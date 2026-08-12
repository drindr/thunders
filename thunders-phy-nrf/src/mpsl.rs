//! MPSL-backed thunders PHY.
//!
//! Nordic's MPSL initializes the radio hardware exactly as Nordic intends
//! (clocks, PLL, DMA, errata) — fixing issues seen with hand-rolled radio
//! init. The thunders link layer (`Central`/`Peripheral`) drives this phy;
//! each TX/RX happens inside a granted MPSL radio timeslot.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::Duration;
use thunders::config::Address;
use thunders::error::Error;
use thunders::phy::Phy;

use crate::radio_phy::RadioMode;

// --- radio base + register offsets (verified against the Nordic SVD) ---
#[cfg(feature = "nrf5340-net")]
const RADIO_BASE: usize = 0x4100_8000; // RADIO_NS (net core)
#[cfg(feature = "_nrf54")]
const RADIO_BASE: usize = 0x5008_A000; // RADIO_S (secure instance, nRF54L)

#[cfg(feature = "nrf5340-net")]
const PCNF0: usize = 0x514;
#[cfg(feature = "_nrf54")]
const PCNF0: usize = 0xE20;
#[cfg(feature = "nrf5340-net")]
const PCNF1: usize = 0x518;
#[cfg(feature = "_nrf54")]
const PCNF1: usize = 0xE28;
#[cfg(feature = "nrf5340-net")]
const CRCCNF: usize = 0x534;
#[cfg(feature = "_nrf54")]
const CRCCNF: usize = 0xE44;
#[cfg(feature = "nrf5340-net")]
const CRCPOLY: usize = 0x538;
#[cfg(feature = "_nrf54")]
const CRCPOLY: usize = 0xE48;
#[cfg(feature = "nrf5340-net")]
const CRCINIT: usize = 0x53C;
#[cfg(feature = "_nrf54")]
const CRCINIT: usize = 0xE4C;
#[cfg(feature = "nrf5340-net")]
const BASE0: usize = 0x51C;
#[cfg(feature = "_nrf54")]
const BASE0: usize = 0xE2C;
#[cfg(feature = "nrf5340-net")]
const PREFIX0: usize = 0x524;
#[cfg(feature = "_nrf54")]
const PREFIX0: usize = 0xE34;
#[cfg(feature = "nrf5340-net")]
const TXADDR: usize = 0x52C;
#[cfg(feature = "_nrf54")]
const TXADDR: usize = 0xE3C;
#[cfg(feature = "nrf5340-net")]
const RXADDRS: usize = 0x530;
#[cfg(feature = "_nrf54")]
const RXADDRS: usize = 0xE40;
#[cfg(feature = "nrf5340-net")]
const FREQ: usize = 0x508;
#[cfg(feature = "_nrf54")]
const FREQ: usize = 0x708;
#[cfg(feature = "nrf5340-net")]
const TXPOWER: usize = 0x50C;
#[cfg(feature = "_nrf54")]
const TXPOWER: usize = 0x710;
#[cfg(feature = "nrf5340-net")]
const MODE: usize = 0x510;
// nRF54L RADIO MODE is at 0x500 (TASKS_TXEN is at 0x0).
#[cfg(feature = "_nrf54")]
const MODE: usize = 0x500;
#[cfg(feature = "nrf5340-net")]
const PACKETPTR: usize = 0x504;
#[cfg(feature = "_nrf54")]
const PACKETPTR: usize = 0xED0;
#[cfg(feature = "nrf5340-net")]
const PHYEND: usize = 0x16C; // EVENTS_PHYEND
#[cfg(feature = "_nrf54")]
const PHYEND: usize = 0x21C;
#[cfg(feature = "nrf5340-net")]
const STATE: usize = 0x400; // RADIO STATE (nRF52-era layout; Disabled=0)
#[cfg(feature = "_nrf54")]
const STATE: usize = 0x520;
#[cfg(feature = "nrf5340-net")]
const CRCSTATUS: usize = 0x400;
#[cfg(feature = "_nrf54")]
const CRCSTATUS: usize = 0xE0C;
// TASKS_DISABLE is 0x10 on both nRF5340 and nRF54L (nRF52 had 0x10 too;
// only the nRF54L task block differs from nRF52 elsewhere).
const DISABLE: usize = 0x10;
#[cfg(feature = "nrf5340-net")]
const SHORTS: usize = 0x200;
#[cfg(feature = "_nrf54")]
const SHORTS: usize = 0x400;
const TASKS_PLLEN: usize = 0x6C;
const EVENTS_PLLREADY: usize = 0x2B0;
// EVENTS_DISABLED: nRF5340 at 0x110, nRF54L at 0x220.
#[cfg(feature = "nrf5340-net")]
const EVENTS_DISABLED: usize = 0x110;
#[cfg(feature = "_nrf54")]
const EVENTS_DISABLED: usize = 0x220;

// READY_START | PHYEND_DISABLE: ramp completes -> START (TX or RX),
// PHYEND (CRC done) -> DISABLE. Matches Nordic ESB on both chip families.
// nRF5340: READY_START=bit0, PHYEND_DISABLE=bit20. nRF54L: bit0, bit19.
// The nRF5340 now uses the BARE-path shorts: END_DISABLE + RXREADY_START
// (RX) / TXREADY_START (TX) + the EVENTS_END poll - the verified-working
// combo on this chip (the PHYEND path was RF-deaf in RX).
#[cfg(feature = "nrf5340-net")]
const SHORTS_RX: u32 = 0x80002; // RXREADY_START(19) | END_DISABLE(1)
#[cfg(feature = "nrf5340-net")]
const SHORTS_TX: u32 = 0x40002; // TXREADY_START(18) | END_DISABLE(1)
#[cfg(feature = "_nrf54")]
const SHORTS_RX: u32 = 0x80001; // READY_START | PHYEND_DISABLE (LM20, verified)
#[cfg(feature = "_nrf54")]
const SHORTS_TX: u32 = 0x80001;
#[cfg(feature = "nrf5340-net")]
const END_EV: usize = 0x10C; // EVENTS_END
/// RX poll bound: must fit the 3.5 ms timeslot with the config/PLL work plus a
/// safety margin (the MPSL asserts 106:179 if the slot work overruns). The
/// 5340's poll loop is ~210 ns/iter -> 12k ~= 2.5 ms.
#[cfg(feature = "nrf5340-net")]
const RX_POLL_BOUND: u32 = 1_000;
/// The nRF54L's poll loop is ~150 ns/iter -> 16k ~= 2.4 ms.
#[cfg(feature = "_nrf54")]
const RX_POLL_BOUND: u32 = 1_000;
#[cfg(feature = "_nrf54")]
const END_EV: usize = 0x21C; // EVENTS_PHYEND (the LM20 completes at PHYEND)

fn rd(off: usize) -> u32 {
    unsafe { (RADIO_BASE as *mut u32).add(off / 4).read_volatile() }
}
fn wr(off: usize, val: u32) {
    unsafe { (RADIO_BASE as *mut u32).add(off / 4).write_volatile(val) }
}



/// Mode value: Nrf2Mbit (0x01) on both chip families.
const MODE_VAL: u32 = 0x01;

/// Run-time cross-check: the raw register offsets above must match the
/// nrf-pac offsets (generated from the official Nordic SVD). A wrong offset
/// is the #1 reason the two boards cannot talk, so fail fast at boot instead
/// of shipping a silent radio misconfiguration.
fn assert_offsets() {
    let r = unsafe { nrf_pac::radio::Radio::from_ptr(core::ptr::null_mut()) };
    // (register offset from nrf-pac, raw offset in this file, name).
    let checks: [(usize, usize, &str); 17] = [
        (unsafe { r.pcnf0().as_ptr() } as usize, PCNF0, "PCNF0"),
        (unsafe { r.pcnf1().as_ptr() } as usize, PCNF1, "PCNF1"),
        (unsafe { r.crccnf().as_ptr() } as usize, CRCCNF, "CRCCNF"),
        (unsafe { r.crcpoly().as_ptr() } as usize, CRCPOLY, "CRCPOLY"),
        (unsafe { r.crcinit().as_ptr() } as usize, CRCINIT, "CRCINIT"),
        (unsafe { r.base0().as_ptr() } as usize, BASE0, "BASE0"),
        (unsafe { r.prefix0().as_ptr() } as usize, PREFIX0, "PREFIX0"),
        (unsafe { r.txaddress().as_ptr() } as usize, TXADDR, "TXADDR"),
        (unsafe { r.rxaddresses().as_ptr() } as usize, RXADDRS, "RXADDRS"),
        (unsafe { r.frequency().as_ptr() } as usize, FREQ, "FREQ"),
        (unsafe { r.txpower().as_ptr() } as usize, TXPOWER, "TXPOWER"),
        (unsafe { r.mode().as_ptr() } as usize, MODE, "MODE"),
        (unsafe { r.packetptr().as_ptr() } as usize, PACKETPTR, "PACKETPTR"),
        (unsafe { r.events_phyend().as_ptr() } as usize, PHYEND, "PHYEND"),
        (unsafe { r.crcstatus().as_ptr() } as usize, CRCSTATUS, "CRCSTATUS"),
        (unsafe { r.shorts().as_ptr() } as usize, SHORTS, "SHORTS"),
        (unsafe { r.tasks_disable().as_ptr() } as usize, DISABLE, "DISABLE"),
    ];
    for (pac, raw, name) in checks {
        if pac != raw {
            report_mismatch(pac, raw, name);
        }
    }
    // nRF5340 has no radio PLL task; nRF54L requires TASKS_PLLEN/EVENTS_PLLREADY.
    #[cfg(feature = "_nrf54")]
    if unsafe { r.tasks_pllen().as_ptr() } as usize != TASKS_PLLEN {
        report_mismatch(unsafe { r.tasks_pllen().as_ptr() } as usize, TASKS_PLLEN, "TASKS_PLLEN");
    }
    #[cfg(feature = "_nrf54")]
    if unsafe { r.events_pllready().as_ptr() } as usize != EVENTS_PLLREADY {
        report_mismatch(unsafe { r.events_pllready().as_ptr() } as usize, EVENTS_PLLREADY, "EVENTS_PLLREADY");
    }
}

/// Record a register offset mismatch where the host can read it, then panic.
/// (Panic without defmt output would otherwise hang silently on the nRF54L
/// boards while probe-rs holds the RTT channel in blocking mode.)
fn report_mismatch(pac: usize, raw: usize, name: &str) {
    let d = unsafe { core::slice::from_raw_parts_mut(0x2000_F020 as *mut u32, 8) };
    d[0] = pac as u32;
    d[1] = raw as u32;
    d[2] = name.as_ptr() as u32;
    d[3] = name.len() as u32;
    panic!("mpsl register offset mismatch: {}", name);
}

static mut SESSIONS: Option<nrf_mpsl::SessionMem<8>> = None;

// --- bridge state between the async phy and the timeslot callback ---
#[repr(u8)]
#[derive(Clone, Copy)]
enum OpKind {
    Idle = 0,
    Tx = 1,
    Rx = 2,
}
static mut OP_KIND: u8 = OpKind::Idle as u8;
/// Set when the RX is event-driven (the callback's SIGNAL_RADIO completes it
/// on the END event); when false, the poll path signals the DONE itself.
static mut TX_DATA: [u8; 64] = [0; 64];
static mut TX_LEN: usize = 0;
static mut TX_OUT: [u8; 64] = [0; 64];
static mut RX_OUT: [u8; 64] = [0; 64];
static mut RX_RESULT: usize = 0;
static mut RX_OK: bool = false;
static DONE: AtomicBool = AtomicBool::new(false);
/// Count of timeslot requests that failed (diagnostic).
pub static REQ_FAILS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Count of timeslot completions (diagnostic).
pub static TS_DONE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

static mut CUR_CHANNEL: u8 = 25;
static mut CUR_BASE0: u32 = 0xE7E7E7E7;
static mut CUR_PREFIX: u32 = 0xE7;

/// Mark the timeslot operation complete (called from the MPSL callback).
pub unsafe fn signal_done() {
    DONE.store(true, Ordering::Release);
    TS_DONE.fetch_add(1, Ordering::Relaxed);
}

/// Configure the radio (inside a granted timeslot) with the current state.
/// The BARE-path wire format (CRC-16, balen 4, crcinc-exclude) is used: it is
/// the verified-working config on this 5340 (the MPSL format was deaf in RX).
unsafe fn radio_configure() {
    wr(MODE, MODE_VAL); // 2 Mbps (0x01) on both families
    wr(PCNF0, 0x0100_0008); // lflen=8, plen=16bit, crcinc=Excl (the bare 2 Mbps)
    wr(PCNF1, 0x0104_00FF); // maxlen=255, balen=4, endian=Big, whiteen=off
    wr(CRCCNF, 0x2); // Len::Two + skipaddr=Include (the bare 2 Mbps CRC-16)
    wr(CRCPOLY, 0x0001_1021);
    wr(CRCINIT, 0x0000_FFFF);
    wr(FREQ, CUR_CHANNEL as u32);
    // TXPOWER: 0 dBm. The encoding differs per chip family - the nRF52/53
    // use 0x00, but the nRF54L's PA output is encoded (0 dBm = 0x18; the raw
    // 0x00 is a reserved value that leaves the PA off - the phantom TX: the
    // radio runs its TX state machine but emits no RF).
    #[cfg(feature = "nrf5340-net")]
    wr(TXPOWER, 0x00);
    #[cfg(feature = "_nrf54")]
    wr(TXPOWER, 0x18);
    wr(BASE0, CUR_BASE0);
    wr(PREFIX0, CUR_PREFIX);
    wr(TXADDR, 0);
    wr(RXADDRS, 0x01);
}

unsafe fn pll_enable() {
    #[cfg(feature = "_nrf54")]
    {
        wr(EVENTS_PLLREADY, 0);
        wr(TASKS_PLLEN, 1);
        let mut i = 0;
        while rd(EVENTS_PLLREADY) == 0 {
            i += 1;
            if i > 100_000 {
                break;
            }
        }
        wr(EVENTS_PLLREADY, 0);
    }
}

/// Perform the pending TX/RX inside the timeslot.
pub unsafe fn timeslot_do_work() {
    radio_configure();
    match OP_KIND {
        x if x == OpKind::Tx as u8 => {
            let len = TX_LEN;
            let data = core::slice::from_raw_parts(TX_DATA.as_ptr(), len);
            // Static TX buffer (the radio DMA needs a never-moved buffer).
            let buf = unsafe { &mut TX_OUT };
            buf.fill(0);
            buf[0] = data.len() as u8;
            buf[1..1 + data.len()].copy_from_slice(data);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            wr(PACKETPTR, buf.as_ptr() as u32);
            pll_enable();
            // nRF54L: the TX DMA amount (TXD.AMOUNT at 0xEE8) must match the
            // PDU length or the radio transmits a 0-length packet.
            #[cfg(feature = "_nrf54")]
            wr(0xEE8, 1 + data.len() as u32);
            wr(SHORTS, SHORTS_TX);
            wr(PHYEND, 0);
            wr(0x10C, 0);
            wr(0x0, 1); // TASKS_TXEN
            let mut i = 0;
            while rd(END_EV) == 0 {
                i += 1;
                if i > 1_000_000 {
                    break;
                }
            }
            wr(END_EV, 0);
            wr(PHYEND, 0);
            wr(DISABLE, 1);
        }
        x if x == OpKind::Rx as u8 => {
            let buf = &mut RX_OUT;
            buf.fill(0);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            pll_enable();
            wr(SHORTS, SHORTS_RX);
            wr(PACKETPTR, buf.as_ptr() as u32);
            wr(0x100, 0);
            wr(0x104, 0);
            wr(0x108, 0);
            wr(0x10C, 0);
            wr(0x110, 0);
            wr(PHYEND, 0);
            // The nRF5340 (net core) starts its RX from the Disabled state
            // exactly like the verified bare path (disable-first + RXEN).
            #[cfg(feature = "nrf5340-net")]
            wr(DISABLE, 1);
            wr(0x4, 1); // TASKS_RXEN
            let rx_poll = RX_POLL_BOUND;
            let mut i = 0;
            while rd(END_EV) == 0 {
                i += 1;
                if i > rx_poll {
                    break;
                }
            }
            wr(END_EV, 0);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            let crc = rd(CRCSTATUS);
            wr(DISABLE, 1);
            wr(DISABLE, 1);
            if crc & 0x1 == 0x1 {
                let len = buf[0] as usize;
                RX_OK = len <= 63;
                RX_RESULT = len.min(63);
            } else {
                RX_OK = false;
                RX_RESULT = 0;
            }
        }
        _ => {}
    }
    signal_done();
}

// --- MPSL timeslot session, owned by the phy ---
static mut SESSION_ID: u8 = 0;
/// What the next timeslot should do (TX or RX).
static mut OP: OpKind = OpKind::Idle;
/// First request must be EARLIEST; afterwards the callback chains NORMAL
/// requests, so we never call mpsl_timeslot_request again from the app.
static mut FIRST_REQUEST: bool = true;
/// The chained (NORMAL) request handed to the MPSL in the callback.
static mut NEXT_REQ: core::mem::MaybeUninit<nrf_mpsl::raw::mpsl_timeslot_request_t> =
    core::mem::MaybeUninit::uninit();

/// MPSL timeslot callback: on START, run the pending TX/RX and chain the
/// next NORMAL timeslot; every other signal returns NULL, exactly like the
/// official MPSL timeslot sample (main.c) - the MPSL handles them.
unsafe extern "C" fn timeslot_cb(
    _sid: u8,
    signal: u32,
) -> *mut nrf_mpsl::raw::mpsl_timeslot_signal_return_param_t {
    static mut RET: nrf_mpsl::raw::mpsl_timeslot_signal_return_param_t =
        unsafe { core::mem::zeroed() };
    if signal == nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_START as u32 {
        timeslot_do_work();
        // Chain the next timeslot: alternate TX/RX.
        OP = if matches!(OP, OpKind::Tx) {
            OpKind::Rx
        } else {
            OpKind::Tx
        };
        let req = NEXT_REQ.assume_init_mut();
        *req = nrf_mpsl::raw::mpsl_timeslot_request_t {
            request_type: nrf_mpsl::raw::MPSL_TIMESLOT_REQ_TYPE_NORMAL as u8,
            params: nrf_mpsl::raw::mpsl_timeslot_request_t__bindgen_ty_1 {
                normal: nrf_mpsl::raw::mpsl_timeslot_request_normal_t {
                    hfclk: nrf_mpsl::raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                    priority: nrf_mpsl::raw::MPSL_TIMESLOT_PRIORITY_HIGH as u8,
                    distance_us: 1000,
                    length_us: 900,
                },
            },
        };
        RET.callback_action = nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
        RET.params.request.p_next = req;
        &mut RET as *mut _
    } else if signal == nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_RADIO as u32 {
        // ESB-style: the MPSL routes the radio interrupt here (SIGNAL_RADIO).
        // The polling TX/RX does not enable the events, so nothing to do.
        RET.callback_action = nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
        &mut RET as *mut _
    } else {
        core::ptr::null_mut()
    }
}

/// Request the first (EARLIEST) timeslot for the given operation; the
/// callback chains subsequent NORMAL timeslots.
fn mpsl_request_timeslot() -> i32 {
    use nrf_mpsl::raw;
    unsafe {
        if !FIRST_REQUEST {
            // Already chained: nothing to do, just wait for the slot.
            return 0;
        }
    }
    let req = raw::mpsl_timeslot_request_t {
        request_type: raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8,
        params: raw::mpsl_timeslot_request_t__bindgen_ty_1 {
            earliest: raw::mpsl_timeslot_request_earliest_t {
                // Same as Zephyr ESB's ts_request_earliest (subsys/esb/esb.c):
                // XTAL_GUARANTEED, NORMAL priority, length filled per-op.
                hfclk: raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                priority: raw::MPSL_TIMESLOT_PRIORITY_HIGH as u8,
                length_us: 900,
                timeout_us: raw::MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US,
            },
        },
    };
    let ret = unsafe { raw::mpsl_timeslot_request(SESSION_ID, &req) };
    unsafe {
        if ret == 0 {
            FIRST_REQUEST = false;
        }
    }
    ret
}

/// Thunders PHY over MPSL timeslots. The application must initialize MPSL and
/// open a timeslot session whose callback calls
/// [`timeslot_do_work`] on `SIGNAL_START`.
pub struct MpslRadioPhy<'d> {
    _mode: RadioMode,
    _phantom: core::marker::PhantomData<&'d ()>,
}

impl<'d> MpslRadioPhy<'d> {
    /// Create a new MPSL-backed PHY.
    ///
    /// Opens an MPSL timeslot session (the callback drives
    /// [`timeslot_do_work`] on `SIGNAL_START`), lets the MPSL low-priority
    /// processing run, and issues the first (EARLIEST) timeslot request.
    /// MPSL must already be initialized.
    pub async fn new(_radio_mode: RadioMode) -> Self {
        #[cfg(any(feature = "nrf5340-net", feature = "_nrf54"))]
        assert_offsets();

        // The bare path power-cycles the radio (POWER off/on = hardware reset)
        // before first use; the MPSL's init does not. On the nRF5340 the RX
        // frontend appears to need that reset (the MPSL RX was RF-deaf: radio
        // in Rx state, rssi=0, no ADDRESS/END/PHYEND, while the LM20 TXed).
        #[cfg(feature = "nrf5340-net")]
        unsafe {
            wr(0xFFC, 0); // RADIO.POWER off
            wr(0xFFC, 1); // on (reset the radio hardware)
        }

        // The phy owns its timeslot session memory and configures it here.
        // Reserve the full session count so a concurrently running SDC (which
        // reconfigures sessions on sdc_enable) still leaves a slot for us.
        // Call this BEFORE building the SDC.
        unsafe {
            let m = SESSIONS.get_or_insert_with(nrf_mpsl::SessionMem::new);
            let ret = nrf_mpsl::raw::mpsl_timeslot_session_count_set(
                m.as_mut_ptr() as *mut _,
                8,
            );
            #[cfg(feature = "defmt")]
            defmt::info!("session_count_set ret={} ptr={:#x}", ret, m.as_mut_ptr() as usize);
        }

        let mut session_id: u8 = 0;
        let ret = unsafe {
            nrf_mpsl::raw::mpsl_timeslot_session_open(Some(timeslot_cb), &mut session_id)
        };
        if ret != 0 {
            // No session: the link will simply never get timeslots.
            #[cfg(feature = "defmt")]
            defmt::error!("mpsl_timeslot_session_open failed: {}", ret);
        } else {
            unsafe {
                SESSION_ID = session_id;
                OP = OpKind::Rx;
                FIRST_REQUEST = true;
            }
            // Insert the first (EARLIEST) request immediately, before any
            // mpsl_low_priority_process pass sees the session: the session
            // processor asserts (106:179) on a freshly opened session (state
            // 1) with no pending request. The request advances the state
            // (-> 3/4, then 5 once the spawned mpsl_task processes), then
            // the RAAL grants the first slot.
            let ret = mpsl_request_timeslot();
            #[cfg(feature = "defmt")]
            defmt::info!("first request ret={} done={}", ret, DONE.load(core::sync::atomic::Ordering::Acquire));
            // Wait for the first slot so the chained NORMAL requests are
            // flowing before the link layer starts. The EARLIEST's arming is
            // deferred (work queue); drive a few processing passes here
            // (thread mode is fine while the mpsl_task is not yet spawned).
            for i in 0..1000 {
                if DONE.load(core::sync::atomic::Ordering::Acquire) {
                    #[cfg(feature = "defmt")]
                    defmt::info!("first slot DONE at iter {}", i);
                    break;
                }
                embassy_futures::yield_now().await;
                unsafe { nrf_mpsl::raw::mpsl_low_priority_process() };
            }
            DONE.store(false, core::sync::atomic::Ordering::Release);
        }

        // MPSL's C init does not enable the RADIO/TIMER/CLOCK NVIC lines on
        // nRF54LM20 (it was built for nrf54l15, whose CLOCK_POWER IRQ is
        // 261; the lm20's is 270); without them the timeslot callback never
        // dispatches and the HFCLK state machine never runs.
        #[cfg(feature = "_nrf54")]
        unsafe {
            use embassy_nrf::interrupt::typelevel::{Interrupt as _, RADIO_0, TIMER10};
            RADIO_0::enable();
            TIMER10::enable();
        }

        Self {
            _mode: _radio_mode,
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<'d> Phy for MpslRadioPhy<'d> {
    type Error = ();

    async fn set_channel(&mut self, ch: u8) {
        unsafe {
            CUR_CHANNEL = ch;
        }
    }

    async fn set_address(&mut self, addr: &Address) {
        // ESB-style: base0 = __REV(bytewise_bit_swap(base_bytes)), prefix = bit-swap.
        let base_raw = u32::from_le_bytes([addr.0[1], addr.0[2], addr.0[3], addr.0[4]]);
        let mut swapped = 0u32;
        for i in 0..4 {
            let byte = ((base_raw >> (i * 8)) & 0xFF) as u8;
            swapped |= (byte.reverse_bits() as u32) << (i * 8);
        }
        unsafe {
            CUR_BASE0 = swapped.swap_bytes();
            CUR_PREFIX = addr.0[0].reverse_bits() as u32;
        }
    }

    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        if pkt.len() > 63 {
            return Err(Error::BufferTooSmall);
        }
        unsafe {
            TX_DATA[..pkt.len()].copy_from_slice(pkt);
            TX_LEN = pkt.len();
            OP_KIND = OpKind::Tx as u8;
            OP = OpKind::Tx;
        }
        DONE.store(false, Ordering::Release);
        let ret = mpsl_request_timeslot();
        if ret != 0 {
            return Ok(());
        }
        while !DONE.load(Ordering::Acquire) {
            embassy_futures::yield_now().await;
        }
        DONE.store(false, Ordering::Release);
        Ok(())
    }

    async fn receive(
        &mut self,
        buf: &mut [u8],
        _timeout: Duration,
    ) -> Result<Option<usize>, Error<Self::Error>> {
        unsafe {
            OP_KIND = OpKind::Rx as u8;
            OP = OpKind::Rx;
        }
        DONE.store(false, Ordering::Release);
        let ret = mpsl_request_timeslot();
        if ret != 0 {
            REQ_FAILS.fetch_add(1, Ordering::Relaxed);
            return Ok(None);
        }
        // Bound the wait so a missing timeslot cannot hang the link layer.
        // Must comfortably outlive the 1.2 ms timeslot: the peripheral can
        // legitimately listen ~900 us before the central TX arrives.
        // The spawned mpsl_task owns mpsl_low_priority_process; calling it
        // from here too would race the session state machine (assert 106:179).
        for _ in 0..100_000 {
            if DONE.load(Ordering::Acquire) {
                break;
            }
            embassy_futures::yield_now().await;
        }
        DONE.store(false, Ordering::Release);
        unsafe {
            if RX_OK && RX_RESULT > 0 && RX_RESULT <= buf.len() {
                buf[..RX_RESULT].copy_from_slice(&RX_OUT[1..1 + RX_RESULT]);
                Ok(Some(RX_RESULT))
            } else {
                Ok(None)
            }
        }
    }

    async fn flush(&mut self) {}
}
