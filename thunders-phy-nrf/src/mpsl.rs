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

// --- radio access via the pac (base + register offsets from the SVD) ---
// The pac's generated instance: RADIO_NS on the nRF5340 net core, RADIO_S
// (secure instance) on the nRF54L. All register access goes through the pac
// accessors - no hand-picked offsets.
#[cfg(feature = "nrf5340-net")]
use nrf_pac::RADIO_NS as RADIO;
#[cfg(feature = "_nrf54")]
use nrf_pac::RADIO_S as RADIO;
use nrf_pac::radio::regs;

// RX completion event: EVENTS_END on the 5340 (the END-completed frame),
// EVENTS_PHYEND on the nRF54L (the LM20's frame completes at PHYEND).
#[cfg(feature = "nrf5340-net")]
fn end_ev_set() -> bool {
    RADIO.events_end().read() != 0
}
#[cfg(feature = "_nrf54")]
fn end_ev_set() -> bool {
    RADIO.events_phyend().read() != 0
}
fn end_ev_clear() {
    #[cfg(feature = "nrf5340-net")]
    RADIO.events_end().write_value(0);
    #[cfg(feature = "_nrf54")]
    RADIO.events_phyend().write_value(0);
}

// SHORTS wiring: READY_START | PHYEND_DISABLE (nRF54L) /
// RXREADY_START|TXREADY_START | END_DISABLE (nRF5340), matching the
// verified ESB-compatible framing on each chip.
#[cfg(feature = "nrf5340-net")]
const SHORTS_RX: u32 = 0x80002; // RXREADY_START(19) | END_DISABLE(1)
#[cfg(feature = "nrf5340-net")]
const SHORTS_TX: u32 = 0x40002; // TXREADY_START(18) | END_DISABLE(1)
#[cfg(feature = "_nrf54")]
const SHORTS_RX: u32 = 0x80001; // READY_START | PHYEND_DISABLE (LM20, verified)
#[cfg(feature = "_nrf54")]
const SHORTS_TX: u32 = 0x80001;
/// RX poll bound: how many iterations the RX listens for the END event
/// inside a granted slot. The slot is 900 us but the poll loop is ~210 ns/iter
/// (5340) / ~150 ns/iter (LM20), so 1000 iters = only ~150-210 us of the slot
/// - the radio sits deaf for the rest. Raised as far as the slot budget allows
/// (the MPSL asserts 106:179 if the slot work overruns):
///   5340: 2000 x 210 ns = 420 us RX + config + TX ~= 720 us of 900 us
///   LM20: 3000 x 150 ns = 450 us RX + config      ~= 550 us of 900 us
/// The free-running chains still drift; this widens the listen window so the
/// overlap lands more often. The real sync (phase-lock) needs a newer MPSL
/// with the ABSOLUTE request type - this vendored version has only
/// EARLIEST + NORMAL, so the fallback is re-anchoring via a fresh EARLIEST.
#[cfg(feature = "nrf5340-net")]
const RX_POLL_BOUND: u32 = 2_000;
#[cfg(feature = "_nrf54")]
const RX_POLL_BOUND: u32 = 3_000;

// --- bridge state between the async phy and the timeslot callback ---
#[repr(u8)]
#[derive(Clone, Copy)]
enum OpKind {
    Idle = 0,
    Tx = 1,
    Rx = 2,
}
static mut OP_KIND: u8 = OpKind::Idle as u8;
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
    let r = RADIO;
    // 2 Mbps (0x01) on both families (the nRF54L pac names the register's
    // struct RadioMode; the nRF5340's is Mode - the values are identical).
    #[cfg(feature = "nrf5340-net")]
    r.mode().write_value(regs::Mode(0x01));
    #[cfg(feature = "_nrf54")]
    r.mode().write_value(regs::RadioMode(0x01));
    r.pcnf0().write_value(regs::Pcnf0(0x0100_0008)); // lflen=8, plen=16bit, crcinc=Excl
    r.pcnf1().write_value(regs::Pcnf1(0x0104_00FF)); // maxlen=255, balen=4, endian=Big, whiteen=off
    r.crccnf().write_value(regs::Crccnf(0x2)); // Len::Two + skipaddr=Include
    r.crcpoly().write_value(regs::Crcpoly(0x0001_1021));
    r.crcinit().write_value(regs::Crcinit(0x0000_FFFF));
    r.frequency().write_value(regs::Frequency(CUR_CHANNEL as u32));
    // TXPOWER: 0 dBm. The encoding differs per chip family - the nRF52/53
    // use 0x00, but the nRF54L's PA output is encoded (0 dBm = 0x18; the raw
    // 0x00 is a reserved value that leaves the PA off - the phantom TX: the
    // radio runs its TX state machine but emits no RF).
    #[cfg(feature = "nrf5340-net")]
    r.txpower().write_value(regs::Txpower(0x00));
    #[cfg(feature = "_nrf54")]
    r.txpower().write_value(regs::Txpower(0x18));
    r.base0().write_value(CUR_BASE0);
    r.prefix0().write_value(regs::Prefix0(CUR_PREFIX));
    r.txaddress().write_value(regs::Txaddress(0));
    r.rxaddresses().write_value(regs::Rxaddresses(0x01));
}

fn pll_enable() {
    #[cfg(feature = "_nrf54")]
    {
        let r = RADIO;
        r.events_pllready().write_value(0);
        r.tasks_pllen().write_value(1);
        let mut i = 0;
        while r.events_pllready().read() == 0 {
            i += 1;
            if i > 100_000 {
                break;
            }
        }
        r.events_pllready().write_value(0);
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
            let r = RADIO;
            r.packetptr().write_value(buf.as_ptr() as u32);
            pll_enable();
            // nRF54L: the TX DMA amount must match the PDU length or the radio
            // transmits a 0-length packet. The SVD has no register at 0xEE8
            // (the pac maps TXD at 0x518), but the write is required on
            // silicon - verified on the LM20.
            #[cfg(feature = "_nrf54")]
            unsafe {
                (r.as_ptr() as *mut u32).add(0xEE8 / 4).write_volatile(1 + data.len() as u32);
            }
            r.shorts().write_value(regs::Shorts(SHORTS_TX));
            r.events_phyend().write_value(0);
            r.events_end().write_value(0);
            r.tasks_txen().write_value(1);
            let mut i = 0;
            while !end_ev_set() {
                i += 1;
                if i > 1_000_000 {
                    break;
                }
            }
            end_ev_clear();
            r.events_phyend().write_value(0);
            r.tasks_disable().write_value(1);
        }
        x if x == OpKind::Rx as u8 => {
            let buf = &mut RX_OUT;
            buf.fill(0);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            pll_enable();
            let r = RADIO;
            r.shorts().write_value(regs::Shorts(SHORTS_RX));
            r.packetptr().write_value(buf.as_ptr() as u32);
            // Clear the pending RX events so the poll keys only on this
            // frame's completion (on the nRF54L the event block sits at
            // 0x200+, not the 5340's 0x100 - the pac accessors handle both).
            r.events_ready().write_value(0);
            r.events_address().write_value(0);
            r.events_payload().write_value(0);
            r.events_end().write_value(0);
            r.events_disabled().write_value(0);
            r.events_phyend().write_value(0);
            // The nRF5340 (net core) starts its RX from the Disabled state
            // exactly like the verified bare path (disable-first + RXEN).
            #[cfg(feature = "nrf5340-net")]
            r.tasks_disable().write_value(1);
            r.tasks_rxen().write_value(1);
            let rx_poll = RX_POLL_BOUND;
            let mut i = 0;
            while !end_ev_set() {
                i += 1;
                if i > rx_poll {
                    break;
                }
            }
            end_ev_clear();
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            let crc = r.crcstatus().read().0;
            r.tasks_disable().write_value(1);
            r.tasks_disable().write_value(1);
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
/// Sync ("connection formed"): the peripheral's chain period is corrected
/// to match the peer's. The MPSL's EARLIEST re-request is blocked while the
/// session is active (-NRF_EAGAIN: the session is not IDLE - the chain never
/// truly ends), so the phase lock instead uses the chained NORMAL request's
/// distance: the app measures the catch-to-catch interval (the peer's period
/// + the peripheral's own phase drift) and nudges the distance +/-1 us to
/// keep the chain period matched. The phase then stays inside the RX window
/// indefinitely - no re-request needed.
static mut DISTANCE_CORRECTION: i32 = 0;
static mut LAST_CATCH: Option<embassy_time::Instant> = None;
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
                    // Distance PLL: the app nudges the period +/-1 us so the
                    // chain stays phase-locked to the peer (see DISTANCE_CORRECTION).
                    distance_us: (1000 + unsafe { DISTANCE_CORRECTION }) as u32,
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
    /// Role-gated "connection formed" sync: only the peripheral re-anchors
    /// its chain on the peer's TX phase. The central must stay free-running
    /// (it is the master - re-anchoring it breaks the PING cadence).
    sync: bool,
    _phantom: core::marker::PhantomData<&'d ()>,
}

impl<'d> MpslRadioPhy<'d> {
    /// Create a new MPSL-backed PHY.
    ///
    /// Opens an MPSL timeslot session (the callback drives
    /// [`timeslot_do_work`] on `SIGNAL_START`), lets the MPSL low-priority
    /// processing run, and issues the first (EARLIEST) timeslot request.
    /// MPSL must already be initialized.
    pub async fn new(_radio_mode: RadioMode, sync: bool) -> Self {
        // The bare path power-cycles the radio (POWER off/on = hardware reset)
        // before first use; the MPSL's init does not. On the nRF5340 the RX
        // frontend appears to need that reset (the MPSL RX was RF-deaf: radio
        // in Rx state, rssi=0, no ADDRESS/END/PHYEND, while the LM20 TXed).
        #[cfg(feature = "nrf5340-net")]
        RADIO.power().write_value(regs::Power(0)); // RADIO.POWER off
        #[cfg(feature = "nrf5340-net")]
        RADIO.power().write_value(regs::Power(1)); // on (reset the radio hardware)

        // The session pool (the count) is configured by the MPSL layer's
        // `with_timeslots`; the phy only opens its own session within it.
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
            // The MPSL's CONSTLAT-only low-latency callbacks (the official
            // upstream) handle the CPU constant-latency. The nRF54L additionally
            // needs the RRAM (code-fetch RAM) out of PowerOff - its high fetch
            // latency made the chained timeslot arming miss its deadline
            // (assert 106:179). The upstream callbacks don't touch it, so set
            // it once here; the RRAM stays in Standby for the session.
            let lowpower = nrf_pac::RRAMC_S.power().lowpowerconfig();
            lowpower.write(|w| w.set_mode(nrf_pac::rramc::vals::Mode::Standby));
        }
        // The MPSL docs: "If the Timeslot API is used for RADIO access, the
        // application is responsible for enabling and disabling the interrupt
        // for RADIO." The MPSL's radio processing (release, SIGNAL_RADIO) runs
        // on the RADIO IRQ - without the NVIC enable it never runs.
        #[cfg(feature = "nrf5340-net")]
        unsafe {
            use embassy_nrf::interrupt::typelevel::{Interrupt as _, RADIO};
            RADIO::enable();
        }

        Self {
            _mode: _radio_mode,
            sync,
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
                if self.sync {
                    // Peripheral: phase-lock the chain to the peer. The
                    // catch-to-catch interval is the peer's 1000 us period
                    // plus the peripheral's own phase drift; nudge the chain
                    // distance to cancel it (bang-bang, +/-1 us per catch).
                    let now = embassy_time::Instant::now();
                    if let Some(prev) = unsafe { LAST_CATCH } {
                        let d = (now - prev).as_micros() as i32;
                        unsafe {
                            DISTANCE_CORRECTION = if d > 1000 {
                                -1
                            } else if d < 1000 {
                                1
                            } else {
                                DISTANCE_CORRECTION
                            };
                        }
                    }
                    unsafe { LAST_CATCH = Some(now) };
                }
                buf[..RX_RESULT].copy_from_slice(&RX_OUT[1..1 + RX_RESULT]);
                Ok(Some(RX_RESULT))
            } else {
                Ok(None)
            }
        }
    }

    async fn flush(&mut self) {}
}
