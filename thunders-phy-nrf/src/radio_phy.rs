//! Nordic RADIO PHY implementation for `thunders`.
//!
//! Supports nRF52, nRF53, and nRF54 series chips.

use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};

use embassy_nrf::interrupt::typelevel::{self, Binding, Handler, Interrupt};
use embassy_nrf::{peripherals, Peri};

/// nRF54L HFXO load-cap trim (call once at boot). The 32 MHz crystal's
/// internal capacitors come from FICR.XOSC32MTRIM + the DK's 15 pF target;
/// untrimmed (INTCAP=0) the carrier sits off-frequency and old-IP receivers
/// (nRF52/53) barely decode it, while the 54L's own CFO-tracking RX copes -
/// a deaf-TX/marginal-link pattern. Zephyr does this in soc.c; nobody does
/// it for us.
#[cfg(feature = "_nrf54")]
pub fn hfxo_cap_trim() {
    unsafe {
        let trim = (0x00FF_C620 as *const u32).read_volatile(); // FICR.XOSC32MTRIM
        let slope_field = trim & 0x1FF;
        let slope = (slope_field ^ 0x100) as i32 - 0x100; // 9-bit two's complement
        let offset = ((trim >> 16) & 0x3FF) as i32;
        let cap_ff = 15000i32; // the DK's internal load capacitance (15 pF)
        let mid = (((cap_ff - 5500) * (slope + 791)) + ((offset << 2) * 1000)) >> 8;
        let mut cap = mid / 1000;
        if mid % 1000 >= 500 {
            cap += 1;
        }
        (0x5012_071C as *mut u32).write_volatile(cap as u32); // OSCILLATORS.XO32M.CONFIG.INTCAP
    }
}

/// nRF54L RRAM fast-fetch (call once at boot, before any MPSL use). The
/// code-fetch RAM comes out of reset in a low-latency-critical-unfriendly
/// mode; the MPSL blob's session arming runs on hard deadlines and asserts
/// (observed: MPSL assert 106:179) when instruction fetch is slow. The
/// first timeslot can be granted before the phy constructor returns, so
/// `MpslRadioPhy::new` calls this before opening the session — but calling
/// it at the top of main is cheaper still.
#[cfg(feature = "_nrf54")]
pub fn rramc_fast_fetch() {
    let lowpower = nrf_pac::RRAMC_S.power().lowpowerconfig();
    lowpower.write(|w| w.set_mode(nrf_pac::rramc::vals::Mode::Standby));
}

use embassy_time::Duration;
use nrf_pac::radio::vals::S1incl;
#[cfg(not(feature = "_nrf54"))]
use nrf_pac::radio::vals::{
    Crcinc, Crcstatus, Endian, Len, Map, Mode, Plen, Ru, Skipaddr, State, Txpower,
};
#[cfg(feature = "_nrf54")]
use nrf_pac::radio::vals::{Crcinc, Crcstatus, Endian, Len, Mode, Plen, Skipaddr, State, Txpower};
#[cfg(not(any(feature = "nrf5340-net", feature = "_nrf54")))]
use nrf_pac::RADIO as PAC_RADIO;
#[cfg(feature = "nrf5340-net")]
use nrf_pac::RADIO_NS as PAC_RADIO;
#[cfg(feature = "_nrf54")]
use nrf_pac::RADIO_S as PAC_RADIO;
use thunders::{config::Address, error::Error, phy::Phy};

/// RADIO mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioMode {
    /// 1 Mbps Nordic proprietary.
    Nrf1Mbit,
    /// 2 Mbps Nordic proprietary.
    Nrf2Mbit,
    /// 1 Mbps BLE.
    Ble1Mbit,
    /// 2 Mbps BLE.
    Ble2Mbit,
}

impl RadioMode {
    /// On-air timing constants for the Nordic proprietary modes used by the
    /// link: (preamble + address anchor in us, us per payload byte).
    ///
    /// Nrf1Mbit uses an 8-bit preamble and 1 Mbit data; Nrf2Mbit uses a
    /// 16-bit preamble and 2 Mbit data.
    pub fn air_timing(self) -> (u32, u32) {
        match self {
            RadioMode::Nrf1Mbit => (48, 8),
            RadioMode::Nrf2Mbit => (28, 4),
            RadioMode::Ble1Mbit => (48, 8),
            RadioMode::Ble2Mbit => (28, 4),
        }
    }

    /// Total on-air time for `len` payload bytes plus the 1 length byte and
    /// 2 CRC bytes, from TX start to the end of the frame.
    pub fn airtime_us(self, len: usize) -> u32 {
        let (prefix, byte_us) = self.air_timing();
        prefix + byte_us * (len as u32 + 3)
    }
}

/// PHY-specific error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioError {
    /// Hardware CCM operation failed (the MIC mismatch or the timeout).
    Crypto,
    /// Provided buffer exceeded the RADIO FIFO.
    BufferTooLong,
    /// CRC check failed on a received packet.
    CrcFailed,
}

/// CPU frequency per chip family (the DWT cycle counter ticks at this rate).
#[cfg(feature = "nrf5340-net")]
const CPU_MHZ: u32 = 64;
#[cfg(feature = "nrf52840")]
const CPU_MHZ: u32 = 64;
#[cfg(feature = "_nrf54")]
const CPU_MHZ: u32 = 128;
#[cfg(not(any(feature = "nrf5340-net", feature = "nrf52840", feature = "_nrf54")))]
const CPU_MHZ: u32 = 64;

/// The fixed slot period for the bare software slot scheduler (us). Both
/// roles pace their slot starts to this grid; the beacon advertises it and
/// the follower phase-locks to the central. It must be large enough for the
/// slowest slot on the slowest chip (the RX poll + the setup/tail), with
/// margin for crystal drift between catches.
const BARE_SLOT_PERIOD_US: u32 = 400;

/// The follower's RX window starts this many us after the slot start. It
/// must be after the follower's RXEN ramp so the radio is listening before
/// the frame starts, and before the central's on-air start (setup + ramp).
const BARE_RX_OFFSET_US: u32 = 30;

/// Where the follower expects the central's frame to be on air, relative to
/// the follower's slot start. The address event is a fixed 28 us after the
/// on-air start at 2 Mbps (16-bit preamble + 4-byte address), so the target
/// address stamp is [`BARE_RX_ON_AIR_TARGET_US`] + 28.
const BARE_RX_ON_AIR_TARGET_US: u32 = 50;
/// The corresponding address-event target (on-air + 28 us).
// The address-event target is board-dependent in practice:
// - nRF54L (LM20) peripheral: the original on-air+28 target (78 us) works
//   for both 52840 and 5340 centrals.
// - nRF52/53 (52840/5340) peripheral: the catch position naturally sits
//   much deeper in the RX window; targeting 78 us makes the PLL fight the
//   natural phase and reduces catches. 156 us is the centre of the RX
//   window's useful range.
#[cfg(feature = "_nrf54")]
const BARE_RX_ADDR_TARGET_US: u32 = BARE_RX_ON_AIR_TARGET_US + 28;
#[cfg(not(feature = "_nrf54"))]
const BARE_RX_ADDR_TARGET_US: u32 = 156;

/// Fixed early margin subtracted from the follower's computed on-air start.
/// Keeps the address event inside the peer's RX window across the measured
/// board-to-board slot-start offsets.
const BARE_TX_PHASE_MARGIN_US: i32 = 10;

/// The TX ramp after TXEN (the Fast ramp; MODECNF0.RU is set in `new`).
/// Used by the follower's echo placement to convert the desired on-air time
/// into the TXEN delay.
const BARE_TX_RAMP_US: u32 = 40;

/// The on-air start target for a paced TX slot, relative to the slot start.
/// All central TX slots (burst-begin, burst-send, and the nRF54 plain-TX
/// fallback) are aligned to this offset so the follower's PLL sees the same
/// phase for every slot, not just the first slot of a burst.
const BARE_TX_ON_AIR_TARGET_US: u32 = BARE_RX_ON_AIR_TARGET_US;

/// The follower's acquisition sweep walks the grid by +2 us per slot. The
/// initial sweep starts immediately; after the first catch the follower
/// re-enables it only after this many consecutive misses.
const BARE_SLOT_RESWEEP_MISSES: u32 = 5_000;
/// The sweep offset added to the slot period while sweeping.
const BARE_SLOT_SWEEP_US: u32 = 2;

/// The follower's phase-lock gain and clamp (a one-shot phase step, the
/// software twin of the MPSL PLL).
const BARE_SLOT_GAIN_NUM: i32 = 1;
/// The PLL gain denominator (correction = err * NUM / DEN).
const BARE_SLOT_GAIN_DEN: i32 = 4;
/// The one-shot phase step clamp (±us).
const BARE_SLOT_CORR_CLAMP_US: i32 = 20;

/// Interrupt handler for the custom RADIO driver.
///
/// On nRF52/nRF53, binds to `RADIO`.  On nRF54, binds to `RADIO_0`.
pub struct RadioIrqHandler;

#[cfg(not(feature = "_nrf54"))]
impl Handler<typelevel::RADIO> for RadioIrqHandler {
    unsafe fn on_interrupt() {
        let r = PAC_RADIO;
        r.intenclr().write(|w| w.0 = 0xffff_ffff);
    }
}

#[cfg(feature = "_nrf54")]
impl Handler<typelevel::RADIO_0> for RadioIrqHandler {
    unsafe fn on_interrupt() {
        let r = PAC_RADIO;
        r.intenclr0(0).write(|w| w.0 = 0xffff_ffff);
    }
}

/// Nordic RADIO PHY.
pub struct NrfRadioPhy<'d> {
    r: nrf_pac::radio::Radio,
    _irq: PhantomData<&'d ()>,
    /// The bare software slot scheduler is enabled (the link path, not the
    /// raw one-way TX bench).
    paced: bool,
    /// True on the peripheral: sweep + phase-lock to the central's grid.
    follower: bool,
    /// The slot grid period in us (the beacon advertises this).
    period_us: u32,
    /// Scheduled start of the next slot (DWT cycles).
    next_slot_cyc: Option<u32>,
    /// Actual start of the current slot (DWT cycles).
    slot_start_cyc: u32,
    /// Consecutive RX polls without an address match (the sweep trigger).
    rx_misses: u32,
    /// True while the follower is sweeping its grid for the first catch.
    sweep: bool,
    /// Our measured RX listen window (us), advertised in the beacon.
    rx_window_us: u32,
    /// The peer's advertised RX listen window (us); 0 = unknown.
    peer_rx_window_us: u32,
    /// Address-event stamp of the last RX poll (us from RXEN).
    addr_poll_us: u32,
    /// Last forward-catch address event measured from the slot start (us).
    /// Used by the follower's echo placement so the echo lands in the
    /// central's RX window even when the forward PLL holds a non-zero phase.
    last_addr_slot_us: u32,
    /// Preamble/address anchor duration for the configured radio mode (us).
    air_prefix_us: u32,
    /// On-air duration per payload/CRC byte for the configured mode (us).
    air_byte_us: u32,
    /// Fastest slot period this board can sustain for the configured mode.
    min_period_us: u32,
    /// Number of paced slot starts seen by this PHY (raw phase diagnostic).
    slot_count: u32,
    /// Peer's advertised TXEN offset and ramp (0 = unknown). Used by the
    /// follower echo formula instead of assuming the master's TX starts at
    /// `BARE_TX_ON_AIR_TARGET_US`.
    peer_tx_en_offset_us: u32,
    peer_tx_ramp_us: u32,
    /// Extra early margin applied to the follower's computed on-air start.
    /// Defaults to [`BARE_TX_PHASE_MARGIN_US`]; boards can override it with
    /// [`Self::set_tx_phase_margin_us`].
    tx_phase_margin_us: i32,
}

// Static packet buffers: the radio DMA must reach them. Stack-allocated
// buffers (the old struct fields) were not DMA-visible on the nRF5340 net core.
static mut TX_BUF: [u8; 256] = [0; 256];
static mut RX_BUF: [u8; 256] = [0; 256];

/// Debug counters for the nRF54 receive path (read from the app).
static RX_STATS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// 1 = TX END fired within the poll bound, 2 = TX poll timed out.
/// Max TX poll iterations (diagnostics).
/// Max RX poll iterations (diagnostics).
static RX_POLL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Last RX poll duration in us (the DWT-capped listen window).
static RX_POLL_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Last RSSI sample from the RADIO RSSISAMPLE register (RX diag).
static RX_RSSI: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// A named, ergonomic snapshot of the bare slot scheduler's state.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BarePllSnapshot {
    /// Last address-event stamp from RXEN (us).
    pub addr_poll_us: u32,
    /// Last address-event stamp from the slot start (us).
    pub addr_slot_us: u32,
    /// Last applied PLL correction (us, signed).
    pub corr_us: i32,
    /// Consecutive RX polls without an address match.
    pub rx_misses: u32,
    /// True while the follower is sweeping.
    pub sweep: bool,
    /// The peer's advertised RX window (us).
    pub peer_rx_window_us: u32,
    /// The slot period in use (us).
    pub period_us: u32,
    /// Last TX on-air offset from the slot start (us).
    pub tx_on_air_us: u32,
    /// Min TX on-air offset in the current window (us).
    pub tx_air_min_us: u32,
    /// Max TX on-air offset in the current window (us).
    pub tx_air_max_us: u32,
    /// Last RX radio-op duration (us).
    pub rx_op_us: u32,
    /// Last TX radio-op duration (us).
    pub tx_op_us: u32,
    /// Address events seen in the current window.
    pub addr_events: u32,
}

/// Snapshot the bare slot scheduler state as a named struct.
pub fn bare_pll() -> BarePllSnapshot {
    BarePllSnapshot {
        addr_poll_us: BARE_ADDR_POLL_US.load(core::sync::atomic::Ordering::Relaxed),
        addr_slot_us: BARE_ADDR_SLOT_US.load(core::sync::atomic::Ordering::Relaxed),
        corr_us: BARE_PLL_CORR_US.load(core::sync::atomic::Ordering::Relaxed),
        rx_misses: BARE_RX_MISSES.load(core::sync::atomic::Ordering::Relaxed),
        sweep: BARE_SWEEP.load(core::sync::atomic::Ordering::Relaxed) != 0,
        peer_rx_window_us: BARE_PEER_WINDOW_US.load(core::sync::atomic::Ordering::Relaxed),
        period_us: BARE_EFFECTIVE_PERIOD_US.load(core::sync::atomic::Ordering::Relaxed),
        tx_on_air_us: BARE_TX_ON_AIR_US.load(core::sync::atomic::Ordering::Relaxed),
        tx_air_min_us: BARE_TX_AIR_MIN.load(core::sync::atomic::Ordering::Relaxed),
        tx_air_max_us: BARE_TX_AIR_MAX.load(core::sync::atomic::Ordering::Relaxed),
        rx_op_us: BARE_RX_OP_US.load(core::sync::atomic::Ordering::Relaxed),
        tx_op_us: BARE_TX_OP_US.load(core::sync::atomic::Ordering::Relaxed),
        addr_events: BARE_ADDR_EVENTS.load(core::sync::atomic::Ordering::Relaxed),
    }
}

/// Bare scheduler diagnostics (read from the app context).
static BARE_ADDR_POLL_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_RX_MISSES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_SWEEP: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_PEER_WINDOW_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_EFFECTIVE_PERIOD_US: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
static BARE_TX_ON_AIR_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_TX_AIR_MIN: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(u32::MAX);
static BARE_TX_AIR_MAX: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_RX_OP_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_TX_OP_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_ADDR_SLOT_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BARE_ADDR_EVENTS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Raw paced TX/RX phase histograms (`slot_count % 10`), mirroring the

static BARE_PLL_CORR_US: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);
#[cfg(feature = "defmt")]
static RXOK_LOG: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// TX sub-phase cycle counters (the DWT CYCCNT deltas; diagnostic).
static TX_PHASE_DIS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static TX_PHASE_SETUP: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static TX_PHASE_POLL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Read the ARM DWT cycle counter (the CPU cycles; the ground-truth timing).
#[inline(always)]
fn dwt_cycles() -> u32 {
    unsafe { (0xE000_1004 as *mut u32).read_volatile() }
}

/// Enable the DWT cycle counter (the bare scheduler and the RX time cap use
/// it; on the nRF53/nRF54 the DWT is not on by default).
fn dwt_enable() {
    unsafe {
        let demcr = 0xE000_EDFC as *mut u32;
        demcr.write_volatile(demcr.read_volatile() | 1 << 24); // TRCENA
        let dwt_ctrl = 0xE000_1000 as *mut u32;
        dwt_ctrl.write_volatile(dwt_ctrl.read_volatile() | 1); // CYCCNTENA
    }
}

/// Cumulative RADIO STATE reads inside disable() (the disable latency proxy).
static DISABLE_READS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
// bit0: phyend fired, bit1: crc ok, bit2: crc fail, bit3: timeout

impl<'d> NrfRadioPhy<'d> {
    /// Create a new RADIO PHY (nRF52 / nRF53 variant).
    #[cfg(not(feature = "_nrf54"))]
    pub fn new(
        _radio: Peri<'d, peripherals::RADIO>,
        _irq: impl Binding<typelevel::RADIO, RadioIrqHandler> + 'd,
        mode: RadioMode,
    ) -> Self {
        let r = PAC_RADIO;

        // Reset the peripheral.
        r.power().write(|w| w.set_power(false));
        r.power().write(|w| w.set_power(true));

        // nRF5340 anomaly 158: powering RADIO clears factory analog trims.
        // Reapply every FICR trim entry targeting the RADIO register block.
        #[cfg(feature = "nrf5340-net")]
        {
            for index in 0..32 {
                let trim = nrf_pac::FICR_NS.trimcnf(index);
                let address = trim.addr().read();
                if address & 0xFFFF_F000 == r.as_ptr() as u32 {
                    unsafe { (address as *mut u32).write_volatile(trim.data().read()) };
                }
            }

            // The nRF5340 high-voltage RADIO supply adds 3 dB to the
            // TXPOWER=0 dBm setting, yielding the chip's +3 dBm maximum.
            nrf_pac::VREQCTRL_NS
                .vregradio()
                .vreqh()
                .modify(|w| w.set_vreqh(true));
        }

        let (mode_val, plen) = match mode {
            RadioMode::Nrf1Mbit => (Mode::Nrf1mbit, Plen::_8bit),
            // Nordic ESB uses a 16-bit preamble at 2 Mbps (2 Mbit sync needs it).
            RadioMode::Nrf2Mbit => (Mode::Nrf2mbit, Plen::_16bit),
            RadioMode::Ble1Mbit => (Mode::Ble1mbit, Plen::_8bit),
            RadioMode::Ble2Mbit => (Mode::Ble2mbit, Plen::_16bit),
        };

        r.mode().write(|w| w.set_mode(mode_val));
        // nRF5340 anomaly 117: changing MODE requires updating the hidden
        // modulation configuration register from FICR.
        #[cfg(feature = "nrf5340-net")]
        unsafe {
            let source = match mode {
                RadioMode::Nrf2Mbit | RadioMode::Ble2Mbit => 0x01FF_0084 as *const u32,
                _ => 0x01FF_0080 as *const u32,
            };
            (0x4100_8588 as *mut u32).write_volatile(source.read_volatile());
        }
        // Fast ramp (40 us, not the 129 us Legacy): both the slot scheduler's
        // TX/RX offset assumptions and the tight RX window depend on it.
        r.modecnf0().modify(|w| w.set_ru(Ru::Fast));

        // CRC config varies by mode. Nordic ESB includes the address in CRC.
        let crc_len = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => Len::Three,
            _ => Len::Two,
        };
        r.crccnf().write(|w| {
            w.set_len(crc_len);
            w.set_skipaddr(Skipaddr::Include);
        });
        let crc_poly = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 0x0000_065B,
            _ => 0x0001_1021,
        };
        r.crcpoly().write(|w| w.set_crcpoly(crc_poly));
        let crc_init = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 0x55_5555,
            _ => 0xFFFF,
        };
        r.crcinit().write(|w| w.set_crcinit(crc_init));

        // Packet configuration.
        r.pcnf0().write(|w| {
            // lflen=8: [length byte | payload]; the length byte lands in
            // RX_BUF[0].
            w.set_lflen(8);
            w.set_s0len(false);
            w.set_s1len(0);
            w.set_s1incl(S1incl::Automatic);
            w.set_cilen(0);
            w.set_plen(plen);
            w.set_crcinc(Crcinc::Exclude);
            w.set_termlen(0);
        });

        let balen = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 3u8,
            _ => 4u8,
        };
        // Whitening disabled: Nordic ESB does not whiten, and keeping it off
        // removes any cross-generation whitening-algorithm mismatch.
        let whiteen = false;
        r.pcnf1().write(|w| {
            w.set_maxlen(255);
            w.set_statlen(0);
            w.set_balen(balen);
            w.set_endian(Endian::Big);
            w.set_whiteen(whiteen);
        });

        // Board TX power ceiling: the 52840 runs +8 dBm; the nRF5340 net
        // core uses TXPOWER=0 dBm plus the +3 dB high-voltage gain path.
        #[cfg(feature = "nrf52840")]
        r.txpower().write(|w| w.set_txpower(Txpower::Pos8dBm));
        #[cfg(feature = "nrf5340-net")]
        r.txpower().write(|w| w.set_txpower(Txpower::_0dBm));
        // Whitening IV kept for completeness (unused while whiteen=false).
        #[cfg(not(feature = "_nrf54"))]
        r.datawhiteiv().write(|w| w.set_datawhiteiv(25));
        r.frequency().write(|w| {
            w.set_frequency(2);
            w.set_map(Map::Default);
        });

        // Base address and prefix default.
        r.base0().write_value(0);
        r.prefix0().write(|w| w.set_ap0(0));
        r.txaddress().write(|w| w.set_txaddress(0));
        r.rxaddresses().write(|w| w.set_addr0(true));

        // Enable the NVIC interrupt.
        typelevel::RADIO::unpend();
        unsafe { typelevel::RADIO::enable() };

        dwt_enable();
        let (air_prefix_us, air_byte_us) = mode.air_timing();

        Self {
            r,
            _irq: PhantomData,
            paced: false,
            follower: false,
            period_us: 0,
            next_slot_cyc: None,
            slot_start_cyc: 0,
            rx_misses: 0,
            sweep: false,
            rx_window_us: 0,
            peer_rx_window_us: 0,
            addr_poll_us: 0,
            last_addr_slot_us: 0,
            air_prefix_us,
            air_byte_us,
            min_period_us: 0,
            slot_count: 0,
            peer_tx_en_offset_us: 0,
            peer_tx_ramp_us: 0,
            tx_phase_margin_us: BARE_TX_PHASE_MARGIN_US,
        }
    }

    /// Create a new RADIO PHY (nRF54 variant).
    #[cfg(feature = "_nrf54")]
    pub fn new(
        _radio: Peri<'d, peripherals::RADIO>,
        _irq: impl Binding<typelevel::RADIO_0, RadioIrqHandler> + 'd,
        mode: RadioMode,
    ) -> Self {
        let r = PAC_RADIO;

        // nRF54 RADIO does not have a power register; the peripheral
        // is always powered when the system is on.

        // NOTE: hfxo_cap_trim() must run BEFORE embassy_nrf::init() starts the
        // HFXO; the examples call it at the top of main().

        // nRF54L errata workarounds required for correct radio operation
        // (mirrors Nordic's ESB driver, esb_glue.c):
        //  - 54L/39: the CLOCK PLL must be started (radio reference clock).
        //  - 54L/20: constant-latency mode (radio timing vs power manager).
        {
            let c = nrf_pac::CLOCK_S;
            c.events_pllstarted().write_value(0);
            c.tasks_pllstart().write_value(1);
            let mut ok = false;
            for _ in 0..1_000_000 {
                if c.events_pllstarted().read() != 0 {
                    ok = true;
                    break;
                }
            }
            nrf_pac::POWER_S.tasks_constlat().write_value(1);
            // Errata 54L/49 workaround (first on-air payload bits): hidden radio reg.
            unsafe {
                (0x5008C58C as *mut u32).write_volatile(1);
            }
            // Report PLL state via a scratch RAM word (readable post-run).
            unsafe {
                (0x2007F000 as *mut u32).write_volatile(if ok { 0x504C4C4F } else { 0x504C4C58 });
            }
        }

        let (mode_val, plen) = match mode {
            RadioMode::Nrf1Mbit => (Mode::Nrf1mbit, Plen::_8bit),
            // Nordic ESB uses a 16-bit preamble at 2 Mbps (2 Mbit sync needs it).
            RadioMode::Nrf2Mbit => (Mode::Nrf2mbit, Plen::_16bit),
            RadioMode::Ble1Mbit => (Mode::Ble1mbit, Plen::_8bit),
            RadioMode::Ble2Mbit => (Mode::Ble2mbit, Plen::_16bit),
        };

        r.mode().write(|w| w.set_mode(mode_val));

        let crc_len = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => Len::Three,
            _ => Len::Two,
        };
        r.crccnf().write(|w| {
            w.set_len(crc_len);
            w.set_skipaddr(Skipaddr::Include);
        });
        let crc_poly = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 0x0000_065B,
            _ => 0x0001_1021,
        };
        r.crcpoly().write(|w| w.set_crcpoly(crc_poly));
        let crc_init = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 0x55_5555,
            _ => 0xFFFF,
        };
        r.crcinit().write(|w| w.set_crcinit(crc_init));

        // Packet configuration.
        r.pcnf0().write(|w| {
            // lflen=8: [length byte | payload]; the length byte lands in
            // RX_BUF[0].
            w.set_lflen(8);
            w.set_s0len(false);
            w.set_s1len(0);
            w.set_s1incl(S1incl::Automatic);
            w.set_cilen(0);
            w.set_plen(plen);
            w.set_crcinc(Crcinc::Exclude);
            w.set_termlen(0);
        });

        let balen = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 3u8,
            _ => 4u8,
        };
        // Whitening disabled: Nordic ESB does not whiten, and keeping it off
        // removes any cross-generation whitening-algorithm mismatch.
        let whiteen = false;
        r.pcnf1().write(|w| {
            w.set_maxlen(255);
            w.set_statlen(0);
            w.set_balen(balen);
            w.set_endian(Endian::Big);
            w.set_whiteen(whiteen);
        });

        // The 54L runs +8 dBm: its reverse link into the 52/53 is marginal
        // at 0 dBm even on-frequency (the MPSL backend does the same).
        r.txpower().write(|w| w.set_txpower(Txpower::Pos8dBm));
        // Whitening IV kept for completeness (unused while whiteen=false).
        #[cfg(feature = "_nrf54")]
        r.datawhite().write(|w| w.set_iv(25));
        r.frequency().write(|w| {
            w.set_frequency(2);
            w.set_map(false);
        });

        // Base address and prefix default.
        r.base0().write_value(0);
        r.prefix0().write(|w| w.set_ap0(0));
        r.txaddress().write(|w| w.set_txaddress(0));
        r.rxaddresses().write(|w| w.set_addr0(true));

        // Shorts: READY_START + PHYEND_DISABLE work for both TX and RX on
        // nRF54 (see Nordic's ESB driver). Set once, never rewrite (rewriting
        // shorts per-operation was observed to break RX).
        r.shorts().write(|w| {
            w.set_ready_start(true);
            w.set_phyend_disable(true);
        });

        // Enable the NVIC interrupt.
        typelevel::RADIO_0::unpend();
        unsafe { typelevel::RADIO_0::enable() };

        dwt_enable();
        let (air_prefix_us, air_byte_us) = mode.air_timing();

        Self {
            r,
            _irq: PhantomData,
            paced: false,
            follower: false,
            period_us: 0,
            next_slot_cyc: None,
            slot_start_cyc: 0,
            rx_misses: 0,
            sweep: false,
            rx_window_us: 0,
            peer_rx_window_us: 0,
            addr_poll_us: 0,
            last_addr_slot_us: 0,
            air_prefix_us,
            air_byte_us,
            min_period_us: 0,
            slot_count: 0,
            peer_tx_en_offset_us: 0,
            peer_tx_ramp_us: 0,
            tx_phase_margin_us: BARE_TX_PHASE_MARGIN_US,
        }
    }

    /// Enable the bare software slot scheduler.
    ///
    /// `follower` is true on the peripheral: it sweeps for the central's
    /// grid and phase-locks once frames are caught. The central free-runs
    /// on the same fixed period and advertises it in its beacons.
    pub fn set_paced(&mut self, follower: bool) {
        self.paced = true;
        self.follower = follower;
        self.min_period_us = BARE_SLOT_PERIOD_US;
        self.period_us = BARE_SLOT_PERIOD_US;
        self.next_slot_cyc = None;
        self.slot_start_cyc = 0;
        self.slot_count = 0;
        self.rx_misses = 0;
        self.sweep = follower;
        self.rx_window_us = 0;
        self.peer_rx_window_us = 0;
        self.peer_tx_en_offset_us = 0;
        self.peer_tx_ramp_us = 0;
        self.addr_poll_us = 0;
        self.last_addr_slot_us = 0;
        dwt_enable();
    }

    /// Override the bare slot period for a mode that needs more airtime than
    /// the 2 Mbit default 400 us. Call before the first `frame`.
    pub fn set_paced_period_us(&mut self, period_us: u32) {
        debug_assert!(self.paced);
        debug_assert!(self.next_slot_cyc.is_none());
        let period_us = period_us.max(BARE_SLOT_PERIOD_US);
        self.min_period_us = period_us;
        self.period_us = period_us;
    }

    /// Override the follower's early TX margin. Call before the first
    /// `frame`; the central role ignores this value.
    pub fn set_tx_phase_margin_us(&mut self, margin_us: i32) {
        self.tx_phase_margin_us = margin_us;
    }

    /// On-air duration for a `pkt` payload (the PHY adds the length byte and
    /// two CRC bytes).
    #[inline(always)]
    fn airtime_us(&self, len: usize) -> u32 {
        self.air_prefix_us + self.air_byte_us * (len as u32 + 3)
    }

    /// The paced TX start offsets for `len` payload bytes:
    /// `(txen_offset_us, on_air_offset_us)`.
    ///
    /// The follower centers its packet in the peer's advertised RX window
    /// and folds in the measured forward-catch phase; the master uses the
    /// shared `BARE_TX_ON_AIR_TARGET_US` anchor. The same offsets are used
    /// by plain TX and by both halves of a burst, so the forward and reverse
    /// data planes are symmetric.
    fn tx_offsets_us(&self, len: usize) -> (u32, u32) {
        if !self.paced {
            return (0, 0);
        }
        if !self.follower {
            return (
                BARE_TX_ON_AIR_TARGET_US.saturating_sub(BARE_TX_RAMP_US),
                BARE_TX_ON_AIR_TARGET_US,
            );
        }
        if self.peer_rx_window_us == 0 {
            // The follower has not heard a beacon yet: transmit at its own
            // slot start (the acquisition sweep covers the peer window).
            return (0, 0);
        }
        let air = self.airtime_us(len);
        let target_on_air = BARE_RX_OFFSET_US + self.peer_rx_window_us.saturating_sub(air) / 2;
        let peer_tx_air = if self.peer_tx_en_offset_us > 0 && self.peer_tx_ramp_us > 0 {
            self.peer_tx_en_offset_us + self.peer_tx_ramp_us
        } else {
            BARE_TX_ON_AIR_TARGET_US
        };
        let desired_on_air = if self.last_addr_slot_us > 0 {
            let s = self.last_addr_slot_us as i32 - self.air_prefix_us as i32;
            (target_on_air as i32 + s - peer_tx_air as i32)
                .saturating_sub(self.tx_phase_margin_us)
                .max(0) as u32
        } else {
            target_on_air
        };
        (
            desired_on_air.saturating_sub(BARE_TX_RAMP_US),
            desired_on_air,
        )
    }

    /// Busy-wait until the scheduled slot start and schedule the next slot.
    ///
    /// The grid is anchored to the first slot start; a slot that overruns
    /// simply makes the next slot start late (it runs immediately), and the
    /// following slots stay on the original phase.
    #[inline(always)]
    fn slot_wait(&mut self) {
        if !self.paced {
            return;
        }
        let mut now = dwt_cycles();
        match self.next_slot_cyc {
            Some(next) => {
                while ((now.wrapping_sub(next)) as i32) < 0 {
                    now = dwt_cycles();
                }
                self.slot_start_cyc = now;
                self.slot_count = self.slot_count.wrapping_add(1);
                let period_us = self.effective_period_us();
                BARE_EFFECTIVE_PERIOD_US.store(period_us, core::sync::atomic::Ordering::Relaxed);
                self.next_slot_cyc = Some(next.wrapping_add(period_us * CPU_MHZ));
            }
            None => {
                self.slot_start_cyc = now;
                self.next_slot_cyc = Some(now.wrapping_add(self.period_us * CPU_MHZ));
            }
        }
    }

    /// The current slot period, including the follower's acquisition sweep.
    #[inline(always)]
    fn effective_period_us(&self) -> u32 {
        if self.follower && self.sweep {
            self.period_us.saturating_add(BARE_SLOT_SWEEP_US)
        } else {
            self.period_us
        }
    }

    /// Busy-wait until `us` microseconds after the current slot start.
    #[inline(always)]
    fn wait_until_slot_offset_us(&self, us: u32) {
        if !self.paced {
            return;
        }
        let target = self.slot_start_cyc.wrapping_add(us * CPU_MHZ);
        while ((dwt_cycles().wrapping_sub(target)) as i32) < 0 {}
    }

    fn track_tx_air(us: u32) {
        BARE_TX_ON_AIR_US.store(us, core::sync::atomic::Ordering::Relaxed);
        BARE_TX_AIR_MIN.fetch_min(us, core::sync::atomic::Ordering::Relaxed);
        BARE_TX_AIR_MAX.fetch_max(us, core::sync::atomic::Ordering::Relaxed);
    }

    /// Apply a one-shot phase step to the next slot's scheduled start.
    fn nudge_next_slot(&mut self, corr_us: i32) {
        let Some(next) = self.next_slot_cyc else {
            return;
        };
        let corr = (corr_us * BARE_SLOT_GAIN_NUM / BARE_SLOT_GAIN_DEN)
            .clamp(-BARE_SLOT_CORR_CLAMP_US, BARE_SLOT_CORR_CLAMP_US);
        BARE_PLL_CORR_US.store(corr, core::sync::atomic::Ordering::Relaxed);
        let corr_cyc = corr as i64 * CPU_MHZ as i64;
        self.next_slot_cyc = Some(if corr_cyc >= 0 {
            next.wrapping_add(corr_cyc as u32)
        } else {
            next.wrapping_sub((-corr_cyc) as u32)
        });
    }

    /// Clear the RX END/PHYEND event (the cfg-specific event register).
    #[inline(always)]
    fn rx_end_clear(&self) {
        #[cfg(not(feature = "_nrf54"))]
        self.r.events_end().write_value(0);
        #[cfg(feature = "_nrf54")]
        self.r.events_phyend().write_value(0);
    }

    /// True while the RX END/PHYEND event is set.
    #[inline(always)]
    fn rx_end_set(&self) -> bool {
        #[cfg(not(feature = "_nrf54"))]
        return self.r.events_end().read() != 0;
        #[cfg(feature = "_nrf54")]
        return self.r.events_phyend().read() != 0;
    }

    /// Read current radio state.
    fn state(&self) -> State {
        self.r.state().read().state()
    }

    /// Move the RADIO to the Disabled state from any state.
    /// Returns the number of STATE register reads (the disable latency
    /// proxy - the 5340's time driver does not tick, so iteration counts
    /// are the reliable timing here).
    fn disable(&self) -> u32 {
        let r = self.r;
        let mut reads = 0u32;
        loop {
            reads += 1;
            match self.state() {
                State::Disabled => {
                    DISABLE_READS.fetch_add(reads, core::sync::atomic::Ordering::Relaxed);
                    return reads;
                }
                State::RxRu | State::RxIdle | State::TxRu | State::TxIdle => {
                    r.tasks_disable().write_value(1);
                    while self.state() != State::Disabled {
                        reads += 1;
                    }
                    DISABLE_READS.fetch_add(reads, core::sync::atomic::Ordering::Relaxed);
                    return reads;
                }
                State::RxDisable | State::TxDisable => {
                    while self.state() != State::Disabled {
                        reads += 1;
                    }
                    DISABLE_READS.fetch_add(reads, core::sync::atomic::Ordering::Relaxed);
                    return reads;
                }
                State::Rx => {
                    r.tasks_stop().write_value(1);
                    while self.state() != State::RxIdle {}
                }
                State::Tx => {
                    r.tasks_stop().write_value(1);
                    while self.state() != State::TxIdle {}
                }
                _ => {
                    // Unknown/substates: wait a bit and retry.
                    r.tasks_disable().write_value(1);
                    while self.state() != State::Disabled {
                        reads += 1;
                    }
                    DISABLE_READS.fetch_add(reads, core::sync::atomic::Ordering::Relaxed);
                    return reads;
                }
            }
        }
    }

    /// Set the RAM pointer for DMA.
    fn set_packet_ptr(&mut self, ptr: *const u8) {
        self.r.packetptr().write_value(ptr as u32);
        // nRF54: the RXD.MAXCNT (0xED4) must match the buffer capacity or
        // the RX transfers nothing.
        #[cfg(feature = "_nrf54")]
        unsafe {
            let base = self.r.as_ptr() as usize;
            ((base + 0xED4) as *mut u32).write_volatile(64);
        }
    }

    /// Bit-reverse each byte of a 4-byte array (ESB nRF24L01+ compatible).
    fn bytewise_bit_swap(v: u32) -> u32 {
        let mut out = 0u32;
        for i in 0..4 {
            let byte = ((v >> (i * 8)) & 0xFF) as u8;
            let swapped = byte.reverse_bits();
            out |= (swapped as u32) << (i * 8);
        }
        out
    }

    /// Convert a raw 5-byte address into BASE0/PREFIX0 configuration.
    ///
    /// Matches the ESB (Enhanced ShockBurst) nRF24L01+ compatible format:
    /// `base0 = __REV(bytewise_bit_swap(base_bytes))` and
    /// `prefix0 = bytewise_bit_swap(prefix_byte)`.
    fn write_address(&self, addr: &Address) {
        // ESB address: [prefix, base0_0, base0_1, base0_2, base0_3]
        // base0 in little-endian bytes is [addr.0[1], addr.0[2], addr.0[3], addr.0[4]]
        let base_raw = u32::from_le_bytes([addr.0[1], addr.0[2], addr.0[3], addr.0[4]]);
        let base = Self::bytewise_bit_swap(base_raw).swap_bytes();
        self.r.base0().write_value(base);
        // Prefix byte: bit-swap the first address byte.
        let prefix = addr.0[0].reverse_bits();
        self.r.prefix0().write(|w| w.set_ap0(prefix));
    }

    /// Synchronous transmit - no await hop, no executor. Measures the raw
    /// radio TX rate (the async wrapper alone costs the executor hop).
    pub fn transmit_blocking(&mut self, pkt: &[u8]) -> Result<(), Error<RadioError>> {
        if pkt.len() > 255 - 1 {
            return Err(Error::Phy(RadioError::BufferTooLong));
        }
        self.slot_wait();
        let c0 = dwt_cycles();
        self.disable();
        let c1 = dwt_cycles();

        // nRF54 requires PLL enable after the radio is disabled.
        #[cfg(feature = "_nrf54")]
        self.pll_enable();

        // ESB format: [S0 = length][payload].
        let tx_buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUF) };
        tx_buf[0] = pkt.len() as u8;
        tx_buf[1..1 + pkt.len()].copy_from_slice(pkt);

        compiler_fence(Ordering::Release);
        self.set_packet_ptr(tx_buf.as_ptr());

        // Shortcuts: ramp-up TX then start packet automatically; disable at END.
        #[cfg(not(feature = "_nrf54"))]
        self.r.shorts().write(|w| {
            w.set_txready_start(true);
            w.set_end_disable(true);
        });

        self.r.events_end().write_value(0);
        let c2 = dwt_cycles();

        // Align the TX start within the slot. Master and follower share the
        // same offset calculation; only the follower folds in peer window
        // and forward-catch phase.
        if self.paced {
            let (txen_offset, _) = self.tx_offsets_us(pkt.len());
            self.wait_until_slot_offset_us(txen_offset);
        }
        let txen_elapsed = dwt_cycles().wrapping_sub(self.slot_start_cyc) / CPU_MHZ;
        Self::track_tx_air(txen_elapsed + BARE_TX_RAMP_US);
        self.r.tasks_txen().write_value(1);

        // Poll for completion (interrupt-driven waits are unreliable here).
        #[cfg(not(feature = "_nrf54"))]
        {
            // 2 Mbps short packet TX completes in ~60 us; bound the poll so a
            // missed END (radio not ready) cannot stall the 1 kHz link.
            let mut t = 0u32;
            while self.r.events_end().read() == 0 {
                t += 1;
                if t > 40_000 {
                    break;
                }
            }
            self.r.events_end().write_value(0);
        }
        let c3 = dwt_cycles();
        TX_PHASE_DIS.fetch_add(c1 - c0, core::sync::atomic::Ordering::Relaxed);
        TX_PHASE_SETUP.fetch_add(c2 - c1, core::sync::atomic::Ordering::Relaxed);
        TX_PHASE_POLL.fetch_add(c3 - c2, core::sync::atomic::Ordering::Relaxed);
        #[cfg(feature = "_nrf54")]
        {
            let mut t = 0u32;
            while self.r.events_phyend().read() == 0 {
                t += 1;
                if t > 40_000 {
                    break;
                }
            }
            self.r.events_phyend().write_value(0);
        }
        compiler_fence(Ordering::Acquire);
        // The END_DISABLE short already ramps the radio down; skip the
        // explicit disable() wait here (the next op's disable() is
        // state-aware and returns instantly when Disabled).
        BARE_TX_OP_US.store(
            dwt_cycles().wrapping_sub(self.slot_start_cyc) / CPU_MHZ,
            core::sync::atomic::Ordering::Relaxed,
        );
        Ok(())
    }

    fn configure_fixed_len(&mut self, len: u8) {
        self.r.pcnf0().write(|w| {
            w.set_lflen(0);
            w.set_s0len(false);
            w.set_s1len(0);
            w.set_s1incl(S1incl::Automatic);
            w.set_cilen(0);
            w.set_plen(Plen::_16bit);
            w.set_crcinc(Crcinc::Exclude);
            w.set_termlen(0);
        });
        self.r.pcnf1().write(|w| {
            w.set_maxlen(len);
            w.set_statlen(len);
            w.set_balen(4);
            w.set_endian(Endian::Big);
            w.set_whiteen(false);
        });
        self.r.crccnf().write(|w| {
            w.set_len(Len::Two);
            w.set_skipaddr(Skipaddr::Include);
        });
        self.r.crcpoly().write(|w| w.set_crcpoly(0x11021));
        self.r.crcinit().write(|w| w.set_crcinit(0xFFFF));
    }

    /// Transmit one fixed-state packet.
    ///
    /// nRF52/53 disables RADIO at END so TXIDLE cannot emit a continuous
    /// carrier between TDMA packets. nRF54 keeps TXIDLE for its streaming path.
    pub fn state_tx_begin(&mut self, state: &[u8; 6]) {
        self.disable();
        #[cfg(feature = "_nrf54")]
        self.pll_enable();
        self.configure_fixed_len(6);
        let tx_buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUF) };
        tx_buf[..6].copy_from_slice(state);
        compiler_fence(Ordering::Release);
        self.set_packet_ptr(tx_buf.as_ptr());
        self.r.shorts().write(|w| {
            #[cfg(not(feature = "_nrf54"))]
            {
                w.set_txready_start(true);
                w.set_end_disable(true);
            }
            #[cfg(feature = "_nrf54")]
            w.set_ready_start(true);
        });
        self.r.events_ready().write_value(0);
        self.rx_end_clear();
        self.r.tasks_txen().write_value(1);
        while !self.rx_end_set() {}
        self.rx_end_clear();
        #[cfg(not(feature = "_nrf54"))]
        while self.state() != State::Disabled {}
    }

    /// Transmit the next state, re-enabling RADIO when it is not in TXIDLE.
    pub fn state_tx_send(&mut self, state: &[u8; 6]) {
        if self.state() != State::TxIdle {
            self.state_tx_begin(state);
            return;
        }
        let tx_buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUF) };
        tx_buf[..6].copy_from_slice(state);
        compiler_fence(Ordering::Release);
        self.set_packet_ptr(tx_buf.as_ptr());
        self.rx_end_clear();
        self.r.tasks_start().write_value(1);
        while !self.rx_end_set() {}
        self.rx_end_clear();
    }

    /// Start exclusive fixed-state RX and leave RADIO listening.
    pub fn state_rx_begin(&mut self) {
        self.disable();
        #[cfg(feature = "_nrf54")]
        self.pll_enable();
        self.configure_fixed_len(6);
        let rx_ptr = core::ptr::addr_of_mut!(RX_BUF) as *mut u8;
        self.set_packet_ptr(rx_ptr);
        self.r.shorts().write(|w| {
            #[cfg(not(feature = "_nrf54"))]
            w.set_rxready_start(true);
            #[cfg(feature = "_nrf54")]
            w.set_ready_start(true);
        });
        self.r.events_ready().write_value(0);
        self.r.events_address().write_value(0);
        self.rx_end_clear();
        self.r.tasks_rxen().write_value(1);
        while self.r.events_ready().read() == 0 {}
    }

    /// Receive the next state with sender identity and a bounded wait.
    pub fn state_rx_next_from_timeout(
        &mut self,
        state: &mut [u8; 6],
        timeout_us: u32,
    ) -> Option<(bool, u32, u8)> {
        let start = dwt_cycles();
        while !self.rx_end_set() {
            if dwt_cycles().wrapping_sub(start) >= timeout_us * CPU_MHZ {
                return None;
            }
        }
        let stamp = dwt_cycles();
        compiler_fence(Ordering::Acquire);
        let crc_ok = self.r.crcstatus().read().0 & 1 == 1;
        let sender = self.r.rxmatch().read().rxmatch();
        if crc_ok {
            let rx_ptr = core::ptr::addr_of!(RX_BUF) as *const u8;
            unsafe { core::ptr::copy_nonoverlapping(rx_ptr, state.as_mut_ptr(), 6) };
        }
        self.rx_end_clear();
        self.r.events_address().write_value(0);
        self.r.tasks_start().write_value(1);
        Some((crc_ok, stamp, sender))
    }

    /// Receive the next state with a bounded wait.
    pub fn state_rx_next_timeout(
        &mut self,
        state: &mut [u8; 6],
        timeout_us: u32,
    ) -> Option<(bool, u32)> {
        self.state_rx_next_from_timeout(state, timeout_us)
            .map(|(crc_ok, stamp, _)| (crc_ok, stamp))
    }

    /// Receive the next state without a timeout.
    pub fn state_rx_next(&mut self, state: &mut [u8; 6]) -> bool {
        loop {
            if let Some((crc_ok, _)) = self.state_rx_next_timeout(state, u32::MAX / CPU_MHZ) {
                return crc_ok;
            }
        }
    }

    /// Send one fixed two-byte feedback frame and leave RADIO disabled.
    pub fn state_rx_send_feedback(&mut self, diff_us: i16) {
        self.disable();
        #[cfg(feature = "_nrf54")]
        self.pll_enable();
        self.configure_fixed_len(2);
        let tx_buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUF) };
        tx_buf[..2].copy_from_slice(&diff_us.to_le_bytes());
        self.set_packet_ptr(tx_buf.as_ptr());
        self.r.shorts().write(|w| {
            #[cfg(not(feature = "_nrf54"))]
            {
                w.set_txready_start(true);
                w.set_end_disable(true);
            }
            #[cfg(feature = "_nrf54")]
            {
                w.set_ready_start(true);
                w.set_phyend_disable(true);
            }
        });
        self.r.events_disabled().write_value(0);
        self.rx_end_clear();
        self.r.tasks_txen().write_value(1);
        while !self.rx_end_set() {}
        self.rx_end_clear();
        while self.state() != State::Disabled {}
    }

    /// Listen for one two-byte feedback frame and leave RADIO disabled.
    pub fn state_tx_receive_feedback(&mut self, timeout_us: u32) -> bool {
        self.disable();
        #[cfg(feature = "_nrf54")]
        self.pll_enable();
        self.configure_fixed_len(2);
        let rx_ptr = core::ptr::addr_of_mut!(RX_BUF) as *mut u8;
        self.set_packet_ptr(rx_ptr);
        self.r.shorts().write(|w| {
            #[cfg(not(feature = "_nrf54"))]
            {
                w.set_rxready_start(true);
                w.set_end_disable(true);
            }
            #[cfg(feature = "_nrf54")]
            {
                w.set_ready_start(true);
                w.set_phyend_disable(true);
            }
        });
        self.r.events_disabled().write_value(0);
        self.r.events_address().write_value(0);
        self.rx_end_clear();
        let start = dwt_cycles();
        self.r.tasks_rxen().write_value(1);
        while !self.rx_end_set() {
            if dwt_cycles().wrapping_sub(start) >= timeout_us * CPU_MHZ {
                self.disable();
                return false;
            }
        }
        let crc_ok = self.r.crcstatus().read().0 & 1 == 1;
        self.rx_end_clear();
        while self.state() != State::Disabled {}
        crc_ok
    }

    /// Change channel while the exclusive state RADIO is disabled.
    pub fn state_set_channel(&mut self, ch: u8) {
        let freq = ch % 101;
        #[cfg(not(feature = "_nrf54"))]
        self.r.frequency().write(|w| {
            w.set_frequency(freq);
            w.set_map(Map::Default);
        });
        #[cfg(feature = "_nrf54")]
        self.r.frequency().write(|w| {
            w.set_frequency(freq);
            w.set_map(false);
        });
    }

    /// Configure up to eight fixed-state logical addresses with one shared base.
    pub fn state_configure_senders(&mut self, addresses: &[Address]) -> bool {
        if addresses.is_empty() || addresses.len() > 8 {
            return false;
        }
        let base_bytes = &addresses[0].0[1..];
        if addresses
            .iter()
            .any(|address| &address.0[1..] != base_bytes)
        {
            return false;
        }
        self.disable();
        let base_raw =
            u32::from_le_bytes([base_bytes[0], base_bytes[1], base_bytes[2], base_bytes[3]]);
        let base = Self::bytewise_bit_swap(base_raw).swap_bytes();
        let mut prefix0 = 0u32;
        let mut prefix1 = 0u32;
        for (index, address) in addresses.iter().enumerate() {
            let prefix = address.0[0].reverse_bits() as u32;
            if index < 4 {
                prefix0 |= prefix << (index * 8);
            } else {
                prefix1 |= prefix << ((index - 4) * 8);
            }
        }
        self.r.base0().write_value(base);
        self.r.base1().write_value(base);
        self.r
            .prefix0()
            .write_value(nrf_pac::radio::regs::Prefix0(prefix0));
        self.r
            .prefix1()
            .write_value(nrf_pac::radio::regs::Prefix1(prefix1));
        let mask = if addresses.len() == 8 {
            0xFF
        } else {
            (1u32 << addresses.len()) - 1
        };
        self.r
            .rxaddresses()
            .write_value(nrf_pac::radio::regs::Rxaddresses(mask));
        true
    }

    /// Current DWT cycle counter used by raw-TDMA examples.
    pub fn state_cycles(&self) -> u32 {
        dwt_cycles()
    }

    /// Busy-wait until an absolute DWT cycle target.
    pub fn state_wait_until(&self, target: u32) {
        while (target.wrapping_sub(dwt_cycles()) as i32) > 0 {}
    }
}

impl<'d> Phy for NrfRadioPhy<'d> {
    type Error = RadioError;

    async fn set_channel(&mut self, ch: u8) {
        let freq = ch % 101;
        #[cfg(not(feature = "_nrf54"))]
        self.r.frequency().write(|w| {
            w.set_frequency(freq);
            w.set_map(Map::Default);
        });
        #[cfg(feature = "_nrf54")]
        self.r.frequency().write(|w| {
            w.set_frequency(freq);
            w.set_map(false);
        });
    }

    async fn set_address(&mut self, addr: &Address) {
        self.write_address(addr);
    }

    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<RadioError>> {
        // The body is fully synchronous (busy-polls, no awaits) - route
        // through the blocking twin so both paths share one implementation.
        self.transmit_blocking(pkt)
    }

    async fn receive(
        &mut self,
        buf: &mut [u8],
        timeout: Duration,
    ) -> Result<Option<usize>, Error<RadioError>> {
        self.slot_wait();

        // nRF54: the radio auto-disables via the PHYEND_DISABLE short; skip the
        // explicit disable before RX (verified: explicit disable broke RX).
        #[cfg(not(feature = "_nrf54"))]
        self.disable();

        // nRF54 requires PLL enable after the radio is disabled.
        #[cfg(feature = "_nrf54")]
        self.pll_enable();

        let rx_buf = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUF) };
        rx_buf.fill(0);
        let ptr = rx_buf.as_mut_ptr();
        self.set_packet_ptr(ptr);

        // Shortcut: ramp-up RX then start listening automatically.
        #[cfg(not(feature = "_nrf54"))]
        self.r.shorts().write(|w| {
            w.set_rxready_start(true);
            w.set_end_disable(true);
            // Sample RSSI for the packet / idle RX slot; the RSSISAMPLE
            // register reads 0 until RSSI is started (the 52840/5340 RSSI
            // diagnostic was dead without these two shorts).
            w.set_address_rssistart(true);
            w.set_disabled_rssistop(true);
        });

        self.r.events_ready().write_value(0);
        self.r.events_phyend().write_value(0);
        self.r.events_address().write_value(0);
        self.r.events_end().write_value(0);

        // Start the RX window at a fixed offset from the slot start. Both
        // roles transmit at their slot start (plus the TX ramp), so the
        // peer's frame falls inside this window when the grids are aligned.
        self.wait_until_slot_offset_us(BARE_RX_OFFSET_US);

        // The DWT-capped RX poll: the old iteration-count cap made the
        // 100 us listen budget last ~400 us on the 5340 net core (the two
        // chips' loops then free-ran at different rates). The hard cap lets
        // an in-flight frame finish even when it started near the end of
        // the listen budget (the same policy as the MPSL poll).
        let timeout_us = timeout.as_micros() as u32;
        let listen_cyc = timeout_us * CPU_MHZ;
        let hard_cyc = listen_cyc.saturating_add(80 * CPU_MHZ);
        let t_rx = dwt_cycles();
        self.r.tasks_rxen().write_value(1);

        let t0 = t_rx;
        let mut t = 0u32;
        let mut got_end = false;
        let mut addr_seen = false;
        let mut addr_us = 0u32;
        let mut elapsed = 0u32;
        loop {
            if self.rx_end_set() {
                got_end = true;
                break;
            }
            t += 1;
            if !addr_seen && self.r.events_address().read() != 0 {
                addr_seen = true;
                addr_us = dwt_cycles().wrapping_sub(t0) / CPU_MHZ;
                BARE_ADDR_EVENTS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            elapsed = dwt_cycles().wrapping_sub(t0);
            if elapsed >= if addr_seen { hard_cyc } else { listen_cyc } {
                break;
            }
        }
        let elapsed_us = elapsed / CPU_MHZ;
        RX_POLL.store(t, core::sync::atomic::Ordering::Relaxed);
        RX_POLL_US.store(elapsed_us, core::sync::atomic::Ordering::Relaxed);
        self.rx_end_clear();

        compiler_fence(Ordering::Acquire);

        RX_RSSI.store(
            self.r.rssisample().read().rssisample() as u32,
            core::sync::atomic::Ordering::Relaxed,
        );

        // The follower's software PLL: correct on the address anchor (a
        // fixed 28 us after the on-air start) regardless of the decode
        // outcome - the MPSL lesson from the 5340 applies here too.
        if addr_seen {
            self.addr_poll_us = addr_us;
        }
        if self.paced {
            if self.follower {
                if addr_seen {
                    self.rx_misses = 0;
                    self.sweep = false;
                    let setup_us = t_rx.wrapping_sub(self.slot_start_cyc) / CPU_MHZ;
                    let addr_from_slot = setup_us + addr_us;
                    self.last_addr_slot_us = addr_from_slot;
                    BARE_ADDR_SLOT_US.store(addr_from_slot, core::sync::atomic::Ordering::Relaxed);
                    let target = (BARE_RX_ADDR_TARGET_US - 28) + self.air_prefix_us;
                    let err = addr_from_slot as i32 - target as i32;
                    self.nudge_next_slot(err);
                } else {
                    self.rx_misses = self.rx_misses.saturating_add(1);
                    // Re-enable the sweep if the phase is truly lost. The
                    // first sweep starts in set_paced; this covers a lost
                    // link.
                    if self.rx_misses >= BARE_SLOT_RESWEEP_MISSES {
                        self.sweep = true;
                    }
                    self.rx_window_us = elapsed_us;
                }
            } else if !addr_seen {
                // The central advertises its measured listen window so the
                // follower can place its echo (the flags beacon field).
                self.rx_window_us = elapsed_us;
            }
            BARE_ADDR_POLL_US.store(self.addr_poll_us, core::sync::atomic::Ordering::Relaxed);
            BARE_RX_MISSES.store(self.rx_misses, core::sync::atomic::Ordering::Relaxed);
            BARE_SWEEP.store(self.sweep as u32, core::sync::atomic::Ordering::Relaxed);
            BARE_PEER_WINDOW_US.store(
                self.peer_rx_window_us,
                core::sync::atomic::Ordering::Relaxed,
            );
        }

        if got_end {
            RX_STATS.fetch_or(1, core::sync::atomic::Ordering::Relaxed);
        } else {
            RX_STATS.fetch_or(8, core::sync::atomic::Ordering::Relaxed);
        }

        let crc_ok = self.r.crcstatus().read().crcstatus() == Crcstatus::CrcOk;
        if crc_ok {
            RX_STATS.fetch_or(2, core::sync::atomic::Ordering::Relaxed);
        } else {
            RX_STATS.fetch_or(4, core::sync::atomic::Ordering::Relaxed);
        }
        if !got_end || !crc_ok {
            self.disable();
            BARE_RX_OP_US.store(
                dwt_cycles().wrapping_sub(self.slot_start_cyc) / CPU_MHZ,
                core::sync::atomic::Ordering::Relaxed,
            );
            return Ok(None);
        }

        // lflen=8: the length byte lands in RX_BUF[0]; the full PDU is in
        // the buffer (the AMOUNT register reads inconsistently on nRF54L,
        // so use the length byte, not the AMOUNT).
        let payload_len = unsafe { RX_BUF[0] } as usize;
        let rx_buf = unsafe { &*core::ptr::addr_of!(RX_BUF) };
        #[cfg(feature = "defmt")]
        if RXOK_LOG.fetch_add(1, core::sync::atomic::Ordering::Relaxed) % 1000 == 0 {
            defmt::info!("RXOK lenbyte={} buf={:02x}", payload_len, &rx_buf[..16]);
        }
        if payload_len == 0 || payload_len > buf.len() {
            self.disable();
            BARE_RX_OP_US.store(
                dwt_cycles().wrapping_sub(self.slot_start_cyc) / CPU_MHZ,
                core::sync::atomic::Ordering::Relaxed,
            );
            return Err(Error::BufferTooSmall);
        }
        buf[..payload_len].copy_from_slice(&rx_buf[1..1 + payload_len]);
        self.disable();
        BARE_RX_OP_US.store(
            dwt_cycles().wrapping_sub(self.slot_start_cyc) / CPU_MHZ,
            core::sync::atomic::Ordering::Relaxed,
        );
        Ok(Some(payload_len))
    }

    async fn flush(&mut self) {
        self.disable();
        self.r.events_end().write_value(0);
        self.r.events_crcok().write_value(0);
        self.r.events_crcerror().write_value(0);
        self.r.events_ready().write_value(0);
        self.r.events_address().write_value(0);
        self.r.events_payload().write_value(0);
        // Reset the software slot grid so the first frame starts immediately.
        self.next_slot_cyc = None;
        // nRF54 requires explicit PLL enable before TX/RX.
        #[cfg(feature = "_nrf54")]
        self.pll_enable();
    }
}

impl<'d> NrfRadioPhy<'d> {
    /// Enable the RADIO PLL (nRF54 only). On nRF52/53 the PLL
    /// auto-enables on TXEN/RXEN, but RADIO3 requires an explicit
    /// TASKS_PLLEN and waits for EVENTS_PLLREADY before it can
    /// ramp to TX/RX.
    #[cfg(feature = "_nrf54")]
    fn pll_enable(&mut self) {
        self.r.events_pllready().write_value(0);
        self.r.tasks_pllen().write_value(1);
        // Busy-wait for the PLL to settle.
        for _ in 0..100_000 {
            if self.r.events_pllready().read() != 0 {
                self.r.events_pllready().write_value(0);
                return;
            }
        }
    }
}
