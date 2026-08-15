//! The MPSL-backed PHY (the Nordic Multiprotocol Service Layer's radio timeslots).
//!
//! The callback runs the chained slots at full cadence and exchanges packets
//! with the link layer through the caller-provided rings in [`MpslState`].
//! The slot cadence (`SLOT_US`), the slot length (`SLOT_LEN_US`) and the RX
//! poll bound (`RX_POLL`) are const generics chosen by the caller.

pub mod callback;
pub mod radio;
pub mod state;

pub use state::{MpslState, Pkt};

use embassy_time::Duration;
use thunders::config::Address;
use thunders::error::Error;
use thunders::phy::Phy;

use crate::radio_phy::RadioMode;

/// The fallback MPSL slot cadence used before negotiation completes.
/// 500 us is the slowest board's physical minimum, so every board can start
/// here and then renegotiate to `max(central_min, peripheral_min)`.
pub const MPSL_FALLBACK_SLOT_US: u32 = 500;

// The phase-lock (the proportional controller on the peripheral).
pub(crate) const PLL_SWEEP_US: u32 = 2;
pub(crate) const PLL_SWEEP_MISSES: u32 = 8;
// The phase error is in exact us (DWT); gain 1/4 converges in a few catches.
pub(crate) const PLL_GAIN_NUM: i32 = 1;
pub(crate) const PLL_GAIN_DEN: i32 = 4;

/// Snapshot the diagnostic counters: (slots granted, works done, other
/// signals, max in-slot work us). Valid after [`MpslRadioPhy::new`].
pub fn mpsl_stats() -> (u32, u32, u32, u32) {
    unsafe {
        let s = &*(STATE as *const MpslState);
        (s.slot_count, s.done_count.load(core::sync::atomic::Ordering::Relaxed), s.other_signals, s.slot_work_max)
    }
}

/// Snapshot the last RSSI sample (the RADIO RSSISAMPLE register).
pub fn mpsl_rssi() -> u32 {
    unsafe {
        let s = &*(STATE as *const MpslState);
        s.rssi_last
    }
}

/// A named, ergonomic snapshot of the MPSL phase-lock state.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MpslPllSnapshot {
    /// Last chained timeslot distance (us).
    pub distance_us: u32,
    /// END-event stamp of the last catch (us from poll start).
    pub catch_poll_us: u32,
    /// Our measured RX listen window (us).
    pub rx_window_us: u32,
    /// The peer's advertised RX listen window (us).
    pub peer_rx_window_us: u32,
    /// Address events seen in RX polls.
    pub addr_events: u32,
    /// First bytes of the last address-matched packet.
    pub last_rx_hdr: [u8; 14],
    /// Address-event stamp of the last catch (us from poll start).
    pub addr_poll_us: u32,
    /// Completed TX ops.
    pub tx_count: u32,
    /// Echo TX delay from slot start (us).
    pub tx_delay_us: u32,
    /// Consecutive RX misses.
    pub rx_misses: u32,
    /// RX polls that ended with CRC ok.
    pub crc_ok: u32,
    /// RX polls that ended without CRC ok.
    pub crc_bad: u32,
}

/// Snapshot the phase-lock state as a named struct.
pub fn mpsl_pll_snapshot() -> MpslPllSnapshot {
    unsafe {
        let s = &*(STATE as *const MpslState);
        MpslPllSnapshot {
            distance_us: s.slot_distance,
            catch_poll_us: s.catch_poll_us,
            rx_window_us: s.rx_window_us,
            peer_rx_window_us: s.peer_rx_window_us,
            addr_events: s.addr_events,
            last_rx_hdr: s.last_rx_hdr,
            addr_poll_us: s.addr_poll_us,
            tx_count: s.tx_count,
            tx_delay_us: s.tx_delay_us,
            rx_misses: s.rx_misses,
            crc_ok: s.crc_ok,
            crc_bad: s.crc_bad,
        }
    }
}

/// Snapshot the phase-lock state: (chain distance, last catch iter, our RX
/// window us, the peer's advertised RX window us).
pub fn mpsl_pll() -> (u32, u32, u32, u32, u32, [u8; 14], u32, u32, u32, u32, u32, u32) {
    let s = mpsl_pll_snapshot();
    (
        s.distance_us,
        s.catch_poll_us,
        s.rx_window_us,
        s.peer_rx_window_us,
        s.addr_events,
        s.last_rx_hdr,
        s.addr_poll_us,
        s.tx_count,
        s.tx_delay_us,
        s.rx_misses,
        s.crc_ok,
        s.crc_bad,
    )
}

// --- the callback's context (the only global the C callback needs) ---
pub(crate) static mut STATE: *mut () = core::ptr::null_mut();

/// The MPSL radio PHY. The caller provides the [`MpslState`] (the rings, the
/// schedule) and the slot constants (as const generics); the callback drives
/// the chained slots through the state.
pub struct MpslRadioPhy<'d, const SLOT_US: u32, const SLOT_LEN_US: u32, const RX_POLL: u32> {
    state: &'d mut MpslState,
    _mode: RadioMode,
}

impl<'d, const SLOT_US: u32, const SLOT_LEN_US: u32, const RX_POLL: u32>
    MpslRadioPhy<'d, SLOT_US, SLOT_LEN_US, RX_POLL>
{
    /// Open the timeslot session and start the chained slots.
    ///
    /// MPSL must already be initialized; `state` must outlive the phy.
    pub fn new(_radio_mode: RadioMode, state: &'d mut MpslState) -> Self {
        // hfxo_cap_trim(): called by the example before embassy_nrf::init.
        // The DWT cycle counter (the follower's echo TX delay; embassy's
        // 30 us tick is too coarse for echo placement).
        unsafe {
            let demcr = 0xE000_EDFC as *mut u32;
            demcr.write_volatile(demcr.read_volatile() | 1 << 24); // TRCENA
            let dwt_ctrl = 0xE000_1000 as *mut u32;
            dwt_ctrl.write_volatile(dwt_ctrl.read_volatile() | 1); // CYCCNTENA
        }
        // Start at the fallback cadence every board can sustain. The
        // const generics still describe this board's physical minimum
        // (SLOT_US) and its RX poll iteration cap (RX_POLL).
        state.slot_nominal = MPSL_FALLBACK_SLOT_US;
        state.slot_len = MPSL_FALLBACK_SLOT_US.saturating_sub(150);
        state.rx_poll = RX_POLL;
        state.slot_distance = MPSL_FALLBACK_SLOT_US;

        unsafe {
            STATE = state as *mut MpslState as *mut ();
        }

        // The bare path power-cycles the radio before first use; the MPSL's
        // init does not. On the nRF5340 the RX frontend needs that reset.
        #[cfg(feature = "nrf5340-net")]
        {
            use nrf_pac::radio::regs;
            state.radio.power().write_value(regs::Power(0));
            state.radio.power().write_value(regs::Power(1));
        }

        // MODECNF0.RU = Fast BEFORE the session opens: the grant machinery
        // sizes the radio ramp from this register.
        #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
        {
            state
                .radio
                .modecnf0()
                .modify(|w| w.set_ru(nrf_pac::radio::vals::Ru::Fast));
            let ru = state.radio.modecnf0().read().ru().to_bits();
            #[cfg(feature = "defmt")]
            defmt::info!("modecnf0 ru={} (1=Fast)", ru);
        }

        let mut session_id: u8 = 0;
        let ret = unsafe {
            nrf_mpsl::raw::mpsl_timeslot_session_open(Some(callback::timeslot_cb), &mut session_id)
        };
        if ret != 0 {
            #[cfg(feature = "defmt")]
            defmt::error!("mpsl_timeslot_session_open failed: {}", ret);
        } else {
            state.session_id = session_id;
            state.first_request = true;
            let ret = callback::mpsl_request_timeslot(state);
            #[cfg(feature = "defmt")]
            defmt::info!("first request ret={}", ret);
        }

        // The nRF54L needs its RADIO/TIMER NVIC lines + the RRAM out of
        // PowerOff (the chained arming asserts otherwise).
        #[cfg(feature = "_nrf54")]
        unsafe {
            use embassy_nrf::interrupt::typelevel::{Interrupt as _, RADIO_0, TIMER10};
            RADIO_0::enable();
            TIMER10::enable();
            let lowpower = nrf_pac::RRAMC_S.power().lowpowerconfig();
            lowpower.write(|w| w.set_mode(nrf_pac::rramc::vals::Mode::Standby));
        }
        // The MPSL's radio processing runs on the RADIO IRQ.
        #[cfg(feature = "nrf5340-net")]
        unsafe {
            use embassy_nrf::interrupt::typelevel::{Interrupt as _, RADIO};
            RADIO::enable();
        }

        Self {
            state,
            _mode: _radio_mode,
        }
    }

    /// Wait until the first timeslot is granted and the chain is flowing
    /// (the callback signals the ready gate). Spawn the MPSL's low-priority
    /// task before awaiting this.
    pub async fn wait_ready(&self) {
        self.state.done_signal.wait().await;
    }
}

impl<'d, const SLOT_US: u32, const SLOT_LEN_US: u32, const RX_POLL: u32> Phy
    for MpslRadioPhy<'d, SLOT_US, SLOT_LEN_US, RX_POLL>
{
    type Error = ();

    async fn set_channel(&mut self, ch: u8) {
        self.state.cur_channel = ch;
    }

    async fn set_address(&mut self, addr: &Address) {
        // ESB-style: base0 = __REV(bytewise_bit_swap(base_bytes)), prefix = bit-swap.
        let base_raw = u32::from_le_bytes([addr.0[1], addr.0[2], addr.0[3], addr.0[4]]);
        let mut swapped = 0u32;
        for i in 0..4 {
            let byte = ((base_raw >> (i * 8)) & 0xFF) as u8;
            swapped |= (byte.reverse_bits() as u32) << (i * 8);
        }
        self.state.cur_base0 = swapped.swap_bytes();
        self.state.cur_prefix = addr.0[0].reverse_bits() as u32;
    }

    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        if pkt.len() > 63 {
            return Err(Error::BufferTooSmall);
        }
        // The on-air frame is [length byte | payload].
        self.state.tx_buf[0] = pkt.len() as u8;
        self.state.tx_buf[1..1 + pkt.len()].copy_from_slice(pkt);
        self.state.tx_ptr = core::ptr::null();
        // Snapshot BEFORE publishing the op: done_count is written only by
        // the callback, so waiting for a change cannot miss a completion
        // (the old done-flag clear-after-submit raced the callback and
        // erased it).
        let done_at = self.state.done_count.load(core::sync::atomic::Ordering::Acquire);
        self.state.op_kind = state::OpKind::Tx as u8;
        // Await this slot's completion on the done signal (the callback
        // signals every slot). Bounded as a dead-man if the chain dies.
        // ponytail: a yield-spin costs ~15us/iter in mpsl_low_priority_process
        for _ in 0..10_000 {
            if self.state.done_count.load(core::sync::atomic::Ordering::Acquire) != done_at {
                break;
            }
            self.state.done_signal.wait().await;
        }
        Ok(())
    }

    async fn receive(
        &mut self,
        buf: &mut [u8],
        _timeout: Duration,
    ) -> Result<Option<usize>, Error<Self::Error>> {
        self.state.rx_ptr = buf.as_mut_ptr();
        self.state.rx_cap = buf.len();
        let done_at = self.state.done_count.load(core::sync::atomic::Ordering::Acquire);
        self.state.op_kind = state::OpKind::Rx as u8;
        for _ in 0..10_000 {
            if self.state.done_count.load(core::sync::atomic::Ordering::Acquire) != done_at {
                break;
            }
            self.state.done_signal.wait().await;
        }
        if self.state.rx_ok {
            // The radio left `[len, payload..]`; the Phy contract is the
            // payload only, so shift it. rx_ok already guarantees the
            // frame fits the buffer (len + 1 <= cap).
            let n = self.state.rx_result.min(buf.len() - 1);
            buf.copy_within(1..1 + n, 0);
            Ok(Some(n))
        } else {
            Ok(None)
        }
    }

    async fn flush(&mut self) {}

    fn rx_window_us(&self) -> u16 {
        self.state.rx_window_us as u16
    }

    fn set_peer_rx_window(&mut self, us: u16) {
        self.state.peer_rx_window_us = us as u32;
    }

    fn slot_period_us(&self) -> u16 {
        self.state.slot_nominal as u16
    }

    fn min_slot_period_us(&self) -> u16 {
        SLOT_US as u16
    }

    fn fallback_slot_period_us(&self) -> u16 {
        MPSL_FALLBACK_SLOT_US as u16
    }

    fn align_slot_period(&mut self, us: u16) {
        // Never adopt a cadence faster than this board's physical minimum;
        // a central that advertised a shorter slot must slow down, not the
        // peripheral starve.
        let us = us.max(SLOT_US as u16) as u32;
        self.state.slot_nominal = us;
        self.state.slot_distance = us;
        // Keep the MPSL inter-slot gap rule (>= 150 us) when the cadence
        // changes at runtime, and give the RX poll a usable budget.
        self.state.slot_len = us.saturating_sub(150);
    }
}
