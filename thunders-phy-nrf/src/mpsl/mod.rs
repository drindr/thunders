//! The MPSL-backed PHY (the Nordic Multiprotocol Service Layer's radio timeslots).
//!
//! The callback runs the chained slots at full cadence and exchanges packets
//! with the link layer through the caller-provided rings in [`MpslState`].
//! The minimum slot cadence (`SLOT_US`) and the RX poll bound (`RX_POLL`)
//! are const generics chosen by the caller. The actual grant length is
//! `slot_nominal - 150` once the cadence is known.

pub mod callback;
pub mod radio;
pub mod state;

pub use state::{MpslState, Pkt};

use embassy_time::Duration;
use thunders::config::Address;
use thunders::error::Error;
use thunders::phy::Phy;

use crate::radio_phy::RadioMode;

/// The MPSL slot cadence floor.
///
/// The physical board minimum is 500 us, but at that cadence the 350 us MPSL
/// grant leaves only ~129 us of legal delay for a 19-byte Data echo after TX
/// setup/ramp/airtime/tail.  Acquisition can measure a peer window that needs
/// 150-180 us of follower delay, so the short SlotRequest still fits while the
/// longer echo is physically impossible to place; the pair then remains in a
/// run-level acquisition dead state.  A 600 us cadence gives a 450 us grant
/// and ~229 us of legal delay, covering the complete 0-210 us acquisition
/// sweep at a 17% cadence cost.
pub const MPSL_FALLBACK_SLOT_US: u32 = 600;

// The phase-lock (the proportional controller on the peripheral).
pub(crate) const PLL_SWEEP_US: u32 = 2;
pub(crate) const PLL_SWEEP_MISSES: u32 = 8;
/// TX tail margin after the on-air frame (END -> DISABLE hand-back).
pub(crate) const MPSL_TX_TAIL_US: i32 = 40;
// The phase error is in exact us (DWT); gain 1/4 converges in a few catches.
pub(crate) const PLL_GAIN_NUM: i32 = 1;
pub(crate) const PLL_GAIN_DEN: i32 = 4;

/// The current MPSL slot count (0 before the first timeslot START).
/// The examples use this to decide TX/RX slots in step with the actual
/// timeslot cadence instead of their own loop counter.
pub fn mpsl_slot_count() -> u32 {
    unsafe {
        let s = &*(STATE as *const MpslState);
        s.slot_count
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
    /// RX polls with an address match, a length byte >= 10, and a bad CRC.
    pub crc_bad_long: u32,
    /// TX length histogram: short (<10) and long (>=10) on-air packets.
    pub tx_short: u32,
    pub tx_long: u32,
    /// Long-packet TX count by slot phase (slot_count % 10).
    pub tx_long_phase: [u32; 10],
    /// All TX ops by raw slot phase.
    pub tx_phase_all: [u32; 10],
    /// All RX ops by raw slot phase.
    pub rx_phase_all: [u32; 10],
    /// Learned follower PLL address target (us from RXEN).
    pub addr_target_us: u32,
    /// Locked catches used for the target calibration.
    pub calib_count: u32,
    /// Our measured RXEN offset from slot START (us).
    pub rx_en_offset_us: u32,
    /// Our measured RXEN -> READY ramp (us).
    pub rx_ramp_us: u32,
    /// Our measured TXEN offset from slot START (us).
    pub tx_en_offset_us: u32,
    /// Our measured TXEN -> READY ramp (us).
    pub tx_ramp_us: u32,
    /// The peer's advertised RXEN offset from its slot START (us).
    pub peer_rx_en_offset_us: u32,
    /// The peer's advertised RXEN -> READY ramp (us).
    pub peer_rx_ramp_us: u32,
    /// The peer's advertised TXEN offset from its slot START (us).
    pub peer_tx_en_offset_us: u32,
    /// The peer's advertised TXEN -> READY ramp (us).
    pub peer_tx_ramp_us: u32,
    /// Last address-matched RX poll that failed CRC (diagnostics).
    pub last_rx_got_end: bool,
    pub last_rx_addr_us: u32,
    pub last_rx_end_us: u32,
    pub last_rx_slot_len: u32,
    pub last_rx_crc: u32,
    pub last_rx_in_flight: bool,
    pub last_rx_len: u32,
    /// Cumulative RSSI sum/count over CRC-ok catches (diff for a
    /// per-window average; dBm = -value).
    pub rssi_catch_sum: u32,
    pub rssi_catch_cnt: u32,
    /// Weakest catch since the previous snapshot (dBm = -value); the
    /// snapshot read resets it.
    pub rssi_catch_max: u32,
    /// Ops skipped because their target slot had already passed (app-loop
    /// lateness counter; cumulative).
    pub op_late: u32,
    /// collect() return-path counters (cumulative): no-op-for-slot /
    /// already-done / catch / empty-listen-or-skip.
    pub coll_noop: u32,
    pub coll_late: u32,
    pub coll_catch: u32,
    pub coll_empty: u32,
    /// TX ops that executed inside their grace slot (cumulative).
    pub op_grace_used: u32,
    /// Max op-publish delay since the previous snapshot (us from slot
    /// START); the snapshot read resets it.
    pub op_publish_max_us: u32,
}

/// Snapshot the phase-lock state as a named struct.
pub fn mpsl_pll_snapshot() -> MpslPllSnapshot {
    unsafe {
        let s = &mut *(STATE as *mut MpslState);
        let rssi_catch_max = s.rssi_catch_max;
        s.rssi_catch_max = 0; // self-resetting "weakest since last read"
        let op_publish_max_us = s.op_publish_max_us;
        s.op_publish_max_us = 0; // self-resetting "tightest budget" probe
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
            crc_bad_long: s.crc_bad_long,
            tx_short: s.tx_short,
            tx_long: s.tx_long,
            tx_long_phase: s.tx_long_phase,
            tx_phase_all: s.tx_phase_all,
            rx_phase_all: s.rx_phase_all,
            addr_target_us: s.addr_target_us,
            calib_count: s.calib_count,
            rx_en_offset_us: s.rx_en_offset_us,
            rx_ramp_us: s.rx_ramp_us,
            tx_en_offset_us: s.tx_en_offset_us,
            tx_ramp_us: s.tx_ramp_us,
            peer_rx_en_offset_us: s.peer_rx_en_offset_us,
            peer_rx_ramp_us: s.peer_rx_ramp_us,
            peer_tx_en_offset_us: s.peer_tx_en_offset_us,
            peer_tx_ramp_us: s.peer_tx_ramp_us,
            last_rx_got_end: s.last_rx_got_end,
            last_rx_addr_us: s.last_rx_addr_us,
            last_rx_end_us: s.last_rx_end_us,
            last_rx_slot_len: s.last_rx_slot_len,
            last_rx_crc: s.last_rx_crc,
            last_rx_in_flight: s.last_rx_in_flight,
            last_rx_len: s.last_rx_len,
            rssi_catch_sum: s.rssi_catch_sum,
            rssi_catch_cnt: s.rssi_catch_cnt,
            rssi_catch_max,
            op_late: s.op_late,
            coll_noop: s.coll_noop,
            coll_late: s.coll_late,
            coll_catch: s.coll_catch,
            coll_empty: s.coll_empty,
            op_grace_used: s.op_grace_used,
            op_publish_max_us,
        }
    }
}

// --- the callback's context (the only global the C callback needs) ---
pub(crate) static mut STATE: *mut () = core::ptr::null_mut();

/// The MPSL radio PHY. The caller provides the [`MpslState`] (the rings, the
/// schedule) and the per-board minimum cadence + RX poll cap (as const
/// generics); the callback drives the chained slots through the state.
pub struct MpslRadioPhy<'d, const SLOT_US: u32, const RX_POLL: u32> {
    state: &'d mut MpslState,
    _mode: RadioMode,
}

impl<'d, const SLOT_US: u32, const RX_POLL: u32> MpslRadioPhy<'d, SLOT_US, RX_POLL>
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
        state.radio_mode = _radio_mode;
        let (air_prefix_us, air_byte_us) = _radio_mode.air_timing();
        state.air_prefix_us = air_prefix_us;
        state.air_byte_us = air_byte_us;

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

    /// Record how deep into the current slot this publish lands. With the
    /// op pipeline the app publishes ~2 slots ahead, so this only approaches
    /// the deadline when the app stalled a whole slot. Self-resetting max.
    fn note_publish_delay(&mut self) {
        let us = radio::cyc().wrapping_sub(self.state.last_start_cyc) / radio::CPU_MHZ;
        // Ignore publishes before the chain is flowing (no START seen yet).
        if us < 4_000 && us > self.state.op_publish_max_us {
            self.state.op_publish_max_us = us;
        }
    }

    /// Publish an RX op for `target` into the parity ring; returns
    /// immediately (the result is picked up by `collect`).
    fn publish_rx(&mut self, buf: &mut [u8], target: u32) {
        self.note_publish_delay();
        let e = &mut self.state.ops[(target % 2) as usize];
        e.rx_ptr = buf.as_mut_ptr();
        e.rx_cap = buf.len();
        e.rx_ok = false;
        e.rx_result = 0;
        e.skipped = false;
        e.kind = state::OpKind::Rx as u8;
        e.target = target;
        e.grace = 0;
        // seq written LAST: a mid-publish IRQ can never consume a
        // half-written op.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
        e.seq = e.seq.wrapping_add(1);
    }

    /// Publish a TX op for `target`; `grace` allows a first-TX-of-run op
    /// to execute one slot late (it still faces a listening peer).
    fn publish_tx(&mut self, pkt: &[u8], target: u32, grace: u8) -> Result<(), Error<()>> {
        if pkt.len() > 63 {
            return Err(Error::BufferTooSmall);
        }
        self.note_publish_delay();
        let e = &mut self.state.ops[(target % 2) as usize];
        // The on-air frame is [length byte | payload].
        e.tx_buf[0] = pkt.len() as u8;
        e.tx_buf[1..1 + pkt.len()].copy_from_slice(pkt);
        e.skipped = false;
        e.kind = state::OpKind::Tx as u8;
        e.target = target;
        e.grace = grace;
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
        e.seq = e.seq.wrapping_add(1);
        Ok(())
    }

    /// Wait for the op published for `slot` (if any) and return its RX
    /// result: `Some(len)` on a catch (the radio left `[len | payload]`
    /// in the published buffer), `None` for a TX op, an idle slot, a
    /// skipped (late) op, or an empty listen.
    async fn collect(&mut self, slot: u32) -> Option<usize> {
        let i = (slot % 2) as usize;
        let seq = self.state.ops[i].seq;
        if self.state.ops[i].target != slot {
            self.state.coll_noop = self.state.coll_noop.wrapping_add(1);
        } else if self.state.ops[i].done_seq == seq {
            self.state.coll_late = self.state.coll_late.wrapping_add(1);
        }
        if self.state.ops[i].target != slot || self.state.ops[i].done_seq == seq {
            // No op was ever published for this slot (an idle slot, or the
            // app stalled and skipped one). Still pace across one START
            // unless the slot has already passed: without this the frame
            // returns instantly and the app spins for the rest of the
            // slot, republishing the same target.
            if self.state.slot_count < slot {
                let before = self.state.slot_count;
                for _ in 0..10_000 {
                    if self.state.slot_count != before {
                        break;
                    }
                    self.state.start_signal.wait().await;
                }
            }
            return None;
        }
        for _ in 0..10_000 {
            if self.state.ops[i].done_seq == seq {
                break;
            }
            self.state.done_signal.wait().await;
        }
        let e = &self.state.ops[i];
        if e.skipped || !e.rx_ok || e.kind != state::OpKind::Rx as u8 {
            self.state.coll_empty = self.state.coll_empty.wrapping_add(1);
            return None;
        }
        self.state.coll_catch = self.state.coll_catch.wrapping_add(1);
        Some(e.rx_result)
    }
}

impl<'d, const SLOT_US: u32, const RX_POLL: u32> Phy for MpslRadioPhy<'d, SLOT_US, RX_POLL>
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
        // Legacy one-slot-ahead synchronous path (composed from the ring
        // primitives): publish for the next slot, wait its completion.
        let target = self.state.slot_count.wrapping_add(1);
        self.publish_tx(pkt, target, 0)?;
        self.collect(target).await;
        Ok(())
    }

    async fn receive(
        &mut self,
        buf: &mut [u8],
        _timeout: Duration,
    ) -> Result<Option<usize>, Error<Self::Error>> {
        let target = self.state.slot_count.wrapping_add(1);
        self.publish_rx(buf, target);
        match self.collect(target).await {
            Some(len) => {
                // The radio left `[len, payload..]`; the Phy contract is
                // the payload only, so shift it.
                let n = len.min(buf.len() - 1);
                buf.copy_within(1..1 + n, 0);
                Ok(Some(n))
            }
            None => Ok(None),
        }
    }

    async fn flush(&mut self) {}

    async fn wait_slot(&mut self) {
        // Pace the app loop across one full slot without a radio op.
        let start_before = self.state.slot_count;
        for _ in 0..10_000 {
            if self.state.slot_count != start_before {
                break;
            }
            self.state.start_signal.wait().await;
        }
        let start_done = self.state.slot_start_done;
        for _ in 0..10_000 {
            if self
                .state
                .done_count
                .load(core::sync::atomic::Ordering::Acquire)
                != start_done
            {
                break;
            }
            self.state.done_signal.wait().await;
        }
    }

    fn rx_window_us(&self) -> u16 {
        self.state.rx_window_us as u16
    }

    fn set_peer_rx_window(&mut self, us: u16) {
        self.state.peer_rx_window_us = us as u32;
    }

    fn op_pipelined(&self) -> bool {
        true
    }

    async fn op_publish_rx(&mut self, buf: &mut [u8], target: u32) {
        self.publish_rx(buf, target);
    }

    async fn op_publish_tx(
        &mut self,
        pkt: &[u8],
        target: u32,
        grace: u8,
    ) -> Result<(), Error<Self::Error>> {
        self.publish_tx(pkt, target, grace)
    }

    async fn op_collect(&mut self, slot: u32) -> Option<usize> {
        self.collect(slot).await
    }

    fn set_peer_rx_en_offset(&mut self, us: u8) {
        if us > 0 {
            self.state.peer_rx_en_offset_us = us as u32;
        }
    }

    fn rx_en_offset_us(&self) -> u8 {
        self.state.rx_en_offset_us as u8
    }

    fn tx_en_offset_us(&self) -> u8 {
        self.state.tx_en_offset_us as u8
    }

    fn rx_ramp_us(&self) -> u8 {
        self.state.rx_ramp_us as u8
    }

    fn tx_ramp_us(&self) -> u8 {
        self.state.tx_ramp_us as u8
    }

    fn set_peer_tx_en_offset(&mut self, us: u8) {
        if us > 0 {
            self.state.peer_tx_en_offset_us = us as u32;
        }
    }

    fn set_peer_rx_ramp(&mut self, us: u8) {
        if us > 0 {
            self.state.peer_rx_ramp_us = us as u32;
        }
    }

    fn set_peer_tx_ramp(&mut self, us: u8) {
        if us > 0 {
            self.state.peer_tx_ramp_us = us as u32;
        }
    }

    fn set_tx_delay_sweep(&mut self, sweep: bool) {
        if sweep && !self.state.tx_delay_sweep {
            self.state.tx_delay_sweep_step = 0;
        }
        self.state.tx_delay_sweep = sweep;
    }

    fn slot_count(&self) -> u32 {
        self.state.slot_count
    }

    fn slot_period_us(&self) -> u16 {
        self.state.slot_nominal as u16
    }

    fn min_slot_period_us(&self) -> u16 {
        SLOT_US as u16
    }

    fn min_short_slot_period_us(&self) -> u16 {
        // Capability, not a theoretical airtime bound. A 450-us experiment
        // left enough nominal RX budget but the 5340 follower's MPSL chain
        // could sustain only 1820-1925 slots/s (central stayed at 2082/s),
        // producing 2/8 deterministic desync failures. Current 2M boards
        // therefore advertise their verified 500-us floor; a future backend
        // may lower its const generic after hardware validation.
        SLOT_US as u16
    }

    fn min_long_slot_period_us(&self) -> u16 {
        (SLOT_US as u16).max(MPSL_FALLBACK_SLOT_US as u16)
    }

    fn schedule_slot_profile(
        &mut self,
        short_us: u16,
        long_us: u16,
        period: u16,
        short_phases: u16,
        phase_offset: u16,
        apply_slot: u32,
    ) {
        let short = short_us.max(self.min_short_slot_period_us()) as u32;
        let long = long_us.max(self.min_long_slot_period_us()) as u32;
        let period = period.max(1) as u32;
        self.state.profile_short_us = short.min(long);
        self.state.profile_long_us = long;
        self.state.profile_period = period;
        self.state.profile_short_phases = (short_phases as u32).min(period);
        self.state.profile_phase_offset = phase_offset as u32 % period;
        self.state.profile_apply_slot = apply_slot;
        self.state.profile_armed = true;
    }

    fn fallback_slot_period_us(&self) -> u16 {
        MPSL_FALLBACK_SLOT_US as u16
    }

    fn align_slot_period(&mut self, us: u16) {
        // Never adopt a cadence faster than this board's physical minimum or
        // the MPSL echo-placement floor.  Negotiation used to shrink the
        // 600-us acquisition cadence back to the boards' 500-us physical
        // minimum, recreating the too-short 350-us grant.
        let us = us
            .max(SLOT_US as u16)
            .max(MPSL_FALLBACK_SLOT_US as u16) as u32;
        self.state.slot_nominal = us;
        self.state.slot_distance = us;
        // A uniform align is the acquisition/fallback mode. A negotiated
        // phase profile is armed later with an absolute apply slot.
        self.state.profile_armed = false;
        // Keep the MPSL inter-slot gap rule (>= 150 us) when the cadence
        // changes at runtime, and give the RX poll a usable budget.
        self.state.slot_len = us.saturating_sub(150);
    }
}
