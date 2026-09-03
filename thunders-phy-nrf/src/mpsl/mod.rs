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
use thunders::mode::{fixed_slot_plan, round_up_us, AirTiming, LinkMode, SlotOverhead};
use thunders::phy::Phy;

use crate::radio_phy::RadioMode;

/// Scheduling granularity applied to mathematically derived slot durations.
pub const MPSL_SLOT_QUANTUM_US: u16 = 5;
/// Extra steady-state callback/radio guard beyond the algebraic fit.
pub const MPSL_STEADY_GUARD_US: u16 = 0;
/// Additional window for the reverse TimeDiff RX event.
pub const MPSL_FEEDBACK_GUARD_US: u16 = 10;
/// One-shot first callback grant. Session/radio initialization performs more
/// register work than steady-state events; subsequent grants use the mode plan.
pub const MPSL_FIRST_CALLBACK_GRANT_US: u32 = 450;

/// Compile-time MPSL schedule for a fixed one-way mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OneWayMpslPlan {
    /// Nominal forward Data event duration.
    pub data_slot_us: u16,
    /// Reverse feedback event duration.
    pub feedback_slot_us: u16,
    /// One long receiver event covering Data and immediate feedback.
    pub receiver_window_us: u16,
    /// Forward Data events per receiver event.
    pub batch: u16,
    /// Whether a reverse TimeDiff event is reserved.
    pub phase_align: bool,
}

impl OneWayMpslPlan {
    /// Hardware events in one transmitter cycle.
    pub const fn transmitter_slots(self) -> u16 {
        self.batch + self.phase_align as u16
    }
}

/// Derive the complete MPSL schedule from mode and radio timing constants.
pub const fn one_way_mpsl_plan<M: LinkMode, const PHASE_ALIGN: bool>(
    air: AirTiming,
    overhead: SlotOverhead,
) -> OneWayMpslPlan {
    let fixed = fixed_slot_plan::<M>(air, overhead);
    let data = round_up_us(
        fixed.data_slot_us.saturating_add(MPSL_STEADY_GUARD_US),
        MPSL_SLOT_QUANTUM_US,
    );
    let feedback_raw = round_up_us(
        fixed.feedback_slot_us.saturating_add(MPSL_STEADY_GUARD_US),
        MPSL_SLOT_QUANTUM_US,
    );
    let feedback_min = data.saturating_add(MPSL_FEEDBACK_GUARD_US);
    let feedback_candidate = if feedback_raw < feedback_min {
        feedback_min
    } else {
        feedback_raw
    };
    let feedback = if PHASE_ALIGN { feedback_candidate } else { 0 };
    let cycle = data as u32 * M::FEEDBACK_EVERY as u32 + feedback as u32;
    OneWayMpslPlan {
        data_slot_us: data,
        feedback_slot_us: feedback,
        receiver_window_us: if cycle > u16::MAX as u32 {
            u16::MAX
        } else {
            cycle as u16
        },
        batch: M::FEEDBACK_EVERY,
        phase_align: PHASE_ALIGN,
    }
}

/// Minimal cumulative MPSL counters used by smoke tests.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MpslStats {
    /// Address matches observed by ordinary RX ops.
    pub addr_events: u32,
    /// Completed hardware transmissions.
    pub tx_count: u32,
    /// CRC-valid received packets.
    pub crc_ok: u32,
    /// CRC failures.
    pub crc_bad: u32,
    /// Current index in the compile-time hopping sequence.
    pub hop_index: u8,
    /// Whether hopping is enabled.
    pub hopping: bool,
    /// Whether the initial hop epoch handshake completed.
    pub hop_locked: bool,
    /// Receiver side: a recall request is asserted in every feedback beacon.
    pub recall_active: bool,
    /// Sender side: this node was recalled into the negotiation phase.
    pub negotiation: bool,
}

/// Read cumulative MPSL counters.
pub fn mpsl_pll_snapshot() -> MpslStats {
    unsafe {
        let state = &*(STATE as *const MpslState);
        MpslStats {
            addr_events: state.addr_events,
            tx_count: state.tx_count,
            crc_ok: state.crc_ok,
            crc_bad: state.crc_bad,
            hop_index: state.hop_index,
            hopping: state.one_way_hopping,
            hop_locked: state.hop_locked,
            recall_active: state.recall_active,
            negotiation: state.negotiation,
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

impl<'d, const SLOT_US: u32, const RX_POLL: u32> MpslRadioPhy<'d, SLOT_US, RX_POLL> {
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
        // The mode period is authoritative immediately. Only the first
        // EARLIEST grant is enlarged for one-time session/radio setup.
        state.slot_nominal = SLOT_US;
        state.slot_len = MPSL_FIRST_CALLBACK_GRANT_US;
        state.rx_poll = RX_POLL;
        state.radio_mode = _radio_mode;

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
        #[cfg(any(feature = "nrf5340-net", feature = "nrf52840", feature = "nrf52833"))]
        {
            state
                .radio
                .modecnf0()
                .modify(|w| w.set_ru(nrf_pac::radio::vals::Ru::Fast));
            #[cfg(feature = "defmt")]
            defmt::info!(
                "modecnf0 ru={} (1=Fast)",
                state.radio.modecnf0().read().ru().to_bits()
            );
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
            let request_result = callback::mpsl_request_timeslot(state);
            #[cfg(feature = "defmt")]
            defmt::info!("first request ret={}", request_result);
            #[cfg(not(feature = "defmt"))]
            let _ = request_result;
        }

        // The nRF54L needs its RADIO/TIMER NVIC lines (the blob's chained
        // arming runs on those IRQs; the RRAM fast-fetch was already set
        // above, before the session could grant anything).
        #[cfg(feature = "_nrf54")]
        unsafe {
            use embassy_nrf::interrupt::typelevel::{Interrupt as _, RADIO_0, TIMER10};
            RADIO_0::enable();
            TIMER10::enable();
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

    /// Configure packet-relative TimeDiff timing for a one-way mode.
    pub fn configure_one_way(
        &mut self,
        data_slot_us: u16,
        feedback_slot_us: u16,
        feedback_every: u16,
        hopping: bool,
    ) {
        self.state.one_way_data_slot_us = data_slot_us as u32;
        self.state.one_way_feedback_slot_us = feedback_slot_us as u32;
        self.state.one_way_feedback_every = feedback_every as u32;
        self.state.one_way_last_address_cyc = 0;
        self.state.one_way_data_phase = 0;
        self.state.one_way_phase_valid = false;
        self.state.one_way_hopping = hopping;
        self.state.hop_index = 0;
        self.state.hop_pending = false;
        self.state.hop_locked = false;
    }

    /// Publish one long RX grant that repeatedly overwrites the latest state.
    pub fn publish_state_rx(&mut self, latest: &mut [u8; 6], target: u32) {
        self.note_publish_delay();
        let e = &mut self.state.ops[(target % 2) as usize];
        e.rx_ptr = latest.as_mut_ptr();
        e.rx_cap = latest.len();
        e.rx_ok = false;
        e.rx_result = 0;
        e.skipped = false;
        e.kind = state::OpKind::RxState as u8;
        e.target = target;
        e.grace = 0;
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
        e.seq = e.seq.wrapping_add(1);
    }

    /// Collect a long state RX grant and return its valid packet count.
    pub async fn collect_state_rx(&mut self, slot: u32) -> usize {
        self.collect(slot).await.unwrap_or(0)
    }

    /// Recall one sender into the negotiation phase, addressed by the first
    /// (prefix) byte of its application-level [`Address`]. The request is
    /// level-triggered: every feedback beacon asserts NEG_REQ with the
    /// target's on-air prefix until [`Self::release_recall`] clears it. A
    /// recalled sender stops streaming and echoes a `ConfigFrame` status in
    /// every forward slot; unmatched senders ignore the request.
    pub fn recall_sender(&mut self, prefix: u8) {
        // Address bytes cross the air bit-reversed (see set_address); the
        // sender compares against its stored on-air prefix.
        self.state.recall_prefix = prefix.reverse_bits();
        self.state.recall_active = true;
    }

    /// Clear the recall request. A recalled sender resumes streaming at the
    /// next batch boundary; hopping restarts from the rendezvous channel.
    pub fn release_recall(&mut self) {
        self.state.recall_active = false;
    }

    /// True while a recall request is asserted in every feedback beacon.
    pub fn recall_active(&self) -> bool {
        self.state.recall_active
    }

    /// Configure up to eight state senders that share the same four-byte base
    /// address and differ by their first (prefix) byte.
    pub fn configure_state_senders(&mut self, addresses: &[Address]) -> bool {
        if addresses.is_empty() || addresses.len() > 8 {
            return false;
        }
        let base = &addresses[0].0[1..];
        if addresses.iter().any(|address| &address.0[1..] != base) {
            return false;
        }
        let base_raw = u32::from_le_bytes([base[0], base[1], base[2], base[3]]);
        let mut swapped = 0u32;
        for i in 0..4 {
            let byte = ((base_raw >> (i * 8)) & 0xFF) as u8;
            swapped |= (byte.reverse_bits() as u32) << (i * 8);
        }
        let encoded_base = swapped.swap_bytes();
        let mut prefix0 = 0u32;
        let mut prefix1 = 0u32;
        for (index, address) in addresses.iter().enumerate() {
            let prefix = address.0[0].reverse_bits() as u32;
            if index < 4 {
                prefix0 |= prefix << (index * 8);
            } else {
                prefix1 |= prefix << ((index - 4) * 8);
            }
            self.state.multi_count[index].store(0, core::sync::atomic::Ordering::Relaxed);
        }
        self.state.cur_base0 = encoded_base;
        self.state.cur_base1 = encoded_base;
        self.state.cur_prefix = prefix0;
        self.state.cur_prefix1 = prefix1;
        self.state.rx_addresses = if addresses.len() == 8 {
            0xFF
        } else {
            (1u8 << addresses.len()) - 1
        };
        true
    }

    /// Copy one sender's latest state and return its cumulative packet count.
    pub fn sender_state(&self, sender: usize, out: &mut [u8; 6]) -> Option<u32> {
        if sender >= 8 || self.state.rx_addresses & (1 << sender) == 0 {
            return None;
        }
        loop {
            let before = self.state.multi_count[sender].load(core::sync::atomic::Ordering::Acquire);
            unsafe {
                out.copy_from_slice(&(*self.state.multi_latest.get())[sender]);
            }
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            let after = self.state.multi_count[sender].load(core::sync::atomic::Ordering::Acquire);
            if before == after {
                return Some(after);
            }
        }
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
        } else if self.state.ops[i].collected_seq == seq {
            self.state.coll_late = self.state.coll_late.wrapping_add(1);
        }
        if self.state.ops[i].target != slot || self.state.ops[i].collected_seq == seq {
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
        if self.state.ops[i].done_seq != seq {
            return None;
        }
        let skipped = self.state.ops[i].skipped;
        let rx_ok = self.state.ops[i].rx_ok;
        let kind = self.state.ops[i].kind;
        let result = self.state.ops[i].rx_result;
        self.state.ops[i].collected_seq = seq;
        if skipped
            || !rx_ok
            || (kind != state::OpKind::Rx as u8 && kind != state::OpKind::RxState as u8)
        {
            self.state.coll_empty = self.state.coll_empty.wrapping_add(1);
            return None;
        }
        self.state.coll_catch = self.state.coll_catch.wrapping_add(1);
        Some(result)
    }
}

impl<'d, const SLOT_US: u32, const RX_POLL: u32> Phy for MpslRadioPhy<'d, SLOT_US, RX_POLL> {
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
        self.state.cur_base1 = self.state.cur_base0;
        self.state.cur_prefix = addr.0[0].reverse_bits() as u32;
        self.state.cur_prefix1 = 0;
        self.state.rx_addresses = 0x01;
    }

    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>> {
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
        Ok(self.collect(target).await)
    }

    async fn flush(&mut self) {}

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

    fn slot_count(&self) -> u32 {
        self.state.slot_count
    }

    fn schedule_slot_profile(
        &mut self,
        short_us: u16,
        long_us: u16,
        period: u16,
        short_phases: u16,
        central_apply_slot: u32,
        local_apply_slot: u32,
    ) -> bool {
        if self
            .state
            .profile_armed
            .load(core::sync::atomic::Ordering::Acquire)
        {
            return false;
        }
        let short = short_us.max(SLOT_US as u16) as u32;
        let long = long_us.max(SLOT_US as u16) as u32;
        let period = period.max(1) as u32;
        self.state.profile_short_us = short.min(long);
        self.state.profile_long_us = long;
        self.state.profile_period = period;
        self.state.profile_short_phases = (short_phases as u32).min(period);
        self.state.profile_central_apply_slot = central_apply_slot;
        self.state.profile_apply_slot = local_apply_slot;
        // Same-core app -> MPSL IRQ publication: armed is written last. If
        // the IRQ preempts any earlier write it still sees the compile-time
        // initial period; after true, the compiler fence guarantees the
        // complete immutable profile is visible.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
        self.state
            .profile_armed
            .store(true, core::sync::atomic::Ordering::Release);
        true
    }
}
