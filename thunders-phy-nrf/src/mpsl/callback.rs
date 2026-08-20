//! The MPSL timeslot callback: the radio work, the phase-lock, and the chain.

use super::radio;
use super::state::{MpslState, OpKind};
use super::{MPSL_TX_TAIL_US, PLL_GAIN_DEN, PLL_GAIN_NUM, PLL_SWEEP_MISSES, PLL_SWEEP_US, STATE};
use core::sync::atomic::Ordering;
use thunders::phy::SlotProbeStats;

#[inline(always)]
fn slot_profile_due(slot: u32, apply: u32) -> bool {
    slot.wrapping_sub(apply) as i32 >= 0
}

#[inline(always)]
fn phase_nominal(
    slot: u32,
    short_us: u32,
    long_us: u32,
    period: u32,
    short_phases: u32,
    phase_offset: u32,
) -> u32 {
    if period == 0 {
        return long_us;
    }
    let phase = slot.wrapping_add(phase_offset) % period;
    if phase < short_phases {
        short_us
    } else {
        long_us
    }
}

#[inline(always)]
fn slot_before(slot: u32, end: u32) -> bool {
    (slot.wrapping_sub(end) as i32) < 0
}

#[inline(always)]
fn probe_trace_index(state: &MpslState, slot: u32) -> Option<usize> {
    if state.probe_start_slot == 0 {
        return None;
    }
    match slot.wrapping_sub(state.probe_start_slot) as i32 {
        -1 => Some(0),
        0 => Some(1),
        1 => Some(2),
        _ => None,
    }
}

/// Negotiated period for hardware slot `slot`: bounded probe overlay,
/// pending commit after its apply epoch, current active profile, then the
/// uniform acquisition fallback.
#[inline(always)]
fn nominal_for_slot(state: &MpslState, slot: u32) -> u32 {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
    if state.probe_armed.load(Ordering::Acquire)
        && slot_profile_due(slot, state.probe_start_slot)
        && slot_before(slot, state.probe_end_slot)
    {
        let phase = state
            .probe_central_start_slot
            .wrapping_add(slot.wrapping_sub(state.probe_start_slot))
            % state.probe_period.max(1);
        return if phase < state.probe_short_phases {
            state.probe_short_us
        } else {
            state.probe_long_us
        };
    }
    if state.profile_armed && slot_profile_due(slot, state.profile_apply_slot) {
        return phase_nominal(
            slot,
            state.profile_short_us,
            state.profile_long_us,
            state.profile_period,
            state.profile_short_phases,
            state.profile_phase_offset,
        );
    }
    if state.active_profile_armed {
        return phase_nominal(
            slot,
            state.active_profile_short_us,
            state.active_profile_long_us,
            state.active_profile_period,
            state.active_profile_short_phases,
            state.active_profile_phase_offset,
        );
    }
    state.slot_nominal
}

#[inline(always)]
fn raw_probe_stats(state: &MpslState) -> SlotProbeStats {
    SlotProbeStats {
        slots: state.slot_count,
        clock_us: 0,
        completed: state.done_count.load(Ordering::Relaxed),
        op_late: state.op_late,
        address_events: state.addr_events,
        crc_ok: state.crc_ok,
        crc_bad_long: state.crc_bad_long,
        tx_count: state.tx_count,
        windows: 0,
        aborted_windows: 0,
    }
}

#[inline(always)]
fn promote_profile_if_due(state: &mut MpslState, slot: u32) {
    if state.probe_armed.load(Ordering::Acquire) && slot == state.probe_start_slot {
        state.probe_clock_start_cyc = state.last_start_cyc;
        state.probe_raw_start = raw_probe_stats(state);
        state.probe_started = true;
    }
    if state.profile_armed && slot_profile_due(slot, state.profile_apply_slot) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
        state.active_profile_short_us = state.profile_short_us;
        state.active_profile_long_us = state.profile_long_us;
        state.active_profile_period = state.profile_period;
        state.active_profile_short_phases = state.profile_short_phases;
        state.active_profile_phase_offset = state.profile_phase_offset;
        state.active_profile_armed = true;
        state.profile_armed = false;
    }
    if state.probe_armed.load(Ordering::Acquire) && slot_profile_due(slot, state.probe_end_slot) {
        let delta = if state.probe_started {
            let mut delta = raw_probe_stats(state).wrapping_delta(state.probe_raw_start);
            delta.clock_us = state
                .last_start_cyc
                .wrapping_sub(state.probe_clock_start_cyc)
                / radio::CPU_MHZ;
            delta.windows = 1;
            delta
        } else {
            SlotProbeStats {
                aborted_windows: 1,
                ..SlotProbeStats::default()
            }
        };
        // Keep a single odd/even generation around atomic per-field updates so
        // readers get a coherent completed-window total without plain-data races.
        state.probe_stats_seq.fetch_add(1, Ordering::AcqRel);
        core::sync::atomic::compiler_fence(Ordering::SeqCst);
        state.probe_stats_total.wrapping_add(delta);
        core::sync::atomic::compiler_fence(Ordering::Release);
        state.probe_stats_seq.fetch_add(1, Ordering::Release);
        state.probe_started = false;
        state.probe_armed.store(false, Ordering::Release);
    }
}

/// MPSL timeslot callback: on START, run the pending TX/RX (the `Phy` trait's
/// `transmit`/`receive` set the op), then chain the next NORMAL timeslot.
pub unsafe extern "C" fn timeslot_cb(
    _sid: u8,
    signal: u32,
) -> *mut nrf_mpsl::raw::mpsl_timeslot_signal_return_param_t {
    static mut RET: nrf_mpsl::raw::mpsl_timeslot_signal_return_param_t =
        unsafe { core::mem::zeroed() };
    if signal == nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_START as u32 {
        let state = unsafe { &mut *(STATE as *mut MpslState) };
        state.slot_count += 1;
        state.slot_start_done = state.done_count.load(core::sync::atomic::Ordering::Acquire);
        state.last_start_cyc = radio::cyc();

        // Consume the published op ONLY in the slot it was published for.
        // Ops live in a depth-2 parity ring (entry = target % 2) and the
        // app publishes ~2 slots ahead, so a publish never clobbers a
        // pending op (the entry is collected before reuse). Priority at
        // each START: the on-target op for this slot; else a grace TX
        // from the previous slot (it still faces a listening peer); else
        // idle - a late op never runs off-phase (the old level-based
        // op_kind smeared the peripheral's echoes into the central's TX
        // phases). No defmt here: MPSL timer-IRQ context.
        let slot = state.slot_count;
        promote_profile_if_due(state, slot);
        // Wake app context only after probe/profile boundary state is coherent.
        state.start_signal.signal(());
        let current_nominal = nominal_for_slot(state, slot);
        let next_nominal = nominal_for_slot(state, slot.wrapping_add(1));
        let on = (slot % 2) as usize;
        let mut exec: Option<usize> = None;
        if state.ops[on].seq != state.ops[on].done_seq {
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            let target = state.ops[on].target;
            let late_by = slot.wrapping_sub(target) as i32;
            if late_by == 0 {
                exec = Some(on);
            } else if late_by > 0 {
                // The target slot has passed: skip, never execute an op
                // one slot off its phase without grace.
                state.op_late = state.op_late.wrapping_add(1);
                state.ops[on].done_seq = state.ops[on].seq;
                state.ops[on].skipped = true;
            }
            // target > slot: the op is for a future slot; leave pending.
        }
        if exec.is_none() {
            // Grace: the first TX of a run may execute one slot late.
            let g = (slot.wrapping_sub(1) % 2) as usize;
            let e = &state.ops[g];
            if e.seq != e.done_seq
                && e.kind == OpKind::Tx as u8
                && e.target == slot.wrapping_sub(1)
                && e.grace > 0
            {
                exec = Some(g);
                state.op_grace_used = state.op_grace_used.wrapping_add(1);
            }
        }
        let executed_kind = exec
            .map(|i| state.ops[i].kind)
            .unwrap_or(OpKind::Idle as u8);
        let trace_index = probe_trace_index(state, slot);
        if let Some(ti) = trace_index {
            state.probe_trace_slot[ti] = slot;
            state.probe_trace_phase[ti] = state
                .probe_central_start_slot
                .wrapping_add(slot.wrapping_sub(state.probe_start_slot))
                % state.probe_period.max(1);
            state.probe_trace_nominal[ti] = current_nominal;
            state.probe_trace_exec_kind[ti] = executed_kind as u32;
            state.probe_trace_exec_target[ti] = exec.map(|i| state.ops[i].target).unwrap_or(0);
        }
        if let Some(i) = exec {
            state.ops[i].done_seq = state.ops[i].seq;
            state.ops[i].skipped = false;
        }

        radio::timeslot_do_work(state, exec);
        if let Some(ti) = trace_index {
            state.probe_trace_event_us[ti] = if executed_kind == OpKind::Tx as u8 {
                state.tx_en_offset_us
            } else if executed_kind == OpKind::Rx as u8 && state.addr_seen {
                state.addr_poll_us
            } else if executed_kind == OpKind::Rx as u8 {
                state.rx_en_offset_us
            } else {
                0
            };
        }

        // The follower's phase-lock (the RX catch iter -> the chain distance).
        // Runs only for an RX op that actually executed in THIS slot.
        if executed_kind == OpKind::Rx as u8 {
            let ei = exec.unwrap_or(0);
            if state.ops[ei].rx_ok {
                state.rx_misses = 0;
                // The runtime align, length side: size the slot to the
                // caught packet (airtime + 140 us of overhead/jitter
                // margin), capped to keep the MPSL inter-slot gap >= 150 us
                // (tighter gaps trip the MPSL scheduler's assert).
                // Runtime align, length side: size the slot to the caught
                // packet + 100 us of poll floor + 40 us ramp + 60 us of
                // phase slack (air+140 was exactly READY+air: zero margin,
                // any jitter missed). Capped to keep the inter-slot gap.
                // Keep the slot as long as the MPSL inter-slot gap rule
                // allows. A shorter slot also clamps the follower's echo TX
                // delay (`max_delay = slot_len - (50 + air + 40)`), and on
                // the LM20 the desired delay needs the full 350 us slot.
                state.slot_len = current_nominal.saturating_sub(150);
                if state.follower {
                    // The echo timing: with mirrored same-period grids the
                    // next-slot echo would otherwise land before the peer's
                    // receiver is ready. Delay it into the slot using the
                    // measured peer TX/RX offsets and ramps below; all
                    // timestamps are DWT-exact microseconds.
                    if state.peer_rx_window_us > 0
                        && state.peer_tx_en_offset_us > 0
                        && state.peer_tx_ramp_us > 0
                        && state.peer_rx_ramp_us > 0
                        && state.tx_ramp_us > 0
                    {
                        // The frame being placed is the NEXT (pending) op -
                        // a TX, published 2 slots ahead in the pipeline -
                        // whose length is known. The old code used the last
                        // RECEIVED frame's length: the echo (a ~19 B Data)
                        // is longer than the beacons it mostly receives
                        // (~15 B), so the centering term (w-air)/2 and the
                        // peer-window clamp both under-shrunk for the real
                        // TX and the frame tail escaped the peer's window
                        // (the reverse dead state).
                        let next = 1 - ei;
                        let pending_len = state.ops[next].tx_buf[0] as usize;
                        let pending_is_next_tx = state.ops[next].seq != state.ops[next].done_seq
                            && state.ops[next].kind == OpKind::Tx as u8
                            && state.ops[next].target == slot.wrapping_add(1)
                            && pending_len > 0;
                        let air = if pending_is_next_tx {
                            state.airtime_us(pending_len)
                        } else {
                            state.airtime_us(state.ops[ei].rx_result as usize)
                        } as i32;
                        let tx_nominal = if pending_is_next_tx {
                            nominal_for_slot(state, state.ops[next].target)
                        } else {
                            next_nominal
                        };
                        let tx_slot_len = tx_nominal.saturating_sub(150) as i32;
                        // Everything below is measured; the only fixed
                        // constant is the named tail margin (the address
                        // anchor is mode-dependent).
                        let own_rx = state.rx_en_offset_us as i32;
                        let own_addr = state.addr_poll_us as i32;
                        let peer_rx = state.peer_rx_en_offset_us as i32;
                        let peer_rx_ramp = state.peer_rx_ramp_us as i32;
                        let peer_tx_en = state.peer_tx_en_offset_us as i32;
                        let peer_tx_ramp = state.peer_tx_ramp_us as i32;
                        let own_tx_ramp = state.tx_ramp_us as i32;
                        let setup = state.tx_pre_delay_us as i32;
                        // Advertised W is measured from RXEN (minus the fixed
                        // 40us shutdown reserve), not from READY. Clamp that
                        // raw duration to both candidate idle and hard edges,
                        // then remove the peer ramp only for centering.
                        let w_raw = (state.peer_rx_window_us as i32)
                            .min((tx_slot_len - 100).max(0))
                            .min((tx_slot_len - 40 - peer_rx).max(0));
                        let w_ready = (w_raw - peer_rx_ramp).max(0);
                        // Forward catch anchor: peer TX on-air at our slot
                        // start = own_rx + own_addr - address_anchor.
                        let forward_catch = own_rx + own_addr - state.air_prefix_us as i32;
                        let delay = forward_catch - (peer_tx_en + peer_tx_ramp)
                            + (peer_rx + peer_rx_ramp)
                            + (w_ready - air) / 2
                            - setup
                            - own_tx_ramp;
                        // The pending TX op must fit its target grant: setup +
                        // delay + own TX ramp + air + tail <= tx_slot_len.
                        let max_delay = tx_slot_len - setup - own_tx_ramp - air - MPSL_TX_TAIL_US;
                        // The peer-window fit (our-slot time): the frame
                        // must END before the peer's listen window ends, or
                        // its tail is clipped against the MPSL hard edge and
                        // the CRC dies (the reverse link's dead state: the
                        // central heard only len-3 SlotRequests, every
                        // 19-byte echo clipped). The centering formula is
                        // exact when all measurements are right; when they
                        // are stale/wrong (a just-frozen anchor, a grid
                        // hiccup) this clamp keeps the frame INSIDE the
                        // window - off-center but caught. The peer's slot
                        // start in our time is the forward-catch anchor
                        // minus where their TX started in their time.
                        let peer_slot_start = forward_catch - (peer_tx_en + peer_tx_ramp);
                        let window_end = peer_slot_start + peer_rx + w_raw;
                        let peer_fit = window_end - (setup + own_tx_ramp + air) - MPSL_TX_TAIL_US;
                        state.tx_delay_us = delay.clamp(0, max_delay.min(peer_fit).max(0)) as u32;
                    }
                }
            } else {
                state.rx_misses = state.rx_misses.saturating_add(1);
                if state.rx_misses >= PLL_SWEEP_MISSES {
                    // Disconnected: widen the budget, keeping the MPSL
                    // inter-slot gap >= 150 us (the scheduler's minimum;
                    // a 100 us gap (nominal-100) lets the chain degrade and
                    // the app fall a slot behind - the 5340's half-rate
                    // RX polls).
                    state.slot_len = current_nominal.saturating_sub(150);
                    // The +2us/slot sweep is for acquisition: before the
                    // first beacon, and again when the phase is truly lost
                    // (500 straight misses). Between those, hold nominal:
                    // a persistent dist!=nominal is a FREQUENCY offset
                    // that walks the grid off the peer's window.
                    if state.follower
                        && (state.peer_rx_window_us == 0 || state.rx_misses >= 500)
                        && state.slot_distance != current_nominal + PLL_SWEEP_US
                    {
                        state.slot_distance = current_nominal + PLL_SWEEP_US;
                        // Re-calibrate from scratch after a real phase loss.
                        state.addr_target_us = 60;
                        state.calib_count = 0;
                    }
                }
            }
            // The phase-lock runs on ANY address anchor, not just a
            // successful decode: a misaligned poll truncates the frame and
            // fails the CRC, and gating the correction on rx_ok makes that
            // failure prevent the correction - the phase error feeds back
            // and never converges (the 5340 peripheral sat at ~90% CRC
            // misses with thousands of address events). The address event
            // is a fixed 28 us after the frame's on-air start, so it is a
            // valid anchor regardless of the decode outcome. (A/B verified:
            // disabling it drops the working pairs from ~12% to 67-95%
            // loss - the tighter lock helps every pair.)
            if state.follower && state.addr_seen {
                // Calibrate the address target from locked catches. The
                // first catches after a sweep can be near the window edge,
                // so only catches while already locked count.
                let locked = state.rx_misses <= 8;
                state.rx_misses = 0;
                if locked && state.calib_count < 32 && (50..=180).contains(&state.addr_poll_us) {
                    state.addr_target_us = (state.addr_target_us * state.calib_count
                        + state.addr_poll_us)
                        / (state.calib_count + 1);
                    state.calib_count += 1;
                }
                state.addr_target_us = state.addr_target_us.clamp(50, 180);
                let err = state.addr_poll_us as i32 - state.addr_target_us as i32;
                // A one-shot phase step (the chain re-bases to nominal after
                // the request): stable for the matched-cadence pairs. An
                // integrating version was tried to compensate the 5340's
                // cadence drift but its dist swings destabilized the LM20
                // pairs (rx 1000+/window -> single digits), so it's reverted.
                let corr = err * PLL_GAIN_NUM / PLL_GAIN_DEN;
                let nominal = current_nominal as i32;
                let new_dist = (nominal + corr).clamp(nominal - 20, nominal + 20) as u32;
                let freeze_probe = cfg!(feature = "pll-probe-freeze")
                    && state.probe_armed.load(Ordering::Acquire)
                    && slot_profile_due(slot, state.probe_start_slot)
                    && slot_before(slot, state.probe_end_slot);
                if !cfg!(feature = "pll-fixed") && !freeze_probe && new_dist != state.slot_distance
                {
                    state.slot_distance = new_dist;
                }
            }
        }

        // Chain the next timeslot. Distance describes CURRENT-start to
        // NEXT-start, while length is the grant of the NEXT slot; this
        // distinction is what makes a phase-indexed cadence safe.
        let next_len = next_nominal.saturating_sub(150);
        let req = state.next_req.assume_init_mut();
        *req = nrf_mpsl::raw::mpsl_timeslot_request_t {
            request_type: nrf_mpsl::raw::MPSL_TIMESLOT_REQ_TYPE_NORMAL as u8,
            params: nrf_mpsl::raw::mpsl_timeslot_request_t__bindgen_ty_1 {
                normal: nrf_mpsl::raw::mpsl_timeslot_request_normal_t {
                    hfclk: nrf_mpsl::raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                    priority: nrf_mpsl::raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                    distance_us: state.slot_distance,
                    length_us: next_len,
                },
            },
        };
        RET.callback_action = nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
        RET.params.request.p_next = req;
        // The PLL's corrected distance is a ONE-SLOT phase step: re-base to
        // the NEXT phase's nominal immediately. The acquisition +2us sweep
        // is the exception and follows the next phase's baseline.
        let keep_sweep = state.follower
            && state.rx_misses > 0
            && state.slot_distance == current_nominal + PLL_SWEEP_US;
        state.slot_distance = if keep_sweep {
            next_nominal + PLL_SWEEP_US
        } else {
            next_nominal
        };
        state.slot_len = next_len;
        &mut RET as *mut _
    } else if signal == nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_RADIO as u32 {
        RET.callback_action = nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
        &mut RET as *mut _
    } else {
        core::ptr::null_mut()
    }
}

/// Request the first (EARLIEST) timeslot; the callback chains the subsequent
/// NORMAL requests.
pub fn mpsl_request_timeslot(state: &mut MpslState) -> i32 {
    use nrf_mpsl::raw;
    unsafe {
        if !state.first_request {
            return 0;
        }
    }
    let req = raw::mpsl_timeslot_request_t {
        request_type: raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8,
        params: raw::mpsl_timeslot_request_t__bindgen_ty_1 {
            earliest: raw::mpsl_timeslot_request_earliest_t {
                hfclk: raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                priority: raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                length_us: state.slot_len,
                timeout_us: raw::MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US,
            },
        },
    };
    let ret = unsafe { raw::mpsl_timeslot_request(state.session_id, &req) };
    unsafe {
        if ret == 0 {
            state.first_request = false;
        }
    }
    ret
}
