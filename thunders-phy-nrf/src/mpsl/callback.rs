//! The MPSL timeslot callback: the radio work, the phase-lock, and the chain.

use super::radio;
use super::state::{MpslState, OpKind};
use super::{
    MPSL_TX_TAIL_US, PLL_GAIN_DEN, PLL_GAIN_NUM, PLL_SWEEP_MISSES,
    PLL_SWEEP_US, STATE,
};

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
        state.start_signal.signal(());

        // Consume the published op ONLY in the slot it was published for.
        // The old level-based op_kind re-ran the previous slot's op when
        // the app published late - a repeated TX/RX one slot off its phase,
        // which smeared the peripheral's echoes into the central's TX
        // phases. A late op now idles its slot (safe) and is counted -
        // except a TX op with grace (the first of a run), which may run
        // one slot late: it still faces a listening peer.
        let mut executed = OpKind::Idle as u8;
        if state.op_seq != state.op_done_seq {
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            let target = state.op_target_slot;
            let late_by = state.slot_count.wrapping_sub(target) as i32;
            if late_by == 0 {
                executed = state.op_kind;
            } else if late_by > 0
                && late_by <= state.op_grace as i32
                && state.op_kind == OpKind::Tx as u8
            {
                executed = OpKind::Tx as u8;
                state.op_grace_used = state.op_grace_used.wrapping_add(1);
            } else if late_by > 0 {
                // The target slot has passed: skip, never execute an op
                // one slot off its phase without grace.
                state.op_late = state.op_late.wrapping_add(1);
                state.op_done_seq = state.op_seq;
                state.op_skipped = true;
            }
            // target > slot_count: the op is for a future slot; leave it
            // pending and idle this one.
            if executed != OpKind::Idle as u8 {
                state.op_done_seq = state.op_seq;
                state.op_skipped = false;
            }
        }
        // Deterministic reverse acquisition: the central is the stable
        // timing reference and never sweeps. Only the peripheral sweeps its
        // RX grid and its SlotRequest TX delay. If both sides swept at
        // +2 us/slot their equal periods would freeze the relative slot
        // phase and acquisition could never converge.
        // No defmt here: this runs in MPSL timer-IRQ context and defmt is
        // not reentrant.

        radio::timeslot_do_work(state, executed);

        // The follower's phase-lock (the RX catch iter -> the chain distance).
        // Runs only for an RX op that actually executed in THIS slot: the
        // op kind is latched at START, not re-read after the work (the app
        // may have published the next op meanwhile).
        if executed == OpKind::Rx as u8 {
            if state.rx_ok {
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
                state.slot_len = state.slot_nominal.saturating_sub(150);
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
                        let air = state.airtime_us(state.rx_result as usize) as i32;
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
                        let w = state.peer_rx_window_us as i32;
                        // Forward catch anchor: peer TX on-air at our slot
                        // start = own_rx + own_addr - address_anchor.
                        let delay = own_rx + own_addr - state.air_prefix_us as i32
                            - (peer_tx_en + peer_tx_ramp)
                            + (peer_rx + peer_rx_ramp)
                            + (w - air) / 2
                            - setup
                            - own_tx_ramp;
                        // The TX op must fit the grant: setup + delay +
                        // own TX ramp + air + tail <= slot_len.
                        let max_delay = state.slot_len as i32
                            - setup
                            - own_tx_ramp
                            - air
                            - MPSL_TX_TAIL_US;
                        state.tx_delay_us = delay.clamp(0, max_delay.max(0)) as u32;
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
                    state.slot_len = state.slot_nominal.saturating_sub(150);
                    // The +2us/slot sweep is for acquisition: before the
                    // first beacon, and again when the phase is truly lost
                    // (500 straight misses). Between those, hold nominal:
                    // a persistent dist!=nominal is a FREQUENCY offset
                    // that walks the grid off the peer's window.
                    if state.follower
                        && (state.peer_rx_window_us == 0 || state.rx_misses >= 500)
                        && state.slot_distance != state.slot_nominal + PLL_SWEEP_US
                    {
                        state.slot_distance = state.slot_nominal + PLL_SWEEP_US;
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
                let nominal = state.slot_nominal as i32;
                let new_dist = (nominal + corr).clamp(nominal - 20, nominal + 20) as u32;
                if new_dist != state.slot_distance {
                    state.slot_distance = new_dist;
                }
            }
        }

        // Chain the next timeslot (the distance is the phase-lock's knob).
        let req = state.next_req.assume_init_mut();
        *req = nrf_mpsl::raw::mpsl_timeslot_request_t {
            request_type: nrf_mpsl::raw::MPSL_TIMESLOT_REQ_TYPE_NORMAL as u8,
            params: nrf_mpsl::raw::mpsl_timeslot_request_t__bindgen_ty_1 {
                normal: nrf_mpsl::raw::mpsl_timeslot_request_normal_t {
                    hfclk: nrf_mpsl::raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                    priority: nrf_mpsl::raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                    distance_us: state.slot_distance,
                    length_us: state.slot_len,
                },
            },
        };
        RET.callback_action = nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
        RET.params.request.p_next = req;
        // The PLL's corrected distance is a ONE-SLOT phase step: re-base to
        // nominal immediately or it persists as a frequency offset and the
        // grid walks away between catches. (The acquisition sweep is the
        // exception: it MEANS to walk.)
        if state.slot_distance != state.slot_nominal + PLL_SWEEP_US
            || (state.follower && state.rx_misses == 0)
        {
            state.slot_distance = state.slot_nominal;
        }
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
