//! The MPSL timeslot callback: the radio work, the phase-lock, and the chain.

use super::radio;
use super::state::{MpslState, OpKind};
use super::{PLL_GAIN_DEN, PLL_GAIN_NUM, PLL_SWEEP_MISSES, PLL_SWEEP_US, STATE};

/// Last-chained request params + counters, for post-mortem assert
/// diagnosis: DIAG = dist; DIAG2 = len | misses<<16 | nominal<<24.
pub static DIAG: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static DIAG2: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// MPSL timeslot callback: on START, run the pending TX/RX (the `Phy` trait's
/// `transmit`/`receive` set the op), then chain the next NORMAL timeslot.
pub unsafe extern "C" fn timeslot_cb(
    _sid: u8,
    signal: u32,
) -> *mut nrf_mpsl::raw::mpsl_timeslot_signal_return_param_t {
    static mut RET: nrf_mpsl::raw::mpsl_timeslot_signal_return_param_t =
        unsafe { core::mem::zeroed() };
    if signal == nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_START as u32 {
        let t0 = embassy_time::Instant::now();
        let state = unsafe { &mut *(STATE as *mut MpslState) };
        state.slot_count += 1;
        // No defmt here: this runs in MPSL timer-IRQ context and defmt is
        // not reentrant (the STATS print from here corrupted the stream
        // and HardFaulted). Counters stay; read them via mpsl_stats().

        radio::timeslot_do_work(state);

        let dt = (embassy_time::Instant::now() - t0).as_micros() as u32;
        if dt > state.slot_work_max {
            state.slot_work_max = dt;
        }

        // The follower's phase-lock (the RX catch iter -> the chain distance).
        if state.op_kind == OpKind::Rx as u8 {
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
                let air = 28 + 4 * (state.rx_result as u32 + 3);
                // The cap keeps the MPSL inter-slot gap >= 150 us (the
                // scheduler's own minimum; tighter gaps trip its assert).
                state.slot_len =
                    (air + 200).min(state.slot_nominal.saturating_sub(150));
                if state.follower {
                    // The echo timing: with mirrored same-period grids the
                    // next-slot echo lands C us BEFORE the peer's receiver
                    // is ready (C = the catch position) - structurally
                    // outside the peer's window. So the echo is delayed
                    // into its slot instead: delta = S - 50 + (W-air)/2
                    // lands it mid-window, where S is the caught frame's
                    // on-air start from our slot start. All DWT-exact us.
                    if state.peer_rx_window_us > 0 {
                        let air = 28 + 4 * (state.rx_result as i32 + 3);
                        // S = RXEN offset (~10 us) + address stamp - 28.
                        let s = 10 + state.addr_poll_us as i32 - 28;
                        let w = state.peer_rx_window_us as i32;
                        let delay = s - 50 + (w - air) / 2;
                        // The frame must still fit the slot after the delay.
                        let max_delay = (state.slot_len as i32) - (50 + air + 40);
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
                if locked
                    && state.calib_count < 32
                    && (50..=180).contains(&state.addr_poll_us)
                {
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
        // DIAG: last-chained dist/len + counters, for post-mortem assert
        // diagnosis.
        DIAG.store(state.slot_distance, core::sync::atomic::Ordering::Relaxed);
        DIAG2.store(
            state.slot_len.min(0xFFFF)
                | ((state.rx_misses as u32) << 16)
                | ((state.slot_nominal.min(255)) << 24),
            core::sync::atomic::Ordering::Relaxed,
        );
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
        if state.slot_distance != state.slot_nominal + PLL_SWEEP_US {
            state.slot_distance = state.slot_nominal;
        }
        &mut RET as *mut _
    } else if signal == nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_RADIO as u32 {
        RET.callback_action = nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
        &mut RET as *mut _
    } else {
        let state = unsafe { &mut *(STATE as *mut MpslState) };
        state.other_signals += 1;
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
