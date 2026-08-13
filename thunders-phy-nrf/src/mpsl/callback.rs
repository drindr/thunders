//! The MPSL timeslot callback: the radio work, the phase-lock, and the chain.

use super::radio;
use super::state::{MpslState, OpKind};
use super::{PLL_GAIN_DEN, PLL_GAIN_NUM, PLL_SWEEP_MISSES, PLL_SWEEP_US, STATE};

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

        radio::timeslot_do_work(state);

        let dt = (embassy_time::Instant::now() - t0).as_micros() as u32;
        if dt > state.slot_work_max {
            state.slot_work_max = dt;
        }

        // The follower's phase-lock (the RX catch iter -> the chain distance).
        if state.op_kind == OpKind::Rx as u8 {
            if state.rx_ok {
                if state.follower {
                    let mid = state.rx_poll as i32 / 2;
                    let err = state.rx_catch_iter as i32 - mid;
                    let corr = err * PLL_GAIN_NUM / PLL_GAIN_DEN;
                    let nominal = state.slot_nominal as i32;
                    let new_dist = (nominal + corr).clamp(nominal - 20, nominal + 20) as u32;
                    if new_dist != state.slot_distance {
                        state.slot_distance = new_dist;
                    }
                    state.rx_misses = 0;
                }
            } else if state.follower {
                state.rx_misses = state.rx_misses.saturating_add(1);
                if state.rx_misses >= PLL_SWEEP_MISSES {
                    if state.slot_distance != state.slot_nominal + PLL_SWEEP_US {
                        state.slot_distance = state.slot_nominal + PLL_SWEEP_US;
                    }
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
