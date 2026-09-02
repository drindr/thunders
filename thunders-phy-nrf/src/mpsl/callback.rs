//! Minimal MPSL callback for compile-time one-way schedules.

use super::radio;
use super::state::{MpslState, OpKind};
use super::STATE;
use core::sync::atomic::Ordering;

#[inline(always)]
fn due(slot: u32, apply: u32) -> bool {
    slot.wrapping_sub(apply) as i32 >= 0
}

#[inline(always)]
fn phase_nominal(
    slot: u32,
    central_start: u32,
    local_start: u32,
    short_us: u32,
    long_us: u32,
    period: u32,
    short_phases: u32,
) -> u32 {
    if period == 0 {
        return long_us;
    }
    let phase = central_start.wrapping_add(slot.wrapping_sub(local_start)) % period;
    if phase < short_phases {
        short_us
    } else {
        long_us
    }
}

#[inline(always)]
fn nominal_for_slot(state: &MpslState, slot: u32) -> u32 {
    core::sync::atomic::compiler_fence(Ordering::Acquire);
    if state.profile_armed.load(Ordering::Acquire) && due(slot, state.profile_apply_slot) {
        return phase_nominal(
            slot,
            state.profile_central_apply_slot,
            state.profile_apply_slot,
            state.profile_short_us,
            state.profile_long_us,
            state.profile_period,
            state.profile_short_phases,
        );
    }
    if state.active_profile_armed.load(Ordering::Acquire) {
        return phase_nominal(
            slot,
            state.active_profile_central_apply_slot,
            state.active_profile_local_apply_slot,
            state.active_profile_short_us,
            state.active_profile_long_us,
            state.active_profile_period,
            state.active_profile_short_phases,
        );
    }
    state.slot_nominal
}

#[inline(always)]
fn promote_profile(state: &mut MpslState, slot: u32) {
    if state.profile_armed.load(Ordering::Acquire) && due(slot, state.profile_apply_slot) {
        core::sync::atomic::compiler_fence(Ordering::Acquire);
        state.active_profile_short_us = state.profile_short_us;
        state.active_profile_long_us = state.profile_long_us;
        state.active_profile_period = state.profile_period;
        state.active_profile_short_phases = state.profile_short_phases;
        state.active_profile_central_apply_slot = state.profile_central_apply_slot;
        state.active_profile_local_apply_slot = state.profile_apply_slot;
        state.active_profile_armed.store(true, Ordering::Release);
        state.profile_armed.store(false, Ordering::Release);
    }
}

/// Execute one published op and chain the next NORMAL request.
pub unsafe extern "C" fn timeslot_cb(
    _sid: u8,
    signal: u32,
) -> *mut nrf_mpsl::raw::mpsl_timeslot_signal_return_param_t {
    static mut RET: nrf_mpsl::raw::mpsl_timeslot_signal_return_param_t =
        unsafe { core::mem::zeroed() };

    if signal == nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_START as u32 {
        let state = unsafe { &mut *(STATE as *mut MpslState) };
        state.slot_count = state.slot_count.wrapping_add(1);
        state.last_start_cyc = radio::cyc();
        let slot = state.slot_count;
        promote_profile(state, slot);
        state.start_signal.signal(());

        let on = (slot % 2) as usize;
        let mut exec = None;
        if state.ops[on].seq != state.ops[on].done_seq {
            core::sync::atomic::compiler_fence(Ordering::Acquire);
            let late_by = slot.wrapping_sub(state.ops[on].target) as i32;
            if late_by == 0 {
                exec = Some(on);
            } else if late_by > 0 {
                state.op_late = state.op_late.wrapping_add(1);
                state.ops[on].done_seq = state.ops[on].seq;
                state.ops[on].skipped = true;
            }
        }
        if exec.is_none() {
            let previous = (slot.wrapping_sub(1) % 2) as usize;
            let op = &state.ops[previous];
            if op.seq != op.done_seq
                && op.kind == OpKind::Tx as u8
                && op.target == slot.wrapping_sub(1)
                && op.grace > 0
            {
                exec = Some(previous);
                state.op_grace_used = state.op_grace_used.wrapping_add(1);
            }
        }
        if let Some(i) = exec {
            state.ops[i].done_seq = state.ops[i].seq;
            state.ops[i].skipped = false;
        }

        radio::timeslot_do_work(state, exec);

        let next_nominal = nominal_for_slot(state, slot.wrapping_add(1));
        let next_len = next_nominal.saturating_sub(150);
        let req = state.next_req.assume_init_mut();
        *req = nrf_mpsl::raw::mpsl_timeslot_request_t {
            request_type: nrf_mpsl::raw::MPSL_TIMESLOT_REQ_TYPE_NORMAL as u8,
            params: nrf_mpsl::raw::mpsl_timeslot_request_t__bindgen_ty_1 {
                normal: nrf_mpsl::raw::mpsl_timeslot_request_normal_t {
                    hfclk: nrf_mpsl::raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                    priority: nrf_mpsl::raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                    distance_us: next_nominal,
                    length_us: next_len,
                },
            },
        };
        RET.callback_action = nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
        RET.params.request.p_next = req;
        state.slot_len = next_len;
        &raw mut RET
    } else if signal == nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_RADIO as u32 {
        RET.callback_action = nrf_mpsl::raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
        &raw mut RET
    } else {
        core::ptr::null_mut()
    }
}

/// Request the first EARLIEST timeslot.
pub fn mpsl_request_timeslot(state: &mut MpslState) -> i32 {
    if !state.first_request {
        return 0;
    }
    let req = nrf_mpsl::raw::mpsl_timeslot_request_t {
        request_type: nrf_mpsl::raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8,
        params: nrf_mpsl::raw::mpsl_timeslot_request_t__bindgen_ty_1 {
            earliest: nrf_mpsl::raw::mpsl_timeslot_request_earliest_t {
                hfclk: nrf_mpsl::raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                priority: nrf_mpsl::raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                length_us: state.slot_len,
                timeout_us: nrf_mpsl::raw::MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US,
            },
        },
    };
    let ret = unsafe { nrf_mpsl::raw::mpsl_timeslot_request(state.session_id, &req) };
    if ret == 0 {
        state.first_request = false;
    }
    ret
}
