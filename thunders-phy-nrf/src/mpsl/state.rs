//! State shared by the MPSL callback and application context.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicU32};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::radio_phy::RadioMode;

/// One fixed-size packet: `[length | payload...]`.
pub type Pkt = [u8; 64];

/// Operation executed in one granted timeslot.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum OpKind {
    /// No radio operation.
    Idle = 0,
    /// Transmit one packet.
    Tx = 1,
    /// Receive one packet.
    Rx = 2,
    /// Continuously receive fixed state packets into one latest-value buffer.
    RxState = 3,
}

/// One entry in the depth-two absolute-slot operation ring.
pub struct OpEntry {
    pub(crate) kind: u8,
    pub(crate) seq: u32,
    pub(crate) target: u32,
    pub(crate) grace: u8,
    pub(crate) rx_ptr: *mut u8,
    pub(crate) rx_cap: usize,
    pub(crate) tx_buf: Pkt,
    pub(crate) done_seq: u32,
    pub(crate) collected_seq: u32,
    pub(crate) skipped: bool,
    pub(crate) rx_ok: bool,
    pub(crate) rx_result: usize,
}

impl OpEntry {
    const fn new() -> Self {
        Self {
            kind: OpKind::Idle as u8,
            seq: 0,
            target: 0,
            grace: 0,
            rx_ptr: core::ptr::null_mut(),
            rx_cap: 0,
            tx_buf: [0; 64],
            done_seq: 0,
            collected_seq: 0,
            skipped: false,
            rx_ok: false,
            rx_result: 0,
        }
    }
}

/// Runtime state owned by [`super::MpslRadioPhy`].
pub struct MpslState {
    /// RADIO peripheral used inside granted callbacks.
    pub radio: nrf_pac::radio::Radio,

    pub(crate) slot_nominal: u32,
    pub(crate) slot_len: u32,
    pub(crate) rx_poll: u32,
    pub(crate) one_way_data_slot_us: u32,
    pub(crate) one_way_feedback_slot_us: u32,
    pub(crate) one_way_feedback_every: u32,
    pub(crate) one_way_last_address_cyc: u32,
    pub(crate) one_way_data_phase: u16,
    pub(crate) one_way_phase_valid: bool,
    pub(crate) one_way_hopping: bool,
    pub(crate) hop_index: u8,
    pub(crate) hop_pending: bool,
    pub(crate) hop_locked: bool,

    /// Receiver side: assert NEG_REQ in every feedback beacon, addressed to
    /// `recall_prefix` (on-air, bit-reversed form).
    pub(crate) recall_active: bool,
    pub(crate) recall_prefix: u8,
    /// Sender side: a matched recall moved this node into the negotiation
    /// phase; forward slots carry ConfigFrame echoes instead of app data.
    pub(crate) negotiation: bool,
    pub(crate) neg_cfg_seq: u8,

    pub(crate) active_profile_short_us: u32,
    pub(crate) active_profile_long_us: u32,
    pub(crate) active_profile_period: u32,
    pub(crate) active_profile_short_phases: u32,
    pub(crate) active_profile_central_apply_slot: u32,
    pub(crate) active_profile_local_apply_slot: u32,
    pub(crate) active_profile_armed: AtomicBool,

    pub(crate) profile_short_us: u32,
    pub(crate) profile_long_us: u32,
    pub(crate) profile_period: u32,
    pub(crate) profile_short_phases: u32,
    pub(crate) profile_central_apply_slot: u32,
    pub(crate) profile_apply_slot: u32,
    pub(crate) profile_armed: AtomicBool,

    pub(crate) tx_count: u32,
    pub(crate) crc_ok: u32,
    pub(crate) crc_bad: u32,
    pub(crate) multi_latest: UnsafeCell<[[u8; 6]; 8]>,
    pub(crate) multi_count: [AtomicU32; 8],

    pub(crate) ops: [OpEntry; 2],
    pub(crate) op_late: u32,
    pub(crate) coll_noop: u32,
    pub(crate) coll_late: u32,
    pub(crate) coll_catch: u32,
    pub(crate) coll_empty: u32,
    pub(crate) op_grace_used: u32,
    pub(crate) last_start_cyc: u32,
    pub(crate) op_publish_max_us: u32,

    pub(crate) session_id: u8,
    pub(crate) first_request: bool,
    pub(crate) next_req: MaybeUninit<nrf_mpsl::raw::mpsl_timeslot_request_t>,
    pub(crate) done_signal: Signal<CriticalSectionRawMutex, ()>,
    pub(crate) start_signal: Signal<CriticalSectionRawMutex, ()>,
    pub(crate) slot_count: u32,
    pub(crate) done_count: AtomicU32,
    pub(crate) addr_events: u32,

    pub(crate) radio_mode: RadioMode,
    pub(crate) cur_channel: u8,
    pub(crate) cur_base0: u32,
    pub(crate) cur_base1: u32,
    pub(crate) cur_prefix: u32,
    pub(crate) cur_prefix1: u32,
    pub(crate) rx_addresses: u8,
}

impl MpslState {
    /// Create zeroed callback state for one RADIO peripheral.
    pub fn new(radio: nrf_pac::radio::Radio) -> Self {
        Self {
            radio,
            slot_nominal: 0,
            slot_len: 0,
            rx_poll: 0,
            one_way_data_slot_us: 0,
            one_way_feedback_slot_us: 0,
            one_way_feedback_every: 1,
            one_way_last_address_cyc: 0,
            one_way_data_phase: 0,
            one_way_phase_valid: false,
            one_way_hopping: false,
            hop_index: 0,
            hop_pending: false,
            hop_locked: false,
            recall_active: false,
            recall_prefix: 0,
            negotiation: false,
            neg_cfg_seq: 0,
            active_profile_short_us: 0,
            active_profile_long_us: 0,
            active_profile_period: 0,
            active_profile_short_phases: 0,
            active_profile_central_apply_slot: 0,
            active_profile_local_apply_slot: 0,
            active_profile_armed: AtomicBool::new(false),
            profile_short_us: 0,
            profile_long_us: 0,
            profile_period: 0,
            profile_short_phases: 0,
            profile_central_apply_slot: 0,
            profile_apply_slot: 0,
            profile_armed: AtomicBool::new(false),
            tx_count: 0,
            crc_ok: 0,
            crc_bad: 0,
            multi_latest: UnsafeCell::new([[0; 6]; 8]),
            multi_count: core::array::from_fn(|_| AtomicU32::new(0)),
            ops: [OpEntry::new(), OpEntry::new()],
            op_late: 0,
            coll_noop: 0,
            coll_late: 0,
            coll_catch: 0,
            coll_empty: 0,
            op_grace_used: 0,
            last_start_cyc: 0,
            op_publish_max_us: 0,
            session_id: 0,
            first_request: true,
            next_req: MaybeUninit::uninit(),
            done_signal: Signal::new(),
            start_signal: Signal::new(),
            slot_count: 0,
            done_count: AtomicU32::new(0),
            addr_events: 0,
            radio_mode: RadioMode::Nrf2Mbit,
            cur_channel: 0,
            cur_base0: 0,
            cur_base1: 0,
            cur_prefix: 0,
            cur_prefix1: 0,
            rx_addresses: 0x01,
        }
    }
}

unsafe impl Send for MpslState {}
unsafe impl Sync for MpslState {}
