//! The MPSL radio's runtime state, provided by the caller during init.

use core::mem::MaybeUninit;
use core::sync::atomic::AtomicBool;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// One fixed-size packet: [0] = length, [1..=len] = payload.
pub type Pkt = [u8; 64];

/// What the next timeslot should do.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum OpKind {
    Idle = 0,
    Tx = 1,
    Rx = 2,
}

/// The radio's runtime state, shared with the interrupt callback.
///
/// The slot schedule (the TX:RX ratio) is driven by the `thunders` link layer
/// through the `Phy` trait's `transmit`/`receive`; the callback only performs
/// the pending op and the phase-lock. The slot constants are filled by
/// [`MpslRadioPhy::new`](super::MpslRadioPhy::new) from its const generics.
pub struct MpslState {
    /// The RADIO peripheral instance (the caller's pac instance).
    pub radio: nrf_pac::radio::Radio,
    /// True on the peripheral (the phase-lock follower).
    pub follower: bool,
    /// Set by the callback when the current slot's work is done.
    pub(crate) done: AtomicBool,

    // The slot constants (filled by the phy from its const generics).
    pub(crate) slot_nominal: u32,
    pub(crate) slot_len: u32,
    pub(crate) rx_poll: u32,

    // The phase-lock.
    pub(crate) slot_distance: u32,
    pub(crate) rx_catch_iter: u32,
    pub(crate) rx_misses: u32,

    // The current RX target (the caller's slice; the radio writes into it).
    pub(crate) rx_buf: [u8; 64],
    pub(crate) rx_ptr: *mut u8,
    pub(crate) rx_cap: usize,
    pub(crate) rx_result: usize,
    pub(crate) rx_ok: bool,

    // The TX DMA buffer (filled by `Phy::transmit`).
    pub(crate) tx_buf: [u8; 64],
    pub(crate) op_kind: u8,

    // The MPSL session.
    pub(crate) session_id: u8,
    pub(crate) first_request: bool,
    pub(crate) next_req: MaybeUninit<nrf_mpsl::raw::mpsl_timeslot_request_t>,
    /// Signaled by the callback on the first granted slot (the ready gate).
    pub(crate) done_signal: Signal<CriticalSectionRawMutex, ()>,
    pub(crate) slot_work_max: u32,

    // The radio config (the channel + the address).
    pub(crate) cur_channel: u8,
    pub(crate) cur_base0: u32,
    pub(crate) cur_prefix: u32,
}

impl MpslState {
    pub fn new(radio: nrf_pac::radio::Radio, follower: bool) -> Self {
        let mut this = Self {
            radio,
            follower,
            done: AtomicBool::new(false),
            slot_nominal: 0,
            slot_len: 0,
            rx_poll: 0,
            slot_distance: 0,
            rx_catch_iter: 0,
            rx_misses: 0,
            rx_buf: [0u8; 64],
            rx_ptr: core::ptr::null_mut(),
            rx_cap: 0,
            rx_result: 0,
            rx_ok: false,
            tx_buf: [0u8; 64],
            op_kind: OpKind::Idle as u8,
            session_id: 0,
            first_request: true,
            next_req: MaybeUninit::uninit(),
            done_signal: Signal::new(),
            slot_work_max: 0,
            cur_channel: 25,
            cur_base0: 0xE7E7E7E7,
            cur_prefix: 0xE7,
        };
        this.rx_ptr = this.rx_buf.as_mut_ptr() as *mut u8;
        this.rx_cap = 64;
        this
    }
}
