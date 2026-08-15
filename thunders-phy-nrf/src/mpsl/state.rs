//! The MPSL radio's runtime state, provided by the caller during init.

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicU32};

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
    /// END-of-catch stamp, us from poll start (DWT-exact). Debug/diag.
    pub(crate) catch_poll_us: u32,
    /// Address-match stamp, us from poll start (DWT-exact): the phase
    /// anchor - a fixed 28 us after the frame's on-air start.
    pub(crate) addr_poll_us: u32,
    /// Whether an ADDRESS event fired in the last RX poll (the phase-lock
    /// corrects on this even when the frame's CRC failed).
    pub(crate) addr_seen: bool,
    pub(crate) rx_misses: u32,
    /// Our measured RX listen window (us, post-ramp), advertised in the beacon.
    pub(crate) rx_window_us: u32,
    /// The follower's TX delay (us from slot start to TXEN): places the echo
    /// in the middle of the peer's advertised RX window. Recomputed on each
    /// catch; 0 = transmit at slot start.
    pub(crate) tx_delay_us: u32,
    /// Completed TX ops (the echo-flow diagnostic).
    pub(crate) tx_count: u32,
    /// The peer's advertised RX listen window (us); 0 = unknown.
    pub(crate) peer_rx_window_us: u32,

    // The current RX target (the caller's slice; the radio writes into it).
    pub(crate) rx_buf: [u8; 64],
    pub(crate) rx_ptr: *mut u8,
    pub(crate) rx_cap: usize,
    pub(crate) rx_result: usize,
    pub(crate) rx_ok: bool,
    /// CRC diagnostics: packets with a good/bad CRCSTATUS (the 5340 net core
    /// decodes ~5% of address-matched frames - these count it).
    pub(crate) crc_ok: u32,
    pub(crate) crc_bad: u32,

    // The TX DMA buffer (filled by `Phy::transmit`).
    pub(crate) tx_buf: [u8; 64],
    /// TX DMA source override: when non-null the radio reads here instead
    /// of `tx_buf` (RAM-region diagnostic).
    pub tx_ptr: *const u8,
    pub(crate) op_kind: u8,

    // The MPSL session.
    pub(crate) session_id: u8,
    pub(crate) first_request: bool,
    pub(crate) next_req: MaybeUninit<nrf_mpsl::raw::mpsl_timeslot_request_t>,
    /// Signaled by the callback on the first granted slot (the ready gate).
    pub(crate) done_signal: Signal<CriticalSectionRawMutex, ()>,
    pub slot_work_max: u32,
    /// Diagnostics: START signals, completed works (atomic: the app
    /// spin-waits on it; a plain load could be hoisted out of the loop),
    /// other (BLOCKED/...) signals.
    pub slot_count: u32,
    pub done_count: AtomicU32,
    pub other_signals: u32,
    /// ADDRESS events seen in RX polls: a packet with our address arrived
    /// (regardless of CRC). Diagnostic for deaf-RX questions.
    pub addr_events: u32,
    /// Learned follower PLL address target (us from RXEN). Starts at the
    /// legacy default and is calibrated from the first locked catches.
    pub addr_target_us: u32,
    /// Number of locked catches used for the address-target calibration.
    pub calib_count: u32,
    /// Last RSSI sample (the RADIO RSSISAMPLE register, read before disable).
    /// Diagnostic for the RF-level marginality questions (the 5340 RX path).
    pub rssi_last: u32,
    /// First 8 bytes ([len | payload...]) of the last address-matched
    /// packet — shows what the peer's TX actually put on the air.
    pub last_rx_hdr: [u8; 14],

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
            catch_poll_us: 0,
            addr_poll_us: 0,
            addr_seen: false,
            rx_misses: 0,
            rx_window_us: 0,
            tx_delay_us: 0,
            tx_count: 0,
            peer_rx_window_us: 0,
            rx_buf: [0u8; 64],
            rx_ptr: core::ptr::null_mut(),
            rx_cap: 0,
            rx_result: 0,
            rx_ok: false,
            crc_ok: 0,
            crc_bad: 0,
            tx_buf: [0u8; 64],
            tx_ptr: core::ptr::null(),
            op_kind: OpKind::Idle as u8,
            session_id: 0,
            first_request: true,
            next_req: MaybeUninit::uninit(),
            done_signal: Signal::new(),
            slot_work_max: 0,
            slot_count: 0,
            done_count: AtomicU32::new(0),
            other_signals: 0,
            addr_events: 0,
            addr_target_us: 60,
            calib_count: 0,
            rssi_last: 0,
            last_rx_hdr: [0; 14],
            cur_channel: 25,
            cur_base0: 0xE7E7E7E7,
            cur_prefix: 0xE7,
        };
        this.rx_ptr = this.rx_buf.as_mut_ptr() as *mut u8;
        this.rx_cap = 64;
        this
    }
}
