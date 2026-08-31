//! The MPSL radio's runtime state, provided by the caller during init.

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::radio_phy::RadioMode;
use thunders::phy::SlotProbeStats;

/// One fixed-size packet: [0] = length, [1..=len] = payload.
pub type Pkt = [u8; 64];

pub(crate) struct AtomicProbeStats {
    slots: AtomicU32,
    clock_us: AtomicU32,
    completed: AtomicU32,
    op_late: AtomicU32,
    address_events: AtomicU32,
    crc_ok: AtomicU32,
    crc_bad_long: AtomicU32,
    tx_count: AtomicU32,
    windows: AtomicU32,
    aborted_windows: AtomicU32,
}

impl AtomicProbeStats {
    pub const fn new() -> Self {
        Self {
            slots: AtomicU32::new(0),
            clock_us: AtomicU32::new(0),
            completed: AtomicU32::new(0),
            op_late: AtomicU32::new(0),
            address_events: AtomicU32::new(0),
            crc_ok: AtomicU32::new(0),
            crc_bad_long: AtomicU32::new(0),
            tx_count: AtomicU32::new(0),
            windows: AtomicU32::new(0),
            aborted_windows: AtomicU32::new(0),
        }
    }

    pub fn wrapping_add(&self, delta: SlotProbeStats) {
        self.slots.fetch_add(delta.slots, Ordering::Relaxed);
        self.clock_us.fetch_add(delta.clock_us, Ordering::Relaxed);
        self.completed.fetch_add(delta.completed, Ordering::Relaxed);
        self.op_late.fetch_add(delta.op_late, Ordering::Relaxed);
        self.address_events
            .fetch_add(delta.address_events, Ordering::Relaxed);
        self.crc_ok.fetch_add(delta.crc_ok, Ordering::Relaxed);
        self.crc_bad_long
            .fetch_add(delta.crc_bad_long, Ordering::Relaxed);
        self.tx_count.fetch_add(delta.tx_count, Ordering::Relaxed);
        self.windows.fetch_add(delta.windows, Ordering::Relaxed);
        self.aborted_windows
            .fetch_add(delta.aborted_windows, Ordering::Relaxed);
    }

    pub fn load(&self) -> SlotProbeStats {
        SlotProbeStats {
            slots: self.slots.load(Ordering::Relaxed),
            clock_us: self.clock_us.load(Ordering::Relaxed),
            completed: self.completed.load(Ordering::Relaxed),
            op_late: self.op_late.load(Ordering::Relaxed),
            address_events: self.address_events.load(Ordering::Relaxed),
            crc_ok: self.crc_ok.load(Ordering::Relaxed),
            crc_bad_long: self.crc_bad_long.load(Ordering::Relaxed),
            tx_count: self.tx_count.load(Ordering::Relaxed),
            windows: self.windows.load(Ordering::Relaxed),
            aborted_windows: self.aborted_windows.load(Ordering::Relaxed),
        }
    }
}

/// What the next timeslot should do.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum OpKind {
    Idle = 0,
    Tx = 1,
    Rx = 2,
    /// One long grant collecting `[len|payload]` records in 64-byte cells.
    RxBatch = 3,
}

/// One published op in the parity ring: the app publishes ops ~2 slots
/// ahead of their target (the publish deadline is no longer the previous
/// op's completion but the target slot's START, ~2.5 slots of budget), so
/// two ops can be pending at once - entry `target % 2` holds the op for
/// `target`, and consecutive targets never alias.
pub struct OpEntry {
    pub(crate) kind: u8,
    /// Publication stamp (per-entry; bumped on each publish, written LAST).
    pub(crate) seq: u32,
    /// The absolute slot_count this op must execute in.
    pub(crate) target: u32,
    /// Extra slots a TX op may execute late (the first TX of a run).
    pub(crate) grace: u8,
    /// The RX target buffer (the caller's slice; the radio writes into it).
    pub(crate) rx_ptr: *mut u8,
    pub(crate) rx_cap: usize,
    /// The TX DMA buffer: [0] = len, [1..=len] = payload.
    pub(crate) tx_buf: [u8; 64],
    /// The last seq the callback consumed (executed or skipped).
    pub(crate) done_seq: u32,
    /// The consumption was a skip (late), not an execution.
    pub(crate) skipped: bool,
    /// RX result (valid once done_seq covers seq).
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
            tx_buf: [0u8; 64],
            done_seq: 0,
            skipped: false,
            rx_ok: false,
            rx_result: 0,
        }
    }
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

    // The slot constants (filled by the phy from its const generics).
    pub(crate) slot_nominal: u32,
    pub(crate) slot_len: u32,
    pub(crate) rx_poll: u32,
    /// Sender Data event duration used for packet-relative TimeDiff replies.
    pub(crate) one_way_data_slot_us: u32,
    /// Valid state frames accumulated since the previous TimeDiff reply.
    pub(crate) one_way_rx_since_feedback: u32,
    /// Currently active negotiated phase profile. Before the first commit,
    /// `active_profile_armed` is false and the uniform `slot_nominal` applies.
    pub(crate) active_profile_short_us: u32,
    pub(crate) active_profile_long_us: u32,
    pub(crate) active_profile_period: u32,
    pub(crate) active_profile_short_phases: u32,
    pub(crate) active_profile_central_apply_slot: u32,
    pub(crate) active_profile_local_apply_slot: u32,
    pub(crate) active_profile_armed: AtomicBool,
    /// Pending profile, atomically promoted to active at `profile_apply_slot`.
    pub(crate) profile_short_us: u32,
    pub(crate) profile_long_us: u32,
    pub(crate) profile_period: u32,
    pub(crate) profile_short_phases: u32,
    pub(crate) profile_central_apply_slot: u32,
    pub(crate) profile_apply_slot: u32,
    pub(crate) profile_armed: AtomicBool,
    /// Bounded trial overlay. It applies only in `[probe_start_slot,
    /// probe_end_slot)` and automatically falls back to the active profile,
    /// even if every control packet is lost during the trial.
    pub(crate) probe_short_us: u32,
    pub(crate) probe_long_us: u32,
    pub(crate) probe_period: u32,
    pub(crate) probe_short_phases: u32,
    pub(crate) probe_central_start_slot: u32,
    pub(crate) probe_start_slot: u32,
    pub(crate) probe_end_slot: u32,
    pub(crate) probe_armed: AtomicBool,
    /// Exact callback-boundary counters for bounded empirical probes.
    pub(crate) probe_clock_start_cyc: u32,
    pub(crate) probe_raw_start: SlotProbeStats,
    pub(crate) probe_stats_total: AtomicProbeStats,
    pub(crate) probe_started: bool,
    /// Odd while the callback publishes the multiword total, even when stable.
    pub(crate) probe_stats_seq: AtomicU32,
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
    /// Measured RX window duration from RXEN, excluding the fixed 40us
    /// shutdown reserve; advertised in the beacon.
    pub(crate) rx_window_us: u32,
    /// The follower's TX delay (us from slot start to TXEN): places the echo
    /// in the middle of the peer's advertised RX window. Recomputed on each
    /// catch; 0 = transmit at slot start.
    pub(crate) tx_delay_us: u32,
    /// Completed TX ops (the echo-flow diagnostic).
    pub(crate) tx_count: u32,
    /// The peer's advertised RX listen window (us); 0 = unknown.
    pub(crate) peer_rx_window_us: u32,

    // RX result diagnostics (the catch/CRC counters stay global: they
    // describe the radio, not one op).
    /// CRC diagnostics: packets with a good/bad CRCSTATUS (the 5340 net core
    /// decodes ~5% of address-matched frames - these count it).
    pub(crate) crc_ok: u32,
    pub(crate) crc_bad: u32,
    pub(crate) crc_bad_long: u32,
    /// TX packet length histogram: short (<10) vs long (>=10) on-air packets.
    pub(crate) tx_short: u32,
    pub(crate) tx_long: u32,
    /// Long-packet TX count by slot phase (slot_count % 10).
    pub(crate) tx_long_phase: [u32; 10],
    /// All TX ops by raw slot phase.
    pub(crate) tx_phase_all: [u32; 10],
    /// All RX ops by raw slot phase.
    pub(crate) rx_phase_all: [u32; 10],
    /// Last address-matched RX poll that failed CRC: diagnostics for the
    /// short-packet-passes/long-packet-fails split.
    pub(crate) last_rx_got_end: bool,
    pub(crate) last_rx_addr_us: u32,
    pub(crate) last_rx_end_us: u32,
    pub(crate) last_rx_slot_len: u32,
    pub(crate) last_rx_crc: u32,
    pub(crate) last_rx_in_flight: bool,
    pub(crate) last_rx_len: u32,

    // The op pipeline (app -> callback): a depth-2 parity ring. The app
    // publishes each op ~2 slots ahead of its target slot; the callback
    // consumes an entry only in its target slot (or one slot late for a
    // grace TX). A late op idles the slot instead of running off-phase -
    // the old level-based op_kind re-ran the previous slot's op whenever
    // the app published late, smearing TX/RX across phases (the LM20
    // pairs' dead reverse link).
    pub(crate) ops: [OpEntry; 2],
    /// Ops skipped because their target slot had already passed at their
    /// START: the app-loop lateness counter.
    pub op_late: u32,
    /// collect() return-path diagnostics (cumulative): no op published for
    /// the slot / op already done when collected / catch / empty-or-skip.
    pub coll_noop: u32,
    pub coll_late: u32,
    pub coll_catch: u32,
    pub coll_empty: u32,
    /// TX ops that executed inside their grace slot (late but useful).
    pub op_grace_used: u32,
    /// DWT cycle count at the current slot's START (for publish latency).
    pub(crate) last_start_cyc: u32,
    /// Max op-publish delay since the last snapshot (us from slot START;
    /// self-resetting). With the pipeline the app publishes ~2 slots
    /// early; this only bites when the app stalls a whole slot.
    pub op_publish_max_us: u32,

    // The MPSL session.
    pub(crate) session_id: u8,
    pub(crate) first_request: bool,
    pub(crate) next_req: MaybeUninit<nrf_mpsl::raw::mpsl_timeslot_request_t>,
    /// Signaled by the callback on the first granted slot (the ready gate).
    pub(crate) done_signal: Signal<CriticalSectionRawMutex, ()>,
    /// Signaled by the callback at every timeslot START (the slot-boundary
    /// wait used by transmit/receive to avoid late-call races).
    pub(crate) start_signal: Signal<CriticalSectionRawMutex, ()>,
    /// `done_count` value at the current slot's START, stored by the
    /// callback so the app can wait exactly one slot completion after the
    /// START it observed (even if the slot completed before the app woke).
    pub(crate) slot_start_done: u32,
    /// Diagnostics: START signals and completed works (atomic: the app
    /// spin-waits on it; a plain load could be hoisted out of the loop).
    pub slot_count: u32,
    pub done_count: AtomicU32,
    /// ADDRESS events seen in RX polls: a packet with our address arrived
    /// (regardless of CRC). Diagnostic for deaf-RX questions.
    pub addr_events: u32,
    /// Learned follower PLL address target in microseconds from RXEN.
    pub addr_target_us: u32,
    /// RXEN offset from slot START, measured on the last RX op (us).
    pub(crate) rx_en_offset_us: u32,
    /// RXEN -> READY ramp measured on the last RX op (us).
    pub(crate) rx_ramp_us: u32,
    /// TXEN offset from slot START, measured on the last TX op (us). For a
    /// follower TX this includes the intentional echo delay; beacons are
    /// only sent by the central, which transmits with zero echo delay.
    pub(crate) tx_en_offset_us: u32,
    /// TXEN -> READY ramp measured on the last TX op (us).
    pub(crate) tx_ramp_us: u32,
    /// TX-branch setup time measured just before the echo delay is applied
    /// (us from slot START). The echo delay is added after this point.
    pub(crate) tx_pre_delay_us: u32,
    /// The peer's measured RXEN offset from its beacon (us); 0 = unknown.
    pub(crate) peer_rx_en_offset_us: u32,
    /// The peer's measured RX ramp from its beacon (us); 0 = unknown.
    pub(crate) peer_rx_ramp_us: u32,
    /// The peer's measured TXEN offset from its beacon (us); 0 = unknown.
    pub(crate) peer_tx_en_offset_us: u32,
    /// The peer's measured TX ramp from its beacon (us); 0 = unknown.
    pub(crate) peer_tx_ramp_us: u32,
    /// When true, the follower is still sending SlotRequest and sweeps its
    /// TX delay through the peer's RX window instead of trusting the echo
    /// formula. Cleared once the reverse link has been acquired.
    pub(crate) tx_delay_sweep: bool,
    /// Current position in the SlotRequest TX-delay sweep.
    pub(crate) tx_delay_sweep_step: u8,
    /// Number of locked catches used for the address-target calibration.
    pub calib_count: u32,
    /// Last RSSI sample (the RADIO RSSISAMPLE register, read before disable).
    /// Diagnostic for the RF-level marginality questions (the 5340 RX path).
    pub rssi_last: u32,
    /// Cumulative RSSI sum over CRC-ok catches only (diff two PLL lines for
    /// a per-window average). Empty windows sample the noise floor, which
    /// says nothing about link margin - catches are the budget probe.
    pub rssi_catch_sum: u32,
    /// Cumulative count feeding `rssi_catch_sum`.
    pub rssi_catch_cnt: u32,
    /// Weakest catch since the last snapshot read (the LARGEST RSSISAMPLE
    /// magnitude = the smallest dBm); self-resetting in mpsl_pll_snapshot.
    /// Headroom estimate: ~95 (2M sensitivity) minus this, in dB.
    pub rssi_catch_max: u32,
    /// First 8 bytes ([len | payload...]) of the last address-matched
    /// packet — shows what the peer's TX actually put on the air.
    pub last_rx_hdr: [u8; 14],

    /// The configured on-air mode (set by `MpslRadioPhy::new`).
    pub(crate) radio_mode: RadioMode,
    /// Preamble/address anchor duration for the configured mode (us).
    pub(crate) air_prefix_us: u32,
    /// On-air duration per payload/CRC byte for the configured mode (us).
    pub(crate) air_byte_us: u32,

    // The radio config (the channel + the address).
    pub(crate) cur_channel: u8,
    pub(crate) cur_base0: u32,
    pub(crate) cur_prefix: u32,
}

impl MpslState {
    /// On-air duration for `len` payload bytes (the PHY frame adds the
    /// length byte and two CRC bytes).
    pub(crate) fn airtime_us(&self, len: usize) -> u32 {
        self.air_prefix_us + self.air_byte_us * (len as u32 + 3)
    }
}

impl MpslState {
    pub fn new(radio: nrf_pac::radio::Radio, follower: bool) -> Self {
        let this = Self {
            radio,
            follower,
            slot_nominal: 0,
            slot_len: 0,
            rx_poll: 0,
            one_way_data_slot_us: 0,
            one_way_rx_since_feedback: 0,
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
            probe_short_us: 0,
            probe_long_us: 0,
            probe_period: 0,
            probe_short_phases: 0,
            probe_central_start_slot: 0,
            probe_start_slot: 0,
            probe_end_slot: 0,
            probe_armed: AtomicBool::new(false),
            probe_clock_start_cyc: 0,
            probe_raw_start: SlotProbeStats::default(),
            probe_stats_total: AtomicProbeStats::new(),
            probe_started: false,
            probe_stats_seq: AtomicU32::new(0),
            slot_distance: 0,
            catch_poll_us: 0,
            addr_poll_us: 0,
            addr_seen: false,
            rx_misses: 0,
            rx_window_us: 0,
            tx_delay_us: 0,
            tx_count: 0,
            peer_rx_window_us: 0,
            crc_ok: 0,
            crc_bad: 0,
            crc_bad_long: 0,
            tx_short: 0,
            tx_long: 0,
            tx_long_phase: [0; 10],
            tx_phase_all: [0; 10],
            rx_phase_all: [0; 10],
            last_rx_got_end: false,
            last_rx_addr_us: 0,
            last_rx_end_us: 0,
            last_rx_slot_len: 0,
            last_rx_crc: 0,
            last_rx_in_flight: false,
            last_rx_len: 0,
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
            slot_start_done: 0,
            slot_count: 0,
            done_count: AtomicU32::new(0),
            addr_events: 0,
            addr_target_us: 60,
            rx_en_offset_us: 0,
            rx_ramp_us: 0,
            tx_en_offset_us: 0,
            tx_ramp_us: 0,
            tx_pre_delay_us: 0,
            peer_rx_en_offset_us: 0,
            peer_rx_ramp_us: 0,
            peer_tx_en_offset_us: 0,
            peer_tx_ramp_us: 0,
            tx_delay_sweep: false,
            tx_delay_sweep_step: 0,
            calib_count: 0,
            rssi_last: 0,
            rssi_catch_sum: 0,
            rssi_catch_cnt: 0,
            rssi_catch_max: 0,
            last_rx_hdr: [0; 14],
            radio_mode: RadioMode::Nrf2Mbit,
            air_prefix_us: 28,
            air_byte_us: 4,
            cur_channel: 25,
            cur_base0: 0xE7E7E7E7,
            cur_prefix: 0xE7,
        };
        this
    }
}
