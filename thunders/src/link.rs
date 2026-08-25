//! Link-layer state machines for central and peripheral.

/// Consecutive missed RX slots before the central advances the channel
/// (transient misses stay put; persistent jamming hops away).
const HOP_MISS_THRESHOLD: u8 = 16;

/// Consecutive missed RX slots before the link is declared lost: the status
/// drops back to `Disconnected` and the node returns to the initial channel,
/// so a recovered link can re-align.
const LINK_LOSS_THRESHOLD: u8 = 16;
const SYNC_LINK_LOSS_THRESHOLD: u8 = 64;

/// Consecutive successful frames before the link is called Connected (one
/// lucky catch on a marginal link must not enable the hop: the peer may
/// still be pinned to the initial channel, and hopping away deafens it).
const CONNECT_STREAK_THRESHOLD: u8 = 8;

/// The link's connection status.
///
/// The status gates channel hopping: while [`LinkStatus::Disconnected`] the
/// scheduler is pinned to the initial channel (no hop), so the two nodes'
/// slot schedules can align without ever landing on different channels. The
/// `CONNECT_STREAK_THRESHOLD`-long receive streak forms the connection and
/// enables the adaptive hop on the central.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinkStatus {
    /// No packet received yet, or the link was lost. Hop is disabled — the
    /// node holds the initial channel.
    Disconnected,
    /// The form-up streak has been reached; the central's adaptive channel
    /// hop is enabled.
    Connected,
}

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use heapless::Vec;

use crate::{
    cadence::{
        CadenceError, CadenceExitPolicy, CadenceNegotiationStatus, CadenceProbeBounds,
        CadenceProbePolicy, CadenceProfile, CadenceSearch, ProbeDecision, ProbeMetrics,
        TrafficContract,
    },
    config::{
        CENTRAL_REPLY_TIMEOUT_US, Config, MAX_PAYLOAD, NACK_BYTES, PERIPHERAL_LISTEN_TIMEOUT_US,
        Role, WINDOW_SIZE,
    },
    error::Error,
    link_mgmt::{
        LinkMgmt, RxWindow, TxRunSlot, nack_from_mask, nack_nonzero, nack_set, nack_vec, seq_gt,
    },
    packet::{CadenceStage, FIXED_DATA_HEADER_LEN, Packet},
    phy::{Phy, SlotProbeStats},
    scheduler::Scheduler,
};

#[cfg(feature = "secure")]
use crate::security::{Cipher, CipherMode, Security, make_nonce, make_nonce_13};

/// Size of the ChaCha20-Poly1305 authentication tag.
#[cfg(feature = "secure")]
const TAG_LEN: usize = 16;

/// Shared link state.
struct LinkState {
    scheduler: Scheduler,
    /// The link-management layer: sender window, receiver window, and the
    /// slot-NACK run bookkeeping.
    lm: LinkMgmt,
    epoch: u32,
    /// Central/forward TX:RX slot ratio.
    tx_rx_ratio: (u8, u8),
    /// Peripheral/reverse TX:RX slot ratio (normalized to the complement of
    /// `tx_rx_ratio` if the caller provided an incompatible schedule).
    reverse_tx_rx_ratio: (u8, u8),
    /// Shared idle slots appended after both local TX/RX runs.
    idle_slots: u8,
    /// The slot step counter (the TX/RX decision).
    ///
    /// Kept as u32 so the ratio phase never jumps when a byte-sized counter
    /// wraps: with a u8, step 255 -> 0 skips part of the period whenever
    /// `tx + rx` does not divide 256 (e.g. the default 8:2 period of 10).
    /// Used only when the PHY has no hardware slot counter.
    slot_step: u32,
    /// Peripheral only: added to the hardware slot count so the mirrored
    /// schedule lines up with the central's phase. Computed from the last
    /// beacon's `slot_phase`.
    slot_offset: u32,
    /// Beacon-anchor voting: a candidate is adopted only when two
    /// CONSECUTIVE beacons agree, so a single late-processed or
    /// gap-shifted measurement cannot freeze the mirror at a wrong
    /// offset.
    beacon_anchor_pending: Option<u32>,
    /// True once the two-beacon vote adopted an absolute hardware-slot
    /// mirror offset. Cadence profiles are not armed before this mapping is
    /// known, because their apply epoch is expressed in central slot space.
    beacon_anchor_ready: bool,
    /// The connection status (the hop gate).
    status: LinkStatus,
    /// Consecutive catches while Disconnected (the form-up streak).
    connect_streak: u8,
    /// The peripheral never advances the hop locally: the beacon's channel
    /// index is the hop authority (it can't hop away without losing the
    /// central); it just follows.
    central: bool,
    /// The channel index both nodes hold while disconnected.
    initial_channel: u8,
    #[cfg(feature = "secure")]
    cipher: Option<Cipher>,
}

impl LinkState {
    fn new(cfg: &Config) -> Self {
        let mut scheduler = Scheduler::new(cfg.network);
        scheduler.sync(cfg.initial_channel);
        // Normalize the ratio even when `Config.tx_rx_ratio` was mutated
        // directly: a zero component would make the slot period zero and
        // `% period` would panic in `next_phase`.
        let tx_rx_ratio = (cfg.tx_rx_ratio.0.max(1), cfg.tx_rx_ratio.1.max(1));
        // The two local schedules must be complementary. An incompatible
        // manually-written reverse ratio is normalized so construction can
        // never produce two nodes that transmit at the same phase.
        let reverse_tx_rx_ratio = if cfg.reverse_tx_rx_ratio == (tx_rx_ratio.1, tx_rx_ratio.0) {
            cfg.reverse_tx_rx_ratio
        } else {
            (tx_rx_ratio.1, tx_rx_ratio.0)
        };
        let idle_slots = cfg.idle_slots.min(255);
        Self {
            scheduler,
            lm: LinkMgmt::new(),
            epoch: 0,
            tx_rx_ratio,
            reverse_tx_rx_ratio,
            idle_slots,
            slot_step: 0,
            slot_offset: 0,
            beacon_anchor_pending: None,
            beacon_anchor_ready: false,
            status: LinkStatus::Disconnected,
            connect_streak: 0,
            central: matches!(cfg.role, Role::Central),
            initial_channel: cfg.initial_channel,
            #[cfg(feature = "secure")]
            cipher: cfg.security.as_ref().map(Security::cipher),
        }
    }

    /// Map a central-schedule phase to this node's local phase. With idle
    /// slots the complement is piecewise: central TX -> peripheral RX,
    /// central RX -> peripheral TX, central idle -> peripheral idle.
    fn local_phase_for(&self, phase: u32) -> u32 {
        let (c_tx, c_rx) = self.tx_rx_ratio;
        let period = c_tx as u32 + c_rx as u32 + self.idle_slots as u32;
        if self.central {
            phase % period
        } else if phase < c_tx as u32 {
            (phase + c_rx as u32) % period
        } else if phase < c_tx as u32 + c_rx as u32 {
            phase - c_tx as u32
        } else {
            phase % period
        }
    }

    /// The phase of the slot that is about to execute.
    ///
    /// With a hardware slot counter (MPSL), the next slot is
    /// `hw_slot + 1`; the peripheral additionally applies the beacon-derived
    /// `slot_offset`. With a software-paced PHY (`hw_slot == 0`), the
    /// link's own `slot_step` is the phase.
    fn next_phase(&self, hw_slot: u32, period: u32) -> u32 {
        if hw_slot == 0 {
            self.slot_step % period
        } else {
            (hw_slot.wrapping_add(1).wrapping_add(self.slot_offset)) % period
        }
    }

    /// A missed RX slot. While disconnected the scheduler is pinned to the
    /// initial channel (no hop). Once connected, a short streak makes the
    /// central hop away from a jammed channel; a long streak declares the
    /// link lost and pins back to the initial channel so a recovered link
    /// can re-align. The peripheral never hops on its own streak.
    fn on_miss(&mut self, streak: &mut u8, loss_threshold: u8) {
        *streak = streak.saturating_add(1);
        self.connect_streak = 0;
        if self.status != LinkStatus::Connected {
            return;
        }
        if *streak >= loss_threshold {
            self.status = LinkStatus::Disconnected;
            self.scheduler.sync(self.initial_channel);
            *streak = 0;
        } else if loss_threshold == LINK_LOSS_THRESHOLD
            && self.central
            && *streak >= HOP_MISS_THRESHOLD
        {
            // Only the central drives the hop; the peripheral follows via
            // the beacon's channel index.
            self.scheduler.advance();
            *streak = 0;
        }
    }

    /// A successful RX slot: the form-up streak forms the connection
    /// (enabling the hop) only after the link proves it can sustain.
    fn on_rx(&mut self, streak: &mut u8) {
        self.connect_streak = self.connect_streak.saturating_add(1);
        if self.connect_streak >= CONNECT_STREAK_THRESHOLD {
            self.status = LinkStatus::Connected;
        }
        *streak = 0;
    }

    /// Ciphertext bytes carried by the fixed Data codec for an application
    /// payload of `len` bytes.
    fn wire_payload_len(&self, len: usize) -> Option<usize> {
        #[cfg(feature = "secure")]
        let tag = self.cipher.as_ref().map_or(0, |cipher| match cipher.mode {
            CipherMode::ChaCha => TAG_LEN,
            CipherMode::Ccm => 4,
        });
        #[cfg(not(feature = "secure"))]
        let tag = 0;
        len.checked_add(tag).filter(|&n| n <= MAX_PAYLOAD)
    }

    /// Encrypt a `Data` payload in place before transmission.
    ///
    /// `sender_central` is `true` when *this* node is encrypting the
    /// outbound payload (central-to-peripheral direction). The nonce binds
    /// only to `seq` + the direction, so a retransmission of the same seq
    /// produces the same ciphertext (the receiver derives the same nonce).
    #[cfg(feature = "secure")]
    fn encrypt_payload<P: Phy>(
        &self,
        phy: &mut P,
        payload: &mut Vec<u8, MAX_PAYLOAD>,
        seq: u16,
        sender_central: bool,
    ) -> Result<(), Error<P::Error>> {
        let cipher = match self.cipher.as_ref() {
            Some(c) => c,
            None => return Ok(()),
        };
        match cipher.mode {
            CipherMode::ChaCha => {
                if payload.len() + TAG_LEN > MAX_PAYLOAD {
                    return Err(Error::BufferTooSmall);
                }
                let nonce = make_nonce(seq, sender_central);
                cipher.encrypt(payload, &nonce)?;
            }
            CipherMode::Ccm => {
                if payload.len() + 4 > MAX_PAYLOAD {
                    return Err(Error::BufferTooSmall);
                }
                let nonce = make_nonce_13(seq, sender_central);
                let mut key16 = [0u8; 16];
                key16.copy_from_slice(&cipher.key[..16]);
                let mut mic = [0u8; 4];
                phy.ccm_crypt(&key16, &nonce, payload, &mut mic, true)?;
                payload
                    .extend_from_slice(&mic)
                    .map_err(|_| Error::BufferTooSmall)?;
            }
        }
        Ok(())
    }

    #[cfg(not(feature = "secure"))]
    fn encrypt_payload<P: Phy>(
        &self,
        _phy: &mut P,
        _payload: &mut Vec<u8, MAX_PAYLOAD>,
        _seq: u16,
        _sender_central: bool,
    ) -> Result<(), Error<P::Error>> {
        Ok(())
    }

    /// Decrypt a received `Data` payload in place.
    ///
    /// `sender_central` is `true` when the *remote* sender was the central.
    #[cfg(feature = "secure")]
    fn decrypt_payload<P: Phy>(
        &self,
        phy: &mut P,
        payload: &mut Vec<u8, MAX_PAYLOAD>,
        seq: u16,
        sender_central: bool,
    ) -> Result<(), Error<P::Error>> {
        let cipher = match self.cipher.as_ref() {
            Some(c) => c,
            None => return Ok(()),
        };
        match cipher.mode {
            CipherMode::ChaCha => {
                let nonce = make_nonce(seq, sender_central);
                cipher.decrypt(payload, &nonce)?;
            }
            CipherMode::Ccm => {
                if payload.len() < 4 {
                    return Err(Error::InvalidPacket);
                }
                let nonce = make_nonce_13(seq, sender_central);
                let mut key16 = [0u8; 16];
                key16.copy_from_slice(&cipher.key[..16]);
                let plen = payload.len() - 4;
                let mut mic = [0u8; 4];
                mic.copy_from_slice(&payload[plen..]);
                phy.ccm_crypt(&key16, &nonce, &mut payload[..plen], &mut mic, false)?;
                payload.truncate(plen);
            }
        }
        Ok(())
    }

    #[cfg(not(feature = "secure"))]
    fn decrypt_payload<P: Phy>(
        &self,
        _phy: &mut P,
        _payload: &mut Vec<u8, MAX_PAYLOAD>,
        _seq: u16,
        _sender_central: bool,
    ) -> Result<(), Error<P::Error>> {
        Ok(())
    }
}

/// Deliver one in-order payload from the reorder window into `rx_buf`.
fn deliver_rx<P>(rx: &mut RxWindow, rx_buf: &mut [u8]) -> Result<Option<usize>, Error<P>> {
    if let Some(len) = rx.peek_len() {
        if len > rx_buf.len() {
            return Err(Error::BufferTooSmall);
        }
        if let Some(entry) = rx.pop_head() {
            rx_buf[..len].copy_from_slice(&entry.payload[..len]);
            return Ok(Some(len));
        }
    }
    Ok(None)
}

const CADENCE_FLAG_STABLE: u8 = 1;
const CADENCE_FLAG_REJECT: u8 = 2;
const CADENCE_FLAG_RELEASE: u8 = 4;
const CADENCE_FLAG_CONFIRM: u8 = 8;
const CADENCE_FLAG_FIXED_WIRE: u8 = 16;
const CADENCE_FLAG_PROBE_ABORT: u8 = 32;
const CADENCE_FLAG_SYNC_SLOT: u8 = 64;
const CADENCE_PROBATION_SUPERFRAMES: u32 = 2048;
const CADENCE_BEACON_WIRE_LEN: u16 = 36;
const BEACON_FLAG_SYNC_SLOT: u8 = 0x80;
const BEACON_FLAG_INITIAL_COMMIT: u8 = 0x40;
const SYNC_PRODUCTION_SHORT_FLOOR_US: u16 = if cfg!(feature = "cadence-fast") {
    450
} else {
    500
};
const SYNC_PRODUCTION_LONG_FLOOR_US: u16 = 600;

fn cadence_exit_triggered(policy: CadenceExitPolicy, failed: u32, misses: u8) -> bool {
    (policy.delivery_failures != 0 && failed >= policy.delivery_failures as u32)
        || (policy.consecutive_misses != 0 && misses >= policy.consecutive_misses)
}

fn cadence_generation_newer(candidate: u8, current: u8) -> bool {
    let delta = candidate.wrapping_sub(current) & 0x7f;
    delta != 0 && delta < 64
}

fn cadence_offer_generation_allowed(stage: CadenceRunStage, current: u8, incoming: u8) -> bool {
    match stage {
        CadenceRunStage::Request => incoming == current,
        CadenceRunStage::Accept => {
            incoming == current || cadence_generation_newer(incoming, current)
        }
        CadenceRunStage::Idle | CadenceRunStage::Stable | CadenceRunStage::Failed => {
            cadence_generation_newer(incoming, current)
        }
        _ => false,
    }
}

fn accept_legacy_data_plane(active_fixed: bool, grace: &mut u8) -> bool {
    if !active_fixed {
        return true;
    }
    if *grace == 0 {
        return false;
    }
    *grace -= 1;
    true
}

fn should_join_central_fallback(active_contract: bool, advertised_apply_epoch: u32) -> bool {
    active_contract && advertised_apply_epoch == 0
}

fn probe_timing_bad(clock_us: u32, expected_us: u32) -> bool {
    clock_us != 0 && clock_us.abs_diff(expected_us) > expected_us / 50 + 30
}

fn required_probe_rx(confirming: bool, expected_rx: u32) -> u32 {
    if confirming {
        u32::from(expected_rx != 0)
    } else {
        expected_rx.div_ceil(2)
    }
}

#[cfg(all(feature = "probe-lead-3", feature = "probe-lead-4"))]
compile_error!("select only one probe lead feature");

const PROBE_ARM_LEAD_SLOTS: i32 = if cfg!(feature = "probe-lead-4") {
    4
} else if cfg!(feature = "probe-lead-3") {
    3
} else {
    2
};

fn probe_has_sufficient_arm_lead(start_delta: i32) -> bool {
    start_delta >= PROBE_ARM_LEAD_SLOTS
}

fn profile_short_phases(forward_data_slots: u16, _sync_slot: bool) -> u16 {
    forward_data_slots
}

fn profile_forward_boundary(profile: CadenceProfile) -> u32 {
    profile.forward_slots as u32 + profile.sync_slot as u32
}

fn data_phase_from_physical(phase: u32, sync_slot: bool) -> Option<u32> {
    if sync_slot {
        phase.checked_sub(1)
    } else {
        Some(phase)
    }
}

fn profile_central_start(central_start: u32, period: u32, sync_slot: bool) -> u32 {
    if sync_slot {
        central_start.wrapping_add(period.max(1).wrapping_sub(1))
    } else {
        central_start
    }
}

fn descriptor_phase(local_slot: u32, local_start: u32, central_start: u32, period: u32) -> u32 {
    central_start.wrapping_add(local_slot.wrapping_sub(local_start)) % period.max(1)
}

fn feedback_uses_previous_run(
    collected_local_phase: u32,
    target_local_phase: u32,
    local_tx: u8,
) -> bool {
    // The first feedback op was published before the peer finalized this run's
    // NACK, so it still describes R-1. The final feedback is collected while
    // phase 0 publication rotates R into the previous map.
    collected_local_phase == local_tx as u32 || target_local_phase == 0
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CadenceRunStage {
    Idle,
    Request,
    Offer,
    Accept,
    ProbePlan,
    Armed,
    Probing,
    Report,
    Commit,
    Applying,
    Probation,
    Stable,
    Failed,
}

struct CadenceRuntime {
    stage: CadenceRunStage,
    generation: u8,
    search: Option<CadenceSearch>,
    contract: TrafficContract,
    fixed_wire: bool,
    sync_slot: bool,
    policy: CadenceProbePolicy,
    candidate: CadenceProfile,
    stable: CadenceProfile,
    central_start: u32,
    central_end: u32,
    local_start: u32,
    local_end: u32,
    stats_start: SlotProbeStats,
    delivery_failures_start: u32,
    probe_started: bool,
    probe_floor_short_us: u16,
    probe_floor_long_us: u16,
    local_probe_floor_short_us: u16,
    local_probe_floor_long_us: u16,
    peer_probe_floor_short_us: u16,
    peer_probe_floor_long_us: u16,
    probe_superframes_current: u16,
    probe_completed_superframes: u16,
    probe_failed_bursts: u16,
    probe_abort_retries: u8,
    confirming: bool,
    local_metrics: Option<ProbeMetrics>,
    peer_metrics: Option<ProbeMetrics>,
    apply_epoch: u32,
    pending_slot_offset: u32,
    probation_deadline: u32,
    probation_failures_start: u32,
    probation_rx_data_start: u32,
    probation_tx_data_start: u32,
    control_deadline: u32,
    commit_changes_profile: bool,
    releasing: bool,
    release_deadline: u32,
    error: Option<CadenceError>,
}

impl CadenceRuntime {
    fn new(stable: CadenceProfile) -> Self {
        Self {
            stage: CadenceRunStage::Idle,
            generation: 0,
            search: None,
            contract: TrafficContract::new(0, 0),
            fixed_wire: false,
            sync_slot: false,
            policy: CadenceProbePolicy::default(),
            candidate: stable,
            stable,
            central_start: 0,
            central_end: 0,
            local_start: 0,
            local_end: 0,
            stats_start: SlotProbeStats::default(),
            delivery_failures_start: 0,
            probe_started: false,
            probe_floor_short_us: 0,
            probe_floor_long_us: 0,
            local_probe_floor_short_us: 0,
            local_probe_floor_long_us: 0,
            peer_probe_floor_short_us: 0,
            peer_probe_floor_long_us: 0,
            probe_superframes_current: 0,
            probe_completed_superframes: 0,
            probe_failed_bursts: 0,
            probe_abort_retries: 0,
            confirming: false,
            local_metrics: None,
            peer_metrics: None,
            apply_epoch: 0,
            pending_slot_offset: 0,
            probation_deadline: 0,
            probation_failures_start: 0,
            probation_rx_data_start: 0,
            probation_tx_data_start: 0,
            control_deadline: 0,
            commit_changes_profile: false,
            releasing: false,
            release_deadline: 0,
            error: None,
        }
    }
}

/// Shared link data plane.
///
/// Forward and reverse share the same TX/RX slot engine. `Role` only changes
/// the synchronization path: the central is the hop/cadence master and sends
/// beacons; the peripheral follows the beacon and sends SlotRequest while
/// acquiring. See `docs/symmetric-link-design.md`.
struct LinkCore<P: Phy> {
    phy: P,
    state: LinkState,
    /// True after the first SlotRequest: cadence is negotiated once, then
    /// the central keeps that period and the peripheral follows it.
    cadence_negotiated: bool,
    /// False until the central's beacon advertises/schedules a cadence this
    /// PHY can sustain. Peripheral-only; the central ignores this field.
    cadence_ok: bool,
    /// Negotiated mixed-cadence profile. ID 0 keeps legacy uniform cadence;
    /// ID 1 is the current short/long phase profile.
    cadence_id: u8,
    cadence_short_us: u16,
    cadence_long_us: u16,
    cadence_short_phases: u16,
    cadence_apply_epoch: u32,
    /// Exact initial proposal epoch advertised by the central.
    initial_sync_proposal_epoch: u32,
    /// Peripheral repeats SyncReady for this proposal.
    initial_sync_ready_epoch: u32,
    /// Peripheral repeats SyncArmed for this scheduled commit.
    initial_sync_armed_epoch: u32,
    /// Central Beacon currently advertises a commit rather than a proposal.
    initial_sync_commit: bool,
    cadence_ack: u8,
    /// API-triggered traffic-contract negotiation and bounded probe state.
    cadence_runtime: CadenceRuntime,
    /// Acquisition-negotiated profile used when releasing a short-payload
    /// contract. API probes never overwrite this safety anchor.
    cadence_safe_profile: CadenceProfile,
    /// Contract currently enforced by the data plane. It remains active while
    /// a replacement or release handshake is in flight.
    cadence_active_contract: Option<TrafficContract>,
    /// Whether the active contract uses the negotiated fixed data codec.
    cadence_active_fixed_wire: bool,
    /// Whether phase 0 is the negotiated long Beacon/resync slot.
    cadence_active_sync_slot: bool,
    /// Fixed contract retained only to decode depth-two packets published
    /// before a synchronized release switched outbound traffic to postcard.
    retired_fixed_contract: Option<TrafficContract>,
    retired_fixed_rx_grace: u8,
    /// Remaining old-postcard data-plane packets tolerated after an epoch
    /// switch, covering operations published by the two-slot pipeline.
    fixed_legacy_rx_grace: u8,
    /// Optional severe-loss policy that requests a synchronized release.
    cadence_exit_policy: Option<CadenceExitPolicy>,
    /// Delivery-failure baseline captured when the active contract stabilized.
    cadence_exit_failure_baseline: u32,
    /// Last channel written to the phy (the phy is only re-tuned on change).
    last_channel: Option<u8>,
    /// Consecutive missed replies (the adaptive-hop trigger).
    consecutive_misses: u8,
    /// Peripheral-only missed-frame counter.
    missed_frames: u8,
    /// A TX burst is in progress (central-only for now; the bare follower
    /// joins this path once burst begin respects follower echo timing).
    in_burst: bool,
    /// Diagnostics: Data packets decoded from the peer (cumulative).
    pub(crate) rx_data: u32,
    /// Diagnostics: Data packets published for TX (cumulative).
    pub(crate) tx_data: u32,
    /// Packets dropped after [`crate::config::MAX_RETRIES`] retransmits
    /// (delivery failures).
    delivery_failures: u32,
    /// Raw `frame` offers that found the TX window full (backpressure).
    window_full: u32,
    /// The raw offer of the last `frame()` call was enqueued into the TX
    /// window (false on RX/idle slots, on rejection, and while a pending
    /// typed-send payload took precedence).
    offer_taken: bool,
    /// Retransmissions performed (a packet re-sent after its first send).
    retransmits: u32,
    /// Receiver re-baselines (a peer-restart resync; diagnostic).
    resyncs: u32,
    /// Non-zero NACK bitmaps sent (diagnostic).
    nack_sent: u32,
    /// Non-zero NACK bitmaps received (diagnostic).
    nacks_recv: u32,
    /// A Data seq we dropped after retry exhaustion and must tell the peer
    /// about (the delivery-failure resync hint). Cleared once the peer's
    /// cumulative ACK covers it.
    pending_drop: Option<u16>,
    pending_tx: [u8; MAX_PAYLOAD],
    pending_tx_len: usize,
    send_done: Signal<CriticalSectionRawMutex, ()>,
    /// Signaled when TX-window space is available.
    tx_space: Signal<CriticalSectionRawMutex, ()>,
    tx_buf: [u8; MAX_PAYLOAD + 32],
    rx_pkt_buf: [u8; MAX_PAYLOAD + 32],
    /// Second RX buffer for the pipelined phy path (the parity
    /// double-buffer: the next op's DMA target while the previous catch
    /// is being processed).
    rx_pkt_buf2: [u8; MAX_PAYLOAD + 32],
}

impl<P: Phy> LinkCore<P> {
    async fn new(mut phy: P, cfg: Config) -> Result<Self, Error<P::Error>> {
        phy.set_address(&cfg.address).await;
        phy.flush().await;
        let profiled = phy.op_pipelined();
        let cadence_id = if profiled { 1 } else { 0 };
        let cadence_short_us = phy.min_short_slot_period_us();
        let cadence_long_us = phy.min_long_slot_period_us();
        let stable_profile = CadenceProfile::new(
            cadence_short_us,
            cadence_long_us,
            cfg.tx_rx_ratio.0.max(1) as u16,
            cfg.tx_rx_ratio.1.max(1) as u16,
            cfg.idle_slots.min(255) as u16,
        )
        .with_sync_slot(profiled);
        Ok(Self {
            cadence_negotiated: false,
            cadence_ok: phy.min_slot_period_us() == 0,
            cadence_id,
            cadence_short_us,
            cadence_long_us,
            cadence_short_phases: cfg.tx_rx_ratio.0.max(1) as u16,
            cadence_apply_epoch: 0,
            initial_sync_proposal_epoch: 0,
            initial_sync_ready_epoch: 0,
            initial_sync_armed_epoch: 0,
            initial_sync_commit: false,
            cadence_ack: 0,
            cadence_runtime: CadenceRuntime::new(stable_profile),
            cadence_safe_profile: stable_profile,
            cadence_active_contract: None,
            cadence_active_fixed_wire: false,
            cadence_active_sync_slot: profiled,
            retired_fixed_contract: None,
            retired_fixed_rx_grace: 0,
            fixed_legacy_rx_grace: 0,
            cadence_exit_policy: None,
            cadence_exit_failure_baseline: 0,
            last_channel: None,
            consecutive_misses: 0,
            missed_frames: 0,
            in_burst: false,
            rx_data: 0,
            tx_data: 0,
            delivery_failures: 0,
            window_full: 0,
            offer_taken: false,
            retransmits: 0,
            resyncs: 0,
            nack_sent: 0,
            nacks_recv: 0,
            pending_drop: None,
            pending_tx: [0u8; MAX_PAYLOAD],
            pending_tx_len: 0,
            send_done: Signal::new(),
            tx_space: Signal::new(),
            tx_buf: [0u8; MAX_PAYLOAD + 32],
            rx_pkt_buf: [0u8; MAX_PAYLOAD + 32],
            rx_pkt_buf2: [0u8; MAX_PAYLOAD + 32],
            state: LinkState::new(&cfg),
            phy,
        })
    }

    fn status(&self) -> LinkStatus {
        self.state.status
    }

    fn tx_window_full(&self) -> bool {
        self.state.lm.tx.is_full()
    }

    fn tx_inflight(&self) -> u8 {
        self.state.lm.tx.inflight
    }

    fn sync_slot_enabled(&self) -> bool {
        self.cadence_safe_profile.sync_slot
    }

    fn physical_period(&self) -> u32 {
        let (c_tx, c_rx) = self.state.tx_rx_ratio;
        c_tx as u32 + c_rx as u32 + self.state.idle_slots as u32 + self.sync_slot_enabled() as u32
    }

    fn link_phase(&self) -> u32 {
        let phase = self.state.next_phase(0, self.physical_period());
        self.to_local_phase(phase).unwrap_or(0)
    }

    /// Map a central physical phase to an application Data phase. The
    /// independent phase-0 sync slot has no Data phase and returns `None`.
    fn to_local_phase(&self, phase: u32) -> Option<u32> {
        data_phase_from_physical(phase, self.sync_slot_enabled())
            .map(|data_phase| self.state.local_phase_for(data_phase))
    }

    fn central_is_tx_phase(&self, phase: u32) -> bool {
        let c_tx = self.state.tx_rx_ratio.0 as u32;
        if self.sync_slot_enabled() {
            (1..=c_tx).contains(&phase)
        } else {
            phase < c_tx
        }
    }

    /// Adjust the follower early TX margin at runtime (central is a no-op).
    fn set_tx_phase_margin_us(&mut self, margin_us: i32) {
        self.phy.set_tx_phase_margin_us(margin_us);
    }

    /// Transmit an encoded packet through the symmetric TX path: burst when
    /// the PHY supports it, plain per-slot transmit otherwise.
    async fn transmit_outbound(&mut self, n: usize) -> Result<(), Error<P::Error>> {
        if self.in_burst {
            match self.phy.transmit_burst_send(&self.tx_buf[..n]) {
                Ok(()) => {}
                Err(Error::Unsupported) => {
                    self.phy.transmit(&self.tx_buf[..n]).await?;
                    self.in_burst = false;
                }
                Err(e) => return Err(e),
            }
        } else {
            match self.phy.transmit_burst_begin(&self.tx_buf[..n]) {
                Ok(()) => self.in_burst = true,
                Err(Error::Unsupported) => {
                    self.phy.transmit(&self.tx_buf[..n]).await?;
                }
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    fn nack_bytes_for_slots(slots: u8) -> usize {
        (slots as usize).div_ceil(8).min(NACK_BYTES)
    }

    fn fixed_nack_is_canonical(&self, packet: &Packet) -> bool {
        let slots = self.local_ratio().0 as usize;
        let rem = slots % 8;
        if rem == 0 {
            return true;
        }
        let nack = match packet {
            Packet::Data { nack, .. } | Packet::Ack { nack, .. } | Packet::Drop { nack, .. } => {
                nack
            }
            _ => return true,
        };
        nack.last()
            .is_some_and(|last| *last & !((1u8 << rem) - 1) == 0)
    }

    fn fixed_codec_lengths(
        &self,
        contract: TrafficContract,
        outgoing: bool,
    ) -> Option<(usize, usize)> {
        let (app_len, nack_slots) = if outgoing {
            let app = if self.state.central {
                contract.forward_payload_len
            } else {
                contract.reverse_payload_len
            };
            (app, self.local_ratio().1)
        } else {
            let app = if self.state.central {
                contract.reverse_payload_len
            } else {
                contract.forward_payload_len
            };
            (app, self.local_ratio().0)
        };
        Some((
            self.state.wire_payload_len(app_len as usize)?,
            Self::nack_bytes_for_slots(nack_slots),
        ))
    }

    /// Serialize `pkt` into the shared TX buffer; negotiated data-plane
    /// packets use the fixed codec, while all control packets remain postcard.
    fn encode_packet(&mut self, pkt: &Packet) -> Result<usize, Error<P::Error>> {
        if matches!(
            pkt,
            Packet::Data { .. } | Packet::Ack { .. } | Packet::Drop { .. }
        ) {
            if let Some(contract) = self
                .cadence_active_contract
                .filter(|_| self.cadence_active_fixed_wire)
            {
                let (payload_len, nack_len) = self
                    .fixed_codec_lengths(contract, true)
                    .ok_or(Error::BufferTooSmall)?;
                return pkt
                    .to_fixed_bytes(payload_len, nack_len, &mut self.tx_buf)
                    .map_err(|_| Error::InvalidPacket);
            }
        }
        pkt.to_bytes(&mut self.tx_buf)
            .map_err(Error::<P::Error>::from)
    }

    fn decode_packet(&mut self, buf: &[u8]) -> Result<Packet, Error<P::Error>> {
        let active = self
            .cadence_active_contract
            .filter(|_| self.cadence_active_fixed_wire);
        let pending = (self.cadence_runtime.fixed_wire
            && matches!(
                self.cadence_runtime.stage,
                CadenceRunStage::Applying | CadenceRunStage::Stable
            ))
        .then_some(self.cadence_runtime.contract);
        let retired = (self.retired_fixed_rx_grace != 0)
            .then_some(self.retired_fixed_contract)
            .flatten();
        if self.retired_fixed_rx_grace != 0 {
            self.retired_fixed_rx_grace -= 1;
            if self.retired_fixed_rx_grace == 0 {
                self.retired_fixed_contract = None;
            }
        }
        for contract in [active, pending, retired].into_iter().flatten() {
            if let Some((payload_len, nack_len)) = self.fixed_codec_lengths(contract, false) {
                if let Ok(packet) = Packet::from_fixed_bytes(buf, payload_len, nack_len) {
                    if self.fixed_nack_is_canonical(&packet) {
                        return Ok(packet);
                    }
                    return Err(Error::InvalidPacket);
                }
            }
        }
        if Packet::has_fixed_marker(buf) {
            return Err(Error::InvalidPacket);
        }
        let packet = Packet::from_bytes(buf).map_err(|_| Error::InvalidPacket)?;
        if matches!(
            packet,
            Packet::Data { .. } | Packet::Ack { .. } | Packet::Drop { .. }
        ) && !accept_legacy_data_plane(
            self.cadence_active_fixed_wire,
            &mut self.fixed_legacy_rx_grace,
        ) {
            return Err(Error::InvalidPacket);
        }
        Ok(packet)
    }

    /// Build the outbound `Packet::Data` for `seq` and record the slot
    /// position mapping for slot-NACK.
    fn data_packet(&mut self, seq: u16, slot: u8, nack_run_len: usize) -> Packet {
        self.state.lm.record_tx_slot(slot, seq);
        let (elen, epayload) = {
            let e = self.state.lm.tx.entry(seq);
            (e.len as usize, e.payload)
        };
        let mut pv = Vec::<u8, MAX_PAYLOAD>::new();
        // The entry length was bounded by MAX_PAYLOAD at enqueue time.
        let _ = pv.extend_from_slice(&epayload[..elen]);
        Packet::Data {
            seq,
            ack: self.state.lm.rx.ack(),
            nack: nack_vec(nack_run_len, &self.state.lm.nack_for_peer),
            payload: pv,
        }
    }

    fn ack_packet(&self, nack_run_len: usize) -> Packet {
        Packet::Ack {
            ack: self.state.lm.rx.ack(),
            nack: nack_vec(nack_run_len, &self.state.lm.nack_for_peer),
        }
    }

    fn drop_packet(&self, nack_run_len: usize) -> Packet {
        Packet::Drop {
            seq: self.pending_drop.unwrap_or(0),
            ack: self.state.lm.rx.ack(),
            nack: nack_vec(nack_run_len, &self.state.lm.nack_for_peer),
        }
    }

    fn cadence_probe_bounds(&self) -> Option<CadenceProbeBounds> {
        (self.cadence_runtime.probe_floor_short_us != 0
            && self.cadence_runtime.probe_floor_long_us != 0)
            .then_some(CadenceProbeBounds {
                local: (
                    self.cadence_runtime.local_probe_floor_short_us,
                    self.cadence_runtime.local_probe_floor_long_us,
                ),
                peer: (self.cadence_runtime.peer_probe_floor_short_us != 0
                    && self.cadence_runtime.peer_probe_floor_long_us != 0)
                    .then_some((
                        self.cadence_runtime.peer_probe_floor_short_us,
                        self.cadence_runtime.peer_probe_floor_long_us,
                    )),
                effective: (
                    self.cadence_runtime.probe_floor_short_us,
                    self.cadence_runtime.probe_floor_long_us,
                ),
            })
    }

    fn cadence_candidate(&self) -> Option<CadenceProfile> {
        matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::ProbePlan
                | CadenceRunStage::Armed
                | CadenceRunStage::Probing
                | CadenceRunStage::Report
        )
        .then_some(self.cadence_runtime.candidate)
    }

    fn cadence_status(&self) -> CadenceNegotiationStatus {
        if self.cadence_runtime.releasing {
            return CadenceNegotiationStatus::Releasing;
        }
        match self.cadence_runtime.stage {
            CadenceRunStage::Idle => CadenceNegotiationStatus::Idle,
            CadenceRunStage::Probing => CadenceNegotiationStatus::Probing {
                candidate: self.cadence_runtime.candidate,
            },
            CadenceRunStage::Commit | CadenceRunStage::Applying | CadenceRunStage::Probation => {
                CadenceNegotiationStatus::Applying {
                    profile: self.cadence_runtime.stable,
                }
            }
            CadenceRunStage::Stable => {
                CadenceNegotiationStatus::Stable(self.cadence_runtime.stable)
            }
            CadenceRunStage::Failed => CadenceNegotiationStatus::Failed(
                self.cadence_runtime
                    .error
                    .unwrap_or(CadenceError::PeerRejected),
            ),
            _ => CadenceNegotiationStatus::Negotiating,
        }
    }

    fn data_wire_lengths(
        &self,
        contract: TrafficContract,
        fixed_wire: bool,
    ) -> Result<(u16, u16), CadenceError> {
        let forward_payload = self
            .state
            .wire_payload_len(contract.forward_payload_len as usize)
            .ok_or(CadenceError::PayloadTooLarge)?;
        let reverse_payload = self
            .state
            .wire_payload_len(contract.reverse_payload_len as usize)
            .ok_or(CadenceError::PayloadTooLarge)?;
        let (forward_slots, reverse_slots) = self.state.tx_rx_ratio;
        let data_header = if fixed_wire {
            FIXED_DATA_HEADER_LEN
        } else {
            // Postcard Data: enum + two Vec lengths + worst-case u16 varints.
            9
        };
        let forward = data_header
            .checked_add(Self::nack_bytes_for_slots(reverse_slots))
            .and_then(|n| n.checked_add(forward_payload))
            .ok_or(CadenceError::WireLengthOverflow)?;
        let reverse = data_header
            .checked_add(Self::nack_bytes_for_slots(forward_slots))
            .and_then(|n| n.checked_add(reverse_payload))
            .ok_or(CadenceError::WireLengthOverflow)?;
        if forward > 63 || reverse > 63 {
            return Err(CadenceError::WireLengthOverflow);
        }
        Ok((forward as u16, reverse as u16))
    }

    fn cadence_local_probe_floors(
        &self,
        contract: TrafficContract,
        policy: CadenceProbePolicy,
        fixed_wire: bool,
        sync_slot: bool,
    ) -> Result<(u16, u16), CadenceError> {
        let (forward_wire, reverse_wire) = self.data_wire_lengths(contract, fixed_wire)?;
        let mut short = policy
            .min_slot_us
            .max(self.phy.min_probe_slot_period_us(forward_wire));
        let mut long = policy.min_slot_us.max(
            self.phy
                .min_probe_slot_period_us(reverse_wire.max(CADENCE_BEACON_WIRE_LEN)),
        );
        if sync_slot {
            short = short.max(SYNC_PRODUCTION_SHORT_FLOOR_US);
            long = long.max(SYNC_PRODUCTION_LONG_FLOOR_US);
        }
        Ok((short, long))
    }

    fn request_cadence(
        &mut self,
        contract: TrafficContract,
        policy: CadenceProbePolicy,
    ) -> Result<u8, CadenceError> {
        if !self.phy.op_pipelined() {
            return Err(CadenceError::Unsupported);
        }
        if !self.phy.slot_profile_active() {
            return Err(CadenceError::Busy);
        }
        if policy.min_slot_us == 0
            || policy.step_us == 0
            || policy.probe_superframes == 0
            || policy.step_us > u8::MAX as u16
            || policy.safety_steps > u8::MAX as u16
            || (policy.probe_superframes as u32)
                .saturating_mul(self.cadence_runtime.stable.superframe_us())
                > 30_000_000
        {
            return Err(CadenceError::InvalidPolicy);
        }
        if contract.forward_payload_len as usize > MAX_PAYLOAD
            || contract.reverse_payload_len as usize > MAX_PAYLOAD
            || self.data_wire_lengths(contract, true).is_err()
        {
            return Err(CadenceError::PayloadTooLarge);
        }
        if self.state.lm.tx.inflight != 0 || self.pending_drop.is_some() || self.pending_tx_len != 0
        {
            return Err(CadenceError::Busy);
        }
        if !matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::Idle | CadenceRunStage::Stable | CadenceRunStage::Failed
        ) {
            return Err(CadenceError::Busy);
        }
        let stable = self.cadence_runtime.stable;
        let (local_short_floor, local_long_floor) = self.cadence_local_probe_floors(
            contract,
            policy,
            true,
            self.cadence_runtime.stable.sync_slot,
        )?;
        if local_short_floor > stable.short_slot_us || local_long_floor > stable.long_slot_us {
            return Err(CadenceError::InvalidPolicy);
        }
        let generation = (self.cadence_runtime.generation.wrapping_add(1) & 0x7f).max(1);
        self.cadence_runtime = CadenceRuntime::new(stable);
        self.cadence_runtime.generation = generation;
        self.cadence_id = generation;
        self.cadence_ack = 0;
        self.cadence_runtime.contract = contract;
        self.cadence_runtime.fixed_wire = true;
        self.cadence_runtime.sync_slot = self.cadence_runtime.stable.sync_slot;
        self.cadence_runtime.policy = policy;
        self.cadence_runtime.candidate = stable;
        self.cadence_runtime.probe_floor_short_us = local_short_floor;
        self.cadence_runtime.probe_floor_long_us = local_long_floor;
        self.cadence_runtime.local_probe_floor_short_us = local_short_floor;
        self.cadence_runtime.local_probe_floor_long_us = local_long_floor;
        if self.state.central {
            // Offer the known-stable anchor first. The peer returns its local
            // feasibility floors in the compact Accept; only then does the
            // central construct the authoritative empirical candidate list.
            self.cadence_runtime.stage = CadenceRunStage::Offer;
        } else {
            // The peripheral API only requests a contract; the central still
            // owns candidate selection and every absolute probe/apply epoch.
            self.cadence_runtime.stage = CadenceRunStage::Request;
        }
        Ok(generation)
    }

    fn set_cadence_exit_policy(&mut self, policy: Option<CadenceExitPolicy>) {
        self.cadence_exit_policy = policy
            .map(|mut p| {
                if p.consecutive_misses > LINK_LOSS_THRESHOLD {
                    p.consecutive_misses = LINK_LOSS_THRESHOLD;
                }
                p
            })
            .filter(|p| p.is_enabled());
        self.cadence_exit_failure_baseline = self.delivery_failures;
    }

    fn request_cadence_exit(&mut self) -> Result<u8, CadenceError> {
        if !self.phy.op_pipelined() {
            return Err(CadenceError::Unsupported);
        }
        if self.cadence_runtime.stage == CadenceRunStage::Idle
            || (self.cadence_runtime.stage == CadenceRunStage::Failed
                && self.cadence_active_contract.is_none())
        {
            return Ok(0);
        }
        if !matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::Stable | CadenceRunStage::Probation | CadenceRunStage::Failed
        ) {
            return Err(CadenceError::Busy);
        }
        let active = self.cadence_runtime.stable;
        let generation = (self.cadence_runtime.generation.wrapping_add(1) & 0x7f).max(1);
        self.cadence_runtime = CadenceRuntime::new(active);
        self.cadence_runtime.generation = generation;
        self.cadence_runtime.releasing = true;
        self.cadence_runtime.contract =
            TrafficContract::new(MAX_PAYLOAD as u16, MAX_PAYLOAD as u16);
        self.cadence_runtime.candidate = self.cadence_safe_profile;
        self.cadence_id = generation;
        self.cadence_ack = 0;
        self.cadence_runtime.stage = if self.state.central {
            CadenceRunStage::Offer
        } else {
            CadenceRunStage::Request
        };
        Ok(generation)
    }

    fn complete_cadence_apply(&mut self) {
        if !self.state.central && self.cadence_runtime.apply_epoch != 0 {
            self.state.slot_offset = self.cadence_runtime.pending_slot_offset;
        }
        if self.cadence_runtime.releasing {
            self.retired_fixed_contract = self
                .cadence_active_contract
                .filter(|_| self.cadence_active_fixed_wire);
            self.retired_fixed_rx_grace = u8::from(self.retired_fixed_contract.is_some()) * 2;
            self.cadence_active_contract = None;
            self.cadence_active_fixed_wire = false;
            self.cadence_active_sync_slot = self.cadence_safe_profile.sync_slot;
            self.fixed_legacy_rx_grace = 0;
            self.cadence_runtime = CadenceRuntime::new(self.cadence_safe_profile);
            // Keep the completed generation/apply descriptor in periodic
            // beacons. A peripheral that lost the first post-apply packet can
            // then finish release without re-arming or guessing a profile.
            self.cadence_short_us = self.cadence_safe_profile.short_slot_us;
            self.cadence_long_us = self.cadence_safe_profile.long_slot_us;
            self.cadence_short_phases = self.cadence_safe_profile.forward_slots;
            self.cadence_ack = self.cadence_id | 0x80;
        } else {
            self.cadence_active_contract = Some(self.cadence_runtime.contract);
            self.cadence_active_fixed_wire = self.cadence_runtime.fixed_wire;
            self.cadence_active_sync_slot = self.cadence_runtime.sync_slot;
            self.fixed_legacy_rx_grace = u8::from(self.cadence_runtime.fixed_wire) * 2;
            self.cadence_runtime.probation_deadline = 0;
            self.cadence_runtime.probation_failures_start = self.delivery_failures;
            self.cadence_runtime.probation_rx_data_start = self.rx_data;
            self.cadence_runtime.probation_tx_data_start = self.tx_data;
            self.cadence_runtime.stage = CadenceRunStage::Probation;
            self.cadence_exit_failure_baseline = self.delivery_failures;
        }
    }

    fn start_failed_probe_release(&mut self) {
        let active = self.cadence_runtime.stable;
        let generation = (self.cadence_runtime.generation.wrapping_add(1) & 0x7f).max(1);
        self.cadence_runtime = CadenceRuntime::new(active);
        self.cadence_runtime.generation = generation;
        self.cadence_runtime.releasing = true;
        self.cadence_runtime.contract =
            TrafficContract::new(MAX_PAYLOAD as u16, MAX_PAYLOAD as u16);
        self.cadence_runtime.candidate = self.cadence_safe_profile;
        self.cadence_id = generation;
        self.cadence_ack = 0;
        self.cadence_runtime.stage = CadenceRunStage::Offer;
    }

    fn emergency_cadence_fallback(&mut self) {
        let fallback = self.phy.fallback_slot_period_us().max(1);
        self.phy.align_slot_period(fallback);
        self.retired_fixed_contract = self
            .cadence_active_contract
            .filter(|_| self.cadence_active_fixed_wire);
        self.retired_fixed_rx_grace = u8::from(self.retired_fixed_contract.is_some()) * 2;
        self.cadence_active_contract = None;
        self.cadence_active_fixed_wire = false;
        self.cadence_active_sync_slot = self.cadence_safe_profile.sync_slot;
        self.cadence_runtime = CadenceRuntime::new(self.cadence_safe_profile);
        self.cadence_id = 1;
        self.cadence_short_us = self.cadence_safe_profile.short_slot_us;
        self.cadence_long_us = self.cadence_safe_profile.long_slot_us;
        self.cadence_short_phases = self.cadence_safe_profile.forward_slots;
        self.cadence_apply_epoch = 0;
        self.initial_sync_proposal_epoch = 0;
        self.initial_sync_ready_epoch = 0;
        self.initial_sync_armed_epoch = 0;
        self.initial_sync_commit = false;
        self.cadence_ack = 0;
        self.cadence_negotiated = false;
        self.cadence_ok = false;
        self.state.lm.rx.have = false;
        self.consecutive_misses = 0;
        self.missed_frames = 0;
    }

    fn cadence_auto_exit_due(&self) -> bool {
        let Some(policy) = self.cadence_exit_policy else {
            return false;
        };
        let failed = self
            .delivery_failures
            .wrapping_sub(self.cadence_exit_failure_baseline);
        let misses = self.consecutive_misses.max(self.missed_frames);
        cadence_exit_triggered(policy, failed, misses)
    }

    fn align_future(slot: u32, period: u32, lead_periods: u32) -> u32 {
        let candidate = slot.wrapping_add(lead_periods.saturating_mul(period));
        let rem = candidate % period.max(1);
        if rem == 0 {
            candidate
        } else {
            candidate.wrapping_add(period - rem)
        }
    }

    fn start_probe_plan(&mut self, slot: u32, period: u32) {
        // Final confirmation gets a longer known-stable lead. Repeated Probe
        // descriptors/Armed replies during this interval let the follower PLL
        // recover from deliberately marginal isolated candidates before the
        // continuous profile is judged.
        let lead_periods = if self.cadence_runtime.confirming {
            64
        } else {
            8
        };
        let base = Self::align_future(slot, period, lead_periods);
        let isolated = !self.cadence_runtime.confirming;
        // Forward candidates exercise one phase-0 interval. Reverse candidates
        // exercise one first-reverse-phase interval. Stable slots surround
        // every trial so an unschedulable period cannot accumulate a whole
        // superframe of hardware-counter phase error.
        let reverse_trial = isolated
            && self.cadence_runtime.candidate.long_slot_us
                != self.cadence_runtime.stable.long_slot_us;
        let start = if reverse_trial {
            base.wrapping_add(profile_forward_boundary(self.cadence_runtime.stable))
        } else if isolated && self.cadence_runtime.sync_slot {
            // Phase 0 is the negotiated long Beacon/resync slot; exercise the
            // first payload-bearing short phase instead.
            base.wrapping_add(1)
        } else {
            base
        };
        let target_samples = if self.cadence_runtime.confirming {
            self.cadence_runtime.policy.probe_superframes.min(32)
        } else {
            self.cadence_runtime.policy.probe_superframes
        };
        let remaining =
            target_samples.saturating_sub(self.cadence_runtime.probe_completed_superframes);
        let probe_samples = if self.cadence_runtime.confirming {
            remaining.min(32).max(1)
        } else {
            remaining.min(1).max(1)
        };
        self.cadence_runtime.probe_superframes_current = probe_samples;
        let end = if isolated {
            start.wrapping_add(1)
        } else {
            start.wrapping_add(period.saturating_mul(probe_samples as u32))
        };
        self.cadence_runtime.central_start = start;
        self.cadence_runtime.central_end = end;
        self.cadence_runtime.local_start = start;
        self.cadence_runtime.local_end = end;
        self.cadence_runtime.probe_started = false;
        self.cadence_runtime.stats_start = self.phy.slot_probe_stats();
        self.cadence_runtime.delivery_failures_start = self.delivery_failures;
        self.cadence_runtime.local_metrics = None;
        self.cadence_runtime.peer_metrics = None;
        // The central deliberately does not arm its PHY yet. The peripheral
        // may arm this bounded descriptor, but the central joins only after a
        // matching Armed reply proves both sides know the same future window.
        self.cadence_runtime.stage = CadenceRunStage::ProbePlan;
    }

    fn arm_central_probe(&mut self) -> bool {
        let p = self.cadence_runtime.candidate;
        self.cadence_runtime.stats_start = self.phy.slot_probe_stats();
        self.cadence_runtime.delivery_failures_start = self.delivery_failures;
        if !self.phy.schedule_slot_probe(
            p.short_slot_us,
            p.long_slot_us,
            p.period_slots() as u16,
            profile_short_phases(p.forward_slots, self.cadence_runtime.sync_slot),
            profile_central_start(
                self.cadence_runtime.central_start,
                p.period_slots(),
                self.cadence_runtime.sync_slot,
            ),
            self.cadence_runtime.local_start,
            self.cadence_runtime.local_end,
        ) {
            return false;
        }
        self.cadence_runtime.stage = CadenceRunStage::Probing;
        true
    }

    fn start_final_commit(&mut self, slot: u32, period: u32, profile: CadenceProfile) {
        let apply = Self::align_future(slot, period, 8);
        let profile_changed = profile != self.cadence_runtime.stable;
        self.cadence_runtime.stable = profile;
        self.cadence_runtime.apply_epoch = apply;
        self.cadence_runtime.local_start = apply;
        self.cadence_runtime.commit_changes_profile = profile_changed;
        self.cadence_id = self.cadence_runtime.generation;
        self.cadence_short_us = profile.short_slot_us;
        self.cadence_long_us = profile.long_slot_us;
        self.cadence_short_phases = profile.forward_slots;
        self.cadence_apply_epoch = apply;
        self.cadence_runtime.stage = CadenceRunStage::Commit;
    }

    fn finish_probe(&mut self, slot: u32, period: u32) {
        let (Some(local), Some(peer)) = (
            self.cadence_runtime.local_metrics,
            self.cadence_runtime.peer_metrics,
        ) else {
            return;
        };
        let burst = ProbeMetrics::new(
            local.completed_superframes.min(peer.completed_superframes),
            local.forward_failures.saturating_add(peer.forward_failures),
            local.reverse_failures.saturating_add(peer.reverse_failures),
        );
        let target_superframes = if self.cadence_runtime.confirming {
            self.cadence_runtime.policy.probe_superframes.min(32)
        } else {
            self.cadence_runtime.policy.probe_superframes
        };
        let candidate = self.cadence_runtime.candidate;
        if burst.completed_superframes == 0 {
            if self.cadence_runtime.probe_abort_retries < 3 {
                self.cadence_runtime.probe_abort_retries += 1;
                self.start_probe_plan(slot, period);
            } else {
                self.start_failed_probe_release();
            }
            return;
        }
        self.cadence_runtime.probe_abort_retries = 0;
        if self.cadence_runtime.confirming {
            if burst.completed_superframes < target_superframes {
                self.start_probe_plan(slot, period);
                return;
            }
            self.cadence_runtime.confirming = false;
            if burst.forward_failures == 0 && burst.reverse_failures == 0 {
                self.start_final_commit(slot, period, candidate);
            } else {
                // Confirmation failure never commits a contract/profile. Use
                // the synchronized release handshake so a reporting peer also
                // leaves the failed generation before the 600us fallback.
                self.start_failed_probe_release();
            }
            return;
        }
        let burst_pass = burst.forward_failures == 0 && burst.reverse_failures == 0;
        self.cadence_runtime.probe_completed_superframes = self
            .cadence_runtime
            .probe_completed_superframes
            .saturating_add(burst.completed_superframes);
        if !burst_pass {
            self.cadence_runtime.probe_failed_bursts =
                self.cadence_runtime.probe_failed_bursts.saturating_add(1);
        }
        if self.cadence_runtime.probe_completed_superframes < target_superframes {
            self.start_probe_plan(slot, period);
            return;
        }
        // Real links have occasional empty/CRC-bad superframes even at the
        // active profile. A candidate passes only when at least 7/8 of its
        // independently restored bursts meet every timing and traffic check.
        let failed = u16::from(
            self.cadence_runtime.probe_failed_bursts > target_superframes.saturating_div(8),
        );
        let combined = ProbeMetrics::new(target_superframes, failed, failed);
        let Some(search) = self.cadence_runtime.search.as_mut() else {
            return;
        };
        match search.record_probe(candidate, combined) {
            Ok(ProbeDecision::Incomplete(_)) => {}
            Ok(ProbeDecision::Continue(next)) => {
                self.cadence_runtime.candidate = next;
                self.cadence_runtime.probe_completed_superframes = 0;
                self.cadence_runtime.probe_failed_bursts = 0;
                self.start_probe_plan(slot, period);
            }
            Ok(ProbeDecision::Passed(next)) if search.final_profile().is_none() => {
                self.cadence_runtime.candidate = next;
                self.cadence_runtime.probe_completed_superframes = 0;
                self.cadence_runtime.probe_failed_bursts = 0;
                self.start_probe_plan(slot, period);
            }
            Ok(ProbeDecision::Passed(profile)) | Ok(ProbeDecision::Failed(profile)) => {
                let tested = search.passed_probes() != 0 || search.failed_probes() != 0;
                if tested {
                    self.cadence_runtime.candidate = profile;
                    self.cadence_runtime.probe_completed_superframes = 0;
                    self.cadence_runtime.probe_failed_bursts = 0;
                    self.cadence_runtime.confirming = true;
                    self.start_probe_plan(slot, period);
                } else {
                    self.start_final_commit(slot, period, profile);
                }
            }
            Err(e) => {
                self.cadence_runtime.error = Some(e);
                self.cadence_runtime.stage = CadenceRunStage::Failed;
            }
        }
    }

    fn cadence_tick(&mut self, slot: u32, period: u32) {
        let loss_threshold = self.loss_threshold();
        if self.state.central
            && !self.cadence_negotiated
            && self.cadence_apply_epoch != 0
            && (slot.wrapping_sub(self.cadence_apply_epoch) as i32) >= 0
        {
            self.emergency_cadence_fallback();
            return;
        }
        if self.cadence_runtime.releasing {
            if self.cadence_runtime.release_deadline == 0 {
                self.cadence_runtime.release_deadline =
                    slot.wrapping_add(period.saturating_mul(256));
            } else if (slot.wrapping_sub(self.cadence_runtime.release_deadline) as i32) >= 0 {
                self.emergency_cadence_fallback();
                return;
            }
        }
        let control_exchange = matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::Request | CadenceRunStage::Offer | CadenceRunStage::Accept
        ) && !self.cadence_runtime.releasing;
        if control_exchange {
            if self.cadence_runtime.control_deadline == 0 {
                self.cadence_runtime.control_deadline =
                    slot.wrapping_add(period.saturating_mul(2048));
            } else if (slot.wrapping_sub(self.cadence_runtime.control_deadline) as i32) >= 0 {
                self.cadence_runtime.control_deadline = 0;
                self.cadence_runtime.error = Some(CadenceError::ControlTimeout);
                self.cadence_runtime.stage = CadenceRunStage::Failed;
                return;
            }
        } else {
            self.cadence_runtime.control_deadline = 0;
        }
        let probe_transition = matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::ProbePlan
                | CadenceRunStage::Armed
                | CadenceRunStage::Probing
                | CadenceRunStage::Report
        );
        let probe_recovery_expired = probe_transition
            && self.cadence_runtime.local_end != 0
            && (slot.wrapping_sub(
                self.cadence_runtime
                    .local_end
                    .wrapping_add(period.saturating_mul(64)),
            ) as i32)
                >= 0;
        if self.state.central
            && probe_recovery_expired
            && self.cadence_runtime.peer_metrics.is_none()
        {
            // Do not depend on link-level misses here: Accept traffic may keep
            // the link alive even when every Probe descriptor was lost.
            self.start_failed_probe_release();
            return;
        }
        if self.missed_frames >= loss_threshold
            && ((!probe_transition
                && (self.cadence_active_contract.is_some() || self.cadence_runtime.releasing))
                || probe_recovery_expired)
        {
            self.emergency_cadence_fallback();
            return;
        }
        if self.cadence_runtime.stage == CadenceRunStage::Probation {
            if self.cadence_runtime.probation_deadline == 0 {
                self.cadence_runtime.probation_deadline =
                    slot.wrapping_add(period.saturating_mul(CADENCE_PROBATION_SUPERFRAMES));
            }
            let probation_failed = self.delivery_failures
                != self.cadence_runtime.probation_failures_start
                || self.missed_frames >= loss_threshold;
            if self.state.central && probation_failed {
                self.start_failed_probe_release();
                return;
            }
            if (slot.wrapping_sub(self.cadence_runtime.probation_deadline) as i32) >= 0 {
                let bidirectional_data = self.rx_data
                    != self.cadence_runtime.probation_rx_data_start
                    && self.tx_data != self.cadence_runtime.probation_tx_data_start;
                if self.state.central && !bidirectional_data {
                    self.start_failed_probe_release();
                    return;
                }
                self.cadence_runtime.stage = CadenceRunStage::Stable;
                self.cadence_exit_failure_baseline = self.delivery_failures;
            }
        }
        if self.cadence_runtime.stage == CadenceRunStage::Stable && self.cadence_auto_exit_due() {
            let _ = self.request_cadence_exit();
        }
        // A central that never receives Armed must not execute its candidate
        // unilaterally. Let the peer's bounded window restore automatically,
        // then retry the same candidate at a fresh future epoch.
        if self.state.central
            && self.cadence_runtime.stage == CadenceRunStage::ProbePlan
            && (slot.wrapping_sub(self.cadence_runtime.local_end) as i32) >= 0
        {
            if self.cadence_runtime.probe_abort_retries < 3 {
                self.cadence_runtime.probe_abort_retries += 1;
                self.start_probe_plan(slot, period);
            } else {
                self.start_failed_probe_release();
            }
            return;
        }
        // The follower already armed the immutable descriptor. Enter Sample
        // mode early enough for the depth-two publisher to fill the first op.
        if !self.state.central
            && self.cadence_runtime.stage == CadenceRunStage::Armed
            && (slot.wrapping_sub(self.cadence_runtime.local_start.wrapping_sub(3)) as i32) >= 0
        {
            self.cadence_runtime.stage = CadenceRunStage::Probing;
        }
        let in_probe_state = self.cadence_runtime.stage == CadenceRunStage::Probing
            || (!self.state.central && self.cadence_runtime.stage == CadenceRunStage::Armed);
        if in_probe_state
            && !self.cadence_runtime.probe_started
            && (slot.wrapping_sub(self.cadence_runtime.local_start) as i32) >= 0
        {
            self.cadence_runtime.probe_started = true;
            if self.cadence_runtime.stage == CadenceRunStage::Armed {
                self.cadence_runtime.stage = CadenceRunStage::Probing;
            }
        }
        if in_probe_state
            && self.cadence_runtime.probe_started
            && self.cadence_runtime.local_metrics.is_none()
            && (slot.wrapping_sub(self.cadence_runtime.local_end) as i32) >= 0
        {
            let delta = self
                .phy
                .slot_probe_stats()
                .wrapping_delta(self.cadence_runtime.stats_start);
            let window_aborted = delta.windows == 0 || delta.aborted_windows != 0;
            let isolated = !self.cadence_runtime.confirming;
            let reverse_trial = isolated
                && self.cadence_runtime.candidate.long_slot_us
                    != self.cadence_runtime.stable.long_slot_us;
            let expected = if isolated {
                if reverse_trial {
                    self.cadence_runtime.candidate.long_slot_us as u32
                } else {
                    self.cadence_runtime.candidate.short_slot_us as u32
                }
            } else {
                self.cadence_runtime
                    .candidate
                    .superframe_us()
                    .saturating_mul(self.cadence_runtime.probe_superframes_current as u32)
            };
            // A translated follower epoch can miss the exact callback START
            // capture and report clock_us=0 even though slot count and traffic
            // cover the complete window. Then those independent checks win.
            let timing_bad = probe_timing_bad(delta.clock_us, expected);
            let (local_tx, local_rx) = if isolated {
                let local_transmits = self.state.central != reverse_trial;
                (u16::from(local_transmits), u16::from(!local_transmits))
            } else {
                let (tx, rx) = self.local_ratio();
                (tx as u16, rx as u16)
            };
            let expected_tx =
                self.cadence_runtime.probe_superframes_current as u32 * local_tx as u32;
            let expected_rx =
                self.cadence_runtime.probe_superframes_current as u32 * local_rx as u32;
            // A timing-only probe could pass after losing every Sample. Require
            // at least half the planned TX completions and RX address catches;
            // the latter still tolerates the measured ~28% weak reverse raw
            // loss before ARQ.
            // Isolated candidates must decode the exact worst-contract frame.
            // Final-profile confirmation instead checks ADDRESS overlap across
            // the continuous multi-superframe run: candidate CRC feasibility
            // was already established per direction, while weak links can lose
            // many payload CRCs without losing cadence synchronization.
            let rx_observed = if self.cadence_runtime.confirming {
                delta.address_events
            } else {
                delta.crc_ok
            };
            let required_rx = required_probe_rx(self.cadence_runtime.confirming, expected_rx);
            let samples_bad = delta.tx_count < expected_tx.div_ceil(2) || rx_observed < required_rx;
            let expected_slots = if isolated {
                1
            } else {
                self.cadence_runtime
                    .probe_superframes_current
                    .saturating_sub(1) as u32
                    * period
            };
            let slots_bad = delta.slots < expected_slots;
            let delivery_bad =
                self.delivery_failures != self.cadence_runtime.delivery_failures_start;
            let local_bad = window_aborted
                || delta.op_late != 0
                || timing_bad
                || samples_bad
                || slots_bad
                || delivery_bad;
            let completed = if window_aborted {
                0
            } else {
                self.cadence_runtime.probe_superframes_current
            };
            self.cadence_runtime.local_metrics = Some(ProbeMetrics::new(
                completed,
                u16::from(local_bad),
                u16::from(local_bad),
            ));
            if self.state.central {
                self.finish_probe(slot, period);
            } else {
                self.cadence_runtime.stage = CadenceRunStage::Report;
            }
        }
        if self.state.central
            && self.cadence_runtime.stage == CadenceRunStage::Commit
            && (slot.wrapping_sub(self.cadence_runtime.local_start) as i32) >= 0
        {
            let changed = self.cadence_runtime.commit_changes_profile;
            self.start_final_commit(slot, period, self.cadence_runtime.stable);
            self.cadence_runtime.commit_changes_profile = changed;
        }
        if self.state.central
            && self.cadence_runtime.stage == CadenceRunStage::Applying
            && (slot.wrapping_sub(self.cadence_runtime.local_start) as i32) >= 0
        {
            self.complete_cadence_apply();
        }
    }

    fn cadence_packet(&self, epoch: u32, local_rx: usize) -> Option<Packet> {
        let central = self.state.central;
        if self.cadence_runtime.stage == CadenceRunStage::Probing {
            // CadenceSample postcard overhead is 3 bytes (variant,
            // generation, Vec length). Match the planner's worst Data wire
            // length instead of adding the large negotiation-control header.
            let (forward_wire, reverse_wire) = self
                .data_wire_lengths(
                    self.cadence_runtime.contract,
                    self.cadence_runtime.fixed_wire,
                )
                .unwrap_or((MAX_PAYLOAD as u16, MAX_PAYLOAD as u16));
            let target_wire = if central { forward_wire } else { reverse_wire } as usize;
            let mut padding = Vec::<u8, { MAX_PAYLOAD + 32 }>::new();
            let _ = padding.resize(target_wire.saturating_sub(3), 0xA5);
            return Some(Packet::CadenceSample {
                generation: self.cadence_runtime.generation,
                padding,
            });
        }
        let stage = match (central, self.cadence_runtime.stage) {
            (false, CadenceRunStage::Request) if self.cadence_runtime.releasing => {
                CadenceStage::Release
            }
            (false, CadenceRunStage::Request) => CadenceStage::Request,
            (true, CadenceRunStage::Offer) if self.cadence_runtime.releasing => {
                CadenceStage::Release
            }
            (true, CadenceRunStage::Offer) => CadenceStage::Offer,
            (false, CadenceRunStage::Accept | CadenceRunStage::Failed) => CadenceStage::Accept,
            (true, CadenceRunStage::ProbePlan) => CadenceStage::Probe,
            (false, CadenceRunStage::Armed) => CadenceStage::Armed,
            (false, CadenceRunStage::Report) => CadenceStage::Report,
            (true, CadenceRunStage::Commit) => CadenceStage::Commit,
            (false, CadenceRunStage::Applying) => CadenceStage::Applied,
            _ => return None,
        };
        let local = self.cadence_runtime.local_metrics.unwrap_or_default();
        let wire_profile = if matches!(stage, CadenceStage::Commit | CadenceStage::Applied) {
            self.cadence_runtime.stable
        } else {
            self.cadence_runtime.candidate
        };
        let mut flags = if stage == CadenceStage::Accept
            && self.cadence_runtime.stage == CadenceRunStage::Failed
        {
            CADENCE_FLAG_REJECT
        } else if stage == CadenceStage::Report
            && local.forward_failures == 0
            && local.reverse_failures == 0
        {
            CADENCE_FLAG_STABLE
        } else {
            0
        };
        if stage == CadenceStage::Report && local.completed_superframes == 0 {
            flags |= CADENCE_FLAG_PROBE_ABORT;
        }
        if self.cadence_runtime.releasing {
            flags |= CADENCE_FLAG_RELEASE;
        }
        if stage == CadenceStage::Probe && self.cadence_runtime.confirming {
            flags |= CADENCE_FLAG_CONFIRM;
        }
        if self.cadence_runtime.fixed_wire && !self.cadence_runtime.releasing {
            flags |= CADENCE_FLAG_FIXED_WIRE;
        }
        if self.cadence_runtime.sync_slot && !self.cadence_runtime.releasing {
            flags |= CADENCE_FLAG_SYNC_SLOT;
        }
        if self.cadence_runtime.releasing
            && matches!(stage, CadenceStage::Release | CadenceStage::Commit)
        {
            return Some(Packet::CadenceAck {
                generation: self.cadence_runtime.generation,
                stage,
                start_epoch: if stage == CadenceStage::Commit {
                    self.cadence_runtime.apply_epoch
                } else {
                    0
                },
                // Compact Release/Commit uses this field for the sender slot
                // so the follower can translate the absolute apply epoch.
                end_epoch: epoch,
                flags,
            });
        }
        if !central && stage != CadenceStage::Request {
            return Some(Packet::CadenceAck {
                generation: self.cadence_runtime.generation,
                stage,
                start_epoch: if stage == CadenceStage::Applied {
                    self.cadence_runtime.apply_epoch
                } else {
                    self.cadence_runtime.central_start
                },
                end_epoch: self.cadence_runtime.central_end,
                flags,
            });
        }
        Some(Packet::Cadence {
            ack: self.state.lm.rx.ack(),
            // Keep the fixed control descriptor within the 64-byte PHY
            // buffer even for unusually wide RX runs. The normal Data/Ack
            // path resumes after the bounded negotiation and carries the full
            // bitmap.
            nack: nack_vec(local_rx.min(16), &self.state.lm.nack_for_peer),
            generation: self.cadence_runtime.generation,
            stage,
            epoch,
            forward_payload: self.cadence_runtime.contract.forward_payload_len as u8,
            reverse_payload: self.cadence_runtime.contract.reverse_payload_len as u8,
            short_us: wire_profile.short_slot_us,
            long_us: wire_profile.long_slot_us,
            min_slot_us: self.cadence_runtime.policy.min_slot_us,
            step_us: self.cadence_runtime.policy.step_us.min(255) as u8,
            safety_steps: self.cadence_runtime.policy.safety_steps.min(255) as u8,
            start_epoch: if stage == CadenceStage::Commit {
                self.cadence_runtime.apply_epoch
            } else {
                self.cadence_runtime.central_start
            },
            end_epoch: self.cadence_runtime.central_end,
            probe_slots: if self.cadence_runtime.probe_superframes_current != 0 {
                self.cadence_runtime.probe_superframes_current
            } else {
                self.cadence_runtime.policy.probe_superframes
            },
            flags,
        })
    }

    fn acquisition_packet(&self, min_slot_us: u16) -> Packet {
        if self.initial_sync_armed_epoch != 0 {
            Packet::SyncArmed {
                generation: self.cadence_id,
                apply_epoch: self.initial_sync_armed_epoch,
            }
        } else if self.initial_sync_ready_epoch != 0 {
            Packet::SyncReady {
                generation: self.cadence_id,
                proposal_epoch: self.initial_sync_ready_epoch,
            }
        } else {
            Packet::SlotRequest {
                min_slot_us,
                min_short_slot_us: self.phy.min_short_slot_period_us(),
                cadence_ack: self.cadence_ack,
                ack: self.state.lm.rx.ack(),
            }
        }
    }

    fn beacon_packet(&self, step: u32, period: u16, beacon_epoch: u32) -> Packet {
        let flags = (self.phy.rx_window_us() / 16).min(0x3f) as u8
            | if self.cadence_active_sync_slot {
                BEACON_FLAG_SYNC_SLOT
            } else {
                0
            }
            | if self.initial_sync_commit {
                BEACON_FLAG_INITIAL_COMMIT
            } else {
                0
            };
        Packet::Beacon {
            epoch: beacon_epoch,
            channel_index: self.state.scheduler.index(),
            flags,
            slot_us: self.phy.slot_period_us(),
            slot_phase: (step % period as u32) as u16,
            rx_en_offset: self.phy.rx_en_offset_us(),
            tx_en_offset: self.phy.tx_en_offset_us(),
            rx_ramp: self.phy.rx_ramp_us(),
            tx_ramp: self.phy.tx_ramp_us(),
            cadence_id: self.cadence_id,
            short_slot_us: self.cadence_short_us,
            long_slot_us: self.cadence_long_us,
            short_phases: self.cadence_short_phases,
            cadence_apply_epoch: self.cadence_apply_epoch,
        }
    }

    /// Enqueue a raw `frame` offer (or the pending typed-send payload).
    /// Returns `true` when a raw offer found the window full.
    fn enqueue_offer(&mut self, tx_payload: Option<&[u8]>) -> Result<bool, Error<P::Error>> {
        let mut pending_buf = [0u8; MAX_PAYLOAD];
        let mut pending_len = 0usize;
        if self.pending_tx_len > 0 {
            pending_len = self.pending_tx_len;
            pending_buf[..pending_len].copy_from_slice(&self.pending_tx[..pending_len]);
        }
        let offered: Option<&[u8]> = if pending_len > 0 {
            Some(&pending_buf[..pending_len])
        } else {
            tx_payload
        };
        let mut rejected = false;
        if let Some(data) = offered {
            if let Some(contract) = self.cadence_active_contract {
                let payload_len = if self.state.central {
                    contract.forward_payload_len
                } else {
                    contract.reverse_payload_len
                } as usize;
                if data.len() != payload_len {
                    return Err(Error::PayloadExceedsCadenceProfile);
                }
            }
            if self.state.lm.tx.is_full() {
                if pending_len == 0 {
                    self.window_full = self.window_full.wrapping_add(1);
                    rejected = true;
                }
            } else {
                let seq = self.state.lm.tx.tx_next;
                let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
                if data.len() > payload.capacity() {
                    return Err(Error::BufferTooSmall);
                }
                payload
                    .extend_from_slice(data)
                    .map_err(|_| Error::BufferTooSmall)?;
                self.state
                    .encrypt_payload(&mut self.phy, &mut payload, seq, self.state.central)?;
                self.state.lm.tx.enqueue(&payload);
                self.offer_taken = pending_len == 0;
                if pending_len > 0 {
                    self.pending_tx_len = 0;
                    self.send_done.signal(());
                }
            }
        }
        Ok(rejected)
    }

    /// Pick the Data seq for this TX slot (retransmit first, then new data,
    /// then the full-window fallback).
    fn pick_data_seq(&self) -> Option<u16> {
        let picked = self.state.lm.tx.pick();
        if picked.is_some() {
            picked
        } else if self.state.lm.tx.is_full() {
            self.state.lm.tx.pick_sent_for_blocked()
        } else {
            None
        }
    }

    /// Mark a picked Data transmission; drop it after retry exhaustion.
    fn mark_data_sent(&mut self, picked: Option<u16>) {
        if let Some(seq) = picked {
            let was_retransmit = self.state.lm.tx.entry(seq).sent;
            if self.state.lm.tx.mark_sent(seq) {
                self.state.lm.tx.drop(seq);
                self.delivery_failures = self.delivery_failures.wrapping_add(1);
                self.pending_drop = Some(seq);
            } else if was_retransmit {
                self.retransmits = self.retransmits.wrapping_add(1);
            }
        }
    }

    /// Apply the peer's ACK/NACK carried by a received packet.
    fn apply_ack_nack(&mut self, ack: u16, nack: &[u8], previous_run: bool) {
        self.state.lm.tx.on_ack(ack);
        let slots = if previous_run {
            self.state.lm.tx_prev_run_slots
        } else {
            self.state.lm.tx_run_slots
        };
        self.state.lm.tx.on_nack_slots(nack, &slots);
        if nack_nonzero(nack) {
            self.nacks_recv = self.nacks_recv.wrapping_add(1);
        }
    }

    /// Clear a pending Drop once the peer's cumulative ACK covers it.
    fn clear_pending_drop(&mut self) {
        if let Some(drop_seq) = self.pending_drop {
            if !seq_gt(drop_seq, self.state.lm.tx.tx_acked) {
                self.pending_drop = None;
            }
        }
    }

    /// The local TX:RX ratio. The central's local ratio is
    /// `Config::tx_rx_ratio`; the peripheral's local ratio is the reverse,
    /// so both roles run exactly the same `tx then rx` data-plane rule and
    /// the ratio is no longer a mirror special-case.
    fn loss_threshold(&self) -> u8 {
        if self.cadence_active_sync_slot
            || (self.cadence_runtime.sync_slot
                && matches!(
                    self.cadence_runtime.stage,
                    CadenceRunStage::Applying
                        | CadenceRunStage::Probation
                        | CadenceRunStage::Stable
                ))
        {
            SYNC_LINK_LOSS_THRESHOLD
        } else {
            LINK_LOSS_THRESHOLD
        }
    }

    fn local_ratio(&self) -> (u8, u8) {
        if self.state.central {
            self.state.tx_rx_ratio
        } else {
            self.state.reverse_tx_rx_ratio
        }
    }

    fn phase_for_target(&self, target: u32, period: u32) -> u32 {
        if self.cadence_runtime.stage == CadenceRunStage::Applying
            && (target.wrapping_sub(self.cadence_runtime.local_start) as i32) >= 0
        {
            return descriptor_phase(
                target,
                self.cadence_runtime.local_start,
                self.cadence_runtime.apply_epoch,
                period,
            );
        }
        let in_window = matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::Armed | CadenceRunStage::Probing
        ) && (target.wrapping_sub(self.cadence_runtime.local_start) as i32) >= 0
            && (target.wrapping_sub(self.cadence_runtime.local_end) as i32) < 0;
        if in_window {
            descriptor_phase(
                target,
                self.cadence_runtime.local_start,
                self.cadence_runtime.central_start,
                period,
            )
        } else {
            self.state.next_phase(target.wrapping_sub(1), period)
        }
    }

    /// Advance epoch/bookkeeping shared by both roles.
    fn advance_epoch(&mut self, hw_slot: u32) {
        if hw_slot != 0 {
            self.state.epoch = hw_slot.wrapping_add(1);
        } else {
            self.state.epoch = self.state.epoch.wrapping_add(1);
        }
    }

    /// Run one slot (the symmetric data plane; role only affects the
    /// acquisition packet and the beacon/SlotRequest sync branches).
    async fn frame(
        &mut self,
        tx_payload: Option<&[u8]>,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        self.offer_taken = false;
        let ch = self.state.scheduler.current();
        if self.last_channel != Some(ch) {
            self.phy.set_channel(ch).await;
            self.last_channel = Some(ch);
        }

        // Negotiation control and Probe samples temporarily replace normal
        // Data/Ack slots. Freeze ARQ retry age so a bounded calibration cannot
        // exhaust application retries merely because its slots were reserved.
        if matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::Idle | CadenceRunStage::Probation | CadenceRunStage::Stable
        ) {
            self.state.lm.tx.tick();
        }
        if !self.state.lm.tx.is_full() {
            self.tx_space.signal(());
        }
        if self.phy.op_pipelined() {
            return self.frame_pipelined(tx_payload, rx_buf).await;
        }

        let (_c_tx, _c_rx) = self.state.tx_rx_ratio;
        let period = self.physical_period();
        let hw_slot = self.phy.slot_count();
        let phase = self.state.next_phase(hw_slot, period);
        if hw_slot == 0 {
            self.state.slot_step = self.state.slot_step.wrapping_add(1);
        }
        let (local_tx, local_rx) = self.local_ratio();
        let local_phase = self.to_local_phase(phase).unwrap_or(0);
        let central = self.state.central;
        let central_is_tx = self.central_is_tx_phase(phase);
        let acquiring = !central && hw_slot != 0 && (!self.cadence_ok || !self.state.lm.rx.have);
        let listen =
            central_is_tx || (acquiring && !self.state.beacon_anchor_ready && phase % 2 == 0);
        let is_tx = if central {
            local_phase < local_tx as u32
        } else if acquiring {
            !listen
        } else {
            local_phase < local_tx as u32
        };
        let active_end = local_tx as u32 + local_rx as u32;

        if is_tx {
            let mut tx_payload = tx_payload;
            let mut offer_rejected = false;

            // Role-specific acquisition gating. Once both directions have
            // traffic, the TX branch below is identical except for the
            // central burst path.
            let min_slot_us = self.phy.min_slot_period_us();
            let slotrequest = !central
                && min_slot_us > 0
                && (!self.cadence_ok
                    || !self.state.lm.rx.have
                    || self.initial_sync_ready_epoch != 0
                    || self.initial_sync_armed_epoch != 0);
            let cadence_pending = central && !self.cadence_negotiated && min_slot_us > 0;
            let profile_countdown = self.cadence_runtime.stage != CadenceRunStage::Commit
                && self.cadence_id != 0
                && self.cadence_apply_epoch != 0
                && (self.state.epoch.wrapping_sub(self.cadence_apply_epoch) as i32) < 0;
            let sync_slot = self.cadence_active_sync_slot
                || (self.cadence_runtime.sync_slot && !self.cadence_runtime.releasing);
            let forced_beacon = central
                && ((sync_slot && phase == 0)
                    || self.state.epoch % 64 == 0
                    || cadence_pending
                    || profile_countdown);

            // Start of a new local TX run: clear the slot-position table.
            if local_phase == 0 {
                self.state.lm.begin_tx_run();
            }

            if slotrequest {
                // Peripheral acquisition: this slot carries our minimum
                // cadence instead of data, with the TX delay swept.
                self.phy.set_tx_delay_sweep(true);
                let outbound = self.acquisition_packet(min_slot_us);
                let n = self.encode_packet(&outbound)?;
                // Acquisition SlotRequests stay on the plain path: they are
                // isolated TX slots and must not inherit a half-open burst
                // state from a previous acquisition attempt.
                self.in_burst = false;
                self.phy.transmit(&self.tx_buf[..n]).await?;
                self.advance_epoch(hw_slot);
                return Ok(None);
            }
            if central {
                if forced_beacon {
                    let outbound = self.beacon_packet(phase, period as u16, self.state.epoch);
                    let n = self.encode_packet(&outbound)?;
                    self.transmit_outbound(n).await?;
                } else {
                    offer_rejected = self.enqueue_offer(tx_payload)?;
                    let picked = self.pick_data_seq();
                    let outbound = if let Some(drop_seq) = self.pending_drop {
                        self.drop_packet(local_rx as usize)
                    } else if let Some(seq) = picked {
                        self.data_packet(seq, local_phase as u8, local_rx as usize)
                    } else if self.state.lm.rx.have {
                        self.ack_packet(local_rx as usize)
                    } else {
                        self.beacon_packet(phase, period as u16, self.state.epoch)
                    };
                    let outbound_is_data = matches!(outbound, Packet::Data { .. });
                    let n = self.encode_packet(&outbound)?;
                    self.transmit_outbound(n).await?;
                    if outbound_is_data {
                        self.tx_data = self.tx_data.wrapping_add(1);
                        self.mark_data_sent(picked);
                    }
                }
            } else {
                offer_rejected = self.enqueue_offer(tx_payload)?;
                let picked = self.pick_data_seq();
                let outbound = if let Some(_) = self.pending_drop {
                    Some(self.drop_packet(local_rx as usize))
                } else if let Some(seq) = picked {
                    Some(self.data_packet(seq, local_phase as u8, local_rx as usize))
                } else if self.state.lm.rx.have {
                    Some(self.ack_packet(local_rx as usize))
                } else {
                    None
                };
                if let Some(outbound) = outbound {
                    let outbound_is_data = matches!(outbound, Packet::Data { .. });
                    let n = self.encode_packet(&outbound)?;
                    self.transmit_outbound(n).await?;
                    if self.state.lm.nack_nonzero() {
                        self.nack_sent = self.nack_sent.wrapping_add(1);
                    }
                    if outbound_is_data {
                        self.tx_data = self.tx_data.wrapping_add(1);
                        self.mark_data_sent(picked);
                    }
                } else {
                    self.phy.wait_slot().await;
                }
            }

            self.advance_epoch(hw_slot);
            if offer_rejected {
                Err(Error::WindowFull)
            } else {
                Ok(None)
            }
        } else if local_phase >= active_end {
            // ---- idle slot (the shared no-radio phase) ----
            self.in_burst = false;
            self.phy.wait_slot().await;
            self.advance_epoch(hw_slot);
            Ok(None)
        } else {
            // ---- RX slot (the shared listen path) ----
            self.in_burst = false;
            if local_phase == local_tx as u32 {
                self.state.lm.rx_run_mask = [0; NACK_BYTES];
            }
            let reply_len = match self
                .phy
                .receive(
                    &mut self.rx_pkt_buf,
                    Duration::from_micros(PERIPHERAL_LISTEN_TIMEOUT_US),
                )
                .await?
            {
                Some(len) => len,
                None => {
                    self.advance_epoch(hw_slot);
                    return self.handle_rx_miss(local_phase, rx_buf);
                }
            };

            let mut encoded = [0u8; MAX_PAYLOAD + 32];
            encoded[..reply_len].copy_from_slice(&self.rx_pkt_buf[..reply_len]);
            let reply = self.decode_packet(&encoded[..reply_len])?;
            let out = self
                .handle_rx_packet(
                    reply,
                    local_phase,
                    period as u32,
                    hw_slot.wrapping_add(1),
                    false,
                    rx_buf,
                )
                .await?;
            self.advance_epoch(hw_slot);
            Ok(out)
        }
    }

    /// RX-run bookkeeping for a listen slot that caught nothing (the miss
    /// path shared by both frame flavors). `local_phase` is the missed
    /// slot's local phase.
    fn handle_rx_miss(
        &mut self,
        local_phase: u32,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        self.missed_frames = self.missed_frames.saturating_add(1);
        if matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::Idle | CadenceRunStage::Stable
        ) {
            let loss_threshold = self.loss_threshold();
            self.state
                .on_miss(&mut self.consecutive_misses, loss_threshold);
        }
        // Negotiation deliberately substitutes control/Sample traffic for
        // Data. Do not let candidate losses drive the central's adaptive hop
        // while beacons are suppressed, or the peripheral cannot follow the
        // channel change before final confirmation.
        let (local_tx, local_rx) = self.local_ratio();
        let rx_run_end = local_tx as u32 + local_rx as u32 - 1;
        if local_phase == rx_run_end {
            self.state.lm.nack_for_peer =
                nack_from_mask(local_rx as usize, &self.state.lm.rx_run_mask);
            self.state.lm.rx_run_mask = [0; NACK_BYTES];
        }
        deliver_rx(&mut self.state.lm.rx, rx_buf)
    }

    /// The follower's current mirror offset (diagnostic).
    pub fn slot_offset(&self) -> u32 {
        self.state.slot_offset
    }

    /// Data packets decoded from the peer (diagnostic; cumulative).
    pub fn rx_data(&self) -> u32 {
        self.rx_data
    }

    /// Data packets published for TX (diagnostic; cumulative).
    pub fn tx_data(&self) -> u32 {
        self.tx_data
    }

    /// The current hardware slot's phase, offset applied (diagnostic).
    pub fn hw_phase(&self) -> u32 {
        self.state.next_phase(
            self.phy.slot_count().wrapping_sub(1),
            self.physical_period(),
        )
    }

    fn handle_cadence_control(
        &mut self,
        generation: u8,
        stage: CadenceStage,
        epoch: u32,
        forward_payload: u8,
        reverse_payload: u8,
        short_us: u16,
        long_us: u16,
        min_slot_us: u16,
        step_us: u8,
        safety_steps: u8,
        start_epoch: u32,
        end_epoch: u32,
        probe_slots: u16,
        flags: u8,
        catch_slot: u32,
        period: u32,
    ) {
        let central = self.state.central;
        if generation == 0 {
            return;
        }
        match (central, stage) {
            (true, CadenceStage::Release)
                if cadence_generation_newer(generation, self.cadence_runtime.generation)
                    && matches!(
                        self.cadence_runtime.stage,
                        CadenceRunStage::Stable
                            | CadenceRunStage::Probation
                            | CadenceRunStage::Failed
                    ) =>
            {
                let active = self.cadence_runtime.stable;
                self.cadence_runtime = CadenceRuntime::new(active);
                self.cadence_runtime.generation = generation;
                self.cadence_runtime.releasing = true;
                self.cadence_runtime.contract =
                    TrafficContract::new(MAX_PAYLOAD as u16, MAX_PAYLOAD as u16);
                self.cadence_runtime.candidate = self.cadence_safe_profile;
                self.cadence_id = generation;
                self.cadence_ack = 0;
                self.cadence_runtime.stage = CadenceRunStage::Offer;
            }
            (false, CadenceStage::Release)
                if matches!(
                    self.cadence_runtime.stage,
                    CadenceRunStage::Stable
                        | CadenceRunStage::Failed
                        | CadenceRunStage::Request
                        | CadenceRunStage::Accept
                        | CadenceRunStage::Armed
                        | CadenceRunStage::Probing
                        | CadenceRunStage::Report
                ) =>
            {
                if matches!(
                    self.cadence_runtime.stage,
                    CadenceRunStage::Stable | CadenceRunStage::Probation | CadenceRunStage::Failed
                ) && !cadence_generation_newer(generation, self.cadence_runtime.generation)
                {
                    return;
                }
                let safe = self.cadence_safe_profile;
                if safe.short_slot_us < self.phy.min_short_slot_period_us()
                    || safe.long_slot_us < self.phy.min_long_slot_period_us()
                    || safe.long_slot_us < safe.short_slot_us
                {
                    self.cadence_runtime.generation = generation;
                    self.cadence_runtime.error = Some(CadenceError::PeerRejected);
                    self.cadence_runtime.stage = CadenceRunStage::Failed;
                    return;
                }
                let active = self.cadence_runtime.stable;
                self.cadence_runtime = CadenceRuntime::new(active);
                self.cadence_runtime.generation = generation;
                self.cadence_runtime.releasing = true;
                self.cadence_runtime.contract =
                    TrafficContract::new(MAX_PAYLOAD as u16, MAX_PAYLOAD as u16);
                self.cadence_runtime.candidate = safe;
                self.cadence_id = generation;
                self.cadence_ack = 0;
                self.cadence_runtime.stage = CadenceRunStage::Accept;
            }
            (true, CadenceStage::Request)
                if matches!(
                    self.cadence_runtime.stage,
                    CadenceRunStage::Idle | CadenceRunStage::Stable | CadenceRunStage::Failed
                ) && self.state.lm.tx.inflight == 0
                    && self.pending_drop.is_none()
                    && self.pending_tx_len == 0 =>
            {
                let contract = TrafficContract::new(forward_payload as u16, reverse_payload as u16);
                let policy = CadenceProbePolicy::new(
                    min_slot_us,
                    step_us as u16,
                    probe_slots,
                    safety_steps as u16,
                );
                let stable = self.cadence_runtime.stable;
                let fixed_wire = flags & CADENCE_FLAG_FIXED_WIRE != 0;
                let peer_sync_slot = flags & CADENCE_FLAG_SYNC_SLOT != 0;
                if stable.sync_slot && !peer_sync_slot {
                    self.cadence_runtime.generation = generation;
                    self.cadence_runtime.error = Some(CadenceError::PeerRejected);
                    self.cadence_runtime.stage = CadenceRunStage::Failed;
                    return;
                }
                let sync_slot = peer_sync_slot && stable.sync_slot;
                match self.cadence_local_probe_floors(contract, policy, fixed_wire, sync_slot) {
                    Ok((short_floor, long_floor))
                        if short_floor <= stable.short_slot_us
                            && long_floor <= stable.long_slot_us =>
                    {
                        self.cadence_runtime = CadenceRuntime::new(stable);
                        self.cadence_runtime.generation = generation;
                        self.cadence_id = generation;
                        self.cadence_ack = 0;
                        self.cadence_runtime.contract = contract;
                        self.cadence_runtime.fixed_wire = fixed_wire;
                        self.cadence_runtime.sync_slot = sync_slot;
                        self.cadence_runtime.policy = policy;
                        self.cadence_runtime.candidate = stable;
                        self.cadence_runtime.probe_floor_short_us = short_floor;
                        self.cadence_runtime.probe_floor_long_us = long_floor;
                        self.cadence_runtime.local_probe_floor_short_us = short_floor;
                        self.cadence_runtime.local_probe_floor_long_us = long_floor;
                        self.cadence_runtime.stage = CadenceRunStage::Offer;
                    }
                    Ok(_) | Err(_) => {
                        self.cadence_runtime.error = Some(CadenceError::InvalidPolicy);
                        self.cadence_runtime.stage = CadenceRunStage::Failed;
                    }
                }
            }
            (false, CadenceStage::Offer)
                if cadence_offer_generation_allowed(
                    self.cadence_runtime.stage,
                    self.cadence_runtime.generation,
                    generation,
                ) && self.state.lm.tx.inflight == 0
                    && self.pending_drop.is_none()
                    && self.pending_tx_len == 0 =>
            {
                let contract = TrafficContract::new(forward_payload as u16, reverse_payload as u16);
                let policy = CadenceProbePolicy::new(
                    min_slot_us,
                    step_us as u16,
                    probe_slots,
                    safety_steps as u16,
                );
                let fixed_wire = flags & CADENCE_FLAG_FIXED_WIRE != 0;
                let peer_sync_slot = flags & CADENCE_FLAG_SYNC_SLOT != 0;
                let sync_slot = peer_sync_slot && self.cadence_runtime.stable.sync_slot;
                let floors =
                    self.cadence_local_probe_floors(contract, policy, fixed_wire, sync_slot);
                if (self.cadence_runtime.stable.sync_slot && !peer_sync_slot)
                    || forward_payload as usize > MAX_PAYLOAD
                    || reverse_payload as usize > MAX_PAYLOAD
                    || long_us < short_us
                    || step_us == 0
                    || probe_slots == 0
                    || !matches!(floors, Ok((forward, reverse))
                        if forward <= self.cadence_runtime.stable.short_slot_us
                            && reverse <= self.cadence_runtime.stable.long_slot_us)
                {
                    self.cadence_runtime.generation = generation;
                    self.cadence_runtime.error = Some(CadenceError::PeerRejected);
                    self.cadence_runtime.stage = CadenceRunStage::Failed;
                    return;
                }
                let (short_floor, long_floor) = floors.unwrap_or((short_us, long_us));
                self.cadence_runtime.generation = generation;
                self.cadence_id = generation;
                self.cadence_ack = 0;
                self.cadence_runtime.contract = contract;
                self.cadence_runtime.fixed_wire = fixed_wire;
                self.cadence_runtime.sync_slot = sync_slot;
                self.cadence_runtime.policy = policy;
                self.cadence_runtime.probe_floor_short_us = short_floor;
                self.cadence_runtime.probe_floor_long_us = long_floor;
                self.cadence_runtime.local_probe_floor_short_us = short_floor;
                self.cadence_runtime.local_probe_floor_long_us = long_floor;
                // Compact Accept returns local feasibility floors in its two
                // otherwise-unused epoch fields. They are overwritten by the
                // first authoritative Probe descriptor.
                self.cadence_runtime.central_start = short_floor as u32;
                self.cadence_runtime.central_end = long_floor as u32;
                self.cadence_runtime.candidate = CadenceProfile::new(
                    short_us,
                    long_us,
                    self.cadence_runtime.stable.forward_slots,
                    self.cadence_runtime.stable.reverse_slots,
                    self.cadence_runtime.stable.idle_slots,
                )
                .with_sync_slot(self.cadence_runtime.sync_slot);
                self.cadence_runtime.stage = CadenceRunStage::Accept;
            }
            (true, CadenceStage::Accept)
                if generation == self.cadence_runtime.generation
                    && self.cadence_runtime.stage == CadenceRunStage::Offer =>
            {
                if flags & CADENCE_FLAG_REJECT != 0 {
                    self.cadence_runtime.error = Some(CadenceError::PeerRejected);
                    self.cadence_runtime.stage = CadenceRunStage::Failed;
                } else if self.cadence_runtime.releasing {
                    self.start_final_commit(catch_slot, period, self.cadence_safe_profile);
                } else {
                    if self.cadence_runtime.sync_slot && flags & CADENCE_FLAG_SYNC_SLOT == 0 {
                        self.cadence_runtime.error = Some(CadenceError::PeerRejected);
                        self.cadence_runtime.stage = CadenceRunStage::Failed;
                        return;
                    }
                    self.cadence_runtime.fixed_wire &= flags & CADENCE_FLAG_FIXED_WIRE != 0;
                    let local_floors = self.cadence_local_probe_floors(
                        self.cadence_runtime.contract,
                        self.cadence_runtime.policy,
                        self.cadence_runtime.fixed_wire,
                        self.cadence_runtime.sync_slot,
                    );
                    let (local_short, local_long) = match local_floors {
                        Ok(floors) => floors,
                        Err(_) => {
                            self.start_failed_probe_release();
                            return;
                        }
                    };
                    let peer_short = start_epoch.min(u16::MAX as u32) as u16;
                    let peer_long = end_epoch.min(u16::MAX as u32) as u16;
                    self.cadence_runtime.local_probe_floor_short_us = local_short;
                    self.cadence_runtime.local_probe_floor_long_us = local_long;
                    self.cadence_runtime.peer_probe_floor_short_us = peer_short;
                    self.cadence_runtime.peer_probe_floor_long_us = peer_long;
                    let short_floor = local_short.max(peer_short);
                    let long_floor = local_long.max(peer_long);
                    let wire_lengths = self.data_wire_lengths(
                        self.cadence_runtime.contract,
                        self.cadence_runtime.fixed_wire,
                    );
                    match wire_lengths.and_then(|(forward_wire, reverse_wire)| {
                        CadenceSearch::new_with_wire_lengths_and_floors(
                            self.cadence_runtime
                                .stable
                                .with_sync_slot(self.cadence_runtime.sync_slot),
                            self.cadence_runtime.contract,
                            self.cadence_runtime.policy,
                            forward_wire,
                            reverse_wire,
                            short_floor,
                            long_floor,
                        )
                    }) {
                        Ok(search) => {
                            self.cadence_runtime.probe_floor_short_us = short_floor;
                            self.cadence_runtime.probe_floor_long_us = long_floor;
                            self.cadence_runtime.candidate =
                                search.next_probe().unwrap_or(self.cadence_runtime.stable);
                            let has_probe = search.next_probe().is_some();
                            self.cadence_runtime.search = Some(search);
                            if has_probe {
                                self.start_probe_plan(catch_slot, period);
                            } else {
                                self.start_final_commit(
                                    catch_slot,
                                    period,
                                    self.cadence_runtime.stable,
                                );
                            }
                        }
                        Err(_) => {
                            self.start_failed_probe_release();
                        }
                    }
                }
            }
            (false, CadenceStage::Probe)
                if generation == self.cadence_runtime.generation
                    && matches!(
                        self.cadence_runtime.stage,
                        CadenceRunStage::Accept | CadenceRunStage::Armed | CadenceRunStage::Report
                    ) =>
            {
                let duplicate_descriptor = matches!(
                    self.cadence_runtime.stage,
                    CadenceRunStage::Armed | CadenceRunStage::Report
                ) && self.cadence_runtime.central_start == start_epoch
                    && self.cadence_runtime.central_end == end_epoch
                    && self.cadence_runtime.probe_superframes_current == probe_slots
                    && self.cadence_runtime.candidate.short_slot_us == short_us
                    && self.cadence_runtime.candidate.long_slot_us == long_us
                    && self.cadence_runtime.confirming == (flags & CADENCE_FLAG_CONFIRM != 0);
                if duplicate_descriptor {
                    // Keep repeating Armed/Report, but never rewrite an
                    // immutable descriptor or replace completed metrics.
                    return;
                }
                if self.cadence_runtime.stage == CadenceRunStage::Armed {
                    // A changed replan may arrive while this follower's slower
                    // counter is still inside the old bounded window. Ignore it
                    // until callback END moves us to Report; central repeats the
                    // new immutable descriptor meanwhile.
                    return;
                }
                let start_delta = start_epoch.wrapping_sub(epoch) as i32;
                let end_delta = end_epoch.wrapping_sub(epoch) as i32;
                if end_delta > start_delta && !probe_has_sufficient_arm_lead(start_delta) {
                    self.cadence_runtime.central_start = start_epoch;
                    self.cadence_runtime.central_end = end_epoch;
                    self.cadence_runtime.probe_superframes_current = probe_slots;
                    self.cadence_runtime.confirming = flags & CADENCE_FLAG_CONFIRM != 0;
                    self.cadence_runtime.candidate = CadenceProfile::new(
                        short_us,
                        long_us,
                        self.cadence_runtime.stable.forward_slots,
                        self.cadence_runtime.stable.reverse_slots,
                        self.cadence_runtime.stable.idle_slots,
                    )
                    .with_sync_slot(self.cadence_runtime.sync_slot);
                    self.cadence_runtime.local_metrics = Some(ProbeMetrics::new(0, 1, 1));
                    self.cadence_runtime.stage = CadenceRunStage::Report;
                    return;
                }
                if probe_has_sufficient_arm_lead(start_delta) && end_delta > start_delta {
                    let local_start = catch_slot.wrapping_add(start_delta as u32);
                    let local_end = catch_slot.wrapping_add(end_delta as u32);
                    self.cadence_runtime.central_start = start_epoch;
                    self.cadence_runtime.central_end = end_epoch;
                    self.cadence_runtime.probe_superframes_current = probe_slots;
                    self.cadence_runtime.confirming = flags & CADENCE_FLAG_CONFIRM != 0;
                    self.cadence_runtime.local_start = local_start;
                    self.cadence_runtime.local_end = local_end;
                    self.cadence_runtime.probe_started = false;
                    self.cadence_runtime.stats_start = self.phy.slot_probe_stats();
                    self.cadence_runtime.delivery_failures_start = self.delivery_failures;
                    self.cadence_runtime.local_metrics = None;
                    self.cadence_runtime.candidate = CadenceProfile::new(
                        short_us,
                        long_us,
                        self.cadence_runtime.stable.forward_slots,
                        self.cadence_runtime.stable.reverse_slots,
                        self.cadence_runtime.stable.idle_slots,
                    )
                    .with_sync_slot(self.cadence_runtime.sync_slot);
                    let p = self.cadence_runtime.candidate;
                    if p.short_slot_us < self.cadence_runtime.probe_floor_short_us
                        || p.long_slot_us < self.cadence_runtime.probe_floor_long_us
                        || p.short_slot_us > p.long_slot_us
                    {
                        self.cadence_runtime.local_metrics = Some(ProbeMetrics::new(
                            self.cadence_runtime.policy.probe_superframes,
                            1,
                            1,
                        ));
                        self.cadence_runtime.stage = CadenceRunStage::Report;
                        return;
                    }
                    if self.phy.schedule_slot_probe(
                        p.short_slot_us,
                        p.long_slot_us,
                        p.period_slots() as u16,
                        profile_short_phases(p.forward_slots, self.cadence_runtime.sync_slot),
                        profile_central_start(
                            start_epoch,
                            p.period_slots(),
                            self.cadence_runtime.sync_slot,
                        ),
                        local_start,
                        local_end,
                    ) {
                        self.cadence_runtime.stage = CadenceRunStage::Armed;
                    } else {
                        self.cadence_runtime.local_metrics = Some(ProbeMetrics::new(0, 1, 1));
                        self.cadence_runtime.stage = CadenceRunStage::Report;
                    }
                }
            }
            (true, CadenceStage::Armed)
                if generation == self.cadence_runtime.generation
                    && self.cadence_runtime.stage == CadenceRunStage::ProbePlan
                    && start_epoch == self.cadence_runtime.central_start
                    && end_epoch == self.cadence_runtime.central_end =>
            {
                let lead = self.cadence_runtime.local_start.wrapping_sub(catch_slot) as i32;
                if probe_has_sufficient_arm_lead(lead) && !self.arm_central_probe() {
                    self.start_failed_probe_release();
                }
                // A late Armed cannot safely fill the first depth-two op. Keep
                // ProbePlan; cadence_tick replans after the peer window ends.
            }
            (true, CadenceStage::Report)
                if generation == self.cadence_runtime.generation
                    && matches!(
                        self.cadence_runtime.stage,
                        CadenceRunStage::ProbePlan | CadenceRunStage::Probing
                    )
                    && start_epoch == self.cadence_runtime.central_start
                    && end_epoch == self.cadence_runtime.central_end
                    && short_us == self.cadence_runtime.candidate.short_slot_us
                    && long_us == self.cadence_runtime.candidate.long_slot_us =>
            {
                let aborted = flags & CADENCE_FLAG_PROBE_ABORT != 0;
                let failures = u16::from(flags & CADENCE_FLAG_STABLE == 0 && !aborted);
                self.cadence_runtime.peer_metrics = Some(ProbeMetrics::new(
                    if aborted {
                        0
                    } else {
                        self.cadence_runtime.probe_superframes_current
                    },
                    failures,
                    failures,
                ));
                self.finish_probe(catch_slot, period);
            }
            (false, CadenceStage::Commit)
                if generation == self.cadence_runtime.generation
                    && matches!(
                        self.cadence_runtime.stage,
                        CadenceRunStage::Accept
                            | CadenceRunStage::Report
                            | CadenceRunStage::Applying
                    ) =>
            {
                if self.cadence_runtime.stage == CadenceRunStage::Applying {
                    if start_epoch == self.cadence_runtime.apply_epoch {
                        return;
                    }
                    if (catch_slot.wrapping_sub(self.cadence_runtime.local_start) as i32) < 0 {
                        return;
                    }
                }
                self.cadence_runtime.releasing = flags & CADENCE_FLAG_RELEASE != 0;
                self.cadence_runtime.contract =
                    TrafficContract::new(forward_payload as u16, reverse_payload as u16);
                let delta = start_epoch.wrapping_sub(epoch) as i32;
                if delta > 0 {
                    let local_apply = catch_slot.wrapping_add(delta as u32);
                    let profile = CadenceProfile::new(
                        short_us,
                        long_us,
                        self.cadence_runtime.stable.forward_slots,
                        self.cadence_runtime.stable.reverse_slots,
                        self.cadence_runtime.stable.idle_slots,
                    )
                    .with_sync_slot(self.cadence_runtime.sync_slot);
                    let profile_changed = self.cadence_runtime.commit_changes_profile
                        || profile != self.cadence_runtime.stable;
                    let commit_slot_offset = start_epoch.wrapping_sub(local_apply);
                    self.cadence_runtime.pending_slot_offset = commit_slot_offset;
                    self.cadence_runtime.commit_changes_profile = profile_changed;
                    if profile_changed
                        && !self.phy.schedule_probed_slot_profile(
                            profile.short_slot_us,
                            profile.long_slot_us,
                            profile.period_slots() as u16,
                            profile_short_phases(
                                profile.forward_slots,
                                self.cadence_runtime.sync_slot,
                            ),
                            profile.sync_slot,
                            profile_central_start(
                                start_epoch,
                                period,
                                self.cadence_runtime.sync_slot,
                            ),
                            local_apply,
                        )
                    {
                        self.cadence_runtime.error = Some(CadenceError::InvalidProfile);
                        self.cadence_runtime.stage = CadenceRunStage::Failed;
                        return;
                    }
                    self.cadence_runtime.stable = profile;
                    self.cadence_runtime.apply_epoch = start_epoch;
                    self.cadence_runtime.local_start = local_apply;
                    self.cadence_apply_epoch = start_epoch;
                    self.cadence_ack = generation | 0x80;
                    self.cadence_runtime.stage = CadenceRunStage::Applying;
                }
            }
            (true, CadenceStage::Applied)
                if generation == self.cadence_runtime.generation
                    && self.cadence_runtime.stage == CadenceRunStage::Commit
                    && start_epoch == self.cadence_runtime.apply_epoch =>
            {
                if self.cadence_runtime.commit_changes_profile
                    && (catch_slot.wrapping_sub(self.cadence_runtime.apply_epoch) as i32) >= 0
                {
                    // The peer armed the old epoch but its confirmation was
                    // delayed past it. Re-issue the same immutable profile at
                    // a fresh future phase-0 boundary; neither side guesses an
                    // immediate asymmetric switch.
                    self.start_final_commit(catch_slot, period, self.cadence_runtime.stable);
                    self.cadence_runtime.commit_changes_profile = true;
                    return;
                }
                if self.cadence_runtime.commit_changes_profile
                    && !self.phy.schedule_probed_slot_profile(
                        self.cadence_runtime.stable.short_slot_us,
                        self.cadence_runtime.stable.long_slot_us,
                        self.cadence_runtime.stable.period_slots() as u16,
                        profile_short_phases(
                            self.cadence_runtime.stable.forward_slots,
                            self.cadence_runtime.sync_slot,
                        ),
                        self.cadence_runtime.stable.sync_slot,
                        profile_central_start(
                            self.cadence_runtime.apply_epoch,
                            period,
                            self.cadence_runtime.sync_slot,
                        ),
                        self.cadence_runtime.apply_epoch,
                    )
                {
                    self.cadence_runtime.error = Some(CadenceError::InvalidProfile);
                    self.cadence_runtime.stage = CadenceRunStage::Failed;
                    return;
                }
                self.cadence_runtime.local_start = self.cadence_runtime.apply_epoch;
                self.cadence_runtime.stage = CadenceRunStage::Applying;
            }
            (_, CadenceStage::Cancel)
                if generation == self.cadence_runtime.generation
                    && self.cadence_runtime.stage != CadenceRunStage::Idle =>
            {
                self.cadence_runtime.error = Some(CadenceError::PeerRejected);
                self.cadence_runtime.stage = CadenceRunStage::Failed;
            }
            _ => {}
        }
    }

    /// Process a caught packet (the shared RX-catch path of both frame
    /// flavors). `local_phase` is the phase of the slot it was caught in.
    ///
    /// The run-end NACK finalize compares the LOCAL phase: the previous
    /// `phase == rx_run_end` form mixed the global phase with a local
    /// index and never fired on the peripheral (its RX branch sees global
    /// phases 0..c_tx while rx_run_end is the last local slot), so the
    /// peripheral never sent a real NACK and the forward direction fell
    /// back to tick-timeout retransmits.
    async fn handle_rx_packet(
        &mut self,
        reply: Packet,
        local_phase: u32,
        period: u32,
        // The hardware slot the packet was caught in (the op's target
        // slot) - the beacon re-anchor must use this, not the slot count
        // at processing time.
        catch_slot: u32,
        previous_tx_run: bool,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        let central = self.state.central;
        let (local_tx, local_rx) = self.local_ratio();
        let active_end = local_tx as u32 + local_rx as u32;
        let rx_run_len = local_rx as usize;
        let rx_run_end = active_end - 1;
        if (local_tx as u32..active_end).contains(&local_phase) {
            let slot_idx = (local_phase - local_tx as u32) as usize;
            nack_set(&mut self.state.lm.rx_run_mask, slot_idx);
        }

        // Sync-only packets (SlotRequest/Beacon) prove a window exists
        // but do NOT form a data link: enabling the adaptive hop on them
        // can move the master away before the first Data ever lands.
        let mut link_rx = false;
        let mut data_plane_rx = false;
        match reply {
            Packet::Data {
                seq,
                ack,
                nack,
                mut payload,
            } => {
                self.state
                    .decrypt_payload(&mut self.phy, &mut payload, seq, central)?;
                self.apply_ack_nack(ack, &nack, previous_tx_run);
                if !central
                    && self.state.status == LinkStatus::Disconnected
                    && !self.state.lm.rx.in_window(seq)
                {
                    self.state.lm.rx.resync(seq);
                    self.resyncs = self.resyncs.wrapping_add(1);
                }
                link_rx = true;
                data_plane_rx = true;
                self.state.lm.rx.receive(seq, &payload);
                self.rx_data = self.rx_data.wrapping_add(1);
            }
            Packet::Ack { ack, nack } => {
                link_rx = true;
                data_plane_rx = true;
                self.apply_ack_nack(ack, &nack, previous_tx_run);
            }
            Packet::Drop { seq, ack, nack } => {
                link_rx = true;
                data_plane_rx = true;
                self.state.lm.rx.skip_to(seq);
                self.apply_ack_nack(ack, &nack, previous_tx_run);
            }
            Packet::Cadence {
                ack,
                nack,
                generation,
                stage,
                epoch,
                forward_payload,
                reverse_payload,
                short_us,
                long_us,
                min_slot_us,
                step_us,
                safety_steps,
                start_epoch,
                end_epoch,
                probe_slots,
                flags,
            } => {
                link_rx = true;
                self.apply_ack_nack(ack, &nack, previous_tx_run);
                self.handle_cadence_control(
                    generation,
                    stage,
                    epoch,
                    forward_payload,
                    reverse_payload,
                    short_us,
                    long_us,
                    min_slot_us,
                    step_us,
                    safety_steps,
                    start_epoch,
                    end_epoch,
                    probe_slots,
                    flags,
                    catch_slot,
                    period,
                );
            }
            Packet::CadenceAck {
                generation,
                stage,
                start_epoch,
                end_epoch,
                flags,
            } => {
                link_rx = true;
                let sender_epoch = if flags & CADENCE_FLAG_RELEASE != 0
                    && matches!(stage, CadenceStage::Release | CadenceStage::Commit)
                {
                    end_epoch
                } else {
                    catch_slot
                };
                self.handle_cadence_control(
                    generation,
                    stage,
                    sender_epoch,
                    self.cadence_runtime.contract.forward_payload_len as u8,
                    self.cadence_runtime.contract.reverse_payload_len as u8,
                    self.cadence_runtime.candidate.short_slot_us,
                    self.cadence_runtime.candidate.long_slot_us,
                    self.cadence_runtime.policy.min_slot_us,
                    self.cadence_runtime.policy.step_us.min(255) as u8,
                    self.cadence_runtime.policy.safety_steps.min(255) as u8,
                    start_epoch,
                    end_epoch,
                    self.cadence_runtime.policy.probe_superframes,
                    flags,
                    catch_slot,
                    period,
                );
            }
            Packet::CadenceSample {
                generation,
                padding: _,
            } => {
                if generation == self.cadence_runtime.generation {
                    link_rx = true;
                }
            }
            Packet::SlotRequest {
                min_slot_us,
                min_short_slot_us,
                cadence_ack,
                ack,
            } if central => {
                if cadence_ack == 0
                    && (self.cadence_negotiated
                        || self.initial_sync_commit
                        || self.cadence_active_contract.is_some()
                        || self.cadence_runtime.releasing)
                {
                    // The peer independently entered uniform acquisition
                    // fallback after severe loss. Join it before processing
                    // this SlotRequest so both counters regain one wall rate.
                    self.emergency_cadence_fallback();
                }
                // The acquiring peer's cumulative ACK advances the central's
                // TX window from liveness traffic itself.
                self.apply_ack_nack(ack, &[0; NACK_BYTES], previous_tx_run);
                self.clear_pending_drop();

                if self.cadence_id == 0 {
                    // Legacy uniform-cadence negotiation (bare PHY).
                    if !self.cadence_negotiated {
                        self.cadence_negotiated = true;
                        let negotiated = self.phy.min_slot_period_us().max(min_slot_us).max(1);
                        if negotiated != self.phy.slot_period_us() {
                            self.phy.align_slot_period(negotiated);
                        }
                    }
                } else {
                    // Mixed MPSL capability negotiation: one received SR
                    // supplies the peer floors; the central then commits a
                    // far-future absolute boundary in repeated beacons.
                    self.cadence_short_us =
                        self.phy.min_short_slot_period_us().max(min_short_slot_us);
                    self.cadence_long_us = self.phy.min_long_slot_period_us().max(min_slot_us);
                    // First SR only creates a proposal. Neither side changes
                    // cadence until exact SyncReady/SyncArmed epochs close.
                    if self.initial_sync_proposal_epoch == 0 {
                        let lead = period.saturating_mul(16).max(period);
                        // Stay in the central hardware-slot coordinate. App
                        // processing can lag the caught SlotRequest by a
                        // complete slot; state.epoch would then arm a
                        // different phase boundary than the beacon epoch the
                        // peripheral translates.
                        let candidate = catch_slot.wrapping_add(lead);
                        let rem = candidate % period;
                        self.cadence_apply_epoch = if rem == 0 {
                            candidate
                        } else {
                            candidate.wrapping_add(period - rem)
                        };
                        self.initial_sync_proposal_epoch = self.cadence_apply_epoch;
                        self.initial_sync_commit = false;
                    }
                    let _ = cadence_ack; // legacy diagnostic only
                }
            }
            Packet::SyncReady {
                generation,
                proposal_epoch,
            } if central => {
                if generation == self.cadence_id
                    && proposal_epoch == self.initial_sync_proposal_epoch
                    && !self.initial_sync_commit
                    && !self.cadence_negotiated
                {
                    let lead = period.saturating_mul(16).max(period);
                    let candidate = catch_slot.wrapping_add(lead);
                    let rem = candidate % period;
                    self.cadence_apply_epoch = if rem == 0 {
                        candidate
                    } else {
                        candidate.wrapping_add(period - rem)
                    };
                    self.initial_sync_commit = true;
                }
            }
            Packet::SyncArmed {
                generation,
                apply_epoch,
            } if central => {
                if generation == self.cadence_id
                    && self.initial_sync_commit
                    && apply_epoch == self.cadence_apply_epoch
                    && !self.cadence_negotiated
                {
                    let lead = apply_epoch.wrapping_sub(catch_slot) as i32;
                    if lead >= PROBE_ARM_LEAD_SLOTS {
                        if self.phy.schedule_slot_profile(
                            self.cadence_short_us,
                            self.cadence_long_us,
                            period as u16,
                            profile_short_phases(
                                self.cadence_short_phases,
                                self.cadence_safe_profile.sync_slot,
                            ),
                            self.cadence_safe_profile.sync_slot,
                            profile_central_start(
                                apply_epoch,
                                period,
                                self.cadence_safe_profile.sync_slot,
                            ),
                            apply_epoch,
                        ) {
                            self.cadence_negotiated = true;
                            self.cadence_runtime.stable.short_slot_us = self.cadence_short_us;
                            self.cadence_runtime.stable.long_slot_us = self.cadence_long_us;
                            self.cadence_safe_profile = self.cadence_runtime.stable;
                            self.initial_sync_proposal_epoch = 0;
                        } else {
                            self.emergency_cadence_fallback();
                        }
                    } else {
                        self.emergency_cadence_fallback();
                    }
                }
            }
            Packet::SyncReady { .. } | Packet::SyncArmed { .. } => {}
            Packet::Beacon {
                epoch: beacon_epoch,
                channel_index,
                flags,
                slot_us,
                slot_phase,
                rx_en_offset,
                tx_en_offset,
                rx_ramp,
                tx_ramp,
                cadence_id,
                short_slot_us,
                long_slot_us,
                short_phases,
                cadence_apply_epoch,
            } if !central => {
                let initial_sync_in_flight = self.initial_sync_proposal_epoch != 0
                    || self.initial_sync_ready_epoch != 0
                    || self.initial_sync_armed_epoch != 0;
                if should_join_central_fallback(
                    self.cadence_active_contract.is_some()
                        || self.cadence_negotiated
                        || initial_sync_in_flight,
                    cadence_apply_epoch,
                ) {
                    // An authoritative central reboot/fallback beacon joins the
                    // peripheral to postcard acquisition even if ordinary
                    // packets still arrive and therefore miss counters stay 0.
                    self.emergency_cadence_fallback();
                }
                self.state.scheduler.sync(channel_index);
                let initial_commit = flags & BEACON_FLAG_INITIAL_COMMIT != 0;
                let rx_window_flags = flags & 0x3f;
                if rx_window_flags > 0 {
                    self.phy.set_peer_rx_window(rx_window_flags as u16 * 16);
                }
                if rx_en_offset > 0 {
                    self.phy.set_peer_rx_en_offset(rx_en_offset);
                }
                if tx_en_offset > 0 {
                    self.phy.set_peer_tx_en_offset(tx_en_offset);
                }
                if rx_ramp > 0 {
                    self.phy.set_peer_rx_ramp(rx_ramp);
                }
                if tx_ramp > 0 {
                    self.phy.set_peer_tx_ramp(tx_ramp);
                }
                // Uniform acquisition alignment is only legal outside an API
                // cadence transaction. Periodic beacons continue during
                // Offer/Probe/Commit; align_slot_period would otherwise disarm
                // the active/probe profile between descriptor and start.
                let api_cadence_in_flight = !matches!(
                    self.cadence_runtime.stage,
                    CadenceRunStage::Idle | CadenceRunStage::Stable | CadenceRunStage::Failed
                );
                if slot_us > 0 && self.cadence_ack & 0x80 == 0 && !api_cadence_in_flight {
                    self.phy.align_slot_period(slot_us);
                }
                let min = self.phy.min_slot_period_us();
                if cadence_id == 0 {
                    self.cadence_ok = min == 0 || slot_us >= min;
                }
                // Re-anchor the mirror phase only while still acquiring.
                // The anchor is exact when the beacon's CATCH slot is used:
                // processing can lag the catch by a whole slot (the 5 s
                // defmt report stalls the app up to ~1 ms), and the old
                // phy.slot_count() at processing time measured that lag as
                // a fake offset shift. Adopt only when two consecutive
                // beacons agree (voting) - a single late-processed beacon
                // then cannot freeze the mirror at a wrong offset.
                if !self.state.lm.rx.have {
                    if catch_slot != 0 {
                        let beacon_phase = slot_phase as u32 % period;
                        let candidate = (beacon_phase.wrapping_sub(catch_slot % period)) % period;
                        if self.state.beacon_anchor_pending == Some(candidate) {
                            self.state.slot_offset = candidate;
                            self.state.beacon_anchor_pending = None;
                            self.state.beacon_anchor_ready = true;
                        } else {
                            self.state.beacon_anchor_pending = Some(candidate);
                        }
                    } else {
                        self.state.slot_step = (slot_phase as u32 + 1) % period;
                        self.state.beacon_anchor_ready = true;
                    }
                }

                // Profile commit. Only after receiving the absolute armed
                // epoch and adopting an exact two-beacon slot mapping does
                // the peripheral schedule the switch and set the high ACK
                // bit (diagnostic/forward-compatible confirmation).
                let api_descriptor = self.cadence_runtime.generation != 0
                    && cadence_id == self.cadence_runtime.generation
                    && self.cadence_runtime.stage != CadenceRunStage::Idle;
                if api_descriptor
                    && self.cadence_runtime.stage == CadenceRunStage::Applying
                    && cadence_apply_epoch == self.cadence_runtime.apply_epoch
                    && (beacon_epoch.wrapping_sub(cadence_apply_epoch) as i32) >= 0
                    && (catch_slot.wrapping_sub(self.cadence_runtime.local_start) as i32) >= 0
                {
                    // A same-generation descriptor sent after the apply epoch
                    // proves the central received Applied and resumed its
                    // normal schedule. Periodic beacons make this idle-link
                    // completion repairable without re-arming the PHY.
                    self.complete_cadence_apply();
                }
                if !api_descriptor && self.cadence_id != 0 && cadence_id == self.cadence_id {
                    self.cadence_short_us = short_slot_us.max(self.phy.min_short_slot_period_us());
                    self.cadence_long_us = long_slot_us.max(self.phy.min_long_slot_period_us());
                    self.cadence_short_phases = short_phases.min(period as u16);
                    if !initial_commit
                        && cadence_apply_epoch != 0
                        && self.state.beacon_anchor_ready
                        && !self.cadence_negotiated
                    {
                        self.initial_sync_proposal_epoch = cadence_apply_epoch;
                        self.initial_sync_ready_epoch = cadence_apply_epoch;
                        self.initial_sync_armed_epoch = 0;
                        self.cadence_apply_epoch = cadence_apply_epoch;
                        self.cadence_ack = cadence_id;
                    } else if initial_commit
                        && cadence_apply_epoch != 0
                        && cadence_apply_epoch != self.initial_sync_proposal_epoch
                        && self.initial_sync_ready_epoch != 0
                        && catch_slot != 0
                        && self.state.beacon_anchor_ready
                        && !self.cadence_negotiated
                    {
                        let delta = cadence_apply_epoch.wrapping_sub(beacon_epoch) as i32;
                        if delta >= PROBE_ARM_LEAD_SLOTS {
                            let local_apply = catch_slot.wrapping_add(delta as u32);
                            if self.phy.schedule_slot_profile(
                                self.cadence_short_us,
                                self.cadence_long_us,
                                period as u16,
                                profile_short_phases(
                                    self.cadence_short_phases,
                                    self.cadence_safe_profile.sync_slot,
                                ),
                                self.cadence_safe_profile.sync_slot,
                                profile_central_start(
                                    cadence_apply_epoch,
                                    period,
                                    self.cadence_safe_profile.sync_slot,
                                ),
                                local_apply,
                            ) {
                                self.cadence_apply_epoch = cadence_apply_epoch;
                                self.initial_sync_ready_epoch = 0;
                                self.initial_sync_armed_epoch = cadence_apply_epoch;
                                self.cadence_ack = cadence_id | 0x80;
                                self.cadence_ok = true;
                                self.cadence_runtime.stable.short_slot_us = self.cadence_short_us;
                                self.cadence_runtime.stable.long_slot_us = self.cadence_long_us;
                                self.cadence_safe_profile = self.cadence_runtime.stable;
                            }
                        }
                    }
                }
            }
            _ => {}
        }

        if central
            && data_plane_rx
            && self.initial_sync_commit
            && (catch_slot.wrapping_sub(self.cadence_apply_epoch) as i32) >= 0
        {
            self.initial_sync_commit = false;
            self.initial_sync_armed_epoch = 0;
        }
        if !central
            && data_plane_rx
            && self.initial_sync_armed_epoch != 0
            && (catch_slot.wrapping_sub(self.cadence_apply_epoch) as i32) >= 0
        {
            self.initial_sync_armed_epoch = 0;
            self.initial_sync_proposal_epoch = 0;
            self.cadence_negotiated = true;
        }
        if !central
            && data_plane_rx
            && self.cadence_runtime.stage == CadenceRunStage::Applying
            && (catch_slot.wrapping_sub(self.cadence_runtime.local_start) as i32) >= 0
        {
            // The central can send normal Data/Ack only after it received our
            // Applied and crossed the same epoch. This is the peripheral's
            // final confirmation, so it may stop repeating Applied now.
            self.complete_cadence_apply();
        }

        self.clear_pending_drop();
        let data_required = matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::Probation | CadenceRunStage::Stable
        );
        if data_plane_rx || !data_required {
            self.missed_frames = 0;
        }
        if link_rx {
            self.state.on_rx(&mut self.consecutive_misses);
        } else if !data_required {
            self.consecutive_misses = 0;
        }
        if local_phase == rx_run_end {
            self.state.lm.nack_for_peer = nack_from_mask(rx_run_len, &self.state.lm.rx_run_mask);
            self.state.lm.rx_run_mask = [0; NACK_BYTES];
        }
        deliver_rx(&mut self.state.lm.rx, rx_buf)
    }

    /// The pipelined frame (phys with a hardware slot counter): publish the
    /// op for `hw_slot + 2` FIRST - the publish deadline is that slot's
    /// START, a ~2.5-slot budget instead of the ~200 us between the
    /// previous op's completion and the next START (the 8-30% op-late rate
    /// on the 5340/LM20) - then collect and process the op for
    /// `hw_slot + 1` (the frame's pacing). Costs: a TX op's NACK/ACK
    /// bitmap is one slot staler, and the echo path has one extra slot of
    /// latency.
    async fn frame_pipelined(
        &mut self,
        tx_payload: Option<&[u8]>,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        let (_c_tx, _c_rx) = self.state.tx_rx_ratio;
        let period = self.physical_period();
        let hw_slot = self.phy.slot_count();
        self.cadence_tick(hw_slot, period);
        let collect_slot = hw_slot.wrapping_add(1);
        let target = hw_slot.wrapping_add(2);
        // next_phase applies the follower's mirror offset (slot_offset);
        // target % period alone would ignore the re-anchor and scramble
        // the peripheral's TX/RX phase decisions.
        let phase = self.phase_for_target(target, period);
        let (local_tx, local_rx) = self.local_ratio();
        let local_phase = self.to_local_phase(phase);
        let is_sync = local_phase.is_none();
        let local_phase = local_phase.unwrap_or(u32::MAX);
        let central = self.state.central;
        let central_is_tx = self.central_is_tx_phase(phase);
        let acquiring = !central && (!self.cadence_ok || !self.state.lm.rx.have);
        let listen = is_sync
            || central_is_tx
            || (acquiring && !self.state.beacon_anchor_ready && phase % 2 == 0);
        let is_tx = if is_sync {
            central
        } else if central {
            local_phase < local_tx as u32
        } else if acquiring {
            !listen
        } else {
            local_phase < local_tx as u32
        };
        let active_end = local_tx as u32 + local_rx as u32;

        // ---- publish the op for `target` ----
        let mut offer_rejected = false;
        if is_tx {
            // The first TX op of a run may execute one slot late: it still
            // lands inside the peer's RX run. Any other late op would face
            // a peer that has stopped listening.
            let grace = if local_phase == 0 && local_tx > 1 {
                1
            } else {
                0
            };
            let min_slot_us = self.phy.min_slot_period_us();
            let slotrequest = !central
                && min_slot_us > 0
                && (!self.cadence_ok
                    || !self.state.lm.rx.have
                    || self.initial_sync_ready_epoch != 0
                    || self.initial_sync_armed_epoch != 0);
            let cadence_pending = central && !self.cadence_negotiated && min_slot_us > 0;
            let profile_countdown = self.cadence_runtime.stage != CadenceRunStage::Commit
                && self.cadence_id != 0
                && self.cadence_apply_epoch != 0
                && (target.wrapping_sub(self.cadence_apply_epoch) as i32) < 0;
            let sync_slot = self.cadence_active_sync_slot
                || (self.cadence_runtime.sync_slot && !self.cadence_runtime.releasing);
            let forced_beacon = central
                && ((sync_slot && phase == 0)
                    || target % 64 == 0
                    || cadence_pending
                    || profile_countdown);

            // Start of a new local TX run: clear the slot-position table.
            if local_phase == 0 {
                self.state.lm.begin_tx_run();
            }

            if slotrequest {
                // Peripheral acquisition: this slot carries our minimum
                // cadence instead of data, with the TX delay swept.
                self.phy.set_tx_delay_sweep(true);
                let outbound = self.acquisition_packet(min_slot_us);
                let n = self.encode_packet(&outbound)?;
                self.phy
                    .op_publish_tx(&self.tx_buf[..n], target, grace)
                    .await?;
            } else {
                self.phy.set_tx_delay_sweep(false);
                if central {
                    if forced_beacon {
                        let outbound = self.beacon_packet(phase, period as u16, target);
                        let n = self.encode_packet(&outbound)?;
                        self.phy
                            .op_publish_tx(&self.tx_buf[..n], target, grace)
                            .await?;
                    } else if let Some(outbound) = self.cadence_packet(target, local_rx as usize) {
                        let n = self.encode_packet(&outbound)?;
                        self.phy
                            .op_publish_tx(&self.tx_buf[..n], target, grace)
                            .await?;
                    } else {
                        offer_rejected = self.enqueue_offer(tx_payload)?;
                        let picked = self.pick_data_seq();
                        let outbound = if self.pending_drop.is_some() {
                            self.drop_packet(local_rx as usize)
                        } else if let Some(seq) = picked {
                            self.data_packet(seq, local_phase as u8, local_rx as usize)
                        } else if self.state.lm.rx.have {
                            self.ack_packet(local_rx as usize)
                        } else {
                            self.beacon_packet(phase, period as u16, target)
                        };
                        let outbound_is_data = matches!(outbound, Packet::Data { .. });
                        let n = self.encode_packet(&outbound)?;
                        self.phy
                            .op_publish_tx(&self.tx_buf[..n], target, grace)
                            .await?;
                        if outbound_is_data {
                            self.tx_data = self.tx_data.wrapping_add(1);
                            self.mark_data_sent(picked);
                        }
                    }
                } else {
                    let control = self.cadence_packet(target, local_rx as usize);
                    let picked = if control.is_none() {
                        offer_rejected = self.enqueue_offer(tx_payload)?;
                        self.pick_data_seq()
                    } else {
                        None
                    };
                    let outbound = if let Some(control) = control {
                        Some(control)
                    } else if self.pending_drop.is_some() {
                        Some(self.drop_packet(local_rx as usize))
                    } else if let Some(seq) = picked {
                        Some(self.data_packet(seq, local_phase as u8, local_rx as usize))
                    } else if self.state.lm.rx.have {
                        Some(self.ack_packet(local_rx as usize))
                    } else {
                        None
                    };
                    if let Some(outbound) = outbound {
                        let outbound_is_data = matches!(outbound, Packet::Data { .. });
                        let n = self.encode_packet(&outbound)?;
                        self.phy
                            .op_publish_tx(&self.tx_buf[..n], target, grace)
                            .await?;
                        if self.state.lm.nack_nonzero() {
                            self.nack_sent = self.nack_sent.wrapping_add(1);
                        }
                        if outbound_is_data {
                            self.tx_data = self.tx_data.wrapping_add(1);
                            self.mark_data_sent(picked);
                        }
                    }
                    // else: nothing to send - the slot idles.
                }
            }
        } else if is_sync || local_phase < active_end {
            // RX or independent sync slot: publish into this slot's parity buffer
            // (the run-start mask reset happens here, one collect before the
            // run's first catch is processed).
            if local_phase == local_tx as u32 {
                self.state.lm.rx_run_mask = [0; NACK_BYTES];
            }
            if (target % 2) as usize == 0 {
                self.phy.op_publish_rx(&mut self.rx_pkt_buf, target).await;
            } else {
                self.phy.op_publish_rx(&mut self.rx_pkt_buf2, target).await;
            }
        }
        // Idle slot: publish nothing; the collect below still paces.

        // ---- collect + process the op for `collect_slot` ----
        let c_phase = self.state.next_phase(collect_slot.wrapping_sub(1), period);
        let c_local_phase = self.to_local_phase(c_phase);
        let c_is_sync = c_local_phase.is_none();
        let c_local_phase = c_local_phase.unwrap_or(u32::MAX);
        let c_central_is_tx = self.central_is_tx_phase(c_phase);
        let c_listen = c_is_sync
            || c_central_is_tx
            || (acquiring && !self.state.beacon_anchor_ready && c_phase % 2 == 0);
        let c_is_tx = if c_is_sync {
            central
        } else if central {
            c_local_phase < local_tx as u32
        } else if acquiring {
            !c_listen
        } else {
            c_local_phase < local_tx as u32
        };
        let collected = self.phy.op_collect(collect_slot).await;
        let out = match collected {
            Some(len) => {
                // The radio left `[len | payload]`; shift to payload-only.
                let buf = if (collect_slot % 2) as usize == 0 {
                    &mut self.rx_pkt_buf
                } else {
                    &mut self.rx_pkt_buf2
                };
                let n = len.min(buf.len() - 1);
                buf.copy_within(1..1 + n, 0);
                let mut encoded = [0u8; MAX_PAYLOAD + 32];
                encoded[..n].copy_from_slice(&buf[..n]);
                let reply = self.decode_packet(&encoded[..n])?;
                self.handle_rx_packet(
                    reply,
                    c_local_phase,
                    period,
                    collect_slot,
                    feedback_uses_previous_run(c_local_phase, local_phase, local_tx),
                    rx_buf,
                )
                .await?
            }
            None => {
                if !c_is_tx && c_local_phase < active_end {
                    // A listen slot with no catch.
                    self.handle_rx_miss(c_local_phase, rx_buf)?
                } else {
                    // A TX or idle slot: nothing to process.
                    None
                }
            }
        };
        self.advance_epoch(hw_slot);
        if offer_rejected {
            Err(Error::WindowFull)
        } else {
            Ok(out)
        }
    }

    async fn send<T: serde::Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), Error<P::Error>> {
        let mut buf = [0u8; MAX_PAYLOAD];
        let written = postcard::to_slice(value, &mut buf).map_err(Error::<P::Error>::from)?;
        let n = written.len();
        self.pending_tx[..n].copy_from_slice(&buf[..n]);
        self.pending_tx_len = n;
        self.send_done.reset();
        self.send_done.wait().await;
        Ok(())
    }

    async fn wait_for_tx_space(&mut self) -> Result<(), Error<P::Error>> {
        if !self.state.lm.tx.is_full() {
            return Ok(());
        }
        self.tx_space.reset();
        self.tx_space.wait().await;
        Ok(())
    }

    async fn recv<T: serde::de::DeserializeOwned>(&mut self) -> Result<Option<T>, Error<P::Error>> {
        let mut rx = [0u8; MAX_PAYLOAD];
        let n = match self.frame(None, &mut rx).await? {
            Some(n) => n,
            None => return Ok(None),
        };
        let msg = postcard::from_bytes(&rx[..n]).map_err(Error::<P::Error>::from)?;
        Ok(Some(msg))
    }
}

/// Central node (the synchronization master).
pub struct Central<P: Phy> {
    core: LinkCore<P>,
}

impl<P: Phy> Central<P> {
    /// Create a new central.
    pub async fn new(phy: P, cfg: Config) -> Result<Self, Error<P::Error>> {
        Ok(Self {
            core: LinkCore::new(phy, cfg).await?,
        })
    }

    pub fn status(&self) -> LinkStatus {
        self.core.status()
    }

    pub fn delivery_failures(&self) -> u32 {
        self.core.delivery_failures
    }

    pub fn window_full(&self) -> u32 {
        self.core.window_full
    }

    pub fn tx_window_full(&self) -> bool {
        self.core.tx_window_full()
    }

    /// True when the raw offer passed to the last `frame()` was enqueued
    /// into the TX window (a TX slot with window space).
    pub fn offer_taken(&self) -> bool {
        self.core.offer_taken
    }

    /// The follower's current mirror offset (diagnostic).
    pub fn slot_offset(&self) -> u32 {
        self.core.slot_offset()
    }

    /// The current hardware slot's phase, offset applied (diagnostic).
    pub fn hw_phase(&self) -> u32 {
        self.core.hw_phase()
    }

    /// Data packets decoded from the peer (diagnostic; cumulative).
    pub fn rx_data(&self) -> u32 {
        self.core.rx_data
    }

    /// Data packets published for TX (diagnostic; cumulative).
    pub fn tx_data(&self) -> u32 {
        self.core.tx_data
    }

    /// In-flight (unacknowledged) Data entries in the TX window.
    pub fn tx_inflight(&self) -> u8 {
        self.core.tx_inflight()
    }

    /// Diagnostic: the link's next software slot phase (0..period-1).
    pub fn slot_phase(&self) -> u32 {
        self.core.link_phase()
    }

    /// Adjust the follower early TX margin at runtime (central is a no-op).
    pub fn set_tx_phase_margin_us(&mut self, margin_us: i32) {
        self.core.set_tx_phase_margin_us(margin_us);
    }

    pub fn retransmits(&self) -> u32 {
        self.core.retransmits
    }

    pub fn nacks_recv(&self) -> u32 {
        self.core.nacks_recv
    }

    /// Negotiate exact directional application payload lengths and a bounded
    /// slot probe. After Commit, Data must match its direction's fixed length;
    /// payload/NACK lengths are omitted from the compact wire format. Existing
    /// TX traffic must drain first, otherwise this returns `Busy`.
    pub fn negotiate_cadence(
        &mut self,
        contract: TrafficContract,
        policy: CadenceProbePolicy,
    ) -> Result<u8, CadenceError> {
        self.core.request_cadence(contract, policy)
    }

    /// Synchronously release the active traffic contract and restore the
    /// acquisition-safe cadence profile. The normal `frame()` loop carries
    /// the Release/Accept/Commit/Applied handshake.
    pub fn exit_cadence(&mut self) -> Result<u8, CadenceError> {
        self.core.request_cadence_exit()
    }

    /// Configure automatic safe release after severe loss. `None` disables
    /// automatic release; packet length never triggers this policy.
    pub fn set_cadence_exit_policy(&mut self, policy: Option<CadenceExitPolicy>) {
        self.core.set_cadence_exit_policy(policy);
    }

    /// Current API-triggered cadence negotiation state.
    pub fn cadence_status(&self) -> CadenceNegotiationStatus {
        self.core.cadence_status()
    }

    /// Effective directional probe floors agreed so far, in microseconds.
    pub fn cadence_probe_bounds(&self) -> Option<CadenceProbeBounds> {
        self.core.cadence_probe_bounds()
    }

    /// Candidate currently being armed, exercised, or reported.
    pub fn cadence_candidate(&self) -> Option<CadenceProfile> {
        self.core.cadence_candidate()
    }

    pub async fn frame(
        &mut self,
        tx_payload: Option<&[u8]>,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        self.core.frame(tx_payload, rx_buf).await
    }

    pub async fn send<T: serde::Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), Error<P::Error>> {
        self.core.send(value).await
    }

    pub async fn wait_for_tx_space(&mut self) -> Result<(), Error<P::Error>> {
        self.core.wait_for_tx_space().await
    }

    pub async fn recv<T: serde::de::DeserializeOwned>(
        &mut self,
    ) -> Result<Option<T>, Error<P::Error>> {
        self.core.recv().await
    }
}

/// Peripheral node (the synchronization follower).
pub struct Peripheral<P: Phy> {
    core: LinkCore<P>,
}

impl<P: Phy> Peripheral<P> {
    /// Create a new peripheral.
    pub async fn new(phy: P, cfg: Config) -> Result<Self, Error<P::Error>> {
        Ok(Self {
            core: LinkCore::new(phy, cfg).await?,
        })
    }

    pub fn status(&self) -> LinkStatus {
        self.core.status()
    }

    pub fn delivery_failures(&self) -> u32 {
        self.core.delivery_failures
    }

    pub fn window_full(&self) -> u32 {
        self.core.window_full
    }

    pub fn tx_window_full(&self) -> bool {
        self.core.tx_window_full()
    }

    /// True when the raw offer passed to the last `frame()` was enqueued
    /// into the TX window (a TX slot with window space).
    pub fn offer_taken(&self) -> bool {
        self.core.offer_taken
    }

    /// The follower's current mirror offset (diagnostic).
    pub fn slot_offset(&self) -> u32 {
        self.core.slot_offset()
    }

    /// The current hardware slot's phase, offset applied (diagnostic).
    pub fn hw_phase(&self) -> u32 {
        self.core.hw_phase()
    }

    /// Data packets decoded from the peer (diagnostic; cumulative).
    pub fn rx_data(&self) -> u32 {
        self.core.rx_data
    }

    /// Data packets published for TX (diagnostic; cumulative).
    pub fn tx_data(&self) -> u32 {
        self.core.tx_data
    }

    /// In-flight (unacknowledged) Data entries in the TX window.
    pub fn tx_inflight(&self) -> u8 {
        self.core.tx_inflight()
    }

    /// Diagnostic: the link's next software slot phase (0..period-1).
    pub fn slot_phase(&self) -> u32 {
        self.core.link_phase()
    }

    /// Adjust the follower early TX margin at runtime (central is a no-op).
    pub fn set_tx_phase_margin_us(&mut self, margin_us: i32) {
        self.core.set_tx_phase_margin_us(margin_us);
    }

    pub fn retransmits(&self) -> u32 {
        self.core.retransmits
    }

    pub fn resyncs(&self) -> u32 {
        self.core.resyncs
    }

    pub fn nack_sent(&self) -> u32 {
        self.core.nack_sent
    }

    pub fn rx_span(&self) -> u16 {
        self.core
            .state
            .lm
            .rx
            .highest_seen
            .wrapping_sub(self.core.state.lm.rx.next_expected)
    }

    /// Request a traffic contract from the peripheral API. The request is
    /// repeated until the central responds with its authoritative Offer;
    /// the central still selects every candidate and absolute epoch.
    pub fn negotiate_cadence(
        &mut self,
        contract: TrafficContract,
        policy: CadenceProbePolicy,
    ) -> Result<u8, CadenceError> {
        self.core.request_cadence(contract, policy)
    }

    /// Ask the central to synchronously release the active traffic contract
    /// and restore the acquisition-safe cadence profile.
    pub fn exit_cadence(&mut self) -> Result<u8, CadenceError> {
        self.core.request_cadence_exit()
    }

    /// Configure automatic safe release after severe loss. `None` disables
    /// automatic release; packet length never triggers this policy.
    pub fn set_cadence_exit_policy(&mut self, policy: Option<CadenceExitPolicy>) {
        self.core.set_cadence_exit_policy(policy);
    }

    /// Current cadence negotiation state driven by either peer's API request.
    pub fn cadence_status(&self) -> CadenceNegotiationStatus {
        self.core.cadence_status()
    }

    /// Effective directional probe floors agreed so far, in microseconds.
    pub fn cadence_probe_bounds(&self) -> Option<CadenceProbeBounds> {
        self.core.cadence_probe_bounds()
    }

    /// Candidate currently being armed, exercised, or reported.
    pub fn cadence_candidate(&self) -> Option<CadenceProfile> {
        self.core.cadence_candidate()
    }

    pub async fn frame(
        &mut self,
        tx_payload: Option<&[u8]>,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        self.core.frame(tx_payload, rx_buf).await
    }

    pub async fn send<T: serde::Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), Error<P::Error>> {
        self.core.send(value).await
    }

    pub async fn wait_for_tx_space(&mut self) -> Result<(), Error<P::Error>> {
        self.core.wait_for_tx_space().await
    }

    pub async fn recv<T: serde::de::DeserializeOwned>(
        &mut self,
    ) -> Result<Option<T>, Error<P::Error>> {
        self.core.recv().await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Address, MAX_RETRIES, RETRY_TIMEOUT_SLOTS};
    use crate::link_mgmt::TxWindow;

    #[test]
    fn cadence_exit_thresholds_are_exact_and_independent() {
        let policy = CadenceExitPolicy::new(2, 3);
        assert!(!cadence_exit_triggered(policy, 1, 2));
        assert!(cadence_exit_triggered(policy, 2, 0));
        assert!(cadence_exit_triggered(policy, 0, 3));
        assert!(!cadence_exit_triggered(
            CadenceExitPolicy::default(),
            99,
            99
        ));
    }

    #[test]
    fn fixed_codec_legacy_grace_is_bounded() {
        let mut grace = 2;
        assert!(accept_legacy_data_plane(true, &mut grace));
        assert!(accept_legacy_data_plane(true, &mut grace));
        assert!(!accept_legacy_data_plane(true, &mut grace));
        assert!(accept_legacy_data_plane(false, &mut grace));
    }

    #[test]
    fn fallback_beacon_only_resets_an_active_contract() {
        assert!(should_join_central_fallback(true, 0));
        assert!(!should_join_central_fallback(true, 123));
        assert!(!should_join_central_fallback(false, 0));
    }

    #[test]
    fn probe_timing_uses_independent_checks_when_callback_stamp_is_missing() {
        assert!(!probe_timing_bad(0, 20_800));
        assert!(!probe_timing_bad(20_790, 20_800));
        assert!(probe_timing_bad(22_000, 20_800));
    }

    #[test]
    fn final_confirmation_requires_bidirectional_overlap() {
        assert_eq!(required_probe_rx(true, 0), 0);
        assert_eq!(required_probe_rx(true, 64), 1);
        assert_eq!(required_probe_rx(false, 8), 4);
    }

    #[test]
    fn probe_arm_lead_covers_depth_two_publication() {
        assert!(!probe_has_sufficient_arm_lead(-1));
        assert!(!probe_has_sufficient_arm_lead(0));
        assert!(!probe_has_sufficient_arm_lead(PROBE_ARM_LEAD_SLOTS - 1));
        assert!(probe_has_sufficient_arm_lead(PROBE_ARM_LEAD_SLOTS));
    }

    #[test]
    fn sync_slot_profile_maps_phase_zero_long() {
        assert_eq!(profile_short_phases(8, true), 8);
        assert_eq!(profile_short_phases(8, false), 8);
        let shifted = profile_central_start(99, 11, true);
        assert_eq!(shifted, 109);
        assert_eq!(descriptor_phase(1_000, 1_000, shifted, 11), 10);
        assert_eq!(descriptor_phase(1_001, 1_000, shifted, 11), 0);
        assert_eq!(descriptor_phase(1_008, 1_000, shifted, 11), 7);
        assert_eq!(descriptor_phase(1_009, 1_000, shifted, 11), 8);
        let profile = CadenceProfile::new(500, 600, 8, 2, 0);
        assert_eq!(profile.with_sync_slot(true).period_slots(), 11);
        assert_eq!(
            profile.with_sync_slot(true).superframe_us(),
            8 * 500 + 3 * 600
        );
        assert_eq!(profile.superframe_us(), 8 * 500 + 2 * 600);
    }

    #[test]
    fn sync_phase_is_outside_eight_to_two_data_map() {
        assert_eq!(data_phase_from_physical(0, true), None);
        for phase in 1..=8 {
            assert_eq!(data_phase_from_physical(phase, true), Some(phase - 1));
        }
        assert_eq!(data_phase_from_physical(9, true), Some(8));
        assert_eq!(data_phase_from_physical(10, true), Some(9));
        assert_eq!(data_phase_from_physical(0, false), Some(0));
    }

    #[test]
    fn descriptor_phase_survives_independent_epoch_wraps() {
        assert_eq!(descriptor_phase(43_621, 43_621, 10_000, 10), 0);
        assert_eq!(descriptor_phase(43_629, 43_621, 10_000, 10), 8);
        assert_eq!(descriptor_phase(1, u32::MAX - 1, 10_000, 10), 3);
        assert_eq!(descriptor_phase(43_624, 43_621, u32::MAX - 1, 10), 1);
    }

    #[test]
    fn pipelined_feedback_selects_the_run_encoded_in_packet() {
        // 8:2: first feedback is stale R-1; final feedback is collected while
        // target phase 0 rotates R into previous.
        assert!(feedback_uses_previous_run(8, 9, 8));
        assert!(feedback_uses_previous_run(9, 0, 8));
        // With a longer feedback run, middle packets already contain R and
        // phase 0 has not rotated it yet.
        assert!(!feedback_uses_previous_run(9, 10, 8));
        assert!(!feedback_uses_previous_run(10, 11, 8));
        assert!(feedback_uses_previous_run(11, 0, 8));
    }

    #[test]
    fn cadence_generation_rejects_duplicates_and_stale_release() {
        assert!(!cadence_generation_newer(7, 7));
        assert!(cadence_generation_newer(8, 7));
        assert!(!cadence_generation_newer(6, 7));
        assert!(cadence_generation_newer(1, 127));
        assert!(!cadence_generation_newer(127, 1));
    }

    #[test]
    fn offer_generation_never_rolls_back_retry_state() {
        assert!(cadence_offer_generation_allowed(
            CadenceRunStage::Request,
            7,
            7
        ));
        assert!(!cadence_offer_generation_allowed(
            CadenceRunStage::Request,
            7,
            8
        ));
        assert!(cadence_offer_generation_allowed(
            CadenceRunStage::Accept,
            7,
            7
        ));
        assert!(cadence_offer_generation_allowed(
            CadenceRunStage::Accept,
            7,
            8
        ));
        assert!(!cadence_offer_generation_allowed(
            CadenceRunStage::Accept,
            7,
            6
        ));
        assert!(cadence_offer_generation_allowed(
            CadenceRunStage::Failed,
            7,
            8
        ));
        assert!(!cadence_offer_generation_allowed(
            CadenceRunStage::Failed,
            7,
            7
        ));
    }

    #[test]
    fn local_phase_mapping_without_idle() {
        let mut cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central).with_tx_rx_ratio(8, 2);
        let central = LinkState::new(&cfg);
        assert_eq!(central.local_phase_for(0), 0);
        assert_eq!(central.local_phase_for(7), 7);
        assert_eq!(central.local_phase_for(8), 8);
        assert_eq!(central.local_phase_for(9), 9);

        cfg.role = Role::Peripheral;
        let peripheral = LinkState::new(&cfg);
        assert_eq!(peripheral.local_phase_for(0), 2);
        assert_eq!(peripheral.local_phase_for(7), 9);
        assert_eq!(peripheral.local_phase_for(8), 0);
        assert_eq!(peripheral.local_phase_for(9), 1);
    }

    #[test]
    fn local_phase_mapping_with_idle() {
        let mut cfg =
            Config::new([0; 4], Address([0xE7; 5]), Role::Central).with_tx_rx_idle(8, 4, 4);
        let central = LinkState::new(&cfg);
        assert_eq!(central.local_phase_for(0), 0);
        assert_eq!(central.local_phase_for(11), 11);
        assert_eq!(central.local_phase_for(12), 12);
        assert_eq!(central.local_phase_for(15), 15);

        cfg.role = Role::Peripheral;
        let peripheral = LinkState::new(&cfg);
        // central TX 0..7 -> peripheral RX 4..11
        assert_eq!(peripheral.local_phase_for(0), 4);
        assert_eq!(peripheral.local_phase_for(7), 11);
        // central RX 8..11 -> peripheral TX 0..3
        assert_eq!(peripheral.local_phase_for(8), 0);
        assert_eq!(peripheral.local_phase_for(11), 3);
        // idle is shared
        assert_eq!(peripheral.local_phase_for(12), 12);
        assert_eq!(peripheral.local_phase_for(15), 15);
    }

    #[test]
    fn reverse_ratio_is_complement_or_normalized() {
        let mut cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central);
        cfg.tx_rx_ratio = (8, 2);
        cfg.reverse_tx_rx_ratio = (3, 7);
        let state = LinkState::new(&cfg);
        assert_eq!(state.reverse_tx_rx_ratio, (2, 8));

        cfg.reverse_tx_rx_ratio = (2, 8);
        let state = LinkState::new(&cfg);
        assert_eq!(state.reverse_tx_rx_ratio, (2, 8));
    }

    #[test]
    fn config_capacity_api() {
        let cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central).with_tx_rx_idle(8, 4, 4);
        assert_eq!(cfg.period_slots(), 16);
        assert_eq!(cfg.physical_period_slots(1), 17);
        assert_eq!(cfg.tx_slots_per_period(Role::Central), 8);
        assert_eq!(cfg.tx_slots_per_period(Role::Peripheral), 4);
    }

    #[test]
    fn config_builder_with_idle_keeps_complement() {
        let cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central).with_tx_rx_idle(6, 2, 2);
        assert_eq!(cfg.tx_rx_ratio, (6, 2));
        assert_eq!(cfg.reverse_tx_rx_ratio, (2, 6));
        assert_eq!(cfg.idle_slots, 2);
    }

    #[test]
    fn config_builder_keeps_ratios_complementary() {
        let cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central).with_tx_rx_ratio(5, 3);
        assert_eq!(cfg.tx_rx_ratio, (5, 3));
        assert_eq!(cfg.reverse_tx_rx_ratio, (3, 5));
    }

    #[test]
    fn seq_gt_is_circular() {
        assert!(seq_gt(1, 0));
        assert!(!seq_gt(0, 1));
        assert!(seq_gt(0, u16::MAX)); // wraps
        assert!(!seq_gt(u16::MAX, 0));
        assert!(!seq_gt(7, 7));
    }

    #[test]
    fn zero_tx_rx_ratio_is_normalized() {
        let mut cfg = Config::new([0xAB, 0xCD, 0xEF, 0x01], Address([0xE7; 5]), Role::Central);
        cfg.tx_rx_ratio = (0, 0);
        let state = LinkState::new(&cfg);
        assert_eq!(state.tx_rx_ratio, (1, 1));
    }

    #[test]
    fn tx_ack_frees_contiguous_prefix() {
        let mut w = TxWindow::new();
        for i in 0..8u16 {
            w.enqueue(&[i as u8; 4]);
        }
        assert_eq!(w.inflight, 8);
        w.on_ack(3); // ack 0,1,2,3
        assert_eq!(w.inflight, 4);
        assert!(!w.is_full());
        // seq 4..7 still in flight
        assert_eq!(w.pick(), Some(4));
    }

    #[test]
    fn tx_blocked_window_picks_oldest_sent_for_fallback() {
        let mut w = TxWindow::new();
        for i in 0..(WINDOW_SIZE as u16) {
            w.enqueue(&[i as u8; 4]);
            w.mark_sent(i);
        }
        assert!(w.is_full());
        assert_eq!(w.pick(), None); // no retransmit/unsent
        assert_eq!(w.pick_sent_for_blocked(), Some(0));
    }

    #[test]
    fn tx_window_full_after_hole_prevents_overwrite() {
        let mut w = TxWindow::new();
        for i in 0..(WINDOW_SIZE as u16) {
            w.enqueue(&[i as u8; 4]);
        }
        w.on_ack(7);
        // A delivery failure at seq 10 frees that slot but leaves
        // 8, 9, 11..15 in flight. The window still has room for the next
        // sequence numbers (16..23), but seq 24 would wrap onto seq 8's
        // slot, so the window must report full before that happens.
        w.drop(10);
        assert_eq!(w.inflight, 7);
        assert!(!w.is_full());
        for _ in 0..8 {
            w.enqueue(&[0u8; 4]); // seqs 16..23
        }
        assert_eq!(w.inflight, 15);
        assert!(w.is_full());
        assert_eq!(w.entry(8).seq, 8);
        assert_eq!(w.entry(23).seq, 23);
    }

    #[test]
    fn tx_ack_ignores_ack_beyond_enqueued_range() {
        let mut w = TxWindow::new();
        for i in 0..4u16 {
            w.enqueue(&[i as u8; 4]);
            w.mark_sent(i);
        }
        // A peer restart/resync can send an ACK far ahead of anything this
        // node has enqueued. Accepting it would free in-flight entries the
        // peer never received, so it must be ignored.
        w.on_ack(999);
        assert_eq!(w.tx_acked, u16::MAX);
        assert_eq!(w.inflight, 4);
        assert!(!w.is_full());
        // A valid cumulative ACK still drains normally.
        w.on_ack(3);
        assert_eq!(w.inflight, 0);
        assert_eq!(w.tx_acked, 3);
    }

    #[test]
    fn tx_on_nack_slots_flags_by_slot_position() {
        let mut w = TxWindow::new();
        for i in 0..8u16 {
            w.enqueue(&[i as u8; 4]);
            w.mark_sent(i);
        }
        let mut slots = [None; WINDOW_SIZE];
        slots[0] = Some(TxRunSlot { slot: 2, seq: 2 });
        slots[1] = Some(TxRunSlot { slot: 4, seq: 4 });
        let mut nack = [0u8; NACK_BYTES];
        nack[0] = 0b0001_0100; // bits 2 and 4
        w.on_nack_slots(&nack, &slots);
        assert_eq!(w.pick(), Some(2));
        w.mark_sent(2);
        assert_eq!(w.pick(), Some(4));
    }

    #[test]
    fn tx_on_nack_slots_ignores_bits_without_entries() {
        let mut w = TxWindow::new();
        for i in 0..4u16 {
            w.enqueue(&[i as u8; 4]);
            w.mark_sent(i);
        }
        // Slot 0 carried no Data in the last run, so NACK bit 0 must not
        // flag seq 0 (which was sent in some earlier run).
        let slots = [None; WINDOW_SIZE];
        let mut nack = [0u8; NACK_BYTES];
        nack[0] = 0b1;
        w.on_nack_slots(&nack, &slots);
        assert_eq!(w.pick(), None);
    }

    #[test]
    fn tx_nack_flags_retransmit() {
        let mut w = TxWindow::new();
        for i in 0..8u16 {
            w.enqueue(&[i as u8; 4]);
        }
        // ack up to 1, holes at 2 and 4 (bits 0 and 2 relative to ack=1)
        w.on_ack(1);
        w.on_nack(1, 0b101);
        // the flagged retransmits are picked first, lowest seq first
        assert_eq!(w.pick(), Some(2));
        w.mark_sent(2);
        assert_eq!(w.pick(), Some(4));
    }

    #[test]
    fn tx_window_full_and_timeout() {
        let mut w = TxWindow::new();
        for i in 0..(WINDOW_SIZE as u16) {
            w.enqueue(&[i as u8]);
        }
        assert!(w.is_full());
        // mark all sent, then tick past the timeout: they get flagged
        for i in 0..(WINDOW_SIZE as u16) {
            assert!(!w.mark_sent(i));
        }
        for _ in 0..RETRY_TIMEOUT_SLOTS {
            w.tick();
        }
        // the lowest seq is now flagged for retransmit
        assert_eq!(w.pick(), Some(0));
    }

    #[test]
    fn tx_retry_exhaustion_drops() {
        let mut w = TxWindow::new();
        w.enqueue(&[9u8; 2]);
        assert!(!w.mark_sent(0)); // first send
        for _ in 1..MAX_RETRIES {
            assert!(!w.mark_sent(0)); // retransmits within budget
        }
        assert!(w.mark_sent(0)); // the MAX_RETRIES-th retransmit is exhausted
        w.drop(0);
        assert_eq!(w.inflight, 0);
    }

    #[test]
    fn rx_in_order_deliver() {
        let mut r = RxWindow::new();
        assert!(r.receive(0, &[10, 11]));
        assert_eq!(r.peek_len(), Some(2));
        let e = r.pop_head().unwrap();
        assert_eq!(e.len, 2);
        assert_eq!(&e.payload[..2], &[10, 11]);
        assert_eq!(r.ack(), 0);
        assert_eq!(r.nack(), 0);
    }

    #[test]
    fn one_ack_after_multi_slot_run_flags_retransmit() {
        let mut tx = TxWindow::new();
        let mut rx = RxWindow::new();

        // One TX run sends eight packets; seq 2 is lost on the air.
        for seq in 0..8u16 {
            tx.enqueue(&[seq as u8; 4]);
        }
        for seq in 0..8u16 {
            tx.mark_sent(seq);
            if seq != 2 {
                rx.receive(seq, &[seq as u8]);
            }
            // The link delivers at most one in-order payload per RX slot.
            if rx.peek_len().is_some() {
                rx.pop_head();
            }
        }

        // The receiver then emits its single ACK/NACK for the whole run.
        let ack = rx.ack();
        let nack = rx.nack();
        assert_eq!(ack, 1); // delivered 0 and 1
        assert_eq!(nack, 0b1); // bit 0 = seq 2 is missing

        tx.on_ack(ack);
        tx.on_nack(ack, nack);
        // The bitmap drives the retransmit: seq 2 is picked first.
        assert_eq!(tx.pick(), Some(2));
    }

    #[test]
    fn rx_out_of_order_nack_and_reorder() {
        let mut r = RxWindow::new();
        // seq 0 and 2 arrive, 1 missing
        assert!(r.receive(0, &[0]));
        assert!(r.receive(2, &[2]));
        // nothing delivered yet: the ack is the "no ack" sentinel
        assert_eq!(r.ack(), 0xFFFF);
        assert_eq!(r.nack(), 0b10); // bit 1 = seq 1 missing
        // deliver 0, then the head is 1 (not yet present)
        assert_eq!(r.pop_head().unwrap().payload[0], 0);
        assert_eq!(r.peek_len(), None);
        assert_eq!(r.ack(), 0);
        // seq 1 arrives: now 1 and 2 are contiguous
        assert!(r.receive(1, &[1]));
        assert_eq!(r.peek_len(), Some(1));
        assert_eq!(r.pop_head().unwrap().payload[0], 1);
        assert_eq!(r.pop_head().unwrap().payload[0], 2);
        assert_eq!(r.ack(), 2);
        assert_eq!(r.nack(), 0);
    }

    #[test]
    fn rx_resync_rebaselines() {
        let mut r = RxWindow::new();
        r.resync(5000);
        assert!(r.in_window(5000));
        assert!(!r.in_window(0)); // far behind the new baseline
        assert!(r.receive(5000, &[1]));
        assert_eq!(r.peek_len(), Some(1));
    }

    /// Full ARQ loop (sender TxWindow + receiver RxWindow) over a 12.5%-lossy
    /// channel: every packet must be delivered exactly once, in order.
    #[test]
    fn arq_recovers_loss_in_order() {
        let mut tx = TxWindow::new();
        let mut rx = RxWindow::new();
        const TOTAL: u16 = 100;
        let mut next_new: u16 = 0;
        let mut delivered: Vec<u8, 128> = Vec::new();
        let mut send_count: u32 = 0;

        for _ in 0..100_000 {
            // 1. generate a new packet (up to TOTAL, while the window has room)
            if next_new < TOTAL && !tx.is_full() {
                tx.enqueue(&[next_new as u8]);
                next_new += 1;
            }
            tx.tick();

            // 2. sender picks one packet and "transmits" it (12.5% lost)
            if let Some(seq) = tx.pick() {
                let payload = tx.entry(seq).payload[0];
                let lost = send_count % 8 == 7;
                if !lost {
                    rx.receive(seq, &[payload]);
                }
                tx.mark_sent(seq);
                send_count += 1;
            }

            // 3. receiver delivers in-order
            while rx.peek_len().is_some() {
                let e = rx.pop_head().unwrap();
                delivered.push(e.payload[0]).ok();
            }

            // 4. feed the receiver's ACK/NACK back to the sender
            let ack = rx.ack();
            let nack = rx.nack();
            tx.on_ack(ack);
            tx.on_nack(ack, nack);

            if delivered.len() >= TOTAL as usize {
                break;
            }
        }

        assert_eq!(delivered.len(), TOTAL as usize);
        for (i, s) in delivered.iter().enumerate() {
            assert_eq!(*s, i as u8);
        }
    }
}
