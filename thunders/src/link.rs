//! Link-layer state machines for central and peripheral.

/// Consecutive missed RX slots before the central advances the channel
/// (transient misses stay put; persistent jamming hops away).
const HOP_MISS_THRESHOLD: u8 = 16;

/// Consecutive missed RX slots before the link is declared lost: the status
/// drops back to `Disconnected` and the node returns to the initial channel,
/// so a recovered link can re-align.
const LINK_LOSS_THRESHOLD: u8 = 16;

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
        CadenceError, CadenceExitPolicy, CadenceNegotiationStatus, CadenceProbePolicy,
        CadenceProfile, CadenceSearch, ProbeDecision, ProbeMetrics, TrafficContract,
    },
    config::{
        Config, Role, CENTRAL_REPLY_TIMEOUT_US, MAX_PAYLOAD, NACK_BYTES,
        PERIPHERAL_LISTEN_TIMEOUT_US, WINDOW_SIZE,
    },
    error::Error,
    link_mgmt::{
        nack_from_mask, nack_nonzero, nack_set, nack_vec, seq_gt, LinkMgmt, RxWindow, TxRunSlot,
    },
    packet::{CadenceStage, Packet},
    phy::{Phy, SlotProbeStats},
    scheduler::Scheduler,
};

#[cfg(feature = "secure")]
use crate::security::{make_nonce, make_nonce_13, Cipher, CipherMode, Security};

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
    fn on_miss(&mut self, streak: &mut u8) {
        *streak = streak.saturating_add(1);
        self.connect_streak = 0;
        if self.status != LinkStatus::Connected {
            return;
        }
        if *streak >= LINK_LOSS_THRESHOLD {
            self.status = LinkStatus::Disconnected;
            self.scheduler.sync(self.initial_channel);
            *streak = 0;
        } else if self.central && *streak >= HOP_MISS_THRESHOLD {
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

const CADENCE_PROTOCOL_OVERHEAD: u16 = 12;
const CADENCE_FLAG_STABLE: u8 = 1;
const CADENCE_FLAG_REJECT: u8 = 2;
const CADENCE_FLAG_RELEASE: u8 = 4;

fn cadence_exit_triggered(policy: CadenceExitPolicy, failed: u32, misses: u8) -> bool {
    (policy.delivery_failures != 0 && failed >= policy.delivery_failures as u32)
        || (policy.consecutive_misses != 0 && misses >= policy.consecutive_misses)
}

fn cadence_generation_newer(candidate: u8, current: u8) -> bool {
    let delta = candidate.wrapping_sub(current) & 0x7f;
    delta != 0 && delta < 64
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
    Stable,
    Failed,
}

struct CadenceRuntime {
    stage: CadenceRunStage,
    generation: u8,
    search: Option<CadenceSearch>,
    contract: TrafficContract,
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
    local_metrics: Option<ProbeMetrics>,
    peer_metrics: Option<ProbeMetrics>,
    apply_epoch: u32,
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
            local_metrics: None,
            peer_metrics: None,
            apply_epoch: 0,
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
    /// Peripheral handshake ACK: profile id, with bit 7 set after the armed
    /// apply epoch has been received and scheduled.
    cadence_ack: u8,
    /// API-triggered traffic-contract negotiation and bounded probe state.
    cadence_runtime: CadenceRuntime,
    /// Acquisition-negotiated profile used when releasing a short-payload
    /// contract. API probes never overwrite this safety anchor.
    cadence_safe_profile: CadenceProfile,
    /// Contract currently enforced by the data plane. It remains active while
    /// a replacement or release handshake is in flight.
    cadence_active_contract: Option<TrafficContract>,
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
        );
        Ok(Self {
            cadence_negotiated: false,
            cadence_ok: phy.min_slot_period_us() == 0,
            cadence_id,
            cadence_short_us,
            cadence_long_us,
            cadence_short_phases: cfg.tx_rx_ratio.0.max(1) as u16,
            cadence_apply_epoch: 0,
            cadence_ack: 0,
            cadence_runtime: CadenceRuntime::new(stable_profile),
            cadence_safe_profile: stable_profile,
            cadence_active_contract: None,
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

    fn link_phase(&self) -> u32 {
        let (c_tx, c_rx) = self.state.tx_rx_ratio;
        let period = c_tx as u16 + c_rx as u16 + self.state.idle_slots as u16;
        let phase = self.state.next_phase(0, period as u32);
        self.to_local_phase(phase)
    }

    /// Map a central-schedule phase to this node's local phase. With idle
    /// slots the complement is piecewise: central TX -> peripheral RX,
    /// central RX -> peripheral TX, central idle -> peripheral idle.
    fn to_local_phase(&self, phase: u32) -> u32 {
        self.state.local_phase_for(phase)
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

    /// Serialize `pkt` into the shared TX buffer; returns the byte length.
    fn encode_packet(&mut self, pkt: &Packet) -> Result<usize, Error<P::Error>> {
        pkt.to_bytes(&mut self.tx_buf)
            .map_err(Error::<P::Error>::from)
    }

    /// Build the outbound `Packet::Data` for `seq` and record the slot
    /// position mapping for slot-NACK.
    fn data_packet(&mut self, seq: u16, slot: u8, nack_run_len: usize) -> Packet {
        for entry in self.state.lm.tx_run_slots.iter_mut() {
            if entry.is_none() {
                *entry = Some(TxRunSlot { slot, seq });
                break;
            }
        }
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

    fn cadence_status(&self) -> CadenceNegotiationStatus {
        if self.cadence_runtime.releasing {
            return CadenceNegotiationStatus::Releasing;
        }
        match self.cadence_runtime.stage {
            CadenceRunStage::Idle => CadenceNegotiationStatus::Idle,
            CadenceRunStage::Probing => CadenceNegotiationStatus::Probing {
                candidate: self.cadence_runtime.candidate,
            },
            CadenceRunStage::Commit | CadenceRunStage::Applying => {
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
        if policy.min_slot_us < self.phy.min_probe_short_slot_period_us()
            || policy.step_us > u8::MAX as u16
            || policy.safety_steps > u8::MAX as u16
            || (policy.probe_superframes as u32)
                .saturating_mul(self.cadence_runtime.stable.superframe_us())
                > 30_000_000
        {
            return Err(CadenceError::InvalidPolicy);
        }
        if contract.forward_max_payload as usize > MAX_PAYLOAD
            || contract.reverse_max_payload as usize > MAX_PAYLOAD
        {
            return Err(CadenceError::PayloadTooLarge);
        }
        if !matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::Idle | CadenceRunStage::Stable | CadenceRunStage::Failed
        ) {
            return Err(CadenceError::Busy);
        }
        let stable = self.cadence_runtime.stable;
        let search = CadenceSearch::new(stable, contract, policy, CADENCE_PROTOCOL_OVERHEAD)?;
        let generation = (self.cadence_runtime.generation.wrapping_add(1) & 0x7f).max(1);
        self.cadence_runtime = CadenceRuntime::new(stable);
        self.cadence_runtime.generation = generation;
        self.cadence_id = generation;
        self.cadence_ack = 0;
        self.cadence_runtime.contract = contract;
        self.cadence_runtime.policy = policy;
        self.cadence_runtime.candidate = search.next_probe().unwrap_or(stable);
        if self.state.central {
            self.cadence_runtime.search = Some(search);
            // Even when the verified floor leaves no lower candidate, still
            // run Offer/Accept and commit the traffic contract to both peers.
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
            CadenceRunStage::Stable | CadenceRunStage::Failed
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
        if self.cadence_runtime.releasing {
            self.cadence_active_contract = None;
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
            self.cadence_runtime.stage = CadenceRunStage::Stable;
            self.cadence_exit_failure_baseline = self.delivery_failures;
        }
    }

    fn emergency_cadence_fallback(&mut self) {
        let fallback = self.phy.fallback_slot_period_us().max(1);
        self.phy.align_slot_period(fallback);
        self.cadence_active_contract = None;
        self.cadence_runtime = CadenceRuntime::new(self.cadence_safe_profile);
        self.cadence_id = 1;
        self.cadence_short_us = self.cadence_safe_profile.short_slot_us;
        self.cadence_long_us = self.cadence_safe_profile.long_slot_us;
        self.cadence_short_phases = self.cadence_safe_profile.forward_slots;
        self.cadence_apply_epoch = 0;
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
        let start = Self::align_future(slot, period, 8);
        let end = start.wrapping_add(
            period.saturating_mul(self.cadence_runtime.policy.probe_superframes as u32),
        );
        self.cadence_runtime.central_start = start;
        self.cadence_runtime.central_end = end;
        self.cadence_runtime.local_start = start;
        self.cadence_runtime.local_end = end;
        self.cadence_runtime.probe_started = false;
        self.cadence_runtime.local_metrics = None;
        self.cadence_runtime.peer_metrics = None;
        let p = self.cadence_runtime.candidate;
        self.phy.schedule_slot_probe(
            p.short_slot_us,
            p.long_slot_us,
            p.period_slots() as u16,
            p.forward_slots,
            0,
            start,
            end,
        );
        self.cadence_runtime.stage = CadenceRunStage::ProbePlan;
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
        let combined = ProbeMetrics::new(
            local.completed_superframes.min(peer.completed_superframes),
            local.forward_failures.saturating_add(peer.forward_failures),
            local.reverse_failures.saturating_add(peer.reverse_failures),
        );
        let candidate = self.cadence_runtime.candidate;
        let Some(search) = self.cadence_runtime.search.as_mut() else {
            return;
        };
        match search.record_probe(candidate, combined) {
            Ok(ProbeDecision::Incomplete(_)) => {}
            Ok(ProbeDecision::Passed(next)) if search.final_profile().is_none() => {
                self.cadence_runtime.candidate = next;
                self.start_probe_plan(slot, period);
            }
            Ok(ProbeDecision::Passed(profile)) | Ok(ProbeDecision::Failed(profile)) => {
                self.start_final_commit(slot, period, profile);
            }
            Err(e) => {
                self.cadence_runtime.error = Some(e);
                self.cadence_runtime.stage = CadenceRunStage::Failed;
            }
        }
    }

    fn cadence_tick(&mut self, slot: u32, period: u32) {
        if self.cadence_runtime.releasing {
            if self.cadence_runtime.release_deadline == 0 {
                self.cadence_runtime.release_deadline =
                    slot.wrapping_add(period.saturating_mul(256));
            } else if (slot.wrapping_sub(self.cadence_runtime.release_deadline) as i32) >= 0 {
                self.emergency_cadence_fallback();
                return;
            }
        }
        if self.missed_frames >= LINK_LOSS_THRESHOLD
            && (self.cadence_active_contract.is_some() || self.cadence_runtime.releasing)
        {
            self.emergency_cadence_fallback();
            return;
        }
        if self.cadence_runtime.stage == CadenceRunStage::Stable && self.cadence_auto_exit_due() {
            let _ = self.request_cadence_exit();
        }
        let in_probe_state = matches!(
            self.cadence_runtime.stage,
            CadenceRunStage::ProbePlan | CadenceRunStage::Armed | CadenceRunStage::Probing
        );
        if in_probe_state
            && !self.cadence_runtime.probe_started
            && (slot.wrapping_sub(self.cadence_runtime.local_start) as i32) >= 0
        {
            self.cadence_runtime.stats_start = self.phy.slot_probe_stats();
            self.cadence_runtime.delivery_failures_start = self.delivery_failures;
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
            let expected = self
                .cadence_runtime
                .candidate
                .superframe_us()
                .saturating_mul(self.cadence_runtime.policy.probe_superframes as u32);
            let timing_bad = delta.clock_us > expected.saturating_add(expected / 50 + 100);
            let (local_tx, local_rx) = self.local_ratio();
            let expected_tx =
                self.cadence_runtime.policy.probe_superframes as u32 * local_tx as u32;
            let expected_rx =
                self.cadence_runtime.policy.probe_superframes as u32 * local_rx as u32;
            // A timing-only probe could pass after losing every Sample. Require
            // at least half the planned TX completions and RX address catches;
            // the latter still tolerates the measured ~28% weak reverse raw
            // loss before ARQ.
            let samples_bad =
                delta.tx_count < expected_tx / 2 || delta.address_events < expected_rx / 2;
            let slots_bad = delta.slots
                < self
                    .cadence_runtime
                    .policy
                    .probe_superframes
                    .saturating_sub(1) as u32
                    * period;
            let local_bad = delta.op_late != 0
                || timing_bad
                || samples_bad
                || slots_bad
                || self.delivery_failures != self.cadence_runtime.delivery_failures_start;
            let crc_bad = delta.crc_bad_long != 0;
            let completed = self.cadence_runtime.policy.probe_superframes;
            self.cadence_runtime.local_metrics = Some(if self.state.central {
                ProbeMetrics::new(
                    completed,
                    u16::from(local_bad),
                    u16::from(local_bad || crc_bad),
                )
            } else {
                ProbeMetrics::new(
                    completed,
                    u16::from(local_bad || crc_bad),
                    u16::from(local_bad),
                )
            });
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
            let app_len = if central {
                self.cadence_runtime.contract.forward_max_payload
            } else {
                self.cadence_runtime.contract.reverse_max_payload
            } as usize;
            let target_wire = app_len.saturating_add(CADENCE_PROTOCOL_OVERHEAD as usize);
            let mut padding = Vec::<u8, { MAX_PAYLOAD + 16 }>::new();
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
            && self.cadence_runtime.error == Some(CadenceError::PeerRejected)
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
        if self.cadence_runtime.releasing {
            flags |= CADENCE_FLAG_RELEASE;
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
            forward_payload: self.cadence_runtime.contract.forward_max_payload as u8,
            reverse_payload: self.cadence_runtime.contract.reverse_max_payload as u8,
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
            probe_slots: self.cadence_runtime.policy.probe_superframes,
            flags,
        })
    }

    fn beacon_packet(&self, step: u32, period: u16, beacon_epoch: u32) -> Packet {
        Packet::Beacon {
            epoch: beacon_epoch,
            channel_index: self.state.scheduler.index(),
            flags: (self.phy.rx_window_us() / 16).min(255) as u8,
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
                let max_payload = if self.state.central {
                    contract.forward_max_payload
                } else {
                    contract.reverse_max_payload
                } as usize;
                if data.len() > max_payload {
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
    fn apply_ack_nack(&mut self, ack: u16, nack: &[u8]) {
        self.state.lm.tx.on_ack(ack);
        self.state
            .lm
            .tx
            .on_nack_slots(nack, &self.state.lm.tx_run_slots);
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
    fn local_ratio(&self) -> (u8, u8) {
        if self.state.central {
            self.state.tx_rx_ratio
        } else {
            self.state.reverse_tx_rx_ratio
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
            CadenceRunStage::Idle | CadenceRunStage::Stable
        ) {
            self.state.lm.tx.tick();
        }
        if !self.state.lm.tx.is_full() {
            self.tx_space.signal(());
        }
        if self.phy.op_pipelined() {
            return self.frame_pipelined(tx_payload, rx_buf).await;
        }

        let (c_tx, c_rx) = self.state.tx_rx_ratio;
        let period = c_tx as u16 + c_rx as u16 + self.state.idle_slots as u16;
        let hw_slot = self.phy.slot_count();
        let phase = self.state.next_phase(hw_slot, period as u32);
        if hw_slot == 0 {
            self.state.slot_step = self.state.slot_step.wrapping_add(1);
        }
        let (local_tx, local_rx) = self.local_ratio();
        let local_phase = self.to_local_phase(phase);
        let central = self.state.central;
        let central_is_tx = phase < c_tx as u32;
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
            let slotrequest =
                !central && min_slot_us > 0 && (!self.cadence_ok || !self.state.lm.rx.have);
            let cadence_pending = central && !self.cadence_negotiated && min_slot_us > 0;
            let profile_countdown = self.cadence_runtime.stage != CadenceRunStage::Commit
                && self.cadence_id != 0
                && self.cadence_apply_epoch != 0
                && (self.state.epoch.wrapping_sub(self.cadence_apply_epoch) as i32) < 0;
            let forced_beacon =
                central && (self.state.epoch % 64 == 0 || cadence_pending || profile_countdown);

            // Start of a new local TX run: clear the slot-position table.
            if local_phase == 0 {
                self.state.lm.tx_run_slots = [None; WINDOW_SIZE];
            }

            if slotrequest {
                // Peripheral acquisition: this slot carries our minimum
                // cadence instead of data, with the TX delay swept.
                self.phy.set_tx_delay_sweep(true);
                let outbound = Packet::SlotRequest {
                    min_slot_us,
                    min_short_slot_us: self.phy.min_short_slot_period_us(),
                    cadence_ack: self.cadence_ack,
                    ack: self.state.lm.rx.ack(),
                };
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
                    let outbound = self.beacon_packet(phase, period, self.state.epoch);
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
                        self.beacon_packet(phase, period, self.state.epoch)
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

            let reply = Packet::from_bytes(&self.rx_pkt_buf[..reply_len])
                .map_err(|_| Error::InvalidPacket)?;
            let out = self
                .handle_rx_packet(
                    reply,
                    local_phase,
                    period as u32,
                    hw_slot.wrapping_add(1),
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
        self.state.on_miss(&mut self.consecutive_misses);
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
        let (c_tx, c_rx) = self.state.tx_rx_ratio;
        let period = c_tx as u16 + c_rx as u16 + self.state.idle_slots as u16;
        self.state
            .next_phase(self.phy.slot_count().wrapping_sub(1), period as u32)
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
                        CadenceRunStage::Stable | CadenceRunStage::Failed
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
                ) =>
            {
                if matches!(
                    self.cadence_runtime.stage,
                    CadenceRunStage::Stable | CadenceRunStage::Failed
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
                ) =>
            {
                let contract = TrafficContract::new(forward_payload as u16, reverse_payload as u16);
                let policy = CadenceProbePolicy::new(
                    min_slot_us,
                    step_us as u16,
                    probe_slots,
                    safety_steps as u16,
                );
                let stable = self.cadence_runtime.stable;
                match CadenceSearch::new(stable, contract, policy, CADENCE_PROTOCOL_OVERHEAD) {
                    Ok(search) => {
                        self.cadence_runtime = CadenceRuntime::new(stable);
                        self.cadence_runtime.generation = generation;
                        self.cadence_id = generation;
                        self.cadence_ack = 0;
                        self.cadence_runtime.contract = contract;
                        self.cadence_runtime.policy = policy;
                        self.cadence_runtime.candidate = search.next_probe().unwrap_or(stable);
                        self.cadence_runtime.search = Some(search);
                        self.cadence_runtime.stage = CadenceRunStage::Offer;
                    }
                    Err(e) => {
                        self.cadence_runtime.error = Some(e);
                        self.cadence_runtime.stage = CadenceRunStage::Failed;
                    }
                }
            }
            (false, CadenceStage::Offer)
                if matches!(
                    self.cadence_runtime.stage,
                    CadenceRunStage::Idle
                        | CadenceRunStage::Request
                        | CadenceRunStage::Stable
                        | CadenceRunStage::Failed
                        | CadenceRunStage::Accept
                ) =>
            {
                if forward_payload as usize > MAX_PAYLOAD
                    || reverse_payload as usize > MAX_PAYLOAD
                    || short_us < self.phy.min_probe_short_slot_period_us()
                    || long_us < self.phy.min_long_slot_period_us()
                    || long_us < short_us
                    || step_us == 0
                    || probe_slots == 0
                {
                    self.cadence_runtime.generation = generation;
                    self.cadence_runtime.error = Some(CadenceError::PeerRejected);
                    self.cadence_runtime.stage = CadenceRunStage::Failed;
                    return;
                }
                self.cadence_runtime.generation = generation;
                self.cadence_id = generation;
                self.cadence_ack = 0;
                self.cadence_runtime.contract =
                    TrafficContract::new(forward_payload as u16, reverse_payload as u16);
                self.cadence_runtime.policy = CadenceProbePolicy::new(
                    min_slot_us,
                    step_us as u16,
                    probe_slots,
                    safety_steps as u16,
                );
                self.cadence_runtime.candidate = CadenceProfile::new(
                    short_us,
                    long_us,
                    self.cadence_runtime.stable.forward_slots,
                    self.cadence_runtime.stable.reverse_slots,
                    self.cadence_runtime.stable.idle_slots,
                );
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
                } else if self
                    .cadence_runtime
                    .search
                    .as_ref()
                    .and_then(CadenceSearch::next_probe)
                    .is_some()
                {
                    self.start_probe_plan(catch_slot, period);
                } else {
                    self.start_final_commit(catch_slot, period, self.cadence_runtime.stable);
                }
            }
            (false, CadenceStage::Probe)
                if generation == self.cadence_runtime.generation
                    && matches!(
                        self.cadence_runtime.stage,
                        CadenceRunStage::Accept | CadenceRunStage::Armed
                    ) =>
            {
                let start_delta = start_epoch.wrapping_sub(epoch) as i32;
                let end_delta = end_epoch.wrapping_sub(epoch) as i32;
                if start_delta > 0 && end_delta > start_delta {
                    let local_start = catch_slot.wrapping_add(start_delta as u32);
                    let local_end = catch_slot.wrapping_add(end_delta as u32);
                    self.cadence_runtime.central_start = start_epoch;
                    self.cadence_runtime.central_end = end_epoch;
                    self.cadence_runtime.local_start = local_start;
                    self.cadence_runtime.local_end = local_end;
                    self.cadence_runtime.probe_started = false;
                    self.cadence_runtime.local_metrics = None;
                    self.cadence_runtime.candidate = CadenceProfile::new(
                        short_us,
                        long_us,
                        self.cadence_runtime.stable.forward_slots,
                        self.cadence_runtime.stable.reverse_slots,
                        self.cadence_runtime.stable.idle_slots,
                    );
                    let p = self.cadence_runtime.candidate;
                    self.phy.schedule_slot_probe(
                        p.short_slot_us,
                        p.long_slot_us,
                        p.period_slots() as u16,
                        p.forward_slots,
                        (self.state.slot_offset % period.max(1)) as u16,
                        local_start,
                        local_end,
                    );
                    self.cadence_runtime.stage = CadenceRunStage::Armed;
                }
            }
            (true, CadenceStage::Armed)
                if generation == self.cadence_runtime.generation
                    && start_epoch == self.cadence_runtime.central_start
                    && end_epoch == self.cadence_runtime.central_end =>
            {
                self.cadence_runtime.stage = CadenceRunStage::Probing;
            }
            (true, CadenceStage::Report)
                if generation == self.cadence_runtime.generation
                    && self.cadence_runtime.stage == CadenceRunStage::Probing
                    && short_us == self.cadence_runtime.candidate.short_slot_us =>
            {
                let failures = u16::from(flags & CADENCE_FLAG_STABLE == 0);
                self.cadence_runtime.peer_metrics = Some(ProbeMetrics::new(
                    self.cadence_runtime.policy.probe_superframes,
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
                    );
                    let profile_changed = self.cadence_runtime.commit_changes_profile
                        || profile != self.cadence_runtime.stable;
                    self.cadence_runtime.commit_changes_profile = profile_changed;
                    if profile_changed
                        && !self.phy.schedule_probed_slot_profile(
                            profile.short_slot_us,
                            profile.long_slot_us,
                            profile.period_slots() as u16,
                            profile.forward_slots,
                            (self.state.slot_offset % period.max(1)) as u16,
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
                        self.cadence_runtime.stable.forward_slots,
                        0,
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
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        let central = self.state.central;
        self.missed_frames = 0;
        let (local_tx, local_rx) = self.local_ratio();
        let active_end = local_tx as u32 + local_rx as u32;
        let slot_idx = (local_phase - local_tx as u32) as usize;
        let rx_run_len = local_rx as usize;
        let rx_run_end = active_end - 1;
        nack_set(&mut self.state.lm.rx_run_mask, slot_idx);

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
                self.apply_ack_nack(ack, &nack);
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
                self.apply_ack_nack(ack, &nack);
            }
            Packet::Drop { seq, ack, nack } => {
                link_rx = true;
                data_plane_rx = true;
                self.state.lm.rx.skip_to(seq);
                self.apply_ack_nack(ack, &nack);
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
                self.apply_ack_nack(ack, &nack);
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
                    self.cadence_runtime.contract.forward_max_payload as u8,
                    self.cadence_runtime.contract.reverse_max_payload as u8,
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
                    && (self.cadence_active_contract.is_some()
                        || self.cadence_runtime.releasing)
                {
                    // The peer independently entered uniform acquisition
                    // fallback after severe loss. Join it before processing
                    // this SlotRequest so both counters regain one wall rate.
                    self.emergency_cadence_fallback();
                }
                // The acquiring peer's cumulative ACK advances the central's
                // TX window from liveness traffic itself.
                self.apply_ack_nack(ack, &[0; NACK_BYTES]);
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
                    // One successful reverse SlotRequest is enough to arm
                    // the profile. The apply boundary is 16 superframes in
                    // the future; until then every central TX slot carries
                    // the armed beacon (~128 delivery opportunities at 8:2).
                    // Waiting for a second/third SR acknowledgement made the
                    // fragile acquisition path itself the handshake.
                    if self.cadence_apply_epoch == 0 {
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
                        self.phy.schedule_slot_profile(
                            self.cadence_short_us,
                            self.cadence_long_us,
                            period as u16,
                            self.cadence_short_phases,
                            0,
                            self.cadence_apply_epoch,
                        );
                        self.cadence_negotiated = true;
                        self.cadence_runtime.stable.short_slot_us = self.cadence_short_us;
                        self.cadence_runtime.stable.long_slot_us = self.cadence_long_us;
                        self.cadence_safe_profile = self.cadence_runtime.stable;
                    }
                    let _ = cadence_ack; // diagnostic/forward-compatible ACK
                }
            }
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
                self.state.scheduler.sync(channel_index);
                if flags > 0 {
                    self.phy.set_peer_rx_window(flags as u16 * 16);
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
                // Uniform 600-us acquisition alignment. Once an armed mixed
                // profile has been scheduled, later beacons must not clear it
                // by re-applying the uniform cadence.
                if slot_us > 0 && self.cadence_ack & 0x80 == 0 {
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
                    // Once this generation's armed epoch was scheduled, keep
                    // the high bit across later beacons. After the epoch is
                    // in the past it is no longer schedulable, but clearing
                    // the bit would make the next beacon call uniform
                    // align_slot_period(600) and silently disable the active
                    // mixed profile on the follower only.
                    if self.cadence_ack & 0x80 == 0 {
                        self.cadence_ack = cadence_id;
                    }
                    if cadence_apply_epoch != 0
                        && catch_slot != 0
                        && self.state.beacon_anchor_ready
                        && (self.cadence_ack & 0x80 == 0
                            || self.cadence_apply_epoch != cadence_apply_epoch)
                    {
                        let delta = cadence_apply_epoch.wrapping_sub(beacon_epoch) as i32;
                        if delta > 0 {
                            let local_apply = catch_slot.wrapping_add(delta as u32);
                            self.phy.schedule_slot_profile(
                                self.cadence_short_us,
                                self.cadence_long_us,
                                period as u16,
                                self.cadence_short_phases,
                                (self.state.slot_offset % period.max(1)) as u16,
                                local_apply,
                            );
                            self.cadence_apply_epoch = cadence_apply_epoch;
                            self.cadence_ack = cadence_id | 0x80;
                            self.cadence_ok = true;
                            self.cadence_runtime.stable.short_slot_us = self.cadence_short_us;
                            self.cadence_runtime.stable.long_slot_us = self.cadence_long_us;
                            self.cadence_safe_profile = self.cadence_runtime.stable;
                        }
                    }
                }
            }
            _ => {}
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
        if link_rx {
            self.state.on_rx(&mut self.consecutive_misses);
        } else {
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
        let (c_tx, c_rx) = self.state.tx_rx_ratio;
        let period = (c_tx as u16 + c_rx as u16 + self.state.idle_slots as u16) as u32;
        let hw_slot = self.phy.slot_count();
        self.cadence_tick(hw_slot, period);
        let collect_slot = hw_slot.wrapping_add(1);
        let target = hw_slot.wrapping_add(2);
        // next_phase applies the follower's mirror offset (slot_offset);
        // target % period alone would ignore the re-anchor and scramble
        // the peripheral's TX/RX phase decisions.
        let phase = self.state.next_phase(target.wrapping_sub(1), period);
        let (local_tx, local_rx) = self.local_ratio();
        let local_phase = self.to_local_phase(phase);
        let central = self.state.central;
        let central_is_tx = phase < c_tx as u32;
        let acquiring = !central && (!self.cadence_ok || !self.state.lm.rx.have);
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
            let slotrequest =
                !central && min_slot_us > 0 && (!self.cadence_ok || !self.state.lm.rx.have);
            let cadence_pending = central && !self.cadence_negotiated && min_slot_us > 0;
            let profile_countdown = self.cadence_runtime.stage != CadenceRunStage::Commit
                && self.cadence_id != 0
                && self.cadence_apply_epoch != 0
                && (target.wrapping_sub(self.cadence_apply_epoch) as i32) < 0;
            let forced_beacon =
                central && (target % 64 == 0 || cadence_pending || profile_countdown);

            // Start of a new local TX run: clear the slot-position table.
            if local_phase == 0 {
                self.state.lm.tx_run_slots = [None; WINDOW_SIZE];
            }

            if slotrequest {
                // Peripheral acquisition: this slot carries our minimum
                // cadence instead of data, with the TX delay swept.
                self.phy.set_tx_delay_sweep(true);
                let outbound = Packet::SlotRequest {
                    min_slot_us,
                    min_short_slot_us: self.phy.min_short_slot_period_us(),
                    cadence_ack: self.cadence_ack,
                    ack: self.state.lm.rx.ack(),
                };
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
                    offer_rejected = self.enqueue_offer(tx_payload)?;
                    let picked = self.pick_data_seq();
                    let outbound =
                        if let Some(control) = self.cadence_packet(target, local_rx as usize) {
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
        } else if local_phase < active_end {
            // RX slot: content-free, publish into this slot's parity buffer
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
        let c_central_is_tx = c_phase < c_tx as u32;
        let c_listen =
            c_central_is_tx || (acquiring && !self.state.beacon_anchor_ready && c_phase % 2 == 0);
        let c_is_tx = if central {
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
                let reply = Packet::from_bytes(&buf[..n]).map_err(|_| Error::InvalidPacket)?;
                self.handle_rx_packet(reply, c_local_phase, period, collect_slot, rx_buf)
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

    /// Start an API-triggered traffic-contract negotiation and bounded slot
    /// probe. The normal `frame()` loop drives Offer/Accept/Probe/Report and
    /// final Commit packets; no packet length changes cadence automatically.
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
        assert!(!cadence_exit_triggered(CadenceExitPolicy::default(), 99, 99));
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
