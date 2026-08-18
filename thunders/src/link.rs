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
    config::{
        Config, Role, CENTRAL_REPLY_TIMEOUT_US, MAX_PAYLOAD, NACK_BYTES,
        PERIPHERAL_LISTEN_TIMEOUT_US, WINDOW_SIZE,
    },
    error::Error,
    link_mgmt::{
        nack_from_mask, nack_nonzero, nack_set, nack_vec, seq_gt, LinkMgmt, RxWindow, TxRunSlot,
    },
    packet::Packet,
    phy::Phy,
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
    /// The previous beacon's (phase, catch_slot) for the differential
    /// anchor update (which measures the count lag accumulated between
    /// beacons - see the beacon arm).
    beacon_anchor_prev: Option<(u32, u32)>,
    /// True once the absolute first anchor was adopted (the differential
    /// form then maintains it).
    beacon_anchor_abs: bool,
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
            beacon_anchor_prev: None,
            beacon_anchor_abs: false,
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

/// Build the beacon packet for a TX slot.
fn make_beacon<P: Phy>(state: &LinkState, phy: &P, step: u32, period: u16) -> Packet {
    Packet::Beacon {
        epoch: state.epoch,
        channel_index: state.scheduler.index(),
        flags: (phy.rx_window_us() / 16).min(255) as u8,
        slot_us: phy.slot_period_us(),
        slot_phase: (step % period as u32) as u16,
        rx_en_offset: phy.rx_en_offset_us(),
        tx_en_offset: phy.tx_en_offset_us(),
        rx_ramp: phy.rx_ramp_us(),
        tx_ramp: phy.tx_ramp_us(),
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
    /// False until the central's beacon advertises a cadence >= our minimum
    /// slot period. Peripheral-only; the central ignores this field.
    cadence_ok: bool,
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
    tx_buf: [u8; MAX_PAYLOAD + 16],
    rx_pkt_buf: [u8; MAX_PAYLOAD + 16],
    /// Second RX buffer for the pipelined phy path (the parity
    /// double-buffer: the next op's DMA target while the previous catch
    /// is being processed).
    rx_pkt_buf2: [u8; MAX_PAYLOAD + 16],
}

impl<P: Phy> LinkCore<P> {
    async fn new(mut phy: P, cfg: Config) -> Result<Self, Error<P::Error>> {
        phy.set_address(&cfg.address).await;
        phy.flush().await;
        Ok(Self {
            cadence_negotiated: false,
            cadence_ok: phy.min_slot_period_us() == 0,
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
            tx_buf: [0u8; MAX_PAYLOAD + 16],
            rx_pkt_buf: [0u8; MAX_PAYLOAD + 16],
            rx_pkt_buf2: [0u8; MAX_PAYLOAD + 16],
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

    fn beacon_packet(&self, step: u32, period: u16) -> Packet {
        make_beacon(&self.state, &self.phy, step, period)
    }

    /// Enqueue a raw `frame` offer (or the pending typed-send payload).
    /// Returns `true` when a raw offer found the window full.
    fn enqueue_offer(
        &mut self,
        tx_payload: Option<&[u8]>,
    ) -> Result<bool, Error<P::Error>> {
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

        self.state.lm.tx.tick();
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
        let acquiring =
            !central && hw_slot != 0 && (!self.cadence_ok || !self.state.lm.rx.have);
        let listen = central_is_tx || (acquiring && phase % 2 == 0);
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
                && (!self.cadence_ok || !self.state.lm.rx.have);
            let cadence_pending =
                central && !self.cadence_negotiated && min_slot_us > 0;
            let forced_beacon = central && (self.state.epoch % 64 == 0 || cadence_pending);

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
                    let outbound = self.beacon_packet(phase, period);
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
                        self.beacon_packet(phase, period)
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
                .handle_rx_packet(reply, local_phase, period as u32, hw_slot.wrapping_add(1), rx_buf)
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
        let central = self.state.central;
        if !central {
            self.missed_frames = self.missed_frames.saturating_add(1);
        }
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
        if !central {
            self.missed_frames = 0;
        }
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
                if !central && self.state.status == LinkStatus::Disconnected
                    && !self.state.lm.rx.in_window(seq)
                {
                    self.state.lm.rx.resync(seq);
                    self.resyncs = self.resyncs.wrapping_add(1);
                }
                link_rx = true;
                self.state.lm.rx.receive(seq, &payload);
                self.rx_data = self.rx_data.wrapping_add(1);
            }
            Packet::Ack { ack, nack } => {
                link_rx = true;
                self.apply_ack_nack(ack, &nack);
            }
            Packet::Drop { seq, ack, nack } => {
                link_rx = true;
                self.state.lm.rx.skip_to(seq);
                self.apply_ack_nack(ack, &nack);
            }
            Packet::SlotRequest { min_slot_us, ack } if central => {
                // The acquiring peer's cumulative ACK: lets the central's
                // TX window advance from the liveness traffic itself. An
                // acquiring peer answers only with SlotRequests (no
                // Data/Ack packets), so without the ACK a dropped Data
                // left the central stuck sending Drop packets forever and
                // no new Data ever (the pair deadlocked).
                self.apply_ack_nack(ack, &[0; NACK_BYTES]);
                self.clear_pending_drop();
                if !self.cadence_negotiated {
                    self.cadence_negotiated = true;
                    let negotiated = self.phy.min_slot_period_us().max(min_slot_us).max(1);
                    if negotiated != self.phy.slot_period_us() {
                        self.phy.align_slot_period(negotiated);
                    }
                }
            }
            Packet::Beacon {
                channel_index,
                flags,
                slot_us,
                slot_phase,
                rx_en_offset,
                tx_en_offset,
                rx_ramp,
                tx_ramp,
                ..
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
                if slot_us > 0 {
                    self.phy.align_slot_period(slot_us);
                }
                let min = self.phy.min_slot_period_us();
                self.cadence_ok = min == 0 || slot_us >= min;
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
                        let candidate =
                            (beacon_phase.wrapping_sub(catch_slot % period)) % period;
                        if self.state.beacon_anchor_pending == Some(candidate) {
                            self.state.slot_offset = candidate;
                            self.state.beacon_anchor_pending = None;
                        } else {
                            self.state.beacon_anchor_pending = Some(candidate);
                        }
                    } else {
                        self.state.slot_step = (slot_phase as u32 + 1) % period;
                    }
                }
            }
            _ => {}
        }

        self.clear_pending_drop();
        if link_rx {
            self.state.on_rx(&mut self.consecutive_misses);
        } else {
            self.consecutive_misses = 0;
        }
        if local_phase == rx_run_end {
            self.state.lm.nack_for_peer =
                nack_from_mask(rx_run_len, &self.state.lm.rx_run_mask);
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
        let listen = central_is_tx || (acquiring && phase % 2 == 0);
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
            let grace = if local_phase == 0 && local_tx > 1 { 1 } else { 0 };
            let min_slot_us = self.phy.min_slot_period_us();
            let slotrequest =
                !central && min_slot_us > 0 && (!self.cadence_ok || !self.state.lm.rx.have);
            let cadence_pending = central && !self.cadence_negotiated && min_slot_us > 0;
            let forced_beacon = central && (target % 64 == 0 || cadence_pending);

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
                        let outbound = self.beacon_packet(phase, period as u16);
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
                            self.beacon_packet(phase, period as u16)
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
                    let outbound = if self.pending_drop.is_some() {
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
        let c_listen = c_central_is_tx || (acquiring && c_phase % 2 == 0);
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
                let reply =
                    Packet::from_bytes(&buf[..n]).map_err(|_| Error::InvalidPacket)?;
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

    async fn recv<T: serde::de::DeserializeOwned>(
        &mut self,
    ) -> Result<Option<T>, Error<P::Error>> {
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
    fn local_phase_mapping_without_idle() {
        let mut cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central)
            .with_tx_rx_ratio(8, 2);
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
        let mut cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central)
            .with_tx_rx_idle(8, 4, 4);
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
        let cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central)
            .with_tx_rx_idle(8, 4, 4);
        assert_eq!(cfg.period_slots(), 16);
        assert_eq!(cfg.tx_slots_per_period(Role::Central), 8);
        assert_eq!(cfg.tx_slots_per_period(Role::Peripheral), 4);
    }

    #[test]
    fn config_builder_with_idle_keeps_complement() {
        let cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central)
            .with_tx_rx_idle(6, 2, 2);
        assert_eq!(cfg.tx_rx_ratio, (6, 2));
        assert_eq!(cfg.reverse_tx_rx_ratio, (2, 6));
        assert_eq!(cfg.idle_slots, 2);
    }

    #[test]
    fn config_builder_keeps_ratios_complementary() {
        let cfg = Config::new([0; 4], Address([0xE7; 5]), Role::Central)
            .with_tx_rx_ratio(5, 3);
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
        let mut cfg = Config::new(
            [0xAB, 0xCD, 0xEF, 0x01],
            Address([0xE7; 5]),
            Role::Central,
        );
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
