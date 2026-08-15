//! Link-layer state machines for central and peripheral.

/// Consecutive failed frames before the adaptive hop advances the channel
/// (transient misses stay put; persistent jamming hops away).
const HOP_MISS_THRESHOLD: u8 = 16;

/// Consecutive failed frames before the link is declared lost: the status
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
/// first successful receive forms the connection and enables the adaptive
/// hop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinkStatus {
    /// No packet received yet, or the link was lost. Hop is disabled — the
    /// node holds the initial channel.
    Disconnected,
    /// A packet was received. Adaptive channel hopping is enabled.
    Connected,
}

/// Frame-phase profile accumulators (us per 1000 frames; diagnostic).
pub static PROFILE_CH: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static PROFILE_TX: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static PROFILE_RX: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static PROFILE_PARSE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static PROFILE_N: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

use embassy_time::Duration;
use heapless::Vec;

use crate::{
    config::{CENTRAL_REPLY_TIMEOUT_US, Config, MAX_PAYLOAD, PERIPHERAL_LISTEN_TIMEOUT_US, Role},
    error::Error,
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
    tx_seq: u8,
    rx_seq: u8,
    epoch: u32,
    /// TX:RX slot ratio (the shared schedule).
    tx_rx_ratio: (u8, u8),
    /// The slot step counter (the TX/RX decision).
    slot_step: u8,
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
        Self {
            scheduler,
            tx_seq: 0,
            rx_seq: 0,
            epoch: 0,
            tx_rx_ratio: cfg.tx_rx_ratio,
            slot_step: 0,
            status: LinkStatus::Disconnected,
            connect_streak: 0,
            central: matches!(cfg.role, Role::Central),
            initial_channel: cfg.initial_channel,
            #[cfg(feature = "secure")]
            cipher: cfg.security.as_ref().map(Security::cipher),
        }
    }

    /// A missed RX slot. While disconnected the scheduler is pinned to the
    /// initial channel (no hop). Once connected, a short streak hops away
    /// from a jammed channel; a long streak declares the link lost and pins
    /// back to the initial channel so a recovered link can re-align.
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

    /// The seq window: accept `seq` when it is within a few steps of the
    /// last accepted seq, or whenever the link is Disconnected (the
    /// connection-state machine's re-sync signal), or before the first
    /// accept (rx_seq still at its initial 0).
    fn accept_seq(&self, seq: u8) -> bool {
        let diff = seq.wrapping_sub(self.rx_seq);
        diff <= 8 || self.rx_seq == 0 || self.status == LinkStatus::Disconnected
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
    /// outbound payload (central-to-peripheral direction).
    #[cfg(feature = "secure")]
    fn encrypt_payload<P: Phy>(
        &self,
        phy: &mut P,
        payload: &mut Vec<u8, MAX_PAYLOAD>,
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
                let nonce =
                    make_nonce(self.epoch, self.tx_seq, self.scheduler.index(), sender_central);
                cipher.encrypt(payload, &nonce)?;
            }
            CipherMode::Ccm => {
                if payload.len() + 4 > MAX_PAYLOAD {
                    return Err(Error::BufferTooSmall);
                }
                let nonce =
                    make_nonce_13(self.epoch, self.tx_seq, self.scheduler.index(), sender_central);
                let mut key16 = [0u8; 16];
                key16.copy_from_slice(&cipher.key[..16]);
                let mut mic = [0u8; 4];
                phy.ccm_crypt(&key16, &nonce, payload, &mut mic, true)?;
                payload.extend_from_slice(&mic).map_err(|_| Error::BufferTooSmall)?;
            }
        }
        Ok(())
    }

    #[cfg(not(feature = "secure"))]
    fn encrypt_payload<P: Phy>(
        &self,
        _phy: &mut P,
        _payload: &mut Vec<u8, MAX_PAYLOAD>,
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
        seq: u8,
        sender_central: bool,
    ) -> Result<(), Error<P::Error>> {
        let cipher = match self.cipher.as_ref() {
            Some(c) => c,
            None => return Ok(()),
        };
        match cipher.mode {
            CipherMode::ChaCha => {
                let nonce = make_nonce(self.epoch, seq, self.scheduler.index(), sender_central);
                cipher.decrypt(payload, &nonce)?;
            }
            CipherMode::Ccm => {
                if payload.len() < 4 {
                    return Err(Error::InvalidPacket);
                }
                let nonce =
                    make_nonce_13(self.epoch, seq, self.scheduler.index(), sender_central);
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
        _seq: u8,
        _sender_central: bool,
    ) -> Result<(), Error<P::Error>> {
        Ok(())
    }
}

/// Central node.
pub struct Central<P: Phy> {
    phy: P,
    state: LinkState,
    /// Last channel written to the phy (the phy is only re-tuned on change).
    last_channel: Option<u8>,
    /// Consecutive missed replies (the adaptive-hop trigger).
    consecutive_misses: u8,
    /// A TX burst is in progress (the radio stays ramped across TX slots).
    in_burst: bool,
    tx_buf: [u8; MAX_PAYLOAD + 16],
    rx_pkt_buf: [u8; MAX_PAYLOAD + 16],
}

impl<P: Phy> Central<P> {
    /// Create a new central.
    pub async fn new(mut phy: P, cfg: Config) -> Result<Self, Error<P::Error>> {
        phy.set_address(&cfg.address).await;
        phy.flush().await;
        Ok(Self {
            phy,
            state: LinkState::new(&cfg),
            last_channel: None,
            consecutive_misses: 0,
            in_burst: false,
            tx_buf: [0u8; MAX_PAYLOAD + 16],
            rx_pkt_buf: [0u8; MAX_PAYLOAD + 16],
        })
    }

    /// The current connection status (the hop gate).
    pub fn status(&self) -> LinkStatus {
        self.state.status
    }

    /// Run one slot from the central side (the TX:RX ratio drives which).
    ///
    /// The TX slots carry the PING/Data (or the Beacon when `tx_payload` is
    /// `None` - the beacon carries the hop index); the RX slots listen for
    /// the peripheral's reverse Data. The TX run uses the phy's burst (the
    /// ramp once, the on-air per packet) and falls back to the plain
    /// transmit on backends without the burst (the MPSL).
    pub async fn frame(
        &mut self,
        tx_payload: Option<&[u8]>,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        let ch = self.state.scheduler.current();
        if self.last_channel != Some(ch) {
            self.phy.set_channel(ch).await;
            self.last_channel = Some(ch);
        }

        // The slot decision from the TX:RX ratio.
        let (tx_n, rx_n) = self.state.tx_rx_ratio;
        let period = tx_n as u16 + rx_n as u16;
        let step = self.state.slot_step;
        let is_tx = (step as u16 % period) < tx_n as u16;
        self.state.slot_step = step.wrapping_add(1);

        if is_tx {
            // ---- the TX slot: the burst (or the plain transmit fallback) ----
            // Every 64th TX slot is a Beacon even under load: it carries the
            // RX-window advertisement the follower aligns to.
            let outbound = if let Some(data) = tx_payload.filter(|_| self.state.epoch % 64 != 0) {
                let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
                if data.len() > payload.capacity() {
                    return Err(Error::BufferTooSmall);
                }
                payload.extend_from_slice(data).map_err(|_| Error::BufferTooSmall)?;
                self.state.encrypt_payload(&mut self.phy, &mut payload, true)?;
                Packet::Data {
                    seq: self.state.tx_seq,
                    payload,
                }
            } else {
                Packet::Beacon {
                    epoch: self.state.epoch,
                    channel_index: self.state.scheduler.index(),
                    flags: (self.phy.rx_window_us() / 16).min(255) as u8,
                    slot_us: self.phy.slot_period_us(),
                    slot_phase: (step as u16 % period) as u8,
                }
            };
            let sent_data = tx_payload.is_some() && !matches!(outbound, Packet::Beacon { .. });
            let n = outbound
                .to_bytes(&mut self.tx_buf)
                .map_err(Error::<P::Error>::from)?;

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
            if sent_data {
                self.state.tx_seq = self.state.tx_seq.wrapping_add(1);
            }
            self.state.epoch = self.state.epoch.wrapping_add(1);
            Ok(None)
        } else {
            // ---- the RX slot: the reverse listen ----
            self.in_burst = false; // the turnaround ends the TX burst
            let reply_len = match self
                .phy
                .receive(
                    &mut self.rx_pkt_buf,
                    Duration::from_micros(CENTRAL_REPLY_TIMEOUT_US),
                )
                .await?
            {
                Some(len) => len,
                None => {
                    self.state.on_miss(&mut self.consecutive_misses);
                    self.state.epoch = self.state.epoch.wrapping_add(1);
                    return Ok(None);
                }
            };

            let reply = Packet::from_bytes(&self.rx_pkt_buf[..reply_len])
                .map_err(|_| Error::InvalidPacket)?;

            let mut received = None;
            if let Packet::Data { seq, mut payload } = reply {
                self.state.decrypt_payload(&mut self.phy, &mut payload, seq, false)?;
                if self.accept_seq(seq) {
                    let len = payload.len();
                    if len > rx_buf.len() {
                        return Err(Error::BufferTooSmall);
                    }
                    rx_buf[..len].copy_from_slice(&payload);
                    received = Some(len);
                    self.state.rx_seq = seq;
                }
            }

            // A healthy RX slot: form the connection + reset the streak.
            self.state.on_rx(&mut self.consecutive_misses);
            self.state.epoch = self.state.epoch.wrapping_add(1);
            Ok(received)
        }
    }

    /// Send a typed value: the postcard-serialize + the crypto + the radio.
    /// The value is serialized into the TX buffer and handed to the frame;
    /// the TX slots carry it, the RX slots ignore it (the ratio is the
    /// frame's internal schedule).
    pub async fn send<T: serde::Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), Error<P::Error>> {
        let mut buf = [0u8; MAX_PAYLOAD + 16];
        let written = postcard::to_slice(value, &mut buf)
            .map_err(Error::<P::Error>::from)?;
        let n = written.len();
        let mut rx = [0u8; MAX_PAYLOAD];
        self.frame(Some(&buf[..n]), &mut rx).await?;
        Ok(())
    }

    /// Receive a typed value: the radio RX + the crypto + the
    /// postcard-deserialize. Returns `None` when the slot had no frame.
    pub async fn recv<T: serde::de::DeserializeOwned>(
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

    fn accept_seq(&self, seq: u8) -> bool {
        self.state.accept_seq(seq)
    }
}

/// Peripheral node.
pub struct Peripheral<P: Phy> {
    phy: P,
    state: LinkState,
    missed_frames: u8,
    /// Last channel written to the phy (the phy is only re-tuned on change).
    last_channel: Option<u8>,
    /// Consecutive missed frames (the mirror of the central's adaptive-hop
    /// trigger: both sides hop together after the same miss streak, so the
    /// peripheral is not left listening on the old channel).
    consecutive_misses: u8,
    tx_buf: [u8; MAX_PAYLOAD + 16],
    rx_pkt_buf: [u8; MAX_PAYLOAD + 16],
}

impl<P: Phy> Peripheral<P> {
    /// Create a new peripheral.
    pub async fn new(mut phy: P, cfg: Config) -> Result<Self, Error<P::Error>> {
        phy.set_address(&cfg.address).await;
        phy.flush().await;
        Ok(Self {
            phy,
            state: LinkState::new(&cfg),
            missed_frames: 0,
            last_channel: None,
            consecutive_misses: 0,
            tx_buf: [0u8; MAX_PAYLOAD + 16],
            rx_pkt_buf: [0u8; MAX_PAYLOAD + 16],
        })
    }

    /// The current connection status (the hop gate).
    pub fn status(&self) -> LinkStatus {
        self.state.status
    }

    /// Run one slot from the peripheral side (the mirror of the central's
    /// ratio): the central's TX slots are our RX slots (the listen), the
    /// central's RX slots are our TX slots (the reverse Data). No ack - the
    /// seq window is the reliability.
    pub async fn frame(
        &mut self,
        tx_payload: Option<&[u8]>,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        let ch = self.state.scheduler.current();
        if self.last_channel != Some(ch) {
            self.phy.set_channel(ch).await;
            self.last_channel = Some(ch);
        }

        // The mirrored slot decision.
        let (tx_n, rx_n) = self.state.tx_rx_ratio;
        let period = tx_n as u16 + rx_n as u16;
        let central_is_tx = (self.state.slot_step as u16 % period) < tx_n as u16;
        self.state.slot_step = self.state.slot_step.wrapping_add(1);

        if central_is_tx {
            // ---- the central's TX = our RX: the listen ----
            let incoming_len = match self
                .phy
                .receive(
                    &mut self.rx_pkt_buf,
                    Duration::from_micros(PERIPHERAL_LISTEN_TIMEOUT_US),
                )
                .await?
            {
                Some(len) => len,
                None => {
                    self.missed_frames = self.missed_frames.saturating_add(1);
                    self.state.on_miss(&mut self.consecutive_misses);
                    return Ok(None);
                }
            };

            let incoming = Packet::from_bytes(&self.rx_pkt_buf[..incoming_len])
                .map_err(|_| Error::InvalidPacket)?;
            self.missed_frames = 0;

            let mut received = None;
            match incoming {
                Packet::Data { seq, mut payload } => {
                    self.state.decrypt_payload(&mut self.phy, &mut payload, seq, true)?;
                    if self.accept_seq(seq) {
                        let len = payload.len();
                        if len > rx_buf.len() {
                            return Err(Error::BufferTooSmall);
                        }
                        rx_buf[..len].copy_from_slice(&payload);
                        received = Some(len);
                        self.state.rx_seq = seq;
                    }
                }
                Packet::Beacon { channel_index, flags, slot_us, slot_phase, .. } => {
                    // The beacon is the hop authority; it also carries the
                    // central's cadence and RX window: align to them at
                    // runtime (to the poorer side's timing).
                    self.state.scheduler.sync(channel_index);
                    if flags > 0 {
                        self.phy.set_peer_rx_window(flags as u16 * 16);
                    }
                    if slot_us > 0 {
                        self.phy.align_slot_period(slot_us);
                    }
                    // The slot phase: our next slot mirrors the central's
                    // next (its TX/RX ratio decision), so our lone TX slot
                    // lands on its lone RX slot.
                    let period =
                        self.state.tx_rx_ratio.0 as u16 + self.state.tx_rx_ratio.1 as u16;
                    self.state.slot_step = ((slot_phase as u16 + 1) % period) as u8;
                }
                _ => {}
            }
            self.state.on_rx(&mut self.consecutive_misses);
            self.state.epoch = self.state.epoch.wrapping_add(1);
            Ok(received)
        } else {
            // ---- the central's RX = our TX: the reverse Data ----
            if let Some(data) = tx_payload {
                let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
                if data.len() > payload.capacity() {
                    return Err(Error::BufferTooSmall);
                }
                payload.extend_from_slice(data).map_err(|_| Error::BufferTooSmall)?;
                self.state.encrypt_payload(&mut self.phy, &mut payload, false)?;
                let outbound = Packet::Data {
                    seq: self.state.tx_seq,
                    payload,
                };
                let n = outbound
                    .to_bytes(&mut self.tx_buf)
                    .map_err(Error::<P::Error>::from)?;
                self.phy.transmit(&self.tx_buf[..n]).await?;
                self.state.tx_seq = self.state.tx_seq.wrapping_add(1);
            } else {
                // No payload: still pace this slot so the bare software slot
                // grid stays time-aligned with the central (the MPSL chain
                // already paces itself; its wait_slot is a no-op).
                self.phy.wait_slot().await;
            }
            self.state.epoch = self.state.epoch.wrapping_add(1);
            Ok(None)
        }
    }

    /// Send a typed value (the peripheral's reverse Data).
    pub async fn send<T: serde::Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), Error<P::Error>> {
        let mut buf = [0u8; MAX_PAYLOAD + 16];
        let written = postcard::to_slice(value, &mut buf)
            .map_err(Error::<P::Error>::from)?;
        let n = written.len();
        let mut rx = [0u8; MAX_PAYLOAD];
        self.frame(Some(&buf[..n]), &mut rx).await?;
        Ok(())
    }

    /// Receive a typed value (the central's Data).
    pub async fn recv<T: serde::de::DeserializeOwned>(
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

    fn handle_ack(&mut self, ack: u8) {
        let _ = ack;
    }

    fn accept_seq(&self, seq: u8) -> bool {
        self.state.accept_seq(seq)
    }
}
