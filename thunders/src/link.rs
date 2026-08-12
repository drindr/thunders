//! Link-layer state machines for central and peripheral.

use embassy_time::Duration;
use heapless::Vec;

use crate::{
    config::{
        Config, CENTRAL_REPLY_TIMEOUT_US, MAX_PAYLOAD, PERIPHERAL_LISTEN_TIMEOUT_US,
    },
    error::Error,
    packet::Packet,
    phy::Phy,
    scheduler::Scheduler,
};

#[cfg(feature = "secure")]
use crate::security::{make_nonce, Cipher, Security};

/// Size of the ChaCha20-Poly1305 authentication tag.
#[cfg(feature = "secure")]
const TAG_LEN: usize = 16;

/// Shared link state.
struct LinkState {
    scheduler: Scheduler,
    tx_seq: u8,
    rx_seq: u8,
    epoch: u32,
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
            #[cfg(feature = "secure")]
            cipher: cfg.security.as_ref().map(Security::cipher),
        }
    }

    /// Encrypt a `Data` payload in place before transmission.
    ///
    /// `sender_central` is `true` when *this* node is encrypting the
    /// outbound payload (central-to-peripheral direction).
    #[cfg(feature = "secure")]
    fn encrypt_payload<E>(
        &self,
        payload: &mut Vec<u8, MAX_PAYLOAD>,
        sender_central: bool,
    ) -> Result<(), Error<E>> {
        let cipher = match self.cipher.as_ref() {
            Some(c) => c,
            None => return Ok(()),
        };
        if payload.len() + TAG_LEN > MAX_PAYLOAD {
            return Err(Error::BufferTooSmall);
        }
        let nonce = make_nonce(self.epoch, self.tx_seq, self.scheduler.index(), sender_central);
        cipher.encrypt(payload, &nonce)?;
        Ok(())
    }

    #[cfg(not(feature = "secure"))]
    fn encrypt_payload<E>(
        &self,
        _payload: &mut Vec<u8, MAX_PAYLOAD>,
        _sender_central: bool,
    ) -> Result<(), Error<E>> {
        Ok(())
    }

    /// Decrypt a received `Data` payload in place.
    ///
    /// `sender_central` is `true` when the *remote* sender was the central.
    #[cfg(feature = "secure")]
    fn decrypt_payload<E>(
        &self,
        payload: &mut Vec<u8, MAX_PAYLOAD>,
        seq: u8,
        sender_central: bool,
    ) -> Result<(), Error<E>> {
        let cipher = match self.cipher.as_ref() {
            Some(c) => c,
            None => return Ok(()),
        };
        let nonce = make_nonce(self.epoch, seq, self.scheduler.index(), sender_central);
        cipher.decrypt(payload, &nonce)?;
        Ok(())
    }

    #[cfg(not(feature = "secure"))]
    fn decrypt_payload<E>(
        &self,
        _payload: &mut Vec<u8, MAX_PAYLOAD>,
        _seq: u8,
        _sender_central: bool,
    ) -> Result<(), Error<E>> {
        Ok(())
    }
}

/// Central node.
pub struct Central<P: Phy> {
    phy: P,
    state: LinkState,
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
            tx_buf: [0u8; MAX_PAYLOAD + 16],
            rx_pkt_buf: [0u8; MAX_PAYLOAD + 16],
        })
    }

    /// Run one 1 ms superframe from the central side.
    ///
    /// `tx_payload` is data the central wishes to send to the
    /// peripheral.  On success, returns the number of bytes received
    /// from the peripheral (or `None` if the peripheral sent an empty
    /// reply).
    pub async fn frame(
        &mut self,
        tx_payload: Option<&[u8]>,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        self.phy
            .set_channel(self.state.scheduler.current())
            .await;

        // Build the outbound packet.
        let outbound = if let Some(data) = tx_payload {
            let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
            if data.len() > payload.capacity() {
                return Err(Error::BufferTooSmall);
            }
            payload.extend_from_slice(data).map_err(|_| Error::BufferTooSmall)?;
            self.state.encrypt_payload(&mut payload, true)?;
            Packet::Data {
                seq: self.state.tx_seq,
                payload,
            }
        } else {
            Packet::Beacon {
                epoch: self.state.epoch,
                channel_index: self.state.scheduler.index(),
                flags: 0,
            }
        };
        // The tx_seq is the DATA packet counter. Beacon frames do not consume
        // it: on free-running radios (MPSL chains) the peer replies to every
        // beacon, and advancing tx_seq on those replies lets it run far ahead
        // of the peer's rx_seq, permanently desyncing the accept window.
        let sent_data = tx_payload.is_some();

        let n = outbound
            .to_bytes(&mut self.tx_buf)
            .map_err(Error::<P::Error>::from)?;
        self.phy.transmit(&self.tx_buf[..n]).await?;

        // Wait for a reply.
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
                // No reply this frame; still advance the hop sequence.
                self.state.scheduler.advance();
                self.state.epoch = self.state.epoch.wrapping_add(1);
                return Ok(None);
            }
        };

        let reply = Packet::from_bytes(&self.rx_pkt_buf[..reply_len])
            .map_err(|_| Error::InvalidPacket)?;

        let mut received = None;
        match reply {
            Packet::Data { seq, mut payload } => {
                self.state.decrypt_payload(&mut payload, seq, false)?;
                if self.accept_seq(seq) {
                    let len = payload.len();
                    if len > rx_buf.len() {
                        return Err(Error::BufferTooSmall);
                    }
                    rx_buf[..len].copy_from_slice(&payload);
                    received = Some(len);
                    self.state.rx_seq = seq;
                }
                if sent_data {
                    self.state.tx_seq = self.state.tx_seq.wrapping_add(1);
                }
            }
            _ => {}
        }

        self.state.scheduler.advance();
        self.state.epoch = self.state.epoch.wrapping_add(1);
        Ok(received)
    }

    fn accept_seq(&self, seq: u8) -> bool {
        // Anti-replay with a small forward window: accept seqs within a few
        // of the last one instead of exactly rx_seq+1, so the two nodes'
        // counters can drift without permanently locking out the link.
        // The first-ever accept (rx_seq still at its initial 0 and never
        // advanced) syncs to ANY seq: on free-running radios (MPSL chains)
        // the peer's tx_seq can run far ahead before the first packet is
        // accepted, and locking onto that first seq is what lets the link
        // establish.
        let diff = seq.wrapping_sub(self.state.rx_seq);
        diff <= 8 || self.state.rx_seq == 0
    }
}

/// Peripheral node.
pub struct Peripheral<P: Phy> {
    phy: P,
    state: LinkState,
    missed_frames: u8,
    /// Last frame catch time (the phase-lock reference for the sync PLL).
    last_catch: Option<embassy_time::Instant>,
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
            last_catch: None,
            tx_buf: [0u8; MAX_PAYLOAD + 16],
            rx_pkt_buf: [0u8; MAX_PAYLOAD + 16],
        })
    }

    /// Run one 1 ms superframe from the peripheral side.
    ///
    /// `tx_payload` is data the peripheral wishes to send to the
    /// central.  On success, returns the number of bytes received from
    /// the central.
    pub async fn frame(
        &mut self,
        tx_payload: Option<&[u8]>,
        rx_buf: &mut [u8],
    ) -> Result<Option<usize>, Error<P::Error>> {
        self.phy
            .set_channel(self.state.scheduler.current())
            .await;

        let incoming_len = match self
            .phy
            .receive(
                &mut self.rx_pkt_buf,
                Duration::from_micros(PERIPHERAL_LISTEN_TIMEOUT_US),
            )
            .await?
        {
            Some(len) => {
                // "Connection formed" sync: phase-lock our chain to the
                // central's. The catch-to-catch interval is the central's
                // 1000 us period plus our own phase drift; nudge the phy's
                // period to cancel it (bang-bang, +/-1 us per catch).
                let now = embassy_time::Instant::now();
                if let Some(prev) = self.last_catch {
                    let d = (now - prev).as_micros() as i32;
                    let corr = if d > 1000 { -1 } else if d < 1000 { 1 } else { 0 };
                    if corr != 0 {
                        self.phy.adjust_period(corr).await;
                    }
                }
                self.last_catch = Some(now);
                len
            }
            None => {
                self.missed_frames = self.missed_frames.saturating_add(1);
                self.state.scheduler.advance();
                return Ok(None);
            }
        };

        let incoming = Packet::from_bytes(&self.rx_pkt_buf[..incoming_len])
            .map_err(|_| Error::InvalidPacket)?;

        self.missed_frames = 0;

        // Extract any data from the central and build the reply.
        let mut received = None;
        let reply = match incoming {
            Packet::Data { seq, mut payload } => {
                self.state.decrypt_payload(&mut payload, seq, true)?;
                if self.accept_seq(seq) {
                    let len = payload.len();
                    if len > rx_buf.len() {
                        return Err(Error::BufferTooSmall);
                    }
                    rx_buf[..len].copy_from_slice(&payload);
                    received = Some(len);
                    self.state.rx_seq = seq;
                }
                self.make_data_reply(tx_payload)?
            }
            Packet::Beacon { channel_index, .. } => {
                self.state.scheduler.sync(channel_index);
                self.make_data_reply(tx_payload)?
            }
            _ => self.make_data_reply(tx_payload)?,
        };

        let n = reply
            .to_bytes(&mut self.tx_buf)
            .map_err(Error::<P::Error>::from)?;
        self.phy.transmit(&self.tx_buf[..n]).await?;

        self.state.tx_seq = self.state.tx_seq.wrapping_add(1);
        self.state.scheduler.advance();
        self.state.epoch = self.state.epoch.wrapping_add(1);
        Ok(received)
    }

    fn make_data_reply(
        &mut self,
        tx_payload: Option<&[u8]>,
    ) -> Result<Packet, Error<P::Error>> {
        let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
        if let Some(data) = tx_payload {
            if data.len() > payload.capacity() {
                return Err(Error::BufferTooSmall);
            }
            payload.extend_from_slice(data).map_err(|_| Error::BufferTooSmall)?;
            self.state.encrypt_payload(&mut payload, false)?;
        }
        Ok(Packet::Data {
            seq: self.state.tx_seq,
            payload,
        })
    }

    fn handle_ack(&mut self, ack: u8) {
        let _ = ack;
    }

    fn accept_seq(&self, seq: u8) -> bool {
        let diff = seq.wrapping_sub(self.state.rx_seq);
        diff <= 8
    }
}
