//! PHY abstraction.

use embassy_time::Duration;

use crate::{config::Address, error::Error};

/// Async interface to a raw radio transceiver.
#[allow(async_fn_in_trait)]
///
/// The link layer owns a type implementing `Phy`.  Implementations are
/// responsible for channel setup, address filtering, CRC, and
/// TX/RX turnaround.
pub trait Phy {
    /// PHY-specific error type.
    type Error;

    /// Set the radio channel.
    ///
    /// The meaning of `ch` is PHY-specific.  For Nordic RADIO it is a
    /// 1 MHz offset from 2400 MHz; for nRF24L01+ it is the raw channel
    /// number 0..125.
    async fn set_channel(&mut self, ch: u8);

    /// Set the receive address.
    async fn set_address(&mut self, addr: &Address);

    /// Transmit a raw packet.
    async fn transmit(&mut self, pkt: &[u8],
    ) -> Result<(), Error<Self::Error>>;

    /// Wait for a raw packet up to `timeout`.
    ///
    /// Returns `Some(n)` when a packet of `n` bytes was written into
    /// `buf`, or `None` when the timeout expired without a packet.
    async fn receive(
        &mut self,
        buf: &mut [u8],
        timeout: Duration,
    ) -> Result<Option<usize>, Error<Self::Error>>;

    /// The receiver's listen window inside a slot, in microseconds
    /// (0 = unknown). Advertised in the beacon so the peer can align its
    /// transmissions to this (possibly poorer) window.
    fn rx_window_us(&self) -> u16 {
        0
    }

    /// The peer's advertised RX window (see [`rx_window_us`](Self::rx_window_us)).
    fn set_peer_rx_window(&mut self, _us: u16) {}

    /// The sender's slot cadence in us, advertised in the beacon
    /// (0 = unknown).
    fn slot_period_us(&self) -> u16 {
        0
    }

    /// The minimum slot period this PHY can physically sustain, in us.
    /// The central adopts `max(current, peer_min)` so a fast central cannot
    /// starve a slower peripheral.
    fn min_slot_period_us(&self) -> u16 {
        0
    }

    /// Adopt the master's advertised cadence at runtime (the align
    /// mechanism): no compile-time cadence matching needed.
    fn align_slot_period(&mut self, _us: u16) {}

    /// Flush any stale RX/TX state.
    async fn flush(&mut self);

    /// Wait for the next software slot boundary without a radio op.
    ///
    /// Backends with their own slot cadence (the MPSL timeslot chain) do
    /// nothing: the slot is already paced by hardware. The bare backend uses
    /// this to keep its software slot grid aligned on empty slots (e.g. the
    /// peripheral's TX slot when it has no payload queued), so the follower's
    /// RX grid does not drift one slot earlier per ratio period.
    async fn wait_slot(&mut self) {}

    /// Transmit then immediately listen for a reply as one combined
    /// operation. The default is [`transmit`](Self::transmit) followed by
    /// [`receive`](Self::receive); a backend may override it to keep the
    /// radio armed across the turnaround (one await point instead of two -
    /// the frame's TX+RX costs one executor hop, not two). Returns the
    /// number of reply bytes, or `None` on a receive timeout.
    async fn transmit_receive(
        &mut self,
        tx: &[u8],
        rx: &mut [u8],
        timeout: Duration,
    ) -> Result<Option<usize>, Error<Self::Error>> {
        self.transmit(tx).await?;
        self.receive(rx, timeout).await
    }

    /// Adjust the RX/TX period by `corr` microseconds (sync, no-op by
    /// default). The link layer's peripheral uses this to phase-lock its
    /// schedule to the central's once frames start flowing ("connection
    /// formed"). The bare radio needs no adjustment; the MPSL backend
    /// nudges its chained timeslot distance.
    async fn adjust_period(&mut self, _corr: i32) {}

    /// Begin a TX burst: ramp the radio once, so the following
    /// [`transmit_burst_send`](Self::transmit_burst_send) packets skip the
    /// per-packet ramp (the on-air time only - the 8 kHz one-way path).
    /// Synchronous (no await hop): the burst is the hot loop. Backends
    /// without the chained TX return [`Error::Unsupported`].
    fn transmit_burst_begin(&mut self, _pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        Err(Error::Unsupported)
    }

    /// Send the next packet in a burst (the radio is already ramped - the
    /// on-air only). Synchronous; see [`transmit_burst_begin`](Self::transmit_burst_begin).
    fn transmit_burst_send(&mut self, _pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        Err(Error::Unsupported)
    }

    /// Encrypt/decrypt a payload with the hardware AES-CCM (the AEAD: the
    /// counter-mode encryption + the CBC-MAC). `mic` is the 4-byte tag (the
    /// output on encrypt, the expected on decrypt); the payload is the
    /// in-place ciphertext/plaintext (the MIC is appended after it).
    /// Synchronous. The default returns [`Error::Unsupported`]; the `secure`
    /// feature's CCM mode uses it.
    fn ccm_crypt(
        &mut self,
        _key: &[u8; 16],
        _nonce: &[u8; 13],
        _payload: &mut [u8],
        _mic: &mut [u8; 4],
        _encrypt: bool,
    ) -> Result<(), Error<Self::Error>> {
        Err(Error::Unsupported)
    }
}
