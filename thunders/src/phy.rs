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
    /// The meaning of `ch` is PHY-specific. For the Nordic RADIO backend it
    /// is the 1 MHz offset from 2400 MHz; other implementations may map it
    /// to a transceiver channel number.
    async fn set_channel(&mut self, ch: u8);

    /// Set the receive address.
    async fn set_address(&mut self, addr: &Address);

    /// Transmit a raw packet.
    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>>;

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

    /// The peer's measured RXEN offset from slot START, in us (0 = unknown).
    fn set_peer_rx_en_offset(&mut self, _us: u8) {}

    /// This node's measured RXEN offset from slot START, in us.
    fn rx_en_offset_us(&self) -> u8 {
        0
    }

    /// This node's measured TXEN offset from slot START, in us.
    fn tx_en_offset_us(&self) -> u8 {
        0
    }

    /// This node's measured RXEN -> READY ramp, in us.
    fn rx_ramp_us(&self) -> u8 {
        0
    }

    /// This node's measured TXEN -> READY ramp, in us.
    fn tx_ramp_us(&self) -> u8 {
        0
    }

    /// The peer's measured TXEN offset from slot START, in us (0 = unknown).
    fn set_peer_tx_en_offset(&mut self, _us: u8) {}

    /// The peer's measured RX ramp, in us (0 = unknown).
    fn set_peer_rx_ramp(&mut self, _us: u8) {}

    /// The peer's measured TX ramp, in us (0 = unknown).
    fn set_peer_tx_ramp(&mut self, _us: u8) {}

    /// Stamp the absolute slot the next `transmit`/`receive` op is meant
    /// for. Backends with a hardware slot counter (MPSL) execute a
    /// published op only in that exact slot; a late publish idles the slot
    /// instead of running the op one slot off its phase. Default no-op
    /// (software-paced PHYs pace their own ops).
    fn set_op_slot(&mut self, _slot: u32) {}

    /// How many extra slots a TX op may execute late (the link stamps 1
    /// for the first TX op of a run: one slot late it still faces a
    /// listening peer). Only honored for TX ops. Default no-op.
    fn set_op_grace(&mut self, _slots: u8) {}

    /// Enable/disable the acquisition TX-delay sweep. Used by the peripheral
    /// while it is still sending SlotRequest and has not received Data yet.
    fn set_tx_delay_sweep(&mut self, _sweep: bool) {}

    /// Adjust the follower's early TX margin at runtime. The PHY reads this
    /// on every paced TX, so it can be tuned while the link is running
    /// (the central role ignores it).
    fn set_tx_phase_margin_us(&mut self, _margin_us: i32) {}

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

    /// The hardware slot counter, when the PHY has its own slot cadence
    /// (the MPSL timeslot chain). Returns 0 when the PHY is software-paced
    /// (the bare radio), so the link layer falls back to its own slot_step.
    fn slot_count(&self) -> u32 {
        0
    }

    /// The fallback slot period used before cadence negotiation completes.
    /// This must be a period every board in the network can sustain; both
    /// sides start here, exchange [`Packet::SlotRequest`], and then switch
    /// to `max(central_min, peripheral_min)`.
    fn fallback_slot_period_us(&self) -> u16 {
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

    /// Begin a TX burst: ramp the radio once, so the following
    /// [`transmit_burst_send`](Self::transmit_burst_send) packets skip the
    /// per-packet ramp (the on-air time only - the one-way path).
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
