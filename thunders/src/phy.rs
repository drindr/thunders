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

    /// Flush any stale RX/TX state.
    async fn flush(&mut self);
}
