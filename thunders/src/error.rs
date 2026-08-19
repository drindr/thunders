//! Error types.

use crate::security::CryptoError;

/// Top-level protocol error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<P> {
    /// PHY-specific failure.
    Phy(P),
    /// Serialization / deserialization failure.
    Serialize,
    /// Received packet could not be parsed.
    InvalidPacket,
    /// The link has been out of sync for too long.
    SyncLost,
    /// Buffer provided by the caller was too small.
    BufferTooSmall,
    /// The requested operation is not supported by this PHY.
    Unsupported,
    /// Encryption or decryption failure.
    Crypto(CryptoError),
    /// Offered payload does not exactly match the active fixed-length contract.
    PayloadExceedsCadenceProfile,
    /// The reliable TX window is full: the caller offered data faster than
    /// the link could deliver it. Backpressure — retry later (or lower the
    /// offered rate; reliable delivery needs spare channel capacity).
    WindowFull,
    /// A packet was retransmitted up to [`crate::config::MAX_RETRIES`] without
    /// an ACK. The link dropped it; the delivery-failure counter records it.
    DeliveryFailed,
}

impl<P> From<postcard::Error> for Error<P> {
    fn from(_: postcard::Error) -> Self {
        Error::Serialize
    }
}

impl<P> From<CryptoError> for Error<P> {
    fn from(e: CryptoError) -> Self {
        Error::Crypto(e)
    }
}

impl<P> Error<P> {
    /// Wrap a PHY error.
    pub fn phy(e: P) -> Self {
        Error::Phy(e)
    }
}
