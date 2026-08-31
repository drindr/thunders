//! Error types.

/// Top-level PHY error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<P> {
    /// PHY-specific failure.
    Phy(P),
    /// Buffer provided by the caller was too small.
    BufferTooSmall,
    /// The requested operation is not supported by this PHY.
    Unsupported,
}

impl<P> Error<P> {
    /// Wrap a PHY error.
    pub fn phy(error: P) -> Self {
        Self::Phy(error)
    }
}
