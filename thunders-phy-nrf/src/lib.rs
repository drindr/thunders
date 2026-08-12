#![no_std]
#![warn(missing_docs)]

//! `thunders-phy-nrf` — Nordic RADIO PHY implementation for `thunders`.

pub mod radio_phy;

#[cfg(feature = "mpsl")]
pub mod mpsl;

pub use radio_phy::{NrfRadioPhy, RadioError, RadioIrqHandler, RadioMode};
#[cfg(feature = "mpsl")]
pub use mpsl::MpslRadioPhy;
