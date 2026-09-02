#![no_std]
#![warn(missing_docs)]

//! `thunders-phy-nrf` — Nordic RADIO PHY implementation for `thunders`.

pub mod radio_phy;

#[cfg(feature = "mpsl")]
pub mod mpsl;

#[cfg(feature = "mpsl")]
pub use mpsl::MpslRadioPhy;
#[cfg(feature = "_nrf54")]
pub use radio_phy::{hfxo_cap_trim, rramc_fast_fetch};
pub use radio_phy::{NrfRadioPhy, RadioError, RadioIrqHandler, RadioMode};
