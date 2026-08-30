#![no_std]
#![warn(missing_docs)]

//! `thunders-phy-nrf` — Nordic RADIO PHY implementation for `thunders`.

pub mod radio_phy;

#[cfg(feature = "_nrf54")]
pub use radio_phy::hfxo_cap_trim;
pub use radio_phy::{NrfRadioPhy, RadioError, RadioIrqHandler, RadioMode};
