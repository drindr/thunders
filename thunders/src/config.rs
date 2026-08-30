//! Core fixed-mode configuration primitives.

use serde::{Deserialize, Serialize};

/// Maximum fixed application payload supported by the in-memory codec.
pub const MAX_PAYLOAD: usize = 32;
/// Default retransmission limit used by reliable one-way adapters.
pub const MAX_RETRIES: u8 = 8;

/// Stable four-byte device identifier used by provisioning layers.
pub type DeviceId = [u8; 4];

/// Five-byte Nordic radio address.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Address(pub [u8; 5]);
