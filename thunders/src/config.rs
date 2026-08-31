//! Core fixed-mode configuration primitives.

/// Maximum fixed application payload supported by the in-memory codec.
pub const MAX_PAYLOAD: usize = 32;

/// Five-byte Nordic radio address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Address(pub [u8; 5]);
