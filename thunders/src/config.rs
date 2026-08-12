//! Protocol configuration constants and types.

use crate::security::Security;

/// Maximum user payload bytes in one Data packet.
///
/// Kept at 32 bytes so the on-air time stays short and the packet fits
/// comfortably into an nRF24L01+ FIFO.
pub const MAX_PAYLOAD: usize = 32;

/// Number of channels in the hop sequence.
pub const HOP_SEQUENCE_LEN: usize = 25;

/// Superframe duration: 1 ms for a 1 kHz HID report rate.
pub const FRAME_DURATION_US: u32 = 1000;

/// Max time the central waits for a peripheral reply within a frame.
pub const CENTRAL_REPLY_TIMEOUT_US: u64 = 400;

/// Max time the peripheral listens for the central beacon/data.
pub const PERIPHERAL_LISTEN_TIMEOUT_US: u64 = 600;

/// Number of retransmissions before a frame is declared lost.
pub const MAX_RETRIES: u8 = 3;

/// Sequence number modulus.
pub const SEQ_MODULUS: u16 = 256;

/// Device identity type.
pub type DeviceId = [u8; 4];

/// Network / piconet identifier.
pub type NetworkId = [u8; 4];

/// Raw address used by the PHY.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Address(pub [u8; 5]);

/// Role of a node in the network.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Role {
    /// Coordinates the 1 ms superframe and polls peripherals.
    Central,
    /// Follows the central's superframe.
    Peripheral,
}

/// Protocol-wide configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Network identifier; used to seed the hop sequence.
    pub network: NetworkId,
    /// Logical address of this node.
    pub address: Address,
    /// Central or peripheral.
    pub role: Role,
    /// Initial channel index (0..HOP_SEQUENCE_LEN).
    pub initial_channel: u8,
    /// Optional pre-shared security context.
    pub security: Option<Security>,
}

impl Config {
    /// Create a new configuration.
    pub const fn new(network: NetworkId, address: Address, role: Role) -> Self {
        Self {
            network,
            address,
            role,
            initial_channel: 0,
            security: None,
        }
    }

    /// Attach a pre-shared security context.
    pub const fn with_security(mut self, security: Security) -> Self {
        self.security = Some(security);
        self
    }
}

/// Default 2.4 GHz hop sequence (channel indices, not raw MHz).
///
/// The actual frequency is `2400 MHz + channel MHz` for Nordic RADIO
/// or the transceiver-specific channel number for external PHYs.
// Benchmark: single fixed channel (25 = 2425 MHz) matching the validated
// ESB configuration. The default multi-channel hop sequence requires the
// two free-running nodes to stay phase-locked, which they do not at
// different frame rates.
pub const DEFAULT_HOP_SEQUENCE: [u8; HOP_SEQUENCE_LEN] = [25; 25];
