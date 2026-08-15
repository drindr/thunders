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
// The slot model: the RX window is one slot. The reply is expected at the
// aligned RX slot's start; the window covers the peer's TX ramp + the
// on-air frame + jitter, inside the bare slot period (400 us). The MPSL phy
// ignores this timeout (it sizes its poll from the timeslot grant).
pub const CENTRAL_REPLY_TIMEOUT_US: u64 = 200;

/// Max time the peripheral listens for the central beacon/data.
pub const PERIPHERAL_LISTEN_TIMEOUT_US: u64 = 200;

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
    /// TX:RX slot ratio - `tx` TX slots per `rx` RX slot. (8, 1) = the 8 kHz
    /// one-way stream + a 1 kHz reverse channel; (1, 1) = the symmetric 4 kHz
    /// round-trip. Both sides agree via this shared config (the roles mirror).
    pub tx_rx_ratio: (u8, u8),
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
            tx_rx_ratio: (8, 1),
        }
    }

    /// Attach a pre-shared security context.
    pub const fn with_security(mut self, security: Security) -> Self {
        self.security = Some(security);
        self
    }

    /// Set the TX:RX slot ratio (both sides must be at least 1 - a zero
    /// period would divide-by-zero the slot scheduler).
    pub const fn with_tx_rx_ratio(mut self, tx: u8, rx: u8) -> Self {
        self.tx_rx_ratio = (if tx == 0 { 1 } else { tx }, if rx == 0 { 1 } else { rx });
        self
    }
}

/// Default 2.4 GHz hop sequence (channel indices, not raw MHz).
///
/// The actual frequency is `2400 MHz + channel MHz` for Nordic RADIO
/// or the transceiver-specific channel number for external PHYs.
// The 25-channel hop sequence (channel indices, 2400 MHz + channel MHz),
// spread over the 2.4 GHz band with 3 MHz spacing. Hopping is beacon-driven:
// the central advances on the interference (the miss threshold), the
// peripheral follows the beacon's channel_index - no free-running local hop,
// so the two nodes stay on the same channel.
pub const DEFAULT_HOP_SEQUENCE: [u8; HOP_SEQUENCE_LEN] = [
    2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38,
    41, 44, 47, 50, 53, 56, 59, 62, 65, 68, 71, 74,
];
