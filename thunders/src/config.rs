//! Protocol configuration constants and types.

use crate::security::Security;

/// Maximum user payload bytes in one Data packet.
///
/// Kept at 32 bytes so the on-air time stays short inside one 400/500 µs
/// slot and the fixed heapless buffers stay small.
pub const MAX_PAYLOAD: usize = 32;

/// Number of channels in the hop sequence.
pub const HOP_SEQUENCE_LEN: usize = 25;

/// Max time the central waits for a peripheral reply in an RX slot.
// The slot model: the RX window is one slot. The reply is expected at the
// aligned RX slot's start; the window covers the peer's TX ramp + the
// on-air frame + jitter, inside the bare slot period (400 us). The MPSL phy
// ignores this timeout (it sizes its poll from the timeslot grant).
pub const CENTRAL_REPLY_TIMEOUT_US: u64 = 200;

/// Max time the peripheral listens for the central beacon/data.
pub const PERIPHERAL_LISTEN_TIMEOUT_US: u64 = 200;

/// Number of retransmissions (after the first send) before a frame is
/// declared lost and dropped as a delivery failure.
pub const MAX_RETRIES: u8 = 8;

/// Sliding-window size: the TX in-flight window and the RX reorder buffer.
///
/// Must be a power of two and ≤ 16. The in-flight window is intentionally
/// independent of the slot-run length: a run can have up to 255 slots, but
/// at most WINDOW_SIZE Data packets can be in flight at once.
pub const WINDOW_SIZE: usize = 16;

/// NACK bitmap size in bytes (256 bits). Supports slot runs up to 255 slots
/// for arbitrary `tx_rx_ratio` choices.
pub const NACK_BYTES: usize = 32;

/// Slots an in-flight packet waits for an ACK before it is retransmitted.
///
/// This is the safety net behind the NACK-driven retransmit: the receiver's
/// NACK bitmap recovers most loss immediately, the timeout covers lost ACKs.
pub const RETRY_TIMEOUT_SLOTS: u16 = 256;

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
    /// Coordinates the shared slot schedule and polls peripherals.
    Central,
    /// Follows the central's slot schedule.
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
    /// Central (forward) TX:RX slot ratio - `tx` TX slots then `rx` RX slots.
    /// (8, 2) = eight forward TX slots followed by two central RX slots.
    /// The peripheral's local ratio is [`Config::reverse_tx_rx_ratio`].
    pub tx_rx_ratio: (u8, u8),
    /// Peripheral (reverse) TX:RX slot ratio - `tx` TX slots then `rx` RX
    /// slots from the peripheral's local point of view. It must be the
    /// complement of [`Config::tx_rx_ratio`] for the two schedules to align:
    /// `reverse == (forward.rx, forward.tx)`.
    pub reverse_tx_rx_ratio: (u8, u8),
    /// Shared idle slots appended after both local TX/RX runs. Idle is what
    /// lets the two directions have different capacities while keeping one
    /// common period: `period = forward_tx + forward_rx + idle`.
    pub idle_slots: u8,
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
            tx_rx_ratio: (8, 2),
            reverse_tx_rx_ratio: (2, 8),
            idle_slots: 0,
        }
    }

    /// Attach a pre-shared security context.
    #[must_use]
    pub const fn with_security(mut self, security: Security) -> Self {
        self.security = Some(security);
        self
    }

    /// Shared slot period: `forward_tx + forward_rx + idle`.
    pub const fn period_slots(&self) -> u16 {
        self.tx_rx_ratio.0 as u16 + self.tx_rx_ratio.1 as u16 + self.idle_slots as u16
    }

    /// TX capacity of `role` in slots per period. This is the theoretical
    /// maximum offered load; ARQ retransmissions need spare capacity, so the
    /// bench offers only when the TX window has room.
    pub const fn tx_slots_per_period(&self, role: Role) -> u8 {
        match role {
            Role::Central => self.tx_rx_ratio.0,
            Role::Peripheral => self.reverse_tx_rx_ratio.0,
        }
    }

    /// Set the central (forward) TX:RX slot ratio. The peripheral's local
    /// ratio is set to the complement `(rx, tx)` automatically. A zero `tx`
    /// or `rx` is clamped to 1 so the slot period can never be zero.
    #[must_use]
    pub const fn with_tx_rx_ratio(mut self, tx: u8, rx: u8) -> Self {
        let tx = if tx == 0 { 1 } else { tx };
        let rx = if rx == 0 { 1 } else { rx };
        self.tx_rx_ratio = (tx, rx);
        self.reverse_tx_rx_ratio = (rx, tx);
        self.idle_slots = 0;
        self
    }

    /// Set a schedule with idle slots. Central local schedule is
    /// `tx` TX slots, `rx` RX slots, `idle` idle slots; peripheral local
    /// schedule is `rx` TX, `tx` RX, `idle` idle. This is the supported way
    /// to give the two directions different capacity while both sides keep
    /// the same period and complementary slot types.
    #[must_use]
    pub const fn with_tx_rx_idle(mut self, tx: u8, rx: u8, idle: u8) -> Self {
        let tx = if tx == 0 { 1 } else { tx };
        let rx = if rx == 0 { 1 } else { rx };
        self.tx_rx_ratio = (tx, rx);
        self.reverse_tx_rx_ratio = (rx, tx);
        self.idle_slots = idle;
        self
    }

    /// Set the peripheral (reverse) TX:RX slot ratio explicitly.
    ///
    /// For the two local schedules to align, `(tx, rx)` must be the
    /// complement of [`Config::tx_rx_ratio`]: `tx == forward.rx` and
    /// `rx == forward.tx`. If it is not, link construction normalizes it
    /// back to the complement, so this setter is useful for documentation
    /// and for future per-link validation; it does not enable physically
    /// incompatible schedules.
    #[must_use]
    pub const fn with_reverse_tx_rx_ratio(mut self, tx: u8, rx: u8) -> Self {
        self.reverse_tx_rx_ratio = (if tx == 0 { 1 } else { tx }, if rx == 0 { 1 } else { rx });
        self
    }
}

/// Default 2.4 GHz hop sequence (channel indices, not raw MHz).
///
/// For the Nordic RADIO the actual frequency is `2400 MHz + channel MHz`;
/// other `Phy` implementations map the index to their own channel number.
// The 25-channel hop sequence (channel indices, 2400 MHz + channel MHz),
// spread over the 2.4 GHz band with 3 MHz spacing. Hopping is beacon-driven:
// the central advances on the interference (the miss threshold), the
// peripheral follows the beacon's channel_index - no free-running local hop,
// so the two nodes stay on the same channel.
pub const DEFAULT_HOP_SEQUENCE: [u8; HOP_SEQUENCE_LEN] = [
    2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53, 56, 59, 62, 65, 68, 71, 74,
];
