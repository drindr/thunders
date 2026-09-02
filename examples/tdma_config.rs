//! Shared compile-time TDMA configuration for the bare multi-sender examples.

use thunders::{Address, StaticTdma};

/// Number of independently addressed state senders.
pub const SENDERS: usize = 2;
/// Measured stable start-to-start period for every sender.
pub const FRAME_US: u32 = 190;
/// Approximate complete fast-ramp fixed-state TX duration.
pub const TX_US: u32 = 100;

/// One schedule shared by every sender and receiver binary.
pub const SCHEDULE: StaticTdma<SENDERS, FRAME_US, TX_US> = StaticTdma::new([
    Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
    Address([0xC3, 0xE7, 0xE7, 0xE7, 0xE7]),
]);
