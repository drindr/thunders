#![no_std]
#![warn(missing_docs)]

//! `thunders` — a small, `no_std`, async RF protocol stack for
//! Nordic nRF radios, designed around a slot-based schedule and
//! `postcard` serialization.

pub mod cadence;
pub mod config;
pub mod error;
pub mod link;
pub mod link_mgmt;
pub mod packet;
pub mod phy;
pub mod scheduler;
pub mod security;

pub use cadence::{
    CadenceError, CadenceNegotiationStatus, CadenceProbePolicy, CadenceProfile, CadenceSearch,
    ProbeDecision, ProbeMetrics, TrafficContract,
};
pub use config::{
    Address, Config, DeviceId, Role, MAX_PAYLOAD, NACK_BYTES, RETRY_TIMEOUT_SLOTS, WINDOW_SIZE,
};
pub use error::Error;
pub use link::{Central, LinkStatus, Peripheral};
pub use packet::Packet;
pub use phy::Phy;
pub use security::{make_nonce, Security};
