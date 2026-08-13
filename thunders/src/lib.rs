#![no_std]
#![warn(missing_docs)]

//! `thunders` — a small, `no_std`, async RF protocol stack for
//! Nordic nRF and external transceivers, designed around a 1 ms
//! superframe and `postcard` serialization.

pub mod config;
pub mod error;
pub mod ipc;
pub mod link;
pub mod packet;
pub mod phy;
pub mod scheduler;
pub mod security;

pub use config::{Address, Config, DeviceId, Role, MAX_PAYLOAD};
pub use error::Error;
pub use link::{Central, LinkStatus, Peripheral};
pub use packet::Packet;
pub use phy::Phy;
pub use security::{make_nonce, Security};
