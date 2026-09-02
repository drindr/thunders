#![no_std]
#![warn(missing_docs)]

//! `thunders` — a `no_std` fixed-packet RF protocol core.
//!
//! Link behavior is selected at compile time through [`LinkMode`]. The first
//! supported family is one-way streaming with reliable ACK feedback or
//! periodic timing-only feedback.

pub mod config;
pub mod error;
pub mod mode;
pub mod one_way;
pub mod packet;
pub mod phy;
pub mod tdma;

pub use config::{Address, MAX_PAYLOAD};
pub use error::Error;
pub use mode::{
    AirTiming, FixedSlotPlan, LinkMode, LinkModeKind, OneWay, OneWayAck, OneWayChanges,
    OneWayNoAck, OneWayState, SlotOverhead, fixed_slot_plan, round_up_us,
};
pub use one_way::{
    FeedbackUpdate, OneWayReceive, OneWayReceiver, OneWaySendError, OneWaySender, TimeDiffAligner,
};
pub use packet::FixedOneWayFrame;
pub use phy::{Phy, RxTiming};
pub use tdma::{StaticTdma, TdmaNode};
