#![no_std]
#![warn(missing_docs)]

//! `thunders` — a `no_std` fixed-packet RF protocol core.
//!
//! Link behavior is selected at compile time through [`LinkMode`]. The first
//! supported family is one-way streaming with periodic timing feedback; the
//! receiver can recall a sender into a reliable negotiation phase over that
//! same feedback channel (see [`negotiation`]).

pub mod config;
pub mod error;
pub mod mode;
pub mod negotiation;
pub mod one_way;
pub mod packet;
pub mod phy;
pub mod tdma;

pub use config::{Address, MAX_PAYLOAD};
pub use error::Error;
pub use mode::{
    fixed_slot_plan, round_up_us, AirTiming, FixedSlotPlan, LinkMode, OneWay, OneWayState,
    SlotOverhead,
};
pub use negotiation::{ConfigFrame, FeedbackBeacon};
pub use one_way::{
    FeedbackUpdate, OneWayReceive, OneWayReceiver, OneWaySendError, OneWaySender, TimeDiffAligner,
};
pub use packet::FixedOneWayFrame;
pub use phy::{Phy, RxTiming};
pub use tdma::{StaticTdma, TdmaNode};
