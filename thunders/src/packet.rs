//! Fixed one-way data frames and connection-control packets.

use heapless::Vec;
use serde::{Deserialize, Serialize};

use crate::config::MAX_PAYLOAD;

const ONE_WAY_DATA: u8 = 0xF0;
const ONE_WAY_ACK: u8 = 0xF1;
const ONE_WAY_TIME_DIFF: u8 = 0xF2;

/// Connection-control traffic. Control remains postcard encoded; Data and
/// feedback use [`FixedOneWayFrame`]'s exact fixed codec.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Packet {
    /// Repeated invitation from the time-master transmitter.
    ConnectOffer {
        /// Wrapping connection generation.
        generation: u8,
        /// Transmitter event counter carrying this offer.
        offer_epoch: u32,
        /// Event counter reserved for the first connection event.
        first_event_epoch: u32,
        /// Delay from this packet's ADDRESS anchor to the first event.
        start_after_us: u32,
        /// Receiver window around the first event.
        window_us: u16,
    },
    /// First transmitter packet at the promised event anchor.
    FirstEvent {
        /// Connection generation.
        generation: u8,
        /// Exact first-event counter.
        event_epoch: u32,
        /// Challenge echoed by the receiver.
        challenge: u16,
    },
    /// Receiver proof tied to the first-event ADDRESS anchor.
    FirstResponse {
        /// Connection generation.
        generation: u8,
        /// Exact first-event counter.
        event_epoch: u32,
        /// Echoed challenge.
        challenge: u16,
    },
    /// Time-master confirms the reverse proof and opens fixed Data.
    Connected {
        /// Connection generation.
        generation: u8,
        /// First connected forward event.
        start_epoch: u32,
    },
}

impl Packet {
    /// Serialize a control packet using postcard.
    pub fn to_bytes(&self, out: &mut [u8]) -> Result<usize, postcard::Error> {
        postcard::to_slice(self, out).map(|encoded| encoded.len())
    }

    /// Deserialize a postcard control packet.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, postcard::Error> {
        postcard::from_bytes(bytes)
    }
}

/// Compact fixed-length frame used by compile-time one-way modes.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FixedOneWayFrame {
    /// Forward fixed-payload Data.
    Data {
        /// Wrapping stream sequence.
        seq: u16,
        /// Payload whose length equals the mode's compile-time value.
        payload: Vec<u8, MAX_PAYLOAD>,
    },
    /// Reverse cumulative ACK plus measured phase error.
    Ack {
        /// Highest accepted forward sequence.
        seq: u16,
        /// Receiver timing error in microseconds.
        diff_us: i16,
    },
    /// Reverse timing-only feedback for no-ACK streaming mode.
    TimeDiff {
        /// Most recently observed forward sequence.
        seq: u16,
        /// Receiver timing error in microseconds.
        diff_us: i16,
    },
}

impl FixedOneWayFrame {
    /// Encode with exactly `PAYLOAD` bytes for Data and five bytes for feedback.
    pub fn encode<const PAYLOAD: usize>(&self, out: &mut [u8]) -> Result<usize, ()> {
        match self {
            Self::Data { seq, payload } => {
                if PAYLOAD > MAX_PAYLOAD || payload.len() != PAYLOAD || out.len() < PAYLOAD + 3 {
                    return Err(());
                }
                out[0] = ONE_WAY_DATA;
                out[1..3].copy_from_slice(&seq.to_le_bytes());
                out[3..3 + PAYLOAD].copy_from_slice(payload);
                Ok(PAYLOAD + 3)
            }
            Self::Ack { seq, diff_us } | Self::TimeDiff { seq, diff_us } => {
                if out.len() < 5 {
                    return Err(());
                }
                out[0] = if matches!(self, Self::Ack { .. }) {
                    ONE_WAY_ACK
                } else {
                    ONE_WAY_TIME_DIFF
                };
                out[1..3].copy_from_slice(&seq.to_le_bytes());
                out[3..5].copy_from_slice(&diff_us.to_le_bytes());
                Ok(5)
            }
        }
    }

    /// Decode a fixed one-way frame for compile-time payload length `PAYLOAD`.
    pub fn decode<const PAYLOAD: usize>(bytes: &[u8]) -> Result<Self, ()> {
        if bytes.is_empty() || PAYLOAD > MAX_PAYLOAD {
            return Err(());
        }
        match bytes[0] {
            ONE_WAY_DATA if bytes.len() == PAYLOAD + 3 => {
                let seq = u16::from_le_bytes([bytes[1], bytes[2]]);
                let mut payload = Vec::new();
                payload.extend_from_slice(&bytes[3..]).map_err(|_| ())?;
                Ok(Self::Data { seq, payload })
            }
            ONE_WAY_ACK | ONE_WAY_TIME_DIFF if bytes.len() == 5 => {
                let seq = u16::from_le_bytes([bytes[1], bytes[2]]);
                let diff_us = i16::from_le_bytes([bytes[3], bytes[4]]);
                if bytes[0] == ONE_WAY_ACK {
                    Ok(Self::Ack { seq, diff_us })
                } else {
                    Ok(Self::TimeDiff { seq, diff_us })
                }
            }
            _ => Err(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_round_trip() {
        for packet in [
            Packet::ConnectOffer {
                generation: 3,
                offer_epoch: u32::MAX - 4,
                first_event_epoch: 8,
                start_after_us: 48_000,
                window_us: 600,
            },
            Packet::FirstEvent {
                generation: 3,
                event_epoch: 8,
                challenge: 0x1234,
            },
            Packet::FirstResponse {
                generation: 3,
                event_epoch: 8,
                challenge: 0x1234,
            },
            Packet::Connected {
                generation: 3,
                start_epoch: 9,
            },
        ] {
            let mut out = [0u8; 64];
            let n = packet.to_bytes(&mut out).unwrap();
            assert_eq!(Packet::from_bytes(&out[..n]).unwrap(), packet);
        }
    }

    #[test]
    fn fixed_codec_has_exact_lengths() {
        let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
        payload.extend_from_slice(&[0xA5; 8]).unwrap();
        let data = FixedOneWayFrame::Data { seq: 7, payload };
        let mut out = [0u8; 64];
        let n = data.encode::<8>(&mut out).unwrap();
        assert_eq!(n, 11);
        assert_eq!(FixedOneWayFrame::decode::<8>(&out[..n]), Ok(data));

        for feedback in [
            FixedOneWayFrame::Ack {
                seq: 7,
                diff_us: -13,
            },
            FixedOneWayFrame::TimeDiff {
                seq: 7,
                diff_us: 19,
            },
        ] {
            let n = feedback.encode::<8>(&mut out).unwrap();
            assert_eq!(n, 5);
            assert_eq!(FixedOneWayFrame::decode::<8>(&out[..n]), Ok(feedback));
        }
    }
}
