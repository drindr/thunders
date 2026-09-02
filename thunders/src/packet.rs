//! Fixed one-way data and feedback frames.

use crate::config::MAX_PAYLOAD;
use heapless::Vec;

const ONE_WAY_DATA: u8 = 0xF0;
const ONE_WAY_TIME_DIFF: u8 = 0xF2;

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
    /// Reverse feedback: the most recently observed forward sequence plus
    /// the measured phase error. When the sender runs its reliable phase,
    /// the sequence doubles as the cumulative ACK.
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
            Self::TimeDiff { seq, diff_us } => {
                if out.len() < 5 {
                    return Err(());
                }
                out[0] = ONE_WAY_TIME_DIFF;
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
            ONE_WAY_TIME_DIFF if bytes.len() == 5 => {
                let seq = u16::from_le_bytes([bytes[1], bytes[2]]);
                let diff_us = i16::from_le_bytes([bytes[3], bytes[4]]);
                Ok(Self::TimeDiff { seq, diff_us })
            }
            _ => Err(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_codec_has_exact_lengths() {
        let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
        payload.extend_from_slice(&[0xA5; 8]).unwrap();
        let data = FixedOneWayFrame::Data { seq: 7, payload };
        let mut out = [0u8; 64];
        let n = data.encode::<8>(&mut out).unwrap();
        assert_eq!(n, 11);
        assert_eq!(FixedOneWayFrame::decode::<8>(&out[..n]), Ok(data));

        let feedback = FixedOneWayFrame::TimeDiff {
            seq: 7,
            diff_us: 19,
        };
        let n = feedback.encode::<8>(&mut out).unwrap();
        assert_eq!(n, 5);
        assert_eq!(FixedOneWayFrame::decode::<8>(&out[..n]), Ok(feedback));
    }
}
