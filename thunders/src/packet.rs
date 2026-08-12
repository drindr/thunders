//! Packet format and `postcard` (de)serialization.

use heapless::Vec;
use serde::{Deserialize, Serialize};

use crate::config::{DeviceId, MAX_PAYLOAD};

/// A protocol packet.
///
/// All variants are serialized with `postcard` and transmitted as raw
/// bytes by the PHY.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Packet {
    /// Central beacon used to keep the 1 ms superframe in sync.
    Beacon {
        /// Monotonically increasing frame counter.
        epoch: u32,
        /// Index into the shared hop sequence for *this* frame.
        ///
        /// Receivers can reset their local scheduler to this index; both
        /// sides then advance one step at the end of the 1 ms frame.
        channel_index: u8,
        /// Flags (reserved for future use).
        flags: u8,
    },
    /// Data packet carrying a payload.
    ///
    /// Reliability is seq-based: the receiver accepts a frame only when its
    /// seq is inside the accept window (freshness/ordering check), and the
    /// peer's replies serve as the implicit acknowledgment that the link is
    /// alive. No explicit ACK field or ACK packet - keeps the on-air time
    /// minimal for high frame rates.
    Data {
        /// Sequence number of this packet.
        seq: u8,
        /// Up to [`MAX_PAYLOAD`] bytes of user data.
        payload: Vec<u8, MAX_PAYLOAD>,
    },
    /// Pairing request from a peripheral.
    PairingRequest {
        /// Device identifier.
        id: DeviceId,
    },
    /// Pairing response from the central.
    PairingResponse {
        /// Device identifier.
        id: DeviceId,
        /// Pre-shared encryption key (placeholder).
        key: [u8; 16],
    },
}

impl Packet {
    /// Serialize the packet into `buf` using `postcard`.
    pub fn to_bytes(&self, buf: &mut [u8]) -> Result<usize, postcard::Error> {
        postcard::to_slice(self, buf).map(|s| s.len())
    }

    /// Deserialize a packet from `buf`.
    pub fn from_bytes(buf: &[u8]) -> Result<Self, postcard::Error> {
        postcard::from_bytes(buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_data() {
        let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
        payload.extend_from_slice(&[1, 2, 3, 4]).unwrap();
        let pkt = Packet::Data { seq: 7, payload };
        let mut buf = [0u8; 64];
        let n = pkt.to_bytes(&mut buf).unwrap();
        let decoded = Packet::from_bytes(&buf[..n]).unwrap();
        assert_eq!(pkt, decoded);
    }

    #[test]
    fn round_trip_beacon() {
        let pkt = Packet::Beacon {
            epoch: 0x1234_5678,
            channel_index: 17,
            flags: 0xAA,
        };
        let mut buf = [0u8; 64];
        let n = pkt.to_bytes(&mut buf).unwrap();
        let decoded = Packet::from_bytes(&buf[..n]).unwrap();
        assert_eq!(pkt, decoded);
    }
}
