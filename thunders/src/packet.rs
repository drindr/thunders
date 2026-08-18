//! Packet format and `postcard` (de)serialization.

use heapless::Vec;
use serde::{Deserialize, Serialize};

use crate::config::{DeviceId, MAX_PAYLOAD, NACK_BYTES};

/// A protocol packet.
///
/// All variants are serialized with `postcard` and transmitted as raw
/// bytes by the PHY.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Packet {
    /// Central beacon used to advertise the slot schedule and the timing
    /// measurements that keep the peripheral aligned.
    Beacon {
        /// Wrapping slot counter (used for the periodic beacon cadence).
        epoch: u32,
        /// Index into the shared hop sequence in effect for this beacon.
        ///
        /// The receiver re-syncs its scheduler to this index; hopping is
        /// driven by the central's miss threshold, not one step per slot.
        channel_index: u8,
        /// The sender's RX listen window, in 16 us units (0 = unknown).
        /// Advertised pre-connection so the peer can align its
        /// transmissions to this (possibly poorer) window.
        flags: u8,
        /// The sender's slot cadence in us (0 = unknown). The follower
        /// adopts it at runtime: no compile-time matching needed.
        slot_us: u16,
        /// The sender's slot phase (slot_step % ratio period). The follower
        /// mirrors it so its TX slots land on the sender's RX slots; without
        /// it the mirrored ratio aligns by luck, 1-in-period per boot.
        /// u16 so periods above 255 are representable for arbitrary ratios.
        slot_phase: u16,
        /// The sender's measured RXEN offset from slot START, in us. The
        /// follower uses this (instead of its own offset) to place its TX in
        /// the middle of the peer's actual RX window.
        rx_en_offset: u8,
        /// The sender's measured TXEN offset from slot START, in us. The
        /// follower subtracts this from the forward-catch estimate so the
        /// echo delay is derived from measurements instead of a fixed
        /// assumed TX offset.
        tx_en_offset: u8,
        /// The sender's measured RXEN -> READY ramp, in us.
        rx_ramp: u8,
        /// The sender's measured TXEN -> READY ramp, in us.
        tx_ramp: u8,
        /// Negotiated cadence-profile generation (0 = uniform fallback).
        cadence_id: u8,
        /// Cadence of central-TX short phases.
        short_slot_us: u16,
        /// Cadence of reverse/idle long phases.
        long_slot_us: u16,
        /// Number of leading central phases that use `short_slot_us`.
        short_phases: u16,
        /// Absolute central hardware slot where the profile takes effect
        /// (0 = offer only, not armed yet).
        cadence_apply_epoch: u32,
    },
    /// Data packet carrying a payload, plus the cumulative ACK and the
    /// NACK bitmap for the opposite direction (selective-repeat reliable
    /// delivery).
    Data {
        /// Sequence number of this packet (u16 — the sliding-window space).
        seq: u16,
        /// Slot cumulative ACK: the highest contiguous Data-slot seq the
        /// sender of *this* packet has delivered to the app from the peer
        /// (0xFFFF = nothing yet). Releases every in-flight slot <= `ack`.
        ack: u16,
        /// NACK bitmap: byte `b`, bit `i` = slot `8*b + i` of the sender's
        /// last TX run was lost. Variable length: only the bytes needed for
        /// the run length are sent (e.g. 1 byte for an 8-slot run).
        nack: Vec<u8, NACK_BYTES>,
        /// Up to [`MAX_PAYLOAD`] bytes of (encrypted) user data.
        payload: Vec<u8, MAX_PAYLOAD>,
    },
    /// A pure ACK/NACK (no data) — sent by a node that has received data
    /// but has nothing to send itself, so the peer's in-flight window can
    /// keep draining.
    Ack {
        /// Slot cumulative ACK (see [`Packet::Data`]).
        ack: u16,
        /// NACK bitmap (see [`Packet::Data`]).
        nack: Vec<u8, NACK_BYTES>,
    },
    /// A dropped-packet notification. Sent by a sender after a Data packet
    /// exhausts its retry budget: the receiver advances its delivery
    /// baseline past `seq`, so one lost middle packet cannot stall the
    /// in-order stream forever. The drop also carries the sender's ACK/NACK
    /// for the opposite direction so two nodes that both have pending drops
    /// can still clear each other (otherwise the drop preemption would
    /// deadlock the ACK path).
    Drop {
        /// The Data seq that the sender has dropped. The receiver skips its
        /// delivery window to `seq + 1`.
        seq: u16,
        /// Slot cumulative ACK (see [`Packet::Data`]).
        ack: u16,
        /// NACK bitmap (see [`Packet::Data`]).
        nack: Vec<u8, NACK_BYTES>,
    },
    /// Slot cadence request from a peripheral: the slowest board in the
    /// network advertises the minimum slot period it can sustain, and the
    /// central adopts `max(current, min_slot_us)`.
    SlotRequest {
        /// The peripheral's minimum uniform/long slot period in microseconds.
        min_slot_us: u16,
        /// The peripheral's minimum short-phase period.
        min_short_slot_us: u16,
        /// Cadence handshake state: low 7 bits are the seen profile id;
        /// bit 7 means the armed apply epoch was seen and scheduled.
        cadence_ack: u8,
        /// The peripheral's cumulative RX ACK. An acquiring peripheral
        /// answers only with SlotRequests (no Data/Ack packets), so
        /// without this the central could never clear a pending_drop via
        /// the normal ACK path - a dropped Data left it sending Drop
        /// packets forever and no new Data ever (the pair deadlocked).
        /// The ACK lets the central's window advance from the liveness
        /// traffic itself.
        ack: u16,
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
        let mut nack = Vec::<u8, NACK_BYTES>::new();
        nack.push(0b101).unwrap();
        let pkt = Packet::Data {
            seq: 7,
            ack: 0xFFFF,
            nack,
            payload,
        };
        let mut buf = [0u8; 64];
        let n = pkt.to_bytes(&mut buf).unwrap();
        let decoded = Packet::from_bytes(&buf[..n]).unwrap();
        assert_eq!(pkt, decoded);
    }

    #[test]
    fn round_trip_drop() {
        let mut nack = Vec::<u8, NACK_BYTES>::new();
        nack.push(0x05).unwrap();
        let pkt = Packet::Drop {
            seq: 0x1234,
            ack: 0xFFFF,
            nack,
        };
        let mut buf = [0u8; 64];
        let n = pkt.to_bytes(&mut buf).unwrap();
        let decoded = Packet::from_bytes(&buf[..n]).unwrap();
        assert_eq!(pkt, decoded);
    }

    #[test]
    fn round_trip_ack() {
        let mut nack = Vec::<u8, NACK_BYTES>::new();
        nack.push(0x01).unwrap();
        nack.push(0x80).unwrap();
        let pkt = Packet::Ack { ack: 0x1234, nack };
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
            slot_us: 0xBEEF,
            slot_phase: 7u16,
            rx_en_offset: 10,
            tx_en_offset: 20,
            rx_ramp: 40,
            tx_ramp: 40,
            cadence_id: 1,
            short_slot_us: 450,
            long_slot_us: 600,
            short_phases: 8,
            cadence_apply_epoch: 1024,
        };
        let mut buf = [0u8; 64];
        let n = pkt.to_bytes(&mut buf).unwrap();
        let decoded = Packet::from_bytes(&buf[..n]).unwrap();
        assert_eq!(pkt, decoded);
    }
}
