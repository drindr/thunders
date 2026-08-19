//! Packet format and `postcard` (de)serialization.

use heapless::Vec;
use serde::{Deserialize, Serialize};

use crate::config::{DeviceId, MAX_PAYLOAD, NACK_BYTES};

/// Fixed-contract data-plane marker. Postcard control variants currently use
/// small enum discriminants, so this byte unambiguously selects the compact
/// codec after a traffic contract is committed.
const FIXED_DATA: u8 = 0xF0;
const FIXED_ACK: u8 = 0xF1;
const FIXED_DROP: u8 = 0xF2;

/// Fixed bytes before a negotiated Data payload: marker + seq + cumulative ACK.
pub const FIXED_DATA_HEADER_LEN: usize = 5;

/// API-triggered cadence-negotiation control stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CadenceStage {
    /// A peripheral API asks the central to start negotiation.
    Request,
    /// The central proposes the contract and first candidate.
    Offer,
    /// Either endpoint requests release of the active traffic contract. The
    /// central repeats this as the authoritative safe-profile offer.
    Release,
    /// The peripheral accepts the offered bounds or release request.
    Accept,
    /// The central publishes a bounded probe interval.
    Probe,
    /// The peripheral has scheduled that exact probe interval.
    Armed,
    /// Probe metrics after automatic restoration of the active profile.
    Report,
    /// The central commits the selected stable profile.
    Commit,
    /// The peripheral scheduled the exact final apply epoch.
    Applied,
    /// Abort while retaining the previous stable profile.
    Cancel,
}

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
        /// last TX run was lost. Postcard mode includes its vector length;
        /// negotiated fixed mode infers the byte count from the slot ratio.
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
    /// API-triggered cadence negotiation/probe control. It is repeated by
    /// the state machine until the peer advances to the next stage and also
    /// carries the normal cumulative ACK/NACK so negotiation cannot block
    /// the data windows.
    Cadence {
        /// Normal cumulative data ACK.
        ack: u16,
        /// Normal selective NACK bitmap.
        nack: Vec<u8, NACK_BYTES>,
        /// Nonzero API negotiation generation.
        generation: u8,
        /// Negotiation state carried by this repeated control packet.
        stage: CadenceStage,
        /// Absolute sender hardware slot carrying this control packet.
        epoch: u32,
        /// Exact central-to-peripheral application payload length.
        forward_payload: u8,
        /// Exact peripheral-to-central application payload length.
        reverse_payload: u8,
        /// Candidate or committed central-TX period.
        short_us: u16,
        /// Candidate or committed reverse/idle period.
        long_us: u16,
        /// Lowest candidate the API permits.
        min_slot_us: u16,
        /// Candidate descent quantum.
        step_us: u8,
        /// Safety quanta added above the lowest passing candidate.
        safety_steps: u8,
        /// Central absolute probe/commit start epoch.
        start_epoch: u32,
        /// Central absolute exclusive probe end epoch.
        end_epoch: u32,
        /// Number of complete superframes in one probe.
        probe_slots: u16,
        /// Stable/reject result flags.
        flags: u8,
    },
    /// Compact negotiation response/control. Accept/Armed/Report/Applied must
    /// fit delayed follower TX placement; Release/Commit must also fit the
    /// currently active short-payload slot while restoring the safe profile.
    CadenceAck {
        /// Active API negotiation generation.
        generation: u8,
        /// Response stage.
        stage: CadenceStage,
        /// Exact central start/apply epoch being acknowledged.
        start_epoch: u32,
        /// Exact central exclusive probe end epoch, or zero for commit ACK.
        end_epoch: u32,
        /// Stable/reject result flags.
        flags: u8,
    },
    /// Compact worst-contract-length trial traffic. Its serialized size is
    /// deliberately adjusted to the Data wire length; using the much larger
    /// negotiation control header would falsely reject short-packet slots.
    CadenceSample {
        /// Active negotiation generation.
        generation: u8,
        /// Bytes used only to reach the requested worst-case wire length.
        padding: Vec<u8, { MAX_PAYLOAD + 32 }>,
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

    /// Serialize negotiated data-plane traffic without vector-length fields.
    ///
    /// `payload_len` and `nack_len` are fixed by the committed directional
    /// traffic contract and slot ratio. Data must match exactly; Ack and Drop
    /// use the same inferred NACK width but carry no payload.
    pub fn to_fixed_bytes(
        &self,
        payload_len: usize,
        nack_len: usize,
        buf: &mut [u8],
    ) -> Result<usize, ()> {
        if payload_len > MAX_PAYLOAD || nack_len > NACK_BYTES {
            return Err(());
        }
        let (marker, seq, ack, nack, payload): (u8, Option<u16>, u16, &[u8], &[u8]) = match self {
            Packet::Data {
                seq,
                ack,
                nack,
                payload,
            } if payload.len() == payload_len && nack.len() == nack_len => {
                (FIXED_DATA, Some(*seq), *ack, nack, payload)
            }
            Packet::Ack { ack, nack } if nack.len() == nack_len => {
                (FIXED_ACK, None, *ack, nack, &[])
            }
            Packet::Drop { seq, ack, nack } if nack.len() == nack_len => {
                (FIXED_DROP, Some(*seq), *ack, nack, &[])
            }
            _ => return Err(()),
        };
        let needed = 1 + usize::from(seq.is_some()) * 2 + 2 + nack_len + payload.len();
        if buf.len() < needed {
            return Err(());
        }
        let mut at = 0;
        buf[at] = marker;
        at += 1;
        if let Some(seq) = seq {
            buf[at..at + 2].copy_from_slice(&seq.to_le_bytes());
            at += 2;
        }
        buf[at..at + 2].copy_from_slice(&ack.to_le_bytes());
        at += 2;
        buf[at..at + nack_len].copy_from_slice(nack);
        at += nack_len;
        buf[at..at + payload.len()].copy_from_slice(payload);
        at += payload.len();
        Ok(at)
    }

    /// True when `buf` starts with a reserved fixed data-plane marker.
    pub fn has_fixed_marker(buf: &[u8]) -> bool {
        matches!(
            buf.first(),
            Some(&FIXED_DATA) | Some(&FIXED_ACK) | Some(&FIXED_DROP)
        )
    }

    /// Decode negotiated fixed-width Data/Ack/Drop traffic.
    pub fn from_fixed_bytes(buf: &[u8], payload_len: usize, nack_len: usize) -> Result<Self, ()> {
        if payload_len > MAX_PAYLOAD || nack_len > NACK_BYTES || buf.is_empty() {
            return Err(());
        }
        let (has_seq, has_payload) = match buf[0] {
            FIXED_DATA => (true, true),
            FIXED_ACK => (false, false),
            FIXED_DROP => (true, false),
            _ => return Err(()),
        };
        let expected =
            1 + usize::from(has_seq) * 2 + 2 + nack_len + if has_payload { payload_len } else { 0 };
        if buf.len() != expected {
            return Err(());
        }
        let mut at = 1;
        let seq = if has_seq {
            let value = u16::from_le_bytes([buf[at], buf[at + 1]]);
            at += 2;
            Some(value)
        } else {
            None
        };
        let ack = u16::from_le_bytes([buf[at], buf[at + 1]]);
        at += 2;
        let mut nack = Vec::<u8, NACK_BYTES>::new();
        nack.extend_from_slice(&buf[at..at + nack_len])
            .map_err(|_| ())?;
        at += nack_len;
        if has_payload {
            let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
            payload
                .extend_from_slice(&buf[at..at + payload_len])
                .map_err(|_| ())?;
            Ok(Packet::Data {
                seq: seq.ok_or(())?,
                ack,
                nack,
                payload,
            })
        } else if let Some(seq) = seq {
            Ok(Packet::Drop { seq, ack, nack })
        } else {
            Ok(Packet::Ack { ack, nack })
        }
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
    fn fixed_contract_data_omits_vector_lengths() {
        let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
        payload.extend_from_slice(&[1, 2, 3, 4]).unwrap();
        let mut nack = Vec::<u8, NACK_BYTES>::new();
        nack.push(0b101).unwrap();
        let pkt = Packet::Data {
            seq: 0x1234,
            ack: 0xABCD,
            nack,
            payload,
        };
        let mut buf = [0u8; 64];
        let n = pkt.to_fixed_bytes(4, 1, &mut buf).unwrap();
        assert_eq!(n, FIXED_DATA_HEADER_LEN + 1 + 4);
        assert_eq!(
            &buf[..n],
            &[FIXED_DATA, 0x34, 0x12, 0xCD, 0xAB, 0x05, 1, 2, 3, 4]
        );
        assert!(Packet::has_fixed_marker(&buf[..n]));
        assert_eq!(Packet::from_fixed_bytes(&buf[..n], 4, 1), Ok(pkt));
        assert!(Packet::from_fixed_bytes(&buf[..n], 3, 1).is_err());
        assert!(Packet::from_fixed_bytes(&buf[..n - 1], 4, 1).is_err());
    }

    #[test]
    fn fixed_contract_ack_and_drop_use_inferred_nack_width() {
        let mut nack = Vec::<u8, NACK_BYTES>::new();
        nack.extend_from_slice(&[0x01, 0x80]).unwrap();
        for pkt in [
            Packet::Ack {
                ack: 7,
                nack: nack.clone(),
            },
            Packet::Drop {
                seq: 9,
                ack: 7,
                nack,
            },
        ] {
            let mut buf = [0u8; 16];
            let n = pkt.to_fixed_bytes(8, 2, &mut buf).unwrap();
            assert_eq!(Packet::from_fixed_bytes(&buf[..n], 8, 2), Ok(pkt));
        }
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
    fn round_trip_cadence_control_fits_phy_buffer() {
        let mut nack = Vec::<u8, NACK_BYTES>::new();
        nack.extend_from_slice(&[0x01, 0x80]).unwrap();
        let pkt = Packet::Cadence {
            ack: 123,
            nack,
            generation: 2,
            stage: CadenceStage::Probe,
            epoch: 100_000,
            forward_payload: MAX_PAYLOAD as u8,
            reverse_payload: MAX_PAYLOAD as u8,
            short_us: 475,
            long_us: 600,
            min_slot_us: 450,
            step_us: 25,
            safety_steps: 1,
            start_epoch: 100_080,
            end_epoch: 100_160,
            probe_slots: 8,
            flags: 1,
        };
        let mut buf = [0u8; 64];
        let n = pkt.to_bytes(&mut buf).unwrap();
        assert!(n <= 64);
        assert_eq!(pkt, Packet::from_bytes(&buf[..n]).unwrap());

        let mut release = pkt.clone();
        if let Packet::Cadence { stage, flags, .. } = &mut release {
            *stage = CadenceStage::Release;
            *flags = 4;
        }
        let n = release.to_bytes(&mut buf).unwrap();
        assert!(n <= 64);
        assert_eq!(release, Packet::from_bytes(&buf[..n]).unwrap());
    }

    #[test]
    fn compact_cadence_ack_round_trip() {
        let pkt = Packet::CadenceAck {
            generation: 7,
            stage: CadenceStage::Applied,
            start_epoch: 0x1234_5678,
            end_epoch: 0,
            flags: 0,
        };
        let mut buf = [0u8; 16];
        let n = pkt.to_bytes(&mut buf).unwrap();
        assert!(n <= 16);
        assert_eq!(pkt, Packet::from_bytes(&buf[..n]).unwrap());

        for stage in [CadenceStage::Release, CadenceStage::Commit] {
            let control = Packet::CadenceAck {
                generation: 8,
                stage,
                start_epoch: 0x2345_6789,
                end_epoch: 0,
                flags: 4,
            };
            let n = control.to_bytes(&mut buf).unwrap();
            assert!(n <= 16);
            assert_eq!(control, Packet::from_bytes(&buf[..n]).unwrap());
        }
    }

    #[test]
    fn cadence_sample_matches_requested_wire_length() {
        let mut padding = Vec::<u8, { MAX_PAYLOAD + 32 }>::new();
        padding.resize(17, 0xA5).unwrap(); // 8-byte app + 12 overhead - 3 header
        let pkt = Packet::CadenceSample {
            generation: 1,
            padding,
        };
        let mut buf = [0u8; 64];
        let n = pkt.to_bytes(&mut buf).unwrap();
        assert_eq!(n, 20);
        assert_eq!(pkt, Packet::from_bytes(&buf[..n]).unwrap());
    }

    #[test]
    fn cadence_sample_can_cover_full_phy_packet() {
        let mut padding = Vec::<u8, { MAX_PAYLOAD + 32 }>::new();
        padding.resize(60, 0xA5).unwrap();
        let pkt = Packet::CadenceSample {
            generation: 1,
            padding,
        };
        let mut buf = [0u8; 64];
        assert_eq!(pkt.to_bytes(&mut buf).unwrap(), 63);
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
