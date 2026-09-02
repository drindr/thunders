//! Addressed recall and negotiation-phase framing over the one-way link.
//!
//! The reverse feedback packet is three bytes. While the link streams, it
//! carries the receiver's phase error as before. The receiver recalls one
//! sender into the negotiation phase by setting [`FEEDBACK_FLAG_NEG_REQ`]
//! and naming the target sender's on-air address prefix; a recalled sender
//! stops streaming application data and transmits [`ConfigFrame`] echoes
//! until the receiver clears the flag.
//!
//! The recall is level-triggered: the receiver asserts the flag in every
//! feedback beacon until it observes the sender's echo, so a lost beacon
//! only delays the transition by one batch. Both peers switch phase at the
//! batch boundary that already synchronizes hopping, so the recall never
//! changes the slot grid.

/// On-air feedback packet length in bytes.
pub const FEEDBACK_LEN: usize = 3;

/// Flags bit 0: the receiver requests negotiation with one sender.
pub const FEEDBACK_FLAG_NEG_REQ: u8 = 0x01;

/// First byte of every negotiation-phase forward frame.
pub const CONFIG_FRAME_MAGIC: u8 = 0xCF;

/// Config op: sender status echo (current on-air prefix and channel).
pub const CONFIG_OP_ECHO: u8 = 0x00;

/// Three-byte reverse feedback beacon.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FeedbackBeacon {
    /// Flags byte (`FEEDBACK_FLAG_*`).
    pub flags: u8,
    /// Phase-dependent byte 1: `diff_us` low byte, or the recall target's
    /// on-air address prefix while [`FEEDBACK_FLAG_NEG_REQ`] is set.
    pub byte1: u8,
    /// Phase-dependent byte 2: `diff_us` high byte, or zero while recalling.
    pub byte2: u8,
}

impl FeedbackBeacon {
    /// Streaming-phase timing feedback: `diff_us` little-endian.
    pub const fn time_diff(diff_us: i16) -> Self {
        let bytes = diff_us.to_le_bytes();
        Self {
            flags: 0,
            byte1: bytes[0],
            byte2: bytes[1],
        }
    }

    /// Addressed recall request for one sender's on-air address prefix.
    pub const fn recall(on_air_prefix: u8) -> Self {
        Self {
            flags: FEEDBACK_FLAG_NEG_REQ,
            byte1: on_air_prefix,
            byte2: 0,
        }
    }

    /// Serialize into the fixed three-byte feedback packet.
    pub const fn encode(&self) -> [u8; FEEDBACK_LEN] {
        [self.flags, self.byte1, self.byte2]
    }

    /// Parse a received feedback packet.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != FEEDBACK_LEN {
            return None;
        }
        Some(Self {
            flags: bytes[0],
            byte1: bytes[1],
            byte2: bytes[2],
        })
    }

    /// The addressed sender's on-air prefix while the receiver recalls.
    pub const fn recall_target(&self) -> Option<u8> {
        if self.flags & FEEDBACK_FLAG_NEG_REQ != 0 {
            Some(self.byte1)
        } else {
            None
        }
    }

    /// Streaming-phase timing error (meaningful only when not recalling).
    pub const fn diff_us(&self) -> i16 {
        i16::from_le_bytes([self.byte1, self.byte2])
    }
}

/// Six-byte negotiation-phase forward frame. Negotiation is an exclusive
/// phase: every forward frame carries this layout, so no streaming payload
/// needs a discriminant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ConfigFrame {
    /// Batch-level sequence for idempotent retries.
    pub seq: u8,
    /// Operation (`CONFIG_OP_*`).
    pub op: u8,
    /// Parameter selector (0 = TX power, 1 = channel).
    pub param: u8,
    /// Operation value, little-endian where numeric.
    pub value: [u8; 2],
}

impl ConfigFrame {
    /// Sender status echo: reports its own on-air prefix and channel so the
    /// receiver can confirm which sender entered the negotiation phase.
    pub const fn echo(seq: u8, on_air_prefix: u8, channel: u8) -> Self {
        Self {
            seq,
            op: CONFIG_OP_ECHO,
            param: 0,
            value: [on_air_prefix, channel],
        }
    }

    /// Serialize into the fixed six-byte state packet.
    pub fn encode(&self) -> [u8; 6] {
        [
            CONFIG_FRAME_MAGIC,
            self.seq,
            self.op,
            self.param,
            self.value[0],
            self.value[1],
        ]
    }

    /// Parse one received state packet as a negotiation frame.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != 6 || bytes[0] != CONFIG_FRAME_MAGIC {
            return None;
        }
        Some(Self {
            seq: bytes[1],
            op: bytes[2],
            param: bytes[3],
            value: [bytes[4], bytes[5]],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beacon_roundtrip() {
        let diff = FeedbackBeacon::time_diff(-513);
        let bytes = diff.encode();
        let parsed = FeedbackBeacon::decode(&bytes).unwrap();
        assert_eq!(parsed, diff);
        assert_eq!(parsed.diff_us(), -513);
        assert_eq!(parsed.recall_target(), None);

        let recall = FeedbackBeacon::recall(0xE7);
        let parsed = FeedbackBeacon::decode(&recall.encode()).unwrap();
        assert_eq!(parsed.recall_target(), Some(0xE7));
        assert_eq!(FeedbackBeacon::decode(&bytes[..2]), None);
    }

    #[test]
    fn config_frame_roundtrip() {
        let echo = ConfigFrame::echo(7, 0xE7, 43);
        let bytes = echo.encode();
        assert_eq!(bytes[0], CONFIG_FRAME_MAGIC);
        assert_eq!(ConfigFrame::decode(&bytes), Some(echo));
        assert_eq!(ConfigFrame::decode(&bytes[..5]), None);
        let mut not_config = bytes;
        not_config[0] = b'S';
        assert_eq!(ConfigFrame::decode(&not_config), None);
    }
}
