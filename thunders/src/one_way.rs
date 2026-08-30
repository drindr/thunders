//! Fixed-packet, compile-time one-way protocol engine.
//!
//! The engine is independent of a particular PHY scheduler. A backend sends
//! [`FixedOneWayFrame`] in the slots from [`FixedSlotPlan`] and supplies the
//! measured phase error used by reverse feedback.

use core::marker::PhantomData;
use heapless::Vec;

use crate::{
    config::MAX_PAYLOAD,
    mode::{LinkMode, LinkModeKind, OneWay},
    packet::FixedOneWayFrame,
};

/// One-way sender errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OneWaySendError {
    /// `PAYLOAD` and the selected mode's payload length differ.
    ModePayloadMismatch,
    /// Reliable mode already has an unacknowledged packet.
    AwaitingAck,
}

/// Result of consuming reverse feedback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FeedbackUpdate {
    /// Receiver-reported phase error in microseconds.
    pub diff_us: i16,
    /// True when the reliable in-flight packet was acknowledged.
    pub acknowledged: bool,
}

/// Bounded timing correction driven by ACK or TimeDiff feedback.
pub struct TimeDiffAligner<const GAIN_DIV: i16 = 4, const MAX_STEP_US: i16 = 20>;

impl<const GAIN_DIV: i16, const MAX_STEP_US: i16> TimeDiffAligner<GAIN_DIV, MAX_STEP_US> {
    /// Convert measured receiver error into a one-event start-time correction.
    pub const fn correction_us(diff_us: i16) -> i16 {
        let divisor = if GAIN_DIV == 0 { 1 } else { GAIN_DIV };
        let raw = diff_us / divisor;
        if raw > MAX_STEP_US {
            MAX_STEP_US
        } else if raw < -MAX_STEP_US {
            -MAX_STEP_US
        } else {
            raw
        }
    }
}

/// Compile-time fixed-payload sender.
pub struct OneWaySender<const PAYLOAD: usize, M: LinkMode> {
    next_seq: u16,
    in_flight_seq: u16,
    in_flight: [u8; PAYLOAD],
    waiting_ack: bool,
    _mode: PhantomData<M>,
}

impl<const PAYLOAD: usize, M: LinkMode> OneWaySender<PAYLOAD, M> {
    /// Create an empty sender.
    pub const fn new() -> Self {
        Self {
            next_seq: 0,
            in_flight_seq: 0,
            in_flight: [0; PAYLOAD],
            waiting_ack: false,
            _mode: PhantomData,
        }
    }

    fn validate_mode() -> Result<(), OneWaySendError> {
        if PAYLOAD == M::PAYLOAD_LEN
            && PAYLOAD <= MAX_PAYLOAD
            && (!M::RETRANSMIT || M::FEEDBACK_EVERY == 1)
        {
            Ok(())
        } else {
            Err(OneWaySendError::ModePayloadMismatch)
        }
    }

    fn frame(seq: u16, payload: &[u8; PAYLOAD]) -> FixedOneWayFrame {
        let mut bytes = Vec::<u8, MAX_PAYLOAD>::new();
        // validate_mode guarantees capacity.
        let _ = bytes.extend_from_slice(payload);
        FixedOneWayFrame::Data {
            seq,
            payload: bytes,
        }
    }

    /// Queue and build the next fixed Data frame.
    pub fn send(&mut self, payload: [u8; PAYLOAD]) -> Result<FixedOneWayFrame, OneWaySendError> {
        Self::validate_mode()?;
        if M::RETRANSMIT && self.waiting_ack {
            return Err(OneWaySendError::AwaitingAck);
        }
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        if M::RETRANSMIT {
            self.in_flight_seq = seq;
            self.in_flight = payload;
            self.waiting_ack = true;
        }
        Ok(Self::frame(seq, &payload))
    }

    /// Consume ACK or TimeDiff feedback.
    pub fn on_feedback(&mut self, frame: &FixedOneWayFrame) -> Option<FeedbackUpdate> {
        match frame {
            FixedOneWayFrame::Ack { seq, diff_us } if M::RETRANSMIT => {
                let acknowledged = self.waiting_ack && *seq == self.in_flight_seq;
                if acknowledged {
                    self.waiting_ack = false;
                }
                Some(FeedbackUpdate {
                    diff_us: *diff_us,
                    acknowledged,
                })
            }
            FixedOneWayFrame::TimeDiff { diff_us, .. } if !M::RETRANSMIT => Some(FeedbackUpdate {
                diff_us: *diff_us,
                acknowledged: false,
            }),
            _ => None,
        }
    }

    /// True while reliable mode waits for feedback.
    pub const fn waiting_ack(&self) -> bool {
        self.waiting_ack
    }
}

impl<const PAYLOAD: usize, const FEEDBACK_EVERY: u16>
    OneWaySender<PAYLOAD, OneWay<PAYLOAD, true, FEEDBACK_EVERY>>
{
    /// Rebuild the reliable in-flight frame after a timeout.
    pub fn retransmit(&self) -> Option<FixedOneWayFrame> {
        self.waiting_ack
            .then(|| Self::frame(self.in_flight_seq, &self.in_flight))
    }
}

impl<const PAYLOAD: usize, M: LinkMode> Default for OneWaySender<PAYLOAD, M> {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of accepting a forward Data frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OneWayReceive<const PAYLOAD: usize> {
    /// Forward sequence number.
    pub seq: u16,
    /// Fixed application payload.
    pub payload: [u8; PAYLOAD],
    /// True when this sequence was already delivered.
    pub duplicate: bool,
}

/// Compile-time fixed-payload receiver and feedback generator.
pub struct OneWayReceiver<const PAYLOAD: usize, M: LinkMode> {
    last_seq: u16,
    have_seq: bool,
    since_feedback: u16,
    _mode: PhantomData<M>,
}

impl<const PAYLOAD: usize, M: LinkMode> OneWayReceiver<PAYLOAD, M> {
    /// Create an empty receiver.
    pub const fn new() -> Self {
        Self {
            last_seq: 0,
            have_seq: false,
            since_feedback: 0,
            _mode: PhantomData,
        }
    }

    /// Decode one fixed Data frame and optionally return reverse feedback.
    pub fn receive(
        &mut self,
        frame: &FixedOneWayFrame,
        diff_us: i16,
    ) -> Result<(OneWayReceive<PAYLOAD>, Option<FixedOneWayFrame>), ()> {
        if PAYLOAD != M::PAYLOAD_LEN
            || PAYLOAD > MAX_PAYLOAD
            || M::FEEDBACK_EVERY == 0
            || (M::RETRANSMIT && M::FEEDBACK_EVERY != 1)
        {
            return Err(());
        }
        let FixedOneWayFrame::Data { seq, payload } = frame else {
            return Err(());
        };
        if payload.len() != PAYLOAD {
            return Err(());
        }
        let duplicate = self.have_seq && *seq == self.last_seq;
        if !duplicate {
            self.last_seq = *seq;
            self.have_seq = true;
        }
        let mut bytes = [0u8; PAYLOAD];
        bytes.copy_from_slice(payload);
        self.since_feedback = self.since_feedback.saturating_add(1);
        let feedback = if M::RETRANSMIT || self.since_feedback >= M::FEEDBACK_EVERY {
            self.since_feedback = 0;
            Some(if M::KIND == LinkModeKind::OneWayAck {
                FixedOneWayFrame::Ack {
                    seq: self.last_seq,
                    diff_us,
                }
            } else {
                FixedOneWayFrame::TimeDiff {
                    seq: self.last_seq,
                    diff_us,
                }
            })
        } else {
            None
        };
        Ok((
            OneWayReceive {
                seq: *seq,
                payload: bytes,
                duplicate,
            },
            feedback,
        ))
    }
}

impl<const PAYLOAD: usize, M: LinkMode> Default for OneWayReceiver<PAYLOAD, M> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mode::{OneWayAck, OneWayNoAck};

    type Ack4 = OneWayAck<4>;
    type NoAck4 = OneWayNoAck<4, 3>;

    #[test]
    fn diff_alignment_is_bounded() {
        type Align = TimeDiffAligner<4, 20>;
        assert_eq!(Align::correction_us(40), 10);
        assert_eq!(Align::correction_us(-40), -10);
        assert_eq!(Align::correction_us(200), 20);
        assert_eq!(Align::correction_us(-200), -20);
    }

    #[test]
    fn reliable_mode_blocks_until_matching_ack() {
        let mut tx = OneWaySender::<4, Ack4>::new();
        let mut rx = OneWayReceiver::<4, Ack4>::new();
        let data = tx.send([1, 2, 3, 4]).unwrap();
        assert_eq!(tx.retransmit(), Some(data.clone()));
        assert_eq!(tx.send([5, 6, 7, 8]), Err(OneWaySendError::AwaitingAck));
        let (received, ack) = rx.receive(&data, -7).unwrap();
        assert_eq!(received.payload, [1, 2, 3, 4]);
        let update = tx.on_feedback(&ack.unwrap()).unwrap();
        assert!(update.acknowledged);
        assert_eq!(update.diff_us, -7);
        assert!(!tx.waiting_ack());
    }

    #[test]
    fn noack_mode_reports_diff_periodically() {
        let mut tx = OneWaySender::<4, NoAck4>::new();
        let mut rx = OneWayReceiver::<4, NoAck4>::new();
        for n in 0..2 {
            let data = tx.send([n; 4]).unwrap();
            assert!(rx.receive(&data, 5).unwrap().1.is_none());
        }
        let data = tx.send([2; 4]).unwrap();
        assert_eq!(
            rx.receive(&data, 5).unwrap().1,
            Some(FixedOneWayFrame::TimeDiff { seq: 2, diff_us: 5 })
        );
    }
}
