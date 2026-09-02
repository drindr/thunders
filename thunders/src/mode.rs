//! Compile-time link modes and fixed-packet timing plans.

/// Compile-time behavior required by the fixed-mode link engine.
pub trait LinkMode {
    /// Exact application payload bytes in every data packet.
    const PAYLOAD_LEN: usize;
    /// Number of forward packets between reverse feedback opportunities.
    const FEEDBACK_EVERY: u16;
}

/// One-way mode family. Reliability is a runtime phase, not a type
/// parameter: the wire format and timing plan are identical whether the
/// link currently streams state or has been recalled into the negotiation
/// phase (see [`crate::negotiation`]).
pub struct OneWay<const PAYLOAD: usize, const FEEDBACK_EVERY: u16>;

impl<const PAYLOAD: usize, const FEEDBACK_EVERY: u16> LinkMode for OneWay<PAYLOAD, FEEDBACK_EVERY> {
    const PAYLOAD_LEN: usize = PAYLOAD;
    const FEEDBACK_EVERY: u16 = FEEDBACK_EVERY;
}

/// Unreliable one-way state snapshots. Delivery is not guaranteed; periodic
/// reverse feedback carries timing error and the optional addressed recall.
pub type OneWayState<const PAYLOAD: usize, const DIFF_EVERY: u16 = 32> =
    OneWay<PAYLOAD, DIFF_EVERY>;

/// Radio framing timing used by the const slot planner.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AirTiming {
    /// Fixed pre-payload time in microseconds.
    pub prefix_us: u16,
    /// Time per payload/length/CRC byte in microseconds.
    pub byte_us: u16,
}

impl AirTiming {
    /// Nordic/BLE-compatible 2 Mbit framing: 16-bit preamble.
    pub const NRF_2MBIT: Self = Self {
        prefix_us: 28,
        byte_us: 4,
    };
    /// Nordic/BLE-compatible 1 Mbit framing: 8-bit preamble.
    pub const NRF_1MBIT: Self = Self {
        prefix_us: 48,
        byte_us: 8,
    };

    /// On-air duration for a fixed-STATLEN frame, including two-byte CRC and
    /// no transmitted length field.
    pub const fn airtime_us(self, wire_len: usize) -> u32 {
        self.prefix_us as u32 + self.byte_us as u32 * (wire_len as u32 + 2)
    }
}

/// Hardware/backend overhead around on-air work.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotOverhead {
    /// Slot START to TXEN.
    pub tx_en_us: u16,
    /// TXEN to READY/on-air.
    pub tx_ramp_us: u16,
    /// Slot START to RXEN.
    pub rx_en_us: u16,
    /// RXEN to READY/listening.
    pub rx_ramp_us: u16,
    /// Receiver DISABLE/PLL/RXEN turnaround before it can catch the next packet.
    pub rx_restart_us: u16,
    /// Reserved shutdown/handback tail after the packet.
    pub tail_us: u16,
    /// Empirical publication/IRQ jitter margin.
    pub margin_us: u16,
    /// Mandatory MPSL gap between grants.
    pub interslot_gap_us: u16,
}

impl SlotOverhead {
    /// Cross-board MPSL timing budget. Packet-to-packet RX restart is kept at
    /// zero here: backend hot-path overhead must be optimized, not hidden in
    /// the protocol period.
    pub const MPSL_CONSERVATIVE: Self = Self {
        tx_en_us: 8,
        tx_ramp_us: 42,
        rx_en_us: 8,
        rx_ramp_us: 42,
        rx_restart_us: 0,
        tail_us: 40,
        margin_us: 0,
        interslot_gap_us: 150,
    };

    /// Minimum safe start-to-start period for a fixed wire length.
    pub const fn slot_us(self, air: AirTiming, wire_len: usize) -> u16 {
        let airtime = air.airtime_us(wire_len);
        let tx = self.tx_en_us as u32 + self.tx_ramp_us as u32 + airtime;
        let rx = self.rx_en_us as u32 + self.rx_ramp_us as u32 + airtime;
        let work = if tx > rx { tx } else { rx };
        let total = work
            + self.rx_restart_us as u32
            + self.tail_us as u32
            + self.margin_us as u32
            + self.interslot_gap_us as u32;
        if total > u16::MAX as u32 {
            u16::MAX
        } else {
            total as u16
        }
    }
}

/// Fully compile-time timing result for a fixed one-way mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FixedSlotPlan {
    /// Serialized bytes in every forward Data packet.
    pub data_wire_len: u16,
    /// Serialized bytes in ACK or TimeDiff feedback.
    pub feedback_wire_len: u16,
    /// Forward data slot period.
    pub data_slot_us: u16,
    /// Reverse feedback slot period.
    pub feedback_slot_us: u16,
    /// Forward packets per reverse feedback packet.
    pub feedback_every: u16,
    /// One long receiver grant covering the complete forward batch.
    pub receiver_window_us: u32,
}

impl FixedSlotPlan {
    /// Logical transmitter events in one data/feedback cycle.
    pub const fn period_slots(self) -> u16 {
        self.feedback_every.saturating_add(1)
    }

    /// Wall-clock duration of one repeating cycle.
    pub const fn period_us(self) -> u32 {
        self.receiver_window_us + self.feedback_slot_us as u32
    }

    /// Receiver uses one long RX grant plus one reverse feedback grant.
    pub const fn receiver_physical_slots(self) -> u8 {
        2
    }

    /// True when `phase` is the reverse feedback slot.
    pub const fn is_feedback_phase(self, phase: u16) -> bool {
        let period = self.period_slots();
        let divisor = if period == 0 { 1 } else { period };
        phase % divisor == self.feedback_every
    }
}

/// Reverse feedback is three bytes on air: one flags byte plus two
/// phase-dependent bytes (see [`crate::negotiation`]).
pub const ONE_WAY_FEEDBACK_LEN: usize = 3;

/// Round a compile-time duration upward to a hardware scheduling quantum.
pub const fn round_up_us(value: u16, quantum: u16) -> u16 {
    if quantum == 0 {
        value
    } else {
        let rem = value % quantum;
        if rem == 0 {
            value
        } else {
            value.saturating_add(quantum - rem)
        }
    }
}

/// Build a const timing plan from a mode's associated constants.
pub const fn fixed_slot_plan<M: LinkMode>(air: AirTiming, overhead: SlotOverhead) -> FixedSlotPlan {
    let data_wire = M::PAYLOAD_LEN;
    FixedSlotPlan {
        data_wire_len: if data_wire > u16::MAX as usize {
            u16::MAX
        } else {
            data_wire as u16
        },
        feedback_wire_len: ONE_WAY_FEEDBACK_LEN as u16,
        data_slot_us: overhead.slot_us(air, data_wire),
        feedback_slot_us: overhead.slot_us(air, ONE_WAY_FEEDBACK_LEN),
        feedback_every: M::FEEDBACK_EVERY,
        receiver_window_us: overhead.slot_us(air, data_wire) as u32 * M::FEEDBACK_EVERY as u32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type Stream8 = OneWay<8, 1>;
    type Stream32 = OneWay<32, 16>;

    #[test]
    fn two_mbit_plan_is_payload_aware() {
        const STREAM8: FixedSlotPlan =
            fixed_slot_plan::<Stream8>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);
        const STREAM32: FixedSlotPlan =
            fixed_slot_plan::<Stream32>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);
        assert_eq!(STREAM8.data_wire_len, 8);
        assert_eq!(STREAM8.data_slot_us, 308);
        assert_eq!(STREAM8.feedback_wire_len, ONE_WAY_FEEDBACK_LEN as u16);
        assert_eq!(STREAM8.feedback_slot_us, 288);
        assert_eq!(STREAM8.receiver_window_us, 308);
        assert_eq!(STREAM8.receiver_physical_slots(), 2);
        assert_eq!(STREAM32.data_wire_len, 32);
        assert_eq!(STREAM32.data_slot_us, 404);
        assert_eq!(STREAM32.feedback_every, 16);
        assert_eq!(STREAM32.receiver_window_us, 16 * 404);
        assert_eq!(STREAM32.period_us(), 16 * 404 + 288);
    }
}
