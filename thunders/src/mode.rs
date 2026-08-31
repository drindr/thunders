//! Compile-time link modes and fixed-packet timing plans.

/// Extensible link-mode discriminator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinkModeKind {
    /// One forward data stream with reverse reliability feedback.
    OneWayAck,
    /// One forward data stream without retransmission; reverse traffic only
    /// carries periodic clock/phase error.
    OneWayNoAck,
}

/// Compile-time behavior required by the fixed-mode link engine.
pub trait LinkMode {
    /// Mode discriminator.
    const KIND: LinkModeKind;
    /// Exact application payload bytes in every data packet.
    const PAYLOAD_LEN: usize;
    /// Number of forward packets between reverse feedback opportunities.
    const FEEDBACK_EVERY: u16;
    /// Whether failed forward packets are retained for retransmission.
    const RETRANSMIT: bool;
}

/// One-way mode family. `ACK` is a compile-time capability, so no-ACK
/// instantiations do not expose a retransmit API.
pub struct OneWay<const PAYLOAD: usize, const ACK: bool, const FEEDBACK_EVERY: u16>;

impl<const PAYLOAD: usize, const ACK: bool, const FEEDBACK_EVERY: u16> LinkMode
    for OneWay<PAYLOAD, ACK, FEEDBACK_EVERY>
{
    const KIND: LinkModeKind = if ACK {
        LinkModeKind::OneWayAck
    } else {
        LinkModeKind::OneWayNoAck
    };
    const PAYLOAD_LEN: usize = PAYLOAD;
    const FEEDBACK_EVERY: u16 = FEEDBACK_EVERY;
    const RETRANSMIT: bool = ACK;
}

/// Reliable one-way stream. The first implementation is stop-and-wait.
pub type OneWayAck<const PAYLOAD: usize> = OneWay<PAYLOAD, true, 1>;

/// Unreliable one-way state snapshots. Delivery is not guaranteed; periodic
/// reverse feedback is used only for time alignment.
pub type OneWayNoAck<const PAYLOAD: usize, const DIFF_EVERY: u16 = 32> =
    OneWay<PAYLOAD, false, DIFF_EVERY>;

/// Semantic alias for continuously refreshed state where the newest value wins.
pub type OneWayState<const PAYLOAD: usize, const DIFF_EVERY: u16 = 32> =
    OneWayNoAck<PAYLOAD, DIFF_EVERY>;

/// Semantic alias for state changes/events that must be delivered.
pub type OneWayChanges<const PAYLOAD: usize> = OneWayAck<PAYLOAD>;

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
        margin_us: 25,
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

/// Reliable state-change Data carries only a two-byte sequence.
pub const ONE_WAY_ACK_DATA_OVERHEAD: usize = 2;
/// Reliable feedback carries sequence plus signed timing difference.
pub const ONE_WAY_ACK_FEEDBACK_LEN: usize = 4;
/// State snapshots carry no protocol header; timing-only feedback is one i16.
pub const ONE_WAY_STATE_FEEDBACK_LEN: usize = 2;

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
    let data_overhead = if M::RETRANSMIT {
        ONE_WAY_ACK_DATA_OVERHEAD
    } else {
        0
    };
    let feedback_wire = if M::RETRANSMIT {
        ONE_WAY_ACK_FEEDBACK_LEN
    } else {
        ONE_WAY_STATE_FEEDBACK_LEN
    };
    let data_wire = M::PAYLOAD_LEN.saturating_add(data_overhead);
    FixedSlotPlan {
        data_wire_len: if data_wire > u16::MAX as usize {
            u16::MAX
        } else {
            data_wire as u16
        },
        feedback_wire_len: feedback_wire as u16,
        data_slot_us: overhead.slot_us(air, data_wire),
        feedback_slot_us: overhead.slot_us(air, feedback_wire),
        feedback_every: M::FEEDBACK_EVERY,
        receiver_window_us: overhead.slot_us(air, data_wire) as u32 * M::FEEDBACK_EVERY as u32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type Ack8 = OneWayAck<8>;
    type Stream32 = OneWayNoAck<32, 16>;

    #[test]
    fn two_mbit_plan_is_payload_aware() {
        const ACK8: FixedSlotPlan =
            fixed_slot_plan::<Ack8>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);
        const STREAM32: FixedSlotPlan =
            fixed_slot_plan::<Stream32>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);
        assert_eq!(ACK8.data_wire_len, 10);
        assert_eq!(ACK8.data_slot_us, 341);
        assert_eq!(ACK8.feedback_slot_us, 317);
        assert_eq!(ACK8.receiver_window_us, 341);
        assert_eq!(ACK8.receiver_physical_slots(), 2);
        assert_eq!(STREAM32.data_wire_len, 32);
        assert_eq!(STREAM32.data_slot_us, 429);
        assert_eq!(STREAM32.feedback_every, 16);
        assert_eq!(STREAM32.receiver_window_us, 16 * 429);
        assert_eq!(STREAM32.period_us(), 16 * 429 + 309);
    }

    #[test]
    fn ack_and_noack_are_compile_time_distinct() {
        assert!(Ack8::RETRANSMIT);
        assert!(!Stream32::RETRANSMIT);
        assert_eq!(Ack8::KIND, LinkModeKind::OneWayAck);
        assert_eq!(Stream32::KIND, LinkModeKind::OneWayNoAck);
    }
}
