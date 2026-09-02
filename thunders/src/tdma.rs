//! Compile-time fixed TDMA schedules.

use crate::Address;

/// Uniform fixed TDMA schedule for up to eight Nordic logical-address senders.
///
/// `SENDERS`, `FRAME_US`, and `TX_US` are part of the type, so incompatible
/// nodes cannot accidentally share one schedule type. `TX_US` is the measured
/// duration of one complete local fixed-state transmit operation; it is used to
/// derive the master's post-END idle interval.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StaticTdma<const SENDERS: usize, const FRAME_US: u32, const TX_US: u32> {
    addresses: [Address; SENDERS],
}

impl<const SENDERS: usize, const FRAME_US: u32, const TX_US: u32>
    StaticTdma<SENDERS, FRAME_US, TX_US>
{
    /// Build and validate a uniform schedule.
    ///
    /// All addresses must have unique prefix bytes and the same four-byte base,
    /// matching Nordic RADIO logical-address demultiplexing.
    pub const fn new(addresses: [Address; SENDERS]) -> Self {
        assert!(SENDERS > 0, "TDMA needs at least one sender");
        assert!(SENDERS <= 8, "Nordic RADIO supports at most eight senders");
        assert!(FRAME_US > TX_US, "TDMA frame must exceed one TX operation");
        assert!(
            FRAME_US % SENDERS as u32 == 0,
            "TDMA frame must divide evenly"
        );

        let mut i = 0;
        while i < SENDERS {
            let mut j = 1;
            while j < 5 {
                assert!(
                    addresses[i].0[j] == addresses[0].0[j],
                    "TDMA addresses must share one four-byte base"
                );
                j += 1;
            }
            let mut other = 0;
            while other < i {
                assert!(
                    addresses[i].0[0] != addresses[other].0[0],
                    "TDMA sender prefixes must be unique"
                );
                other += 1;
            }
            i += 1;
        }

        Self { addresses }
    }

    /// Number of active senders in this schedule.
    pub const fn sender_count(&self) -> usize {
        SENDERS
    }

    /// Start-to-start period of each sender.
    pub const fn frame_us(&self) -> u32 {
        FRAME_US
    }

    /// Uniform logical phase width within one frame.
    pub const fn phase_us(&self) -> u32 {
        FRAME_US / SENDERS as u32
    }

    /// Measured duration of one complete fixed-state TX operation.
    pub const fn tx_us(&self) -> u32 {
        TX_US
    }

    /// Master's idle delay from the previous END to the next TX operation.
    pub const fn master_idle_us(&self) -> u32 {
        FRAME_US - TX_US
    }

    /// Logical addresses in sender-index order.
    pub const fn addresses(&self) -> &[Address; SENDERS] {
        &self.addresses
    }

    /// Select one compile-time sender index from this schedule.
    pub const fn sender<const INDEX: usize>(self) -> TdmaNode<SENDERS, INDEX, FRAME_US, TX_US> {
        assert!(INDEX < SENDERS, "TDMA sender index is out of range");
        TdmaNode { schedule: self }
    }
}

/// One compile-time-selected sender in a [`StaticTdma`] schedule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TdmaNode<const SENDERS: usize, const INDEX: usize, const FRAME_US: u32, const TX_US: u32>
{
    schedule: StaticTdma<SENDERS, FRAME_US, TX_US>,
}

impl<const SENDERS: usize, const INDEX: usize, const FRAME_US: u32, const TX_US: u32>
    TdmaNode<SENDERS, INDEX, FRAME_US, TX_US>
{
    /// Sender index used by RADIO logical-address matching.
    pub const fn index(&self) -> usize {
        INDEX
    }

    /// Sender's RADIO address.
    pub const fn address(&self) -> Address {
        self.schedule.addresses[INDEX]
    }

    /// Start-to-start period of this sender.
    pub const fn frame_us(&self) -> u32 {
        FRAME_US
    }

    /// Master's idle delay after END. Valid for sender index zero.
    pub const fn master_idle_us(&self) -> u32 {
        self.schedule.master_idle_us()
    }

    /// Delay after sender zero's END before this follower starts its TX op.
    ///
    /// Sender one starts immediately. Additional senders are separated by one
    /// uniform logical phase each.
    pub const fn after_master_end_us(&self) -> u32 {
        if INDEX <= 1 {
            0
        } else {
            (INDEX as u32 - 1) * self.schedule.phase_us()
        }
    }

    /// Add a microsecond delay to an absolute wrapping cycle timestamp.
    pub const fn deadline_after(&self, stamp: u32, delay_us: u32, cpu_mhz: u32) -> u32 {
        stamp.wrapping_add(delay_us.wrapping_mul(cpu_mhz))
    }

    /// Advance an absolute wrapping cycle deadline by one TDMA frame.
    pub const fn advance_deadline(&self, deadline: u32, cpu_mhz: u32) -> u32 {
        self.deadline_after(deadline, FRAME_US, cpu_mhz)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SCHEDULE: StaticTdma<2, 190, 100> = StaticTdma::new([
        Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
        Address([0xC3, 0xE7, 0xE7, 0xE7, 0xE7]),
    ]);

    #[test]
    fn derives_uniform_two_sender_schedule() {
        let sender0 = SCHEDULE.sender::<0>();
        let sender1 = SCHEDULE.sender::<1>();
        assert_eq!(SCHEDULE.sender_count(), 2);
        assert_eq!(SCHEDULE.phase_us(), 95);
        assert_eq!(sender0.master_idle_us(), 90);
        assert_eq!(sender1.after_master_end_us(), 0);
        assert_eq!(sender1.address().0[0], 0xC3);
        assert_eq!(sender1.advance_deadline(u32::MAX - 9, 1), 180);
    }
}
