//! Frequency-hop scheduler.

use crate::config::{NetworkId, DEFAULT_HOP_SEQUENCE, HOP_SEQUENCE_LEN};

/// Maintains the shared channel hopping sequence.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Scheduler {
    sequence: [u8; HOP_SEQUENCE_LEN],
    index: u8,
}

impl Scheduler {
    /// Build a scheduler from a network identifier.
    ///
    /// The network ID seeds a simple LCG so different networks use
    /// different hop orders.
    pub fn new(network: NetworkId) -> Self {
        let mut sequence = DEFAULT_HOP_SEQUENCE;
        // Simple LCG permutation.
        let seed = u32::from_le_bytes(network);
        let mut state = seed.wrapping_add(0x9E37_79B9);
        for i in 0..HOP_SEQUENCE_LEN {
            state = state.wrapping_mul(11035_15245).wrapping_add(12345);
            let j = (state as usize) % HOP_SEQUENCE_LEN;
            sequence.swap(i, j);
        }
        Self {
            sequence,
            index: 0,
        }
    }

    /// Current channel index.
    pub fn index(&self) -> u8 {
        self.index
    }

    /// Current raw channel value.
    pub fn current(&self) -> u8 {
        self.sequence[self.index as usize % HOP_SEQUENCE_LEN]
    }

    /// Advance to the next channel.
    pub fn advance(&mut self) {
        self.index = self.index.wrapping_add(1) % HOP_SEQUENCE_LEN as u8;
    }

    /// Set the scheduler to a known index (e.g. from a received beacon).
    pub fn sync(&mut self, index: u8) {
        self.index = index % HOP_SEQUENCE_LEN as u8;
    }

    /// Predict the channel `offset` hops ahead.
    pub fn predict(&self, offset: u8) -> u8 {
        let idx = self.index.wrapping_add(offset) % HOP_SEQUENCE_LEN as u8;
        self.sequence[idx as usize]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_around() {
        let mut s = Scheduler::new([0xAA, 0xBB, 0xCC, 0xDD]);
        let first = s.current();
        for _ in 0..HOP_SEQUENCE_LEN {
            s.advance();
        }
        assert_eq!(s.current(), first);
        assert_eq!(s.index(), 0);
    }

    #[test]
    fn default_sequence_is_fixed() {
        // The default hop sequence is a single fixed channel while the two
        // free-running nodes cannot stay phase-locked (see config.rs).
        let a = Scheduler::new([1, 2, 3, 4]);
        let b = Scheduler::new([5, 6, 7, 8]);
        assert_eq!(a.sequence, DEFAULT_HOP_SEQUENCE);
        assert_eq!(b.sequence, DEFAULT_HOP_SEQUENCE);
    }
}
