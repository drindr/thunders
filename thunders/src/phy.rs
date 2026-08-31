//! Minimal asynchronous PHY abstraction.

use embassy_time::Duration;

use crate::{config::Address, error::Error};

/// Hardware timestamp paired with a received fixed packet.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxTiming {
    /// Hardware slot that executed RX.
    pub local_slot: u32,
    /// DWT cycle at the slot START callback.
    pub slot_start_cyc: u32,
    /// RADIO ADDRESS offset from slot START, in microseconds.
    pub address_offset_us: u32,
    /// DWT cycles per microsecond.
    pub cycles_per_us: u32,
}

/// Radio interface shared by the bare and MPSL adapters.
#[allow(async_fn_in_trait)]
pub trait Phy {
    /// PHY-specific error type.
    type Error;

    /// Select the radio channel.
    async fn set_channel(&mut self, ch: u8);
    /// Set the five-byte radio address.
    async fn set_address(&mut self, addr: &Address);
    /// Transmit one packet.
    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>>;
    /// Receive one packet or return `None` on timeout.
    async fn receive(
        &mut self,
        buf: &mut [u8],
        timeout: Duration,
    ) -> Result<Option<usize>, Error<Self::Error>>;

    /// Publish an RX op for an absolute hardware slot.
    async fn op_publish_rx(&mut self, _buf: &mut [u8], _target: u32) {}
    /// Publish a TX op for an absolute hardware slot.
    async fn op_publish_tx(
        &mut self,
        _pkt: &[u8],
        _target: u32,
        _grace: u8,
    ) -> Result<(), Error<Self::Error>> {
        Ok(())
    }
    /// Collect a previously published slot operation.
    async fn op_collect(&mut self, _slot: u32) -> Option<usize> {
        None
    }
    /// Arm a two-duration compile-time profile at an exact slot.
    fn schedule_slot_profile(
        &mut self,
        _short_us: u16,
        _long_us: u16,
        _period: u16,
        _short_phases: u16,
        _central_apply_slot: u32,
        _local_apply_slot: u32,
    ) -> bool {
        false
    }
    /// Current hardware slot counter, or zero for unpaced PHYs.
    fn slot_count(&self) -> u32 {
        0
    }
    /// Flush stale radio state.
    async fn flush(&mut self);
}
