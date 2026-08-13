//! Shared-memory mailbox between the nRF5340's two cores (the host/controller
//! split).
//!
//! The network core owns the RADIO and runs the link; the application core
//! runs the host and exchanges payloads through this fixed-address mailbox,
//! with the nRF5340 IPC peripheral used for signalling (see the examples).
//!
//! IPC convention:
//!   - channel 0 / event 0: application -> network ("TX ready")
//!   - channel 1 / event 1: network -> application ("RX ready")

use crate::MAX_PAYLOAD;

/// Base address of the shared mailbox, in the application core's RAM. The
/// net core's RAM (0x21000000) is private to it; only the app core's RAM
/// (0x20000000) is reachable by both cores through the DCNF.
pub const SHARED_MEM_BASE: usize = 0x2007_F000;

/// One fixed-size payload slot.
#[repr(C)]
pub struct Slot {
    /// Payload length (0..=MAX_PAYLOAD).
    pub len: u8,
    /// 1 = a fresh payload is present, 0 = consumed.
    pub valid: u8,
    /// Payload bytes.
    pub data: [u8; MAX_PAYLOAD],
}

/// The mailbox: `tx` (application -> network) and `rx` (network -> application).
#[repr(C)]
pub struct SharedMailbox {
    pub tx: Slot,
    pub rx: Slot,
}

/// Borrow the shared mailbox at its fixed address.
pub fn mailbox() -> &'static mut SharedMailbox {
    // Safety: SHARED_MEM_BASE is reserved for this structure by the net core's
    // linker script; both cores agree on the layout.
    unsafe { &mut *(SHARED_MEM_BASE as *mut SharedMailbox) }
}

impl SharedMailbox {
    /// Application core: queue a payload for the network core to transmit.
    pub fn put_tx(&mut self, data: &[u8]) {
        let n = data.len().min(MAX_PAYLOAD);
        self.tx.data[..n].copy_from_slice(&data[..n]);
        self.tx.len = n as u8;
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
        self.tx.valid = 1;
    }

    /// Network core: take the next TX payload into `out` (non-blocking).
    /// Returns the payload length, or `None` when nothing is queued.
    pub fn take_tx(&mut self, out: &mut [u8]) -> Option<usize> {
        if self.tx.valid == 0 {
            return None;
        }
        let n = (self.tx.len as usize).min(MAX_PAYLOAD).min(out.len());
        out[..n].copy_from_slice(&self.tx.data[..n]);
        self.tx.valid = 0;
        Some(n)
    }

    /// Network core: deliver a received payload to the application core.
    pub fn put_rx(&mut self, data: &[u8]) {
        let n = data.len().min(MAX_PAYLOAD);
        self.rx.data[..n].copy_from_slice(&data[..n]);
        self.rx.len = n as u8;
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
        self.rx.valid = 1;
    }

    /// Application core: take the next RX payload into `out` (non-blocking).
    /// Returns the payload length, or `None` when nothing is received.
    pub fn take_rx(&mut self, out: &mut [u8]) -> Option<usize> {
        if self.rx.valid == 0 {
            return None;
        }
        let n = (self.rx.len as usize).min(MAX_PAYLOAD).min(out.len());
        out[..n].copy_from_slice(&self.rx.data[..n]);
        self.rx.valid = 0;
        Some(n)
    }
}
