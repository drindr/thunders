//! Link-management layer.
//!
//! The fragment primitives in this module are intentionally kept as pending
//! building blocks until `frame` gains a wire marker for fragment payloads;
//! the legacy seq-based `on_nack`/`nack` helpers remain for the window tests.
#![allow(missing_docs)]
#![allow(dead_code)]
//!
//! This layer sits between the raw PHY slot framing and the application
//! transport. It owns:
//!
//! - slot-based NACK retransmission state,
//! - slot cumulative ACK (the `ack` field, which releases delivered slots),
//! - packet fragmentation / reassembly for payloads larger than one Data
//!   slot.
//!
//! The goal is that `Central` / `Peripheral` only move user bytes; all
//! ordering, duplicate detection, and reliability bookkeeping lives here.

use heapless::Vec;

use crate::config::{MAX_PAYLOAD, MAX_RETRIES, NACK_BYTES, RETRY_TIMEOUT_SLOTS, WINDOW_SIZE};

/// Maximum number of fragments a single application payload may be split
/// into. Fits a 4-bit fragment index in the fragment header.
pub const MAX_FRAGMENTS: usize = 16;

/// Application bytes per fragment (the fragment header occupies 3 of the
/// [`MAX_PAYLOAD`] Data bytes).
pub const MAX_FRAGMENT_CHUNK: usize = MAX_PAYLOAD - 3;

/// Maximum reassembled application-payload size (all fragments combined).
pub const MAX_FRAGMENT_PAYLOAD: usize = MAX_FRAGMENTS * MAX_FRAGMENT_CHUNK;

/// Fragment header layout (3 bytes):
/// - byte 0: base seq low
/// - byte 1: base seq high
/// - byte 2 bits 0..3: fragment index (0-based)
/// - byte 2 bits 4..7: total fragment count minus one
///
/// The payload of each fragment is the raw application byte slice, so each
/// fragment can carry up to `MAX_PAYLOAD - 3` application bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FragmentHeader {
    /// The Data seq of the first fragment of this application payload.
    pub base_seq: u16,
    /// 0-based fragment index.
    pub index: u8,
    /// Total number of fragments minus one.
    pub total_minus_one: u8,
}

impl FragmentHeader {
    pub fn encode(base_seq: u16, index: u8, total: u8) -> Option<[u8; 3]> {
        if index >= MAX_FRAGMENTS as u8 || total == 0 || total > MAX_FRAGMENTS as u8 {
            return None;
        }
        Some([
            (base_seq & 0xFF) as u8,
            (base_seq >> 8) as u8,
            (index << 4) | (total - 1),
        ])
    }

    pub fn decode(h: &[u8; 3]) -> Self {
        Self {
            base_seq: u16::from_le_bytes([h[0], h[1]]),
            index: h[2] >> 4,
            total_minus_one: h[2] & 0x0F,
        }
    }

    pub fn total(&self) -> u8 {
        self.total_minus_one + 1
    }
}

/// Split `payload` into fragments that fit into `MAX_PAYLOAD - 1` bytes each.
pub fn split_payload(
    base_seq: u16,
    payload: &[u8],
    out: &mut [Vec<u8, MAX_PAYLOAD>; MAX_FRAGMENTS],
) -> Result<usize, ()> {
    let chunk = MAX_FRAGMENT_CHUNK;
    let total = payload.len().div_ceil(chunk.max(1)).max(1);
    if total > MAX_FRAGMENTS {
        return Err(());
    }
    for i in 0..total {
        let h = FragmentHeader::encode(base_seq, i as u8, total as u8).ok_or(())?;
        let start = i * chunk;
        let end = (start + chunk).min(payload.len());
        let mut v = Vec::new();
        v.extend_from_slice(&h).map_err(|_| ())?;
        v.extend_from_slice(&payload[start..end]).map_err(|_| ())?;
        out[i] = v;
    }
    Ok(total)
}

/// Reassemble a payload from fragments.
///
/// The caller provides a buffer and fills `fragments[index]` with the payload
/// bytes (without the fragment header). `delivered` is set to `true` once the
/// last missing fragment is filled and the complete payload is copied into
/// `out`.
pub struct Reassembler {
    pub fragments: [Option<Vec<u8, MAX_PAYLOAD>>; MAX_FRAGMENTS],
    pub(crate) total: u8,
    pub(crate) remaining: u8,
    pub(crate) base_seq: u16,
}

impl Reassembler {
    pub fn new(total: u8, base_seq: u16) -> Option<Self> {
        if total == 0 || total as usize > MAX_FRAGMENTS {
            return None;
        }
        Some(Self {
            fragments: core::array::from_fn(|_| None),
            total,
            remaining: total,
            base_seq,
        })
    }

    pub fn add(&mut self, index: u8, bytes: &[u8]) -> bool {
        if index >= self.total || self.fragments[index as usize].is_some() {
            return false;
        }
        let mut v = Vec::new();
        if v.extend_from_slice(bytes).is_err() {
            return false;
        }
        self.fragments[index as usize] = Some(v);
        self.remaining -= 1;
        true
    }

    pub fn complete(&self) -> bool {
        self.remaining == 0
    }

    pub fn total(&self) -> u8 {
        self.total
    }

    pub fn base_seq(&self) -> u16 {
        self.base_seq
    }

    pub fn assemble(&self, out: &mut Vec<u8, MAX_FRAGMENT_PAYLOAD>) -> bool {
        if !self.complete() {
            return false;
        }
        for i in 0..self.total as usize {
            if let Some(frag) = &self.fragments[i] {
                if out.extend_from_slice(frag).is_err() {
                    return false;
                }
            } else {
                return false;
            }
        }
        true
    }
}

/// The link-management facade: owns the sender and receiver windows plus the
/// slot-NACK run bookkeeping. `Central` / `Peripheral` should only talk to
/// this type, never to the raw windows.
pub(crate) struct LinkMgmt {
    pub tx: TxWindow,
    pub rx: RxWindow,
    pub tx_run_slots: [Option<TxRunSlot>; WINDOW_SIZE],
    pub rx_run_mask: [u8; NACK_BYTES],
    pub nack_for_peer: [u8; NACK_BYTES],
    pub tx_frags: [Vec<u8, MAX_PAYLOAD>; MAX_FRAGMENTS],
    pub tx_frag_total: usize,
    pub tx_frag_next: usize,
    pub rx_reasm: Option<Reassembler>,
}

impl LinkMgmt {
    pub fn new() -> Self {
        Self {
            tx: TxWindow::new(),
            rx: RxWindow::new(),
            tx_run_slots: [None; WINDOW_SIZE],
            rx_run_mask: [0; NACK_BYTES],
            nack_for_peer: [0; NACK_BYTES],
            tx_frags: core::array::from_fn(|_| Vec::new()),
            tx_frag_total: 0,
            tx_frag_next: 0,
            rx_reasm: None,
        }
    }

    pub fn record_tx_slot(&mut self, slot: u8, seq: u16) {
        for entry in self.tx_run_slots.iter_mut() {
            if entry.is_none() {
                *entry = Some(TxRunSlot { slot, seq });
                break;
            }
        }
    }

    pub fn set_rx_slot(&mut self, slot_idx: usize) {
        nack_set(&mut self.rx_run_mask, slot_idx);
    }

    pub fn finish_rx_run(&mut self, run_len: usize) {
        self.nack_for_peer = nack_from_mask(run_len, &self.rx_run_mask);
        self.rx_run_mask = [0; NACK_BYTES];
    }

    pub fn nack_vec_for_peer(&self, run_len: usize) -> Vec<u8, NACK_BYTES> {
        nack_vec(run_len, &self.nack_for_peer)
    }

    pub fn queue_tx_payload(&mut self, base_seq: u16, payload: &[u8]) -> Result<(), ()> {
        let total = split_payload(base_seq, payload, &mut self.tx_frags)?;
        self.tx_frag_total = total;
        self.tx_frag_next = 0;
        Ok(())
    }

    pub fn has_pending_tx_frags(&self) -> bool {
        self.tx_frag_next < self.tx_frag_total
    }

    pub fn next_tx_fragment(&mut self) -> Option<Vec<u8, MAX_PAYLOAD>> {
        if self.tx_frag_next >= self.tx_frag_total {
            return None;
        }
        let frag = self.tx_frags[self.tx_frag_next].clone();
        self.tx_frag_next += 1;
        Some(frag)
    }

    pub fn handle_rx_fragment(
        &mut self,
        payload: &[u8],
    ) -> Result<Option<Vec<u8, MAX_FRAGMENT_PAYLOAD>>, ()> {
        if payload.len() < 3 {
            return Err(());
        }
        let mut hbuf = [0u8; 3];
        hbuf.copy_from_slice(&payload[..3]);
        let hdr = FragmentHeader::decode(&hbuf);
        let frag_payload = &payload[3..];
        if hdr.total() == 1 {
            let mut out = Vec::<u8, MAX_FRAGMENT_PAYLOAD>::new();
            out.extend_from_slice(frag_payload).map_err(|_| ())?;
            return Ok(Some(out));
        }
        if self.rx_reasm.as_ref().map(|r| (r.total(), r.base_seq))
            != Some((hdr.total(), hdr.base_seq))
        {
            self.rx_reasm = Reassembler::new(hdr.total(), hdr.base_seq);
        }
        let r = self.rx_reasm.as_mut().ok_or(())?;
        if !r.add(hdr.index, frag_payload) {
            return Err(());
        }
        if r.complete() {
            let mut out = Vec::<u8, MAX_FRAGMENT_PAYLOAD>::new();
            if r.assemble(&mut out) {
                self.rx_reasm = None;
                return Ok(Some(out));
            }
            return Err(());
        }
        Ok(None)
    }

    pub fn nack_nonzero(&self) -> bool {
        nack_nonzero(&self.nack_for_peer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn link_mgmt_fragment_queue() {
        let mut lm = LinkMgmt::new();
        let payload: [u8; 40] = core::array::from_fn(|i| i as u8);
        lm.queue_tx_payload(0, &payload).unwrap();
        assert_eq!(lm.tx_frag_total, 2);
        assert_eq!(lm.tx_frag_next, 0);
        let f0 = lm.next_tx_fragment().unwrap();
        let f1 = lm.next_tx_fragment().unwrap();
        assert_eq!(lm.next_tx_fragment(), None);
        let mut hbuf = [0u8; 3];
        hbuf.copy_from_slice(&f0[..3]);
        assert_eq!(FragmentHeader::decode(&hbuf).index, 0);
        let mut hbuf = [0u8; 3];
        hbuf.copy_from_slice(&f1[..3]);
        assert_eq!(FragmentHeader::decode(&hbuf).index, 1);
    }

    #[test]
    fn link_mgmt_fragment_short_payload_is_error() {
        let mut lm = LinkMgmt::new();
        assert!(lm.handle_rx_fragment(&[1, 2]).is_err());
    }

    #[test]
    fn link_mgmt_reassemble_fragments() {
        let mut lm = LinkMgmt::new();
        let payload: [u8; 40] = core::array::from_fn(|i| i as u8);
        lm.queue_tx_payload(0, &payload).unwrap();
        let f0 = lm.next_tx_fragment().unwrap();
        let f1 = lm.next_tx_fragment().unwrap();
        assert_eq!(lm.handle_rx_fragment(&f1).unwrap(), None); // out of order
        let out = lm.handle_rx_fragment(&f0).unwrap().unwrap();
        assert_eq!(&out, &payload);
    }

    #[test]
    fn rx_skip_to_unblocks_in_order_delivery() {
        let mut rx = RxWindow::new();
        // seq 2 arrives first; seq 0 and 1 are never coming because the
        // sender dropped them.
        assert!(rx.receive(2, &[2]));
        assert_eq!(rx.ack(), 0xFFFF);
        rx.skip_to(0);
        // The baseline moves to 1; the sender drops seq 1 as well, so
        // skip_to(1) makes seq 2 the in-order head.
        assert_eq!(rx.ack(), 0);
        rx.skip_to(1);
        assert_eq!(rx.peek_len(), Some(1));
        assert_eq!(rx.pop_head().unwrap().payload[0], 2);
        assert_eq!(rx.ack(), 2);
    }

    #[test]
    fn arq_drop_hint_recovers_stream() {
        let mut tx = TxWindow::new();
        let mut rx = RxWindow::new();
        const TOTAL: u16 = 40;
        let mut next_new: u16 = 0;
        let mut delivered: Vec<u8, 128> = Vec::new();
        let mut pending_drop: Option<u16> = None;
        let mut drops: u32 = 0;
        let mut dropped_seqs: Vec<u16, 128> = Vec::new();
        let mut send_count: u32 = 0;

        for _ in 0..500_000 {
            tx.tick();

            // Offer the next application packet whenever the window has room.
            while next_new < TOTAL && !tx.is_full() {
                tx.enqueue(&[next_new as u8]);
                next_new += 1;
            }

            // One slot: either the drop notification (re-sent until ACKed),
            // or a normal data transmission.
            if let Some(drop_seq) = pending_drop.take() {
                rx.skip_to(drop_seq);
            } else if let Some(seq) = tx.pick() {
                let payload = tx.entry(seq).payload[0];
                // 20% background loss, plus seq 3 is always lost so the
                // retry budget is guaranteed to be exercised once.
                let lost = seq == 3 || send_count % 5 == 4;
                if !lost {
                    rx.receive(seq, &[payload]);
                }
                if tx.mark_sent(seq) {
                    tx.drop(seq);
                    drops += 1;
                    dropped_seqs.push(seq).ok();
                    pending_drop = Some(seq);
                }
                send_count += 1;
            }

            // Deliver the reorder buffer in order.
            while rx.peek_len().is_some() {
                let e = rx.pop_head().unwrap();
                delivered.push(e.payload[0]).ok();
            }

            // Lossless ACK path. The receiver's ACK clears a pending drop
            // once it has advanced past that seq.
            tx.on_ack(rx.ack());
            if let Some(drop_seq) = pending_drop {
                if !seq_gt(drop_seq, tx.tx_acked) {
                    pending_drop = None;
                }
            }

            if next_new == TOTAL
                && tx.inflight == 0
                && pending_drop.is_none()
                && delivered.len() + drops as usize == TOTAL as usize
            {
                break;
            }
        }

        assert_eq!(delivered.len() + drops as usize, TOTAL as usize);
        assert!(drops > 0, "the lossy channel should drop someone");
        let mut expected: Vec<u8, 128> = Vec::new();
        for i in 0..TOTAL {
            if !dropped_seqs.contains(&i) {
                expected.push(i as u8).ok();
            }
        }
        assert_eq!(delivered, expected);
    }

    #[test]
    fn rx_skip_to_marks_heard_before_any_data() {
        let mut rx = RxWindow::new();
        // The sender dropped the only packet it ever sent. The receiver
        // hears the drop notification before any Data, so it must still be
        // able to ACK the skip and let the sender move on to seq 1.
        assert!(!rx.have);
        rx.skip_to(0);
        assert!(rx.have);
        assert_eq!(rx.ack(), 0);
        assert!(rx.in_window(1));
        assert!(rx.receive(1, &[1]));
        assert_eq!(rx.peek_len(), Some(1));
        assert_eq!(rx.pop_head().unwrap().payload[0], 1);
    }

    #[test]
    fn link_mgmt_run_bookkeeping() {
        let mut lm = LinkMgmt::new();
        lm.record_tx_slot(2, 7);
        lm.record_tx_slot(4, 9);
        assert_eq!(lm.tx_run_slots[0], Some(TxRunSlot { slot: 2, seq: 7 }));
        assert_eq!(lm.tx_run_slots[1], Some(TxRunSlot { slot: 4, seq: 9 }));

        lm.set_rx_slot(1);
        lm.set_rx_slot(3);
        lm.finish_rx_run(8);
        // Slots 0,2,4,5,6,7 missing -> byte0 bits 0,2,4,5,6,7 = 0b1111_0101 = 0xF5
        assert_eq!(lm.nack_for_peer[0], 0xF5);
        assert!(lm.nack_nonzero());

        let v = lm.nack_vec_for_peer(8);
        assert_eq!(v.len(), 1);
        assert_eq!(v[0], 0xF5);
    }

    #[test]
    pub(crate) fn fragment_round_trip() {
        let payload: [u8; 40] = core::array::from_fn(|i| i as u8);
        let mut frags: [Vec<u8, MAX_PAYLOAD>; MAX_FRAGMENTS] = core::array::from_fn(|_| Vec::new());
        let n = split_payload(0, &payload, &mut frags).unwrap();
        assert_eq!(n, 2);
        let mut hbuf = [0u8; 3];
        hbuf.copy_from_slice(&frags[0][..3]);
        let h0 = FragmentHeader::decode(&hbuf);
        let mut hbuf = [0u8; 3];
        hbuf.copy_from_slice(&frags[1][..3]);
        let h1 = FragmentHeader::decode(&hbuf);
        assert_eq!(h0.index, 0);
        assert_eq!(h0.total(), 2);
        assert_eq!(h1.index, 1);
        assert_eq!(h1.total(), 2);

        let mut r = Reassembler::new(2, 0).unwrap();
        assert!(r.add(1, &frags[1][3..]));
        assert!(!r.complete());
        assert!(r.add(0, &frags[0][3..]));
        assert!(r.complete());
        let mut out = Vec::<u8, MAX_FRAGMENT_PAYLOAD>::new();
        assert!(r.assemble(&mut out));
        assert_eq!(&out, &payload);
    }
}

/// True when `a` is strictly ahead of `b` in the circular u16 sequence space
/// (within the half-range). The sliding-window comparison primitive: `a` is
/// "newer" than `b` without ambiguity as long as the two never drift by more
/// than 32768 apart.
pub(crate) fn nack_set(mask: &mut [u8; NACK_BYTES], idx: usize) {
    if idx < NACK_BYTES * 8 {
        mask[idx / 8] |= 1 << (idx % 8);
    }
}

pub(crate) fn nack_from_mask(run_len: usize, mask: &[u8; NACK_BYTES]) -> [u8; NACK_BYTES] {
    let mut nack = [0u8; NACK_BYTES];
    let bytes = (run_len + 7) / 8;
    for i in 0..bytes.min(NACK_BYTES) {
        nack[i] = !mask[i];
    }
    let rem = run_len % 8;
    if rem != 0 && bytes <= NACK_BYTES {
        nack[bytes - 1] &= (1u8 << rem) - 1;
    }
    nack
}

pub(crate) fn nack_nonzero(nack: &[u8]) -> bool {
    nack.iter().any(|&b| b != 0)
}

pub(crate) fn nack_vec(run_len: usize, nack: &[u8; NACK_BYTES]) -> Vec<u8, NACK_BYTES> {
    let mut v = Vec::new();
    let bytes = ((run_len + 7) / 8).min(NACK_BYTES);
    for &b in &nack[..bytes] {
        v.push(b).ok();
    }
    v
}

pub(crate) fn seq_gt(a: u16, b: u16) -> bool {
    let d = a.wrapping_sub(b);
    d != 0 && d < 0x8000
}

/// One slot-position -> seq mapping for the sender's last TX run. Used by
/// the slot-position NACK path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TxRunSlot {
    /// Slot index within the run (0..run_len).
    pub(crate) slot: u8,
    pub(crate) seq: u16,
}

/// One in-flight TX entry: the ciphertext of an encrypted `Data` payload,
/// kept so a lost packet can be retransmitted without re-serializing the app
/// value (the nonce binds only to `seq`, so the ciphertext is stable).
#[derive(Clone, Copy)]
pub(crate) struct TxEntry {
    pub(crate) seq: u16,
    pub(crate) len: u8,
    pub(crate) payload: [u8; MAX_PAYLOAD],
    /// Transmitted at least once.
    pub(crate) sent: bool,
    /// Flagged for retransmit (NACK hit or timeout).
    pub(crate) retransmit: bool,
    /// Retransmissions so far (after the first send).
    pub(crate) retries: u8,
    /// Slots since the last send (the timeout counter).
    pub(crate) idle: u16,
}

/// One buffered RX entry: a decrypted, out-of-order `Data` payload held
/// until the gap before it is filled (selective repeat's reorder buffer).
#[derive(Clone, Copy)]
pub(crate) struct RxEntry {
    pub(crate) len: u8,
    pub(crate) payload: [u8; MAX_PAYLOAD],
}

/// The sender's sliding window: the in-flight, unacknowledged ciphertexts.
///
/// In-flight entries always live in the sequence range
/// `(tx_acked, tx_next)`. A cumulative ACK frees a contiguous prefix; a NACK
/// flags the named holes for retransmit; the idle timeout is the safety net
/// for lost ACKs. A delivery failure can remove a hole from that range, so
/// the window-full test is sequence-space based, not just an `inflight`
/// count.
pub(crate) struct TxWindow {
    pub(crate) slots: [Option<TxEntry>; WINDOW_SIZE],
    pub(crate) tx_next: u16,
    pub(crate) tx_acked: u16,
    pub(crate) inflight: u8,
}

impl TxWindow {
    pub(crate) fn new() -> Self {
        Self {
            slots: [None; WINDOW_SIZE],
            tx_next: 0,
            tx_acked: u16::MAX,
            inflight: 0,
        }
    }

    /// The window is full when every slot is occupied, or when the next
    /// sequence number has advanced a full window past the cumulative ACK.
    /// The second condition is what keeps `enqueue` from wrapping around
    /// onto a slot still held by an older in-flight entry after a delivery
    /// failure has punched a hole in the window (the `inflight` count is
    /// then less than [`WINDOW_SIZE`], but the next seq would still
    /// collide with the oldest un-acked entry).
    pub(crate) fn is_full(&self) -> bool {
        self.inflight as usize >= WINDOW_SIZE
            || self.tx_next.wrapping_sub(self.tx_acked) > WINDOW_SIZE as u16
    }

    /// Store a ciphertext payload under the next seq.
    ///
    /// The caller must check [`Self::is_full`] first; `enqueue` always
    /// writes at `tx_next % WINDOW_SIZE` and the sequence-space full check
    /// guarantees that slot is free.
    pub(crate) fn enqueue(&mut self, ciphertext: &[u8]) -> u16 {
        let seq = self.tx_next;
        let mut payload = [0u8; MAX_PAYLOAD];
        let len = ciphertext.len().min(MAX_PAYLOAD);
        payload[..len].copy_from_slice(&ciphertext[..len]);
        let idx = (seq % WINDOW_SIZE as u16) as usize;
        debug_assert!(self.slots[idx].is_none());
        self.slots[idx] = Some(TxEntry {
            seq,
            len: len as u8,
            payload,
            sent: false,
            retransmit: false,
            retries: 0,
            idle: 0,
        });
        self.tx_next = self.tx_next.wrapping_add(1);
        self.inflight += 1;
        seq
    }

    /// Advance the slot cumulative ACK, freeing every in-flight slot it
    /// covers (every seq <= ack is a Data slot the receiver has delivered
    /// in order).
    ///
    /// A slot cumulative ACK can only cover packets this node has actually
    /// enqueued (`seq < tx_next`). A peer restart or re-baseline can send
    /// an ACK far ahead of `tx_next`; accepting it would free in-flight
    /// entries the peer never received, so it is ignored.
    pub(crate) fn on_ack(&mut self, ack: u16) {
        let max_ack = self.tx_next.wrapping_sub(1);
        if seq_gt(ack, max_ack) {
            return;
        }
        if !seq_gt(ack, self.tx_acked) {
            return;
        }
        let mut s = self.tx_acked.wrapping_add(1);
        loop {
            let idx = (s % WINDOW_SIZE as u16) as usize;
            let slot = self.slots[idx];
            if matches!(slot, Some(e) if e.seq == s) {
                self.slots[idx] = None;
                self.inflight -= 1;
            }
            if s == ack {
                break;
            }
            s = s.wrapping_add(1);
        }
        self.tx_acked = ack;
    }

    /// Flag the holes named by the peer's NACK bitmap for retransmit.
    pub(crate) fn on_nack(&mut self, ack: u16, nack: u16) {
        for i in 0..WINDOW_SIZE {
            if nack & (1 << i) != 0 {
                let s = ack.wrapping_add(1 + i as u16);
                let idx = (s % WINDOW_SIZE as u16) as usize;
                if let Some(e) = self.slots[idx].as_mut() {
                    if e.seq == s {
                        e.retransmit = true;
                    }
                }
            }
        }
    }

    /// Flag the slots named by the peer's slot-position NACK bitmap for
    /// retransmit. `run_slots` is the list of (slot, seq) pairs this node
    /// sent in its last TX run.
    pub(crate) fn on_nack_slots(
        &mut self,
        nack: &[u8],
        run_slots: &[Option<TxRunSlot>; WINDOW_SIZE],
    ) {
        for slot_entry in run_slots.iter().flatten() {
            let byte = slot_entry.slot as usize / 8;
            let bit = slot_entry.slot as usize % 8;
            if byte < nack.len() && nack[byte] & (1 << bit) != 0 {
                let seq = slot_entry.seq;
                let idx = (seq % WINDOW_SIZE as u16) as usize;
                if let Some(e) = self.slots[idx].as_mut() {
                    if e.seq == seq {
                        e.retransmit = true;
                    }
                }
            }
        }
    }

    /// Pick the next entry to send: flagged retransmits first (lowest seq),
    /// then never-sent new data (lowest seq).
    pub(crate) fn pick(&self) -> Option<u16> {
        for pass in 0..2 {
            let mut s = self.tx_acked.wrapping_add(1);
            while s != self.tx_next {
                let idx = (s % WINDOW_SIZE as u16) as usize;
                if let Some(e) = &self.slots[idx] {
                    if e.seq == s {
                        let want = if pass == 0 { e.retransmit } else { !e.sent };
                        if want {
                            return Some(s);
                        }
                    }
                }
                s = s.wrapping_add(1);
            }
        }
        None
    }

    /// Pick the lowest sent-but-unacked entry, for the full-window fallback.
    ///
    /// When the window is full and there is neither a NACK-flagged nor a
    /// never-sent packet, the sender used to fall back to Beacon/Ack-only
    /// slots. If the receiver had not caught any Data yet, it then had
    /// nothing to ACK and the link could sit dead until the idle timeout.
    /// Re-sending the oldest in-flight Data keeps a packet in the air for
    /// the peer to catch and ACK, without touching the retry budget.
    pub(crate) fn pick_sent_for_blocked(&self) -> Option<u16> {
        let mut s = self.tx_acked.wrapping_add(1);
        while s != self.tx_next {
            let idx = (s % WINDOW_SIZE as u16) as usize;
            if let Some(e) = &self.slots[idx] {
                if e.seq == s && e.sent {
                    return Some(s);
                }
            }
            s = s.wrapping_add(1);
        }
        None
    }

    /// The entry for `seq` (caller guarantees it exists and is in-window).
    pub(crate) fn entry(&self, seq: u16) -> &TxEntry {
        let idx = (seq % WINDOW_SIZE as u16) as usize;
        self.slots[idx].as_ref().unwrap()
    }

    /// Mark an entry transmitted; bump its retry count on a retransmit.
    /// Returns `true` when the retransmission budget [`MAX_RETRIES`] has
    /// been used up (the caller drops the entry after the transmission
    /// that returns `true`).
    pub(crate) fn mark_sent(&mut self, seq: u16) -> bool {
        let idx = (seq % WINDOW_SIZE as u16) as usize;
        let Some(e) = self.slots[idx].as_mut() else {
            return false;
        };
        if e.seq != seq {
            return false;
        }
        let was_retransmit = e.sent;
        e.sent = true;
        e.retransmit = false;
        e.idle = 0;
        if was_retransmit {
            e.retries = e.retries.saturating_add(1);
            // Drop after the packet has been retransmitted MAX_RETRIES
            // times (the caller drops after the transmission that returns
            // `true`). The first send is never a retransmit.
            return e.retries >= MAX_RETRIES;
        }
        false
    }

    /// Drop an entry after its retries were exhausted (delivery failure).
    pub(crate) fn drop(&mut self, seq: u16) {
        let idx = (seq % WINDOW_SIZE as u16) as usize;
        let slot = self.slots[idx];
        if matches!(slot, Some(e) if e.seq == seq) {
            self.slots[idx] = None;
            self.inflight -= 1;
        }
    }

    /// Advance the per-entry idle counters once per slot; flag the timeout.
    pub(crate) fn tick(&mut self) {
        let mut s = self.tx_acked.wrapping_add(1);
        while s != self.tx_next {
            let idx = (s % WINDOW_SIZE as u16) as usize;
            if let Some(e) = self.slots[idx].as_mut() {
                if e.seq == s && e.sent && !e.retransmit {
                    e.idle = e.idle.saturating_add(1);
                    if e.idle >= RETRY_TIMEOUT_SLOTS {
                        e.retransmit = true;
                        e.idle = 0;
                    }
                }
            }
            s = s.wrapping_add(1);
        }
    }
}

/// The receiver's reorder window: buffered out-of-order payloads plus the
/// ACK/NACK state derived from them.
pub(crate) struct RxWindow {
    pub(crate) buf: [Option<RxEntry>; WINDOW_SIZE],
    /// Next seq to deliver in order.
    pub(crate) next_expected: u16,
    /// Highest seq received so far (the hole-detection horizon).
    pub(crate) highest_seen: u16,
    /// True once any packet has been received.
    pub(crate) have: bool,
}

impl RxWindow {
    pub(crate) fn new() -> Self {
        Self {
            buf: [None; WINDOW_SIZE],
            next_expected: 0,
            highest_seen: 0,
            have: false,
        }
    }

    pub(crate) fn reset(&mut self) {
        *self = Self::new();
    }

    /// True when `seq` sits inside the current delivery window.
    pub(crate) fn in_window(&self, seq: u16) -> bool {
        seq.wrapping_sub(self.next_expected) < WINDOW_SIZE as u16
    }

    /// Cumulative ACK: everything below `next_expected` has been delivered
    /// or explicitly skipped after the sender dropped it.
    pub(crate) fn ack(&self) -> u16 {
        self.next_expected.wrapping_sub(1)
    }

    /// NACK bitmap: bit `i` = seq `next_expected + i` is a detected hole
    /// (a later seq has been seen, so this one is known-missing).
    pub(crate) fn nack(&self) -> u16 {
        if !self.have {
            return 0;
        }
        let mut bits = 0u16;
        for i in 0..WINDOW_SIZE {
            let s = self.next_expected.wrapping_add(i as u16);
            if seq_gt(self.highest_seen, s) {
                let idx = (s % WINDOW_SIZE as u16) as usize;
                if self.buf[idx].is_none() {
                    bits |= 1 << i;
                }
            }
        }
        bits
    }

    /// Re-baseline to `seq` after a peer restart (used while Disconnected).
    pub(crate) fn resync(&mut self, seq: u16) {
        self.reset();
        self.next_expected = seq;
        self.highest_seen = seq;
        self.have = true;
    }

    /// Advance the delivery baseline to just past `seq` when a sender has
    /// dropped that Data packet after its retry budget.
    ///
    /// This is the receiver half of the delivery-failure resync: the sender
    /// stops retransmitting `seq`, so waiting for it would stall every later
    /// in-order packet. The baseline moves to `seq + 1` (or stays where it
    /// is when `seq` is already behind it). The sender keeps re-sending the
    /// drop notification until our cumulative ACK confirms the skip.
    pub(crate) fn skip_to(&mut self, seq: u16) {
        if seq == self.next_expected || seq_gt(seq, self.next_expected) {
            self.next_expected = seq.wrapping_add(1);
            // The skipped slots are dead; leave them empty so the next
            // in-order head (if any) is delivered on the next call.
        }
        if !self.have {
            // A drop notification is still a valid packet from the sender:
            // mark the receiver as having heard it, so the next outbound
            // slot sends the ACK that clears the sender's pending drop.
            self.have = true;
            self.highest_seen = self.highest_seen.max(seq);
        }
    }

    /// Buffer an in-window payload. Returns `false` when out of window.
    pub(crate) fn receive(&mut self, seq: u16, payload: &[u8]) -> bool {
        if !self.in_window(seq) {
            return false;
        }
        let idx = (seq % WINDOW_SIZE as u16) as usize;
        if self.buf[idx].is_none() {
            let mut p = [0u8; MAX_PAYLOAD];
            let len = payload.len().min(MAX_PAYLOAD);
            p[..len].copy_from_slice(&payload[..len]);
            self.buf[idx] = Some(RxEntry {
                len: len as u8,
                payload: p,
            });
        }
        if !self.have || seq_gt(seq, self.highest_seen) {
            self.highest_seen = seq;
            self.have = true;
        }
        true
    }

    /// The length of the in-order head, if ready.
    pub(crate) fn peek_len(&self) -> Option<usize> {
        let idx = (self.next_expected % WINDOW_SIZE as u16) as usize;
        self.buf[idx].as_ref().map(|e| e.len as usize)
    }

    /// Pop and return the in-order head (advancing `next_expected`).
    pub(crate) fn pop_head(&mut self) -> Option<RxEntry> {
        let idx = (self.next_expected % WINDOW_SIZE as u16) as usize;
        let entry = self.buf[idx].take()?;
        self.next_expected = self.next_expected.wrapping_add(1);
        Some(entry)
    }
}
