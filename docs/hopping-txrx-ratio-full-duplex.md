# Hopping + TX/RX Ratio + Protocol-Level Full-Duplex

The thunders link runs the same slot protocol on **both** the bare RADIO and
the MPSL timeslot backends. The slot is the phy's atomic op — **400 µs on the
bare path in the default 2 Mbit mode** (600 µs when the examples are built
with `radio-1m`), **500 µs on the MPSL fallback** (its common grant floor;
the negotiated cadence becomes `max(central_min, peripheral_min)`). The
ratio, the hopping and the full-duplex are identical at each backend's slot
rate; the radio mode only changes the on-air time and therefore the slot
budget.

## 1. The Slot Model

The slot is the atomic unit. Every slot is **either TX or RX** — never both
(the TX+RX-in-one-slot does not fit the radio's turnaround). The ratio
decides the mix.

```
slot: | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | ...
type: | T | T | T | T | T | T | T | T | R | R | ...   (an 8:2 ratio)
```

`Central::frame` and `Peripheral::frame` run one slot each. When the PHY has
its own slot cadence (`Phy::slot_count() != 0`, the MPSL timeslot chain) the
hardware slot count is the authority; otherwise the link's own `slot_step`
counter paces the bare radio. In both cases the phase is
`(slot_count % (tx + rx)) < tx` → a TX slot (or a mirrored listen slot on
the peripheral).

## 2. The TX/RX Ratio

```rust
// thunders/src/config.rs
pub struct Config {
    ...
    pub tx_rx_ratio: (u8, u8),   // (8, 2) by default
}
// Config::with_tx_rx_ratio(tx, rx) sets it.
```

- **Central**: local schedule = `tx TX, rx RX, idle IDLE`; `(8, 2, 0)` →
  eight TX slots followed by two RX slots.
- **Peripheral**: local schedule = `rx TX, tx RX, idle IDLE`; it runs the
  same `tx then rx then idle` data-plane rule in a shifted phase coordinate.
- Both roles execute one shared `local_phase < local_tx` check; role only
  supplies the local schedule. `Config::with_tx_rx_idle(tx, rx, idle)` lets
  the two directions carry different capacity while keeping one common
  period and complementary slot types. `Config::with_tx_rx_ratio(tx, rx)` clamps a zero
  `tx` or `rx` to 1 so the slot period can never be zero.
- The **cadence** is negotiated separately: the peripheral sends
  `Packet::SlotRequest { min_slot_us }`, the central adopts
  `max(own_min, peer_min)`, and beacons advertise the chosen `slot_us`.

## 3. The Hopping (beacon-driven)

The `Scheduler` holds the 25-channel sequence (a network-seeded LCG
permutation of `DEFAULT_HOP_SEQUENCE`, the 2.4 GHz channels at 3 MHz spacing).
Hopping is beacon-driven — no free-running local hop, so the two nodes cannot
drift off the shared channel:

- **The central is the hop master**: while `Connected` it advances the index
  only after `HOP_MISS_THRESHOLD` (16) consecutive failed RX slots — the
  interference signal. A healthy slot resets the streak.
- **The peripheral follows**: it re-syncs its index from every received
  beacon's `channel_index` (the authority). It does not advance the hop on
  its own miss streak; following the beacon is what keeps both sides on the
  same channel.
- The `last_channel` set-once optimization means the phy re-tunes only when
  the channel actually changes.

The hop is **gated by the connection state machine**
(`connection-state-machine.md`): until the pair has sustained
`CONNECT_STREAK_THRESHOLD` (8) successful RX slots, the scheduler is pinned
to the initial channel (no hop), so the two free-running schedules align
without ever landing on different channels. The adaptive hop only starts
once `Connected`.

## 4. The Full-Duplex (selective-repeat ARQ)

- The central's TX slots carry the Data (the `seq`); the RX slots listen for
  the peripheral's reverse Data.
- The peripheral's TX slots carry its reverse Data; the RX slots take the
  central's Data.
- Reliability is **selective repeat, slot-position NACK**: each `Data` piggybacks
  a cumulative ACK (`ack`, still sequence based for freeing the window) and a
  variable-length NACK bitmap (`Vec<u8, NACK_BYTES>`) that maps to the **slots
  of the last TX run**, not to seq numbers. For an 8:2 ratio the forward run
  has 8 slots and the reverse run has 2. The receiver records which slot
  positions of the run had a valid packet and NACKs the missing positions;
  the sender maps each NACK bit back to the seq it sent in that slot and flags
  exactly those entries for retransmit. A delivery failure after
  `MAX_RETRIES` is announced with `Packet::Drop { seq, ack, nack }`, which
  advances the receiver past the dropped seq while still draining the
  opposite direction. The sender holds a 16-packet in-flight window and a
  cumulative ACK drains it; the receiver buffers out-of-order payloads and
  delivers in order.

**Capacity requirement.** ARQ is not free — retransmissions consume channel
slots. A link offered at 100% of its slot rate cannot be 100% reliable under
loss: there is no spare slot to retransmit in. The bench loops therefore use
`tx_window_full()` and only offer a new PING/echo when the TX window has
space, so the offered rate adapts to the confirmed link capacity and
`window_full` stays 0. Raw `frame` callers that ignore backpressure get a
`window_full` count; `Central::delivery_failures()` /
`Peripheral::delivery_failures()` count packets dropped after
`MAX_RETRIES`.

The application-facing surface is the typed interface (`send<T>` / `recv<T>`),
which postcard-serializes the caller's own struct and hides the bytes, the
MIC, the CRC and the radio entirely. `send<T>` applies backpressure: it stores the pending payload and awaits a
completion signal until a future `frame` enqueues it, so it never silently
drops the value because the window is full and it does not spin. Raw `frame`
callers get the one-slot offer semantics instead: the offer is consumed only
when the ratio makes that call a TX slot (on an RX slot it is ignored), and
`window_full` counts TX-slot offers that were rejected for lack of space.

## 5. The Phy Layer

- **Radio mode**: the examples default to `RadioMode::Nrf2Mbit`; the
  `radio-1m` example feature selects `Nrf1Mbit` and also sets the bare slot
  period to 600 µs (`NrfRadioPhy::set_paced_period_us(600)`) so the longer
  1 Mbit on-air frame still fits.
- **TX**: the burst interface — `transmit_burst_begin` ramps the radio once,
  `transmit_burst_send` sends each subsequent packet (the on-air only,
  ~10-12 kHz one-way on the bare). The MPSL backend returns
  `Error::Unsupported` and the link falls back to the plain per-slot
  `transmit` — the same slot-aware frame, the slower path.
- **RX**: on the bare backend the RXEN is asserted at a fixed
  `BARE_RX_OFFSET_US` (30 µs) after slot start, and the link passes a 200 µs
  listen timeout (`CENTRAL_REPLY_TIMEOUT_US` / `PERIPHERAL_LISTEN_TIMEOUT_US`);
  an in-flight frame may run a little past that timeout. On MPSL the RX starts
  at the timeslot START and is time-capped against the grant (`slot_len −
  100 µs`, with a hard edge for an in-flight frame), not by the link timeout.
- **Turnaround**: the TX↔RX switch pays one ramp per direction-switch
  (amortized across the burst run).
- **CRC**: the radio's hardware CRC-16 (the `crcstatus` gate) — no software
  CRC anywhere.

## 6. The Time Sync (the follower phase-lock)

Two boards each run their own MPSL timeslot chain at 500 µs. The chains start
at arbitrary times, so the peripheral's RX slot is offset from the central's
TX slot by an arbitrary phase `δ`. The MPSL callback closes this loop in
software in `thunders-phy-nrf/src/mpsl/callback.rs`.

- **Measurement.** The RX poll records `addr_poll_us`: the ADDRESS event
  measured with DWT from this node's own RXEN. The follower calibrates an
  address target (50..180 µs) from locked catches.
- **Slot-boundary synchronization.** `transmit`/`receive` publish their op,
  wait for the next timeslot START, then wait exactly that slot's completion.
  The link phase and the radio op can therefore never silently drift apart
  when an app call is late.
- **Acquisition sweep.** After 8 misses the follower chains
  `nominal + 2 µs` per slot so the RX window slides across the peer's TX.
  The central never sweeps: it stays at nominal and is the stable reference.
  If both sides swept at +2 us/slot their equal periods would freeze the
  relative phase.
- **Acquisition duty cycle.** Until the peripheral has received Data, it
  listens on even phases and sends `SlotRequest` on odd phases. Any central
  RX window (two consecutive phases) therefore receives a SlotRequest every
  period, independent of the initial phase offset; half of the central's TX
  run is still listened to. The exact mirrored ratio starts after the first
  Data catch.
- **Tracking.** On any address anchor (not only CRC-ok frames) the follower
  applies a one-shot phase step:
  `distance = nominal + (addr_poll - target) / 4`, clamped to ±20 µs, then
  re-bases the next request to nominal. A catch exits the sweep.
- **Echo placement.** The follower delays its TXEN so its echo/SlotRequest
  lands mid-window at the peer. `Packet::Beacon` advertises the central's
  measured TXEN offset, TXEN→READY ramp, RXEN offset and RXEN→READY ramp;
  the follower combines those with its own measured RXEN offset, address
  stamp, TX setup and TX ramp. The fixed constants are the address-anchor
  delay (28 us) and a named tail margin.
- **Slot length.** The stable grant is `nominal - 150 µs` (350 µs at the
  500 µs cadence).
- **Bare twin.** `NrfRadioPhy::set_paced` implements the same idea on a
  software grid: 400 µs by default, or 600 µs after
  `set_paced_period_us(600)` when the example is built for 1 Mbit. The
  follower sweeps +2 µs/slot and phase-locks on its own `last_addr_slot_us`
  (own RXEN + address stamp).

## 7. Where it lives

| piece | file |
|---|---|
| the slot decision + the ratio | `thunders/src/link.rs` (`Central::frame`, `Peripheral::frame`) |
| the ratio config | `thunders/src/config.rs` (`tx_rx_ratio`, `with_tx_rx_ratio`) |
| the hop sequence + the permutation | `thunders/src/scheduler.rs` + `DEFAULT_HOP_SEQUENCE` |
| the burst + the CCM + the CRC | `thunders-phy-nrf/src/radio_phy.rs` |
| the MPSL slot chain + the follower PLL | `thunders-phy-nrf/src/mpsl/{callback,radio,state}.rs` |

## 8. The limitations

- **The MPSL runs the same protocol at ~2 kHz** (its 500 µs fallback slot).
  The bare runs at ~2.5 kHz in the default 2 Mbit mode (400 µs slot) and
  ~1.67 kHz with `radio-1m` (600 µs slot). The bare one-way *burst* path is
  faster (~10-12 kHz at 2 Mbit) because it skips the per-packet ramp, but
  that is a TX-only bench, not a full-duplex link. The burst and the
  hardware CCM are bare-only; CCM is unavailable on nRF54.
- **The sync is the sensitive piece**: if the peripheral's phase-lock slips,
  its RX windows miss the central's TX slots (the free-running drift).
- **The turnaround cost**: every TX→RX switch burns a ramp (~40-129 µs). The
  ratio decides how often — `(8, 2)` pays it once per 10 slots, `(1, 1)` every
  other slot.
