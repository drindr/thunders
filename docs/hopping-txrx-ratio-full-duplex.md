# Hopping + TX/RX Ratio + Protocol-Level Full-Duplex

The thunders link runs the same slot protocol on **both** the bare RADIO and
the MPSL timeslot backends. The slot is the phy's atomic op — **400 µs on the
bare path** (the software-paced slot grid; see the bare scheduler note in
section 6), **500 µs on the MPSL** (its grant floor, ~375-500 µs). The
ratio, the hopping and the full-duplex are identical at each backend's slot
rate.

## 1. The Slot Model

The slot is the atomic unit. Every slot is **either TX or RX** — never both
(the TX+RX-in-one-slot does not fit the radio's turnaround). The ratio
decides the mix.

```
slot: | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | ...
type: | T | T | T | T | T | T | T | T | R | ...   (an 8:1 ratio)
```

`Central::frame` and `Peripheral::frame` run one slot each: the
`Config::tx_rx_ratio` and a `slot_step` counter decide which type this slot is
(`(slot_step % (tx + rx)) < tx` is a TX slot).

## 2. The TX/RX Ratio

```rust
// thunders/src/config.rs
pub struct Config {
    ...
    pub tx_rx_ratio: (u8, u8),   // (8, 1) by default
}
// Config::with_tx_rx_ratio(tx, rx) sets it.
```

- **Central**: `tx` TX slots (the PING/Data) then `rx` RX slots (the reverse
  listen). `(8, 1)` → 8 kHz TX / 1 kHz RX on the bare.
- **Peripheral**: mirrors — RX on the central's TX slots, TX on the central's
  RX slots (the reverse Data).
- The ratio is symmetric by construction (the same config on both sides, the
  roles mirror), so there is no negotiation — the two sides agree via the
  shared schedule.

## 3. The Hopping (beacon-driven)

The `Scheduler` holds the 25-channel sequence (a network-seeded LCG
permutation of `DEFAULT_HOP_SEQUENCE`, the 2.4 GHz channels at 3 MHz spacing).
Hopping is beacon-driven — no free-running local hop, so the two nodes cannot
drift off the shared channel:

- **The central is the hop master**: it advances the index only after
  `HOP_MISS_THRESHOLD` (4) consecutive failed RX slots — the interference
  signal. A healthy slot resets the streak.
- **The peripheral follows**: it re-syncs its index from every received
  beacon's `channel_index` (the authority) and mirrors the same miss
  threshold for the local fallback.
- The `last_channel` set-once optimization means the phy re-tunes only when
  the channel actually changes.

The hop is **gated by the connection state machine**
(`docs/connection-state-machine.md`): before the pair has exchanged a packet,
the scheduler is pinned to the initial channel (no hop), so the two
free-running schedules align without ever landing on different channels. The
adaptive hop only starts once `Connected`.

## 4. The Full-Duplex (seq-based, no ACK)

- The central's TX slots carry the Data (the `seq`); the RX slots listen for
  the peripheral's reverse Data.
- The peripheral's TX slots carry its reverse Data; the RX slots take the
  central's Data.
- Reliability is the seq window (`accept_seq`), not acks — each direction has
  its own `tx_seq`/`rx_seq`.

The application-facing surface is the typed interface (`send<T>` / `recv<T>`),
which postcard-serializes the caller's own struct and hides the bytes, the
MIC, the CRC and the radio entirely.

## 5. The Phy Layer

- **TX**: the burst interface — `transmit_burst_begin` ramps the radio once,
  `transmit_burst_send` sends each subsequent packet (the on-air only,
  ~10-12 kHz one-way on the bare). The MPSL backend returns
  `Error::Unsupported` and the link falls back to the plain per-slot
  `transmit` — the same slot-aware frame, the slower path.
- **RX**: the DWT-capped listen window passed by the link layer
  (`CENTRAL_REPLY_TIMEOUT_US` / `PERIPHERAL_LISTEN_TIMEOUT_US`, 200 µs), and
  the RX window opens at a fixed 30 µs offset from the slot start.
- **Turnaround**: the TX↔RX switch pays one ramp per direction-switch
  (amortized across the burst run).
- **CRC**: the radio's hardware CRC-16 (the `crcstatus` gate) — no software
  CRC anywhere.

## 6. The Time Sync (the peripheral's phase-lock)

Two boards each run their own MPSL timeslot chain at 500 µs. The chains start
at arbitrary times, so the peripheral's RX slot is offset from the central's
TX slot by an arbitrary phase `δ`. A packet is caught only when the central's
on-air packet falls inside the peripheral's RX listen window, so a free-running
pair catches `window / period` of slots — and exactly 0 while the phase sits in
the blind spot.

The MPSL phy closes this loop in software (`thunders-phy-nrf/src/mpsl.rs`). The
knob is the chained timeslot distance `SLOT_DISTANCE` (nominal 500 µs); the
phase error is measured as `RX_CATCH_ITER`, the RX poll count at the moment the
frame's END event fires.

- **Measurement.** The RX poll loop counts iterations until the END event. On
  a valid (CRC-ok) catch it stores that count: a small count means the peer's
  TX landed *early* in our window (we are late — `δ` too positive); a large
  count means *late* (we are early — `δ` too negative).
- **Acquisition (the sweep).** After `PLL_SWEEP_MISSES` (8) consecutive misses
  the distance is set to `500 + PLL_SWEEP_US` (502 µs). A constant +2 µs per
  slot slides `δ` across the whole 500 µs period in ~125 ms, so the RX window
  sweeps the peer's TX until the first catch.
- **Tracking (proportional).** On each catch the phase error (the offset
  minus the window midpoint, in poll iterations) is multiplied by the gain
  (`PLL_GAIN_NUM/PLL_GAIN_DEN`, ~0.021 µs/iter) and applied directly to the
  chained distance: `SLOT_DISTANCE = 500 + gain × error`, clamped to
  480–520 µs. The gain keeps the loop gain < 1, so the catch converges to the
  window centre in a few slots without the integrator overshoot of a ±1 µs
  bang-bang (which climbed the distance and limit-cycled).
- **It lives in the phy** (`MpslRadioPhy::receive`), not the link layer: the
  phy self-aligns its own RX window, so `Central`/`Peripheral` stay
  backend-agnostic (`Phy::adjust_period` remains an unused hook).
- **The bare RADIO backend now has its own software twin** of this loop in
  `NrfRadioPhy` (`set_paced`): both roles pace their slot starts to the same
  400 µs grid, the follower sweeps for the central's phase and then
  phase-locks on the RX address anchor. The bare path needs it for a
  different reason than the MPSL: there is no timeslot chain, so the app
  loop's natural per-slot time differs per chip and per slot type; the fixed
  software grid is what makes the two free-running CPUs share one cadence.

Crystal drift (ppm) walks `δ` out of the deadband between catches and the
bang-bang pulls it back, so the loop re-centres continuously rather than
drifting apart.

## 7. Where it lives

| piece | file |
|---|---|
| the slot decision + the ratio | `thunders/src/link.rs` (`Central::frame`, `Peripheral::frame`) |
| the ratio config | `thunders/src/config.rs` (`tx_rx_ratio`, `with_tx_rx_ratio`) |
| the hop sequence + the permutation | `thunders/src/scheduler.rs` + `DEFAULT_HOP_SEQUENCE` |
| the burst + the CCM + the CRC | `thunders-phy-nrf/src/radio_phy.rs` |
| the MPSL slot (500/490) + the time sync | `thunders-phy-nrf/src/mpsl.rs` (`SLOT_DISTANCE`, `RX_CATCH_ITER`, the proportional PLL) |

## 8. The limitations

- **The MPSL runs the same protocol at ~2 kHz** (its 500 µs grant-floor slot)
  instead of the bare's 8 kHz — the burst and the hardware CCM are bare-only.
- **The sync is the sensitive piece**: if the peripheral's phase-lock slips,
  its RX windows miss the central's TX slots (the free-running drift).
- **The turnaround cost**: every TX→RX switch burns a ramp (~40-129 µs). The
  ratio decides how often — `(8, 1)` pays it once per 9 slots, `(1, 1)` every
  other slot.
