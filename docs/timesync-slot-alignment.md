# Time Sync and Slot Alignment

How two boards with free-running clocks and possibly different compile-time
slot defaults end up on a shared grid — and stay there.

## The slot chain (the clock)

The MPSL backend runs the radio inside a **self-chaining timeslot chain**:

- The app requests one EARLIEST slot at boot. Every slot's START callback
  chains the next NORMAL request before returning.
- A request is `(distance_us, length_us)`:
  - **distance** = start-to-start time — the slot *cadence*. This is the
    sync knob.
  - **length** = the granted interval — the *work budget*. The radio op
    must fully fit inside it.
- The chain never stops: a missed op just idles its slot.

Rule of the gap: **distance − length ≥ 150 us**. MPSL needs the inter-slot
margin for its own scheduling; tighter gaps trip its internal assert
(`MultiprotocolServiceLayer: 106:179`) and kill the chain.

The const generics are `MpslRadioPhy<SLOT_US, RX_POLL>`: `SLOT_US` is this
board's minimum cadence (used during negotiation), `RX_POLL` is the poll
iteration cap. The stable grant length is `cadence - 150` (350 us at the
500 us fallback cadence).

## The in-slot timeline

```
|slot start
|  ~10 us  radio config (the registers, every slot — MPSL owns the radio between slots)
|  ~40 us  ramp (MODECNF0.RU = Fast; 129 us with the legacy ramp)
|  work    TX: the packet on air   /   RX: the listen window
|  ~40 us  tail margin (the disable + the MPSL hand-back)
```

- Airtime is mode-dependent (`RadioMode::airtime_us(len)`): at 2 Mbit it is
  `28 + 4*(len+3)` us (preamble+address 28 us; 4 us/byte for the length
  byte, the payload, and the 2 CRC bytes), at 1 Mbit `48 + 8*(len+3)` us.
  A 28-byte packet = **152 us** on the air at 2 Mbit and **296 us** at
  1 Mbit.
- The RX poll is capped two ways: the iteration count (`RX_POLL`) **and** the
  time (`length − 100 us`, checked every 32 iterations). Iteration cost is
  chip-dependent (~0.28 us @64 MHz, ~0.12 us @128 MHz), so only the time cap
  is portable. **The poll must end inside the grant**: an overrun keeps
  `mpsl_low_priority_process()` from returning, starves the executor (the
  "frozen app" symptom), and trips the MPSL assert.
- Echo fit: a same-slot echo lands in the peer's RX window only if
  `window ≥ airtime + 2*jitter` (~192 us for 28-byte frames at 2 Mbit).
  A 250 us slot (a ~160 us window) cannot close a 28-byte reverse link at
  2 Mbit; 300 can. 1 Mbit frames need roughly twice the airtime, which is
  why the bare `radio-1m` build raises its slot period to 600 us.

## The completion protocol

Every START and every completed slot are observable by the app:

- At START the callback atomically publishes the boundary: `slot_count += 1`,
  stores `slot_start_done = done_count`, and signals `start_signal`.
- After the op (including an idle op) the callback increments `done_count`
  and signals `done_signal`.

`transmit`/`receive` then synchronize to the slot that actually executes
their op:

1. Publish the op (`op_kind` + the TX frame / the RX buffer pointer).
2. Wait on `start_signal` until `slot_count` advances. This is the slot that
   will execute the published op; a late app call can no longer mistake the
   previous slot's completion for its own.
3. Read `slot_start_done` and wait on `done_signal` until `done_count`
   advances past it — exactly one slot completion.

All waits are bounded by 10_000 wake-ups as a dead-man if the chain dies.

## The follower PLL (staying locked)

The central is the master; the peripheral phase-locks to it:

- **Observable**: `addr_poll_us` — the ADDRESS event measured with DWT from
  this node's own RXEN. The target is calibrated from locked catches and
  clamped to 50..180 us.
- **Correction**: `err = addr_poll − target`;
  `distance = clamp(nominal + err/4, nominal ± 20 us)`, then the next
  request re-bases to nominal. Any address anchor (not only a CRC-ok frame)
  is used for the correction.
- **Sweep**: after 8 misses the follower chains `nominal + 2 us`. The
  central never sweeps — it is the stable timing reference. This avoids the
  both-sides-sweep deadlock where two 502 us chains freeze their relative
  phase. A catch exits the follower sweep, including the old 502 us fixed
  point.
- **Echo placement**: the follower delays TXEN by a measurement-derived
  formula: it uses the peer's advertised TXEN offset + TX ramp, RXEN offset +
  RX ramp, and its own measured RXEN offset, TX setup and TX ramp. The only
  fixed constants are the address-anchor delay (28 us) and the tail margin.

## The runtime alignment (the align mechanism)

Heterogeneous boards must align **over the air**, not by matching builds.

**Advertise** — `Packet::Beacon` carries the timing measurements:

| field | meaning |
|---|---|
| `channel_index` | the hop authority (the peripheral's scheduler syncs to it) |
| `flags` | the sender's **measured RX listen window**, in 16 us units (0 = unknown) |
| `slot_us` | the sender's slot cadence in us (0 = unknown) |
| `slot_phase` | the sender's next-slot phase (the peripheral derives `slot_offset`) |
| `rx_en_offset`, `rx_ramp` | the sender's RXEN offset and RXEN→READY ramp, in us |
| `tx_en_offset`, `tx_ramp` | the sender's TXEN offset and TXEN→READY ramp, in us |

The central sends a beacon whenever there is no Data/ACK to send; during
cadence negotiation every TX slot is beacon-only, and `epoch % 64 == 0`
forces another beacon on the TX slot that carries that epoch. A connecting
peripheral therefore hears one promptly.

The window is measured, not configured: every no-catch RX poll ran to its
bound, so its duration (minus the 40 us ramp) *is* the window.

**Adopt** — the peripheral, on decoding a beacon:

- `scheduler.sync(channel_index)` — the channel.
- `align_slot_period(slot_us)` — `slot_nominal = slot_distance = slot_us`:
  the coarse align, instantly; the PLL fine-locks from there. `cadence_ok`
  is set once `slot_us >= min_slot_period_us()`.
- `set_peer_rx_window(flags * 16)` — the central's window, for the echo
  placement.
- `set_peer_rx_en_offset` / `set_peer_tx_en_offset` /
  `set_peer_rx_ramp` / `set_peer_tx_ramp` — the central's measured radio
  timings, for the same echo placement.
- For the hardware slot cadence (MPSL), store
  `slot_offset = beacon_phase − slot_count (mod period)` so the next slot
  mirrors the central's advertised phase; the bare path instead re-seeds its
  software `slot_step` from `slot_phase`.

**Align the reverse path** — the follower delays TXEN inside its own TX slot
so the echo on-air interval lands in the centre of the peer's measured RX
window. The delay is computed from the beacon's advertised RX/TX offsets and
ramps plus the follower's own RXEN offset, address stamp, TX setup and TX
ramp; the fixed constants are only the address-anchor delay (28 us) and the
named tail margin. The same placement is used for the acquisition
`SlotRequest`.

**Stable slot length** — both TX and RX slots use

```
slot_len = nominal − 150
```

so the full 350 us grant (at 500 us cadence) is available for Data RX and
for the follower's echo TX delay. Short catches do not shrink the slot.

## Connection formation (the disconnected profile)

While no link exists, acquisition is deterministic:

- **No hopping** — the link-layer gate pins the scheduler to
  `initial_channel` until the `CONNECT_STREAK_THRESHOLD` (8) form-up streak
  is reached.
- **Stable central** — the central holds `slot_nominal`; it never sweeps.
  Only the peripheral sweeps its grid (+2 us/slot after misses) and its
  SlotRequest TX delay.
- **Peripheral acquisition duty cycle** — until the peripheral has received
  Data, it listens on even phases and sends `SlotRequest` on odd phases.
  This guarantees one SlotRequest lands in the central's two-slot RX window
  every period regardless of the initial phase guess, while still listening
  to half of the central's TX run for beacons/Data. After the first Data
  catch it switches to the exact mirrored ratio.
- **Wide budget** — `slot_len` is always `nominal − 150`; acquisition only
  changes `distance` (the sweep), never shrinks the RX budget.

## Where it lives

| piece | file |
|---|---|
| the beacon fill (central) / parse + adopt (peripheral) | `thunders/src/link.rs` |
| the align trait surface (`slot_period_us`, `min_slot_period_us`, `fallback_slot_period_us`, `align_slot_period`, `rx_window_us`, `set_peer_rx_window`, `set_peer_*_offset/ramp`, `slot_count`) | `thunders/src/phy.rs` (default no-op/`0`), `MpslRadioPhy` impl in `thunders-phy-nrf/src/mpsl/mod.rs` |
| the chain, the follower PLL/sweep, the echo TX delay | `thunders-phy-nrf/src/mpsl/callback.rs` |
| the time-capped poll, the window measurement, the radio quiesce (`disable_wait`) | `thunders-phy-nrf/src/mpsl/radio.rs` |
| the completion protocol, the named PLL/RSSI snapshots | `thunders-phy-nrf/src/mpsl/mod.rs` |
| the hop gate (`LinkStatus`) | `thunders/src/link.rs` (`on_miss` / `on_rx`) |

## Rules that fell out of hardware (do not relearn the hard way)

- Never print (defmt) from the timeslot callback — it runs in MPSL timer-IRQ
  context and defmt is not reentrant (stream corruption → HardFault). Keep
  counters; read them from the app context.
- Quiesce the radio before the slot ends (`disable_wait`): an unfinished
  disable leaks a live receiver into the next slot — its trailing END/CRC
  land in that slot's poll setup (instant exits, torn catches).
- A received frame is valid only if `len > 0 && len + 1 <= rx_cap` (the phy
  frame is `[len | payload]` and `receive` shifts left by one).
- The MPSL gap is ≥ 150 us, not 100.
- The state's backing storage must be genuinely `'static` (a stack local
  borrowed into the phy is UB that only *sometimes* crashes).
