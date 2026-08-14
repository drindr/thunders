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

The const generics (`MpslRadioPhy<SLOT_US, SLOT_LEN_US, RX_POLL>`) are only
the **initial** values — the runtime alignment below overrides them. The
verified per-chip defaults: nRF5340/nRF52840 = `500/400`, nRF54LM20 =
`300/250`.

## The in-slot timeline

```
|slot start
|  ~10 us  radio config (the registers, every slot — MPSL owns the radio between slots)
|  ~40 us  ramp (MODECNF0.RU = Fast; 129 us with the legacy ramp)
|  work    TX: the packet on air   /   RX: the listen window
|  ~40 us  tail margin (the disable + the MPSL hand-back)
```

- Airtime at 2 Mbps: `28 + 4*(len+3)` us (preamble+address 28 us; 4 us/byte
  for the length byte, the payload, the 2 CRC bytes). A 28-byte packet =
  **152 us** on the air.
- The RX poll is capped two ways: the iteration count (`RX_POLL`) **and** the
  time (`length − 100 us`, checked every 32 iterations). Iteration cost is
  chip-dependent (~0.28 us @64 MHz, ~0.12 us @128 MHz), so only the time cap
  is portable. **The poll must end inside the grant**: an overrun keeps
  `mpsl_low_priority_process()` from returning, starves the executor (the
  "frozen app" symptom), and trips the MPSL assert.
- Echo fit: a same-slot echo lands in the peer's RX window only if
  `window ≥ airtime + 2*jitter` (~192 us for 28-byte frames). A 250 us slot
  (a ~160 us window) cannot close a 28-byte reverse link; 300 can.

## The completion protocol

`transmit`/`receive` publish the op into the shared state and await its
completion:

1. Snapshot `done_count` **before** publishing `op_kind`.
2. Publish (`op_kind` + the TX frame / the RX buffer pointer).
3. Wait on `done_signal` until `done_count` changes (bounded by 10_000
   wake-ups as a dead-man if the chain dies).

The callback increments `done_count` (an AtomicU32, not a flag) after the op
runs and stores the signal. Snapshot-before-publish + a monotonic counter is
what makes a completion that happened *before* the wait started unmissable —
the old clear-after-publish flag raced the callback and erased completions.

## The follower PLL (staying locked)

The central is the master; the peripheral phase-locks to it:

- **Observable**: `rx_catch_iter` — the poll iteration at which the END event
  fired, i.e. *where in our window* the central's packet landed.
- **Correction**: `err = catch_iter − target`;
  `distance = clamp(nominal + err * 21/1000, nominal ± 20 us)`.
  A small proportional gain with a clamp: it tracks drift, it cannot
  teleport.
- **Sweep**: after 8 consecutive misses (`PLL_SWEEP_MISSES`) the follower is
  not locked at all; it runs `distance = nominal + 2 us` so the two grids
  slide past each other until catches resume.

## The runtime alignment (the align mechanism)

Heterogeneous boards must align **over the air**, not by matching builds.

**Advertise** — the beacon carries three things (`Packet::Beacon`):

| field | meaning |
|---|---|
| `channel_index` | the hop authority (the peripheral's scheduler syncs to it) |
| `flags` | the sender's **measured RX listen window**, in 16 us units (0 = unknown) |
| `slot_us` | the sender's slot cadence in us (0 = unknown) |

The central sends a beacon whenever the payload queue is empty **and** every
64th TX slot under load, so a connecting peripheral hears one promptly.

The window is measured, not configured: every no-catch RX poll ran to its
bound, so its duration (minus the 40 us ramp) *is* the window.

**Adopt** — the peripheral, on decoding a beacon:

- `scheduler.sync(channel_index)` — the channel.
- `align_slot_period(slot_us)` — `slot_nominal = slot_distance = slot_us`:
  the coarse align, instantly; the PLL fine-locks from there.
- `set_peer_rx_window(flags * 16)` — the central's window, for the echo
  below.

**Align with the poor one** — the reverse path. The follower's echo goes out
at its next slot start (+~40 us ramp); the central's listen window sits
`[50, 50 + W_c]` us into *its* RX slot. Rather than delaying the TX, the
follower moves its whole grid: it retargets the PLL catch point to

```
c* = A − (W_peer − A) / 2        (us, post-ramp; A = the frame's airtime)
```

so that the next slot's echo lands centered in the peer's (possibly narrower)
window. Clamped to the middle half of the poll. Without a peer window, the
target is plain mid-window.

**Self-size** — on each catch, the follower sets

```
slot_len = min(airtime + 140, nominal − 150)
```

so the length converges to what the actual packets need (plus overhead and
jitter), capped by the MPSL gap — whichever side is poorer dictates.

## Connection formation (the disconnected profile)

While no link exists, both sides make acquisition easy:

- **No hopping** — the link-layer gate pins the scheduler to
  `initial_channel` until the first successful RX (see
  `connection-state-machine.md`). A miss before connection is not
  interference; it is two unsynchronized grids. The central holding still is
  what lets the follower's sweep find it.
- **Wide budget** — while `rx_misses ≥ PLL_SWEEP_MISSES` (not catching =
  disconnected), the phy widens `slot_len` to `nominal − 150` — the most
  window the cadence allows. The first catch re-sizes it (above).

## Where it lives

| piece | file |
|---|---|
| the beacon fill (central) / parse + adopt (peripheral) | `thunders/src/link.rs` |
| the align trait surface (`slot_period_us`, `align_slot_period`, `rx_window_us`, `set_peer_rx_window`) | `thunders/src/phy.rs` (default no-op/`0`), `MpslRadioPhy` impl in `thunders-phy-nrf/src/mpsl/mod.rs` |
| the chain, the PLL, the sweep, the widen, the self-size | `thunders-phy-nrf/src/mpsl/callback.rs` |
| the time-capped poll, the window measurement, the radio quiesce (`disable_wait`) | `thunders-phy-nrf/src/mpsl/radio.rs` |
| the completion protocol, `mpsl_stats` / `mpsl_pll` (the diagnostics) | `thunders-phy-nrf/src/mpsl/mod.rs` |
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
