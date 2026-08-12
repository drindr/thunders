# thunders

A minimal reliable link layer for the Nordic 2.4 GHz RADIO, with two backends:
a raw RADIO driver and an MPSL timeslot backend (the Zephyr ESB-on-MPSL
pattern). Verified across two heterogeneous boards — an **nRF5340** (net core)
and an **nRF54LM20** — in either role, over either backend.

```
┌─────────────┐   PING (frame)   ┌─────────────┐
│ Central     │ ───────────────▶ │ Peripheral  │
│  (TX+echo)  │ ◀─────────────── │  (RX+echo)  │
└─────────────┘                  └─────────────┘
```

## Architecture

| Crate | Role |
|---|---|
| `thunders` | Protocol core: framing, CRC-16, sequence sync, `Phy` trait, `Central`/`Peripheral` link loops. Board-agnostic. |
| `thunders-phy-nrf` | nRF RADIO PHY. Two backends: **bare** (direct RADIO, interrupt-driven) and **mpsl** (radio inside MPSL timeslots — coexists with BLE). All register access goes through the `nrf-pac` accessors (SVD-derived, no hand-picked offsets). |
| `examples/nrf5340/{bare,mpsl}` | nRF5340 net-core firmware. One binary, either role. |
| `examples/nrf54lm20/{bare,mpsl}` | nRF54LM20 firmware. One binary, either role. |

The MPSL layer is the **stock official** [`alexmoon/nrf-sdc`](https://github.com/alexmoon/nrf-sdc)
as a git dependency — no vendored fork. The nRF54L-specific bits that upstream
doesn't provide (RRAM fetch-latency, NVIC enables) live in `thunders-phy-nrf`.

## Roles

Each example builds once, and the role is selected at build time:

```sh
cargo build --release                          # central (default)
cargo build --release --no-default-features \
  --features peripheral                        # peripheral
```

The central polls the link and PINGs; the peripheral RXes and echoes.

## Building & flashing

Prereqs: `probe-rs`, the `thumbv8m.main-none-eabi` (nRF5340) and
`thumbv8m.main-none-eabihf` (nRF54L) targets, and the patched embassy HAL at
`../../embassy-nrf54` (see the `[patch.crates-io]` in each example's Cargo.toml).

```sh
# nRF5340 (net core), DAPLink probe
probe-rs run --chip nRF5340_xxAA \
  --probe "0d28:0204-3:13040003001100e10465599500004fca0000000097969921" \
  --allow-erase-all target/thumbv8m.main-none-eabi/release/thunders-<name>

# nRF54LM20, J-Link probe
probe-rs run --chip nRF54LM20A --probe 1366:1069 --speed 100 \
  target/thumbv8m.main-none-eabihf/release/thunders-<name>
```

Run the peripheral first, then the central — the logs show the PINGs, the
`GOT reply` round-trips and the periodic `BENCH` summary.

## Benchmarks

30 s runs, 5340 central ↔ LM20 peripheral and the swapped pair, both backends.
PING payload is 8 bytes; bandwidth = received payload bytes/s.

| Pairing | Backend | Central poll | TX emitted | **Peer catches (data rate)** | Round-trip RX | Errors |
|---|---|---|---|---|---|---|
| 5340 central ↔ LM20 peripheral | **bare** | 542 Hz | 4.3 KB/s | 322 RXOK / 2 s = **1.3 KB/s** | 322 / 2 s = 1.3 KB/s | 0 |
| LM20 central ↔ 5340 peripheral | **bare** | 233 Hz | 1.9 KB/s | 329 RXOK / 2 s = **1.3 KB/s** | 320 / 2 s = 1.3 KB/s | 0 |
| 5340 central ↔ LM20 peripheral | **mpsl** | **500 Hz** | 4.0 KB/s | **~660 B/s steady** (phase-locked) | GOT=22 | 0 |
| LM20 central ↔ 5340 peripheral | **mpsl** | **500 Hz** | 4.0 KB/s | **~600 B/s steady** (phase-locked) | GOT=67 | 0 |

Two different ceilings are visible:

- **Poll rate** — the MPSL backend matches the raw radio (500 Hz vs 542 Hz
  central poll); the old 18× gap is gone. The central always frames RX+TX (2
  slots), the peripheral mostly RX (1 slot, ~830 Hz on the MPSL).
- **Data rate** — the *caught* bytes are lower than the emitted rate on both
  backends. The bare radio stays in RX between TXes (long poll windows),
  catching ~60% of the emitted PINGs. The MPSL's free-running timeslot chains
  drift against each other, so the peer's RX window only intermittently
  overlaps the central's TX. Two knobs already help: the RX listen window per
  slot was raised (`RX_POLL_BOUND` 1000→2000 on the 5340, 1000→3000 on the
  LM20 — the slot budget allows it, the 106:179 overrun assert still holds),
  which lifted the peer catch from ~100 B/s to **~800 B/s**. The remaining
  lever is phase sync, and it is now implemented: a **distance PLL** on the
  peripheral (see `thunders-phy-nrf/src/mpsl.rs`). The MPSL's absolute-time
  requests don't exist here (only EARLIEST + NORMAL), and re-anchoring via a
  fresh EARLIEST is blocked while the session is active (-NRF_EAGAIN: the
  chain never truly goes idle). So the peripheral instead measures its
  catch-to-catch interval (the peer's 1000 us period plus its own phase
  drift) and nudges the chained request's `distance_us` by ±1 us per catch —
  a bang-bang PLL that holds the phase inside the RX window indefinitely.
  The catch is now **steady ~600-660 B/s** instead of the bursty
  drift-dependent pattern. The central stays free-running (it is the master;
  re-anchoring it breaks the PING cadence).

## nRF54L field notes (things the SVD won't tell you)

- **TXPOWER is encoded** on the nRF54L: 0 dBm = `0x18`. The raw `0x00` is a
  reserved value that leaves the PA off — the radio runs its TX state machine
  but emits no RF ("phantom TX").
- **RRAM fetch latency**: the MPSL's timeslot arming needs the code-fetch RAM
  out of PowerOff (`RRAMC.LOWPOWERCONFIG = Standby`, set once at phy init) or
  the session deadline asserts (106:179). The official upstream's low-latency
  callbacks only handle the CPU CONSTLAT.
- **Event block at 0x200+**: the nRF54L radio's events sit at 0x200-0x220 (the
  nRF5340's at 0x100-0x110). Use the pac accessors — the earlier raw `0x100`
  clears were hitting reserved space.
- **TXD/RXD DMA amounts**: the SVD has no registers at 0xEE8/0xED4 (the pac
  maps TXD/RXD to the DFE sub-registers instead), but the writes are required
  on silicon — the TX transmits 0-length PDUs and the RX transfers nothing
  without them. Verified, kept raw.
- **Errata 54L/49**: the first on-air payload bits need a hidden register
  (0x5008C58C) set — not in the SVD by definition.

## Repository layout

```
thunders/            protocol core (framing, CRC, seq, Central/Peripheral)
thunders-phy-nrf/    nRF PHY: bare + MPSL backends (pac-typed register access)
examples/
  nrf5340/{bare,mpsl}    role-agnostic examples (central default / peripheral)
  nrf54lm20/{bare,mpsl}  role-agnostic examples
```
