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

| Pairing | Backend | Central poll | Frame | Round-trip | Peer RX | Errors |
|---|---|---|---|---|---|---|
| 5340 central ↔ LM20 peripheral | **bare** | 542 Hz | 1.84 ms | 322 RXOK / 2 s window | 322 frames | 0 |
| LM20 central ↔ 5340 peripheral | **bare** | 233 Hz | — | 320 RXOK | 329 frames | 0 |
| 5340 central ↔ LM20 peripheral | **mpsl** | **500 Hz** | 2.0 ms (RX+TX) | **GOT=57** | 395 PINGs | 0 |
| LM20 central ↔ 5340 peripheral | **mpsl** | **500 Hz** | 2.0 ms | **GOT=56** | 373 PINGs | 0 |

The MPSL backend runs at the same order as the raw radio (500 Hz vs 540 Hz
central poll) — the old 18× gap is gone; both are limited by the per-frame RX
poll, not the MPSL. The central always frames RX+TX (2 slots), the peripheral
mostly RX (1 slot, ~830 Hz on the MPSL).

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
