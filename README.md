# thunders

A compile-time, fixed-packet **one-way** link layer for the Nordic 2.4 GHz
RADIO, over two backends — a bare exclusive RADIO driver and an MPSL
timeslot backend. Three heterogeneous boards interoperate on one wire
format: **nRF52840**, **nRF5340** (net core), **nRF54LM20**.

```
┌──────────────────┐  fixed 6-byte state   ┌──────────────────┐
│ Sender           │ ─────────────────────▶ │ Receiver         │
│  data slots      │                        │  one long window │
│  feedback listen │ ◀───────────────────── │  3-byte beacon   │
└──────────────────┘   per batch boundary   └──────────────────┘
```

The sender publishes fixed-length state packets on a compile-time slot plan
(`OneWayState<PAYLOAD, FEEDBACK_EVERY>` + `fixed_slot_plan`); the receiver
keeps the newest value and answers once per batch with a 3-byte feedback
beacon — timing error, plus an **addressed recall** that pulls one sender
into a reliable negotiation phase. Details: [`docs/architecture.md`](docs/architecture.md)
(protocol/timing), [`docs/negotiation.md`](docs/negotiation.md) (recall).

## Crates

| Crate | Role |
|---|---|
| `thunders` | Protocol core: one-way modes, const slot planner, negotiation codec, `StaticTdma`, the `Phy` trait. `no_std`, board-agnostic. |
| `thunders-phy-nrf` | nRF PHY: **bare** (exclusive RADIO, TXIDLE/RXIDLE restart per packet) and **mpsl** (RADIO inside MPSL timeslots). Registers via `nrf-pac`. |
| `examples/{nrf52840,nrf5340,nrf54lm20}/{bare,mpsl}` | Board firmware; sender/receiver roles are separate binaries/Cargo features. |

MPSL is the stock [`alexmoon/nrf-sdc`](https://github.com/alexmoon/nrf-sdc)
git dependency; embassy HAL is upstream `embassy-rs/embassy` via
`[patch.crates-io]`. No vendored forks.

## Measured rates (full bench over every example combination)

| Link | TX | RX | Throughput | Integrity |
|---|---|---|---|---|
| MPSL one-way, default | LM20 3333/s | 52840 | 3332 pkt/s | 0 CRC-bad, 0 invalid |
| MPSL one-way, `phase-align` | LM20 3306/s | 52840 | 3306 pkt/s | 0 CRC-bad over 83k frames; recall demo echo confirmed |
| MPSL one-way, `hopping` | LM20 3304/s | 52840 | 3267 pkt/s | 0 CRC-bad while hopping 8 channels |
| MPSL multi-sender | LM20 1666/s + 5340 1111/s | 52840 | 1623/s + 898/s | both senders demultiplexed concurrently |
| Bare one-way | LM20 14705/s | 52840 | 14627 pkt/s | 0% loss, feedback flowing |
| Bare one-way, `hopping` | LM20 14723/s | 52840 | 14625 pkt/s | 0% loss |
| Bare TDMA, 2 senders | 52840 5265/s + 5340 5221/s | LM20 | 5258/s + 5160/s | 0 invalid; ~2% CRC-bad from collisions |

## Optional features (MPSL examples)

| Feature | Effect |
|---|---|
| `phase-align` | Per-batch feedback event (receiver replies, sender locks). Enables recall. |
| `hopping` | Synchronized 8-channel hopping at the batch boundary (implies `phase-align`). |
| `multi-sender` | nrf52840 receiver demultiplexes up to 8 senders by RADIO logical address. |
| `multi-sender-bench` / `multi-receiver` | The 2-TX/1-RX benchmark roles. |
| `sender-1` | nrf54lm20 transmits with the 0xC3 prefix (second-sender role). |

The bare examples add a const-generic TDMA pair (`--bin tdma-sender` /
`--bin tdma-receiver`, shared schedule in `examples/tdma_config.rs`).

## Building & flashing

Prereqs: `probe-rs`, Rust targets `thumbv7em-none-eabihf` (nRF52840),
`thumbv8m.main-none-eabi` (nRF5340), `thumbv8m.main-none-eabihf` (nRF54LM20).

```sh
# receiver: nRF52840 (DAPLink)
cd examples/nrf52840/mpsl
cargo build --release --no-default-features --features phase-align
probe-rs run --chip nRF52840_xxAA --probe "0d28:0204-3:0700..." \
  target/thumbv7em-none-eabihf/release/thunders-mpsl

# sender: nRF54LM20 (J-Link; auto-detect fails, --chip is required)
cd examples/nrf54lm20/mpsl
cargo build --release --no-default-features --features phase-align
probe-rs run --verify --disable-double-buffering --chip nRF54LM20A \
  --probe 1366:1069 target/thumbv8m.main-none-eabihf/release/thunders-mpsl
```

The receiver's RTT log runs the recall demo (foreign prefix, addressed
recall, echo, release). The nRF5340 net core is debug-locked: every flash
needs `--allow-erase-all`.

## nRF54L field notes (things the SVD won't tell you)

- **TXPOWER is encoded**: 0 dBm = `0x18`. Raw `0x00` leaves the PA off (the
  radio runs its TX state machine but emits no RF).
- **Event block at 0x200+**: the nRF54L radio's events sit at 0x200-0x220
  (the nRF5340's at 0x100-0x110).
- **TXD/RXD DMA amounts**: no SVD registers at 0xEE8/0xED4, but the writes
  are required on silicon (0-length PDUs / no RX otherwise).
- **Errata 54L/49**: the first on-air payload bits need a hidden register
  (0x5008C58C) set.
- **Post-flash boot faults = corrupted flash content**: probe-rs's
  *double-buffered* flashing downloads the next page buffer over MEM-AP
  while the CPU runs the RRAMC buffered-write algorithm; the bus contention
  mis-slots one word near the start of each `ProgramPage` call (dropped +
  next word duplicated). Boots then fault semi-randomly (blob assert, null
  call, bus fault); resets don't help because the corruption is in the RRAM
  content itself. Always flash the LM20 with `--disable-double-buffering`
  and/or `--verify`. Both LM20 examples install a precise HardFault handler
  (CFSR/HFSR/BFAR, stacked PC/R0/LR, fault-context dump over defmt) so any
  recurrence is diagnosable.
- **Retracted**: an early `RRAMC.LOWPOWERCONFIG = Standby` write turned out
  unnecessary — the boot asserts attributed to fetch latency were
  flash-corruption artifacts. Don't cargo-cult it back.
