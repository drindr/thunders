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

The protocol is one-way streaming: the sender publishes fixed-length state
packets on a compile-time slot plan, the receiver keeps the newest value.
Once per batch the receiver answers with a three-byte feedback beacon —
timing error today, plus an **addressed recall** that pulls one sender into
a reliable negotiation phase for link configuration. Reliability is a
runtime phase, not a type parameter.

> Documentation map: [`docs/architecture.md`](docs/architecture.md) is the
> protocol/timing reference with measured numbers;
> [`docs/negotiation.md`](docs/negotiation.md) specifies the recall and the
> negotiation phase.

## Architecture

```
thunders         OneWay<PAYLOAD, FEEDBACK_EVERY>, FixedSlotPlan, negotiation codec
thunders-phy-nrf Phy trait → NrfRadioPhy (bare exclusive) / MpslRadioPhy (timeslots)
examples/        one firmware per board/backend; roles are separate binaries
```

| Crate | Role |
|---|---|
| `thunders` | Protocol core: the `OneWay` mode family and const slot planner, the one-way engine (`OneWaySender`/`OneWayReceiver`), the negotiation beacon/config codec, the const-generic `StaticTdma` schedule, the `Phy` trait. Board-agnostic, `no_std`. |
| `thunders-phy-nrf` | The nRF PHY. Two backends: **bare** (exclusive RADIO, one initial ramp, TXIDLE/RXIDLE restart per packet) and **mpsl** (the radio inside MPSL timeslots, chained grants). All registers via `nrf-pac`. |
| `examples/{nrf52840,nrf5340,nrf54lm20}/{bare,mpsl}` | Board firmware; the sender/receiver roles are selected by binary and Cargo feature, not by a runtime role. |

The MPSL layer is the stock official [`alexmoon/nrf-sdc`](https://github.com/alexmoon/nrf-sdc)
git dependency — no vendored fork. The nRF54L-specific bits (the RRAM
fetch latency, the NVIC enables, the HFXO trim) live in `thunders-phy-nrf`.
The embassy HAL comes from upstream `embassy-rs/embassy` pinned by
`[patch.crates-io]` in the nRF54LM20 examples; no local checkout needed.

## The protocol in one paragraph

`OneWayState<PAYLOAD, FEEDBACK_EVERY>` fixes the payload length, the batch
size and both slot durations at compile time (`fixed_slot_plan`). The MPSL
sender runs N data slots plus one feedback listen; the receiver holds one
long RX window per batch and answers after the last data phase — that beacon
is also the hop/negotiation epoch boundary. See `docs/architecture.md` for
the exact arithmetic and the measured rates (3.3k packets/s over MPSL,
16.2k packets/s bare, both with CRC16 and zero invalid frames).

## Optional features (MPSL examples)

| Feature | Effect |
|---|---|
| `phase-align` | Reserve the per-batch feedback event (receiver replies, sender locks). Enables recall. |
| `hopping` | Synchronized 8-channel hopping at the batch boundary (implies `phase-align`). |
| `multi-sender` | nrf52840 receiver demultiplexes up to 8 senders by RADIO logical address (RXMATCH). |
| `multi-sender-bench` / `multi-receiver` | The 2-TX/1-RX benchmark roles. |
| `sender-1` | nrf54lm20 transmits with the 0xC3 prefix (second-sender role). |

The bare examples carry the same one-way stream at ~16.2k packets/s and a
const-generic TDMA pair (`--bin tdma-sender` / `--bin tdma-receiver`, shared
schedule in `examples/tdma_config.rs`).

## Addressed recall (negotiation phase)

The receiver recalls one sender by asserting `NEG_REQ` + the target's
on-air address prefix in every feedback beacon (level-triggered). The
addressed sender stops streaming and echoes a `ConfigFrame` status in every
slot until released; unmatched senders ignore the request. Phase switches
happen at the batch boundary that already synchronizes hopping, so the slot
grid never changes. Hardware-verified (LM20 → 52840): foreign-prefix recall
ignored, addressed recall confirmed by echo in 232 ms, seamless release.
Details and the power/frequency command-loop roadmap: `docs/negotiation.md`.

## Building & flashing

Prereqs: `probe-rs`, and the `thumbv7em-none-eabihf` (nRF52840),
`thumbv8m.main-none-eabi` (nRF5340) and `thumbv8m.main-none-eabihf`
(nRF54LM20) Rust targets.

```sh
# receiver: nRF52840 (DAPLink)
cd examples/nrf52840/mpsl
cargo build --release --no-default-features --features phase-align
probe-rs run --chip nRF52840_xxAA --probe "0d28:0204-3:0700..." \
  target/thumbv7em-none-eabihf/release/thunders-mpsl

# sender: nRF54LM20 (J-Link; auto-detect fails, --chip is required)
cd examples/nrf54lm20/mpsl
cargo build --release --no-default-features --features phase-align
probe-rs run --verify --chip nRF54LM20A --probe 1366:1069 \
  target/thumbv8m.main-none-eabihf/release/thunders-mpsl
```

The receiver's RTT log shows the recall demo (foreign prefix, addressed
recall, echo, release); the sender logs its negotiation transitions. The
nRF5340 net core is debug-locked: every flash needs `--allow-erase-all`.

## nRF54L field notes (things the SVD won't tell you)

- **TXPOWER is encoded**: 0 dBm = `0x18`. The raw `0x00` leaves the PA off
  (the "phantom TX" — the radio runs its TX state machine but emits no RF).
- **RRAM fetch latency**: the MPSL blob's timeslot arming runs on hard
  deadlines and asserts (MPSL assert `106:179`) when instruction fetch is
  slow. `RRAMC.LOWPOWERCONFIG = Standby` must be set *before* the session
  opens — the first timeslot can be granted immediately after the first
  request. `rramc_fast_fetch()` runs at the top of `MpslRadioPhy::new` and
  at the top of the LM20 examples' main.
- **Event block at 0x200+**: the nRF54L radio's events sit at 0x200-0x220 (the
  nRF5340's at 0x100-0x110).
- **TXD/RXD DMA amounts**: no SVD registers at 0xEE8/0xED4, but the writes are
  required on silicon (0-length PDUs / no RX otherwise).
- **Errata 54L/49**: the first on-air payload bits need a hidden register
  (0x5008C58C) set.
- **Post-flash boot faults = corrupted flash content**: a boot right after a
  *real* flash (different image written) used to fault semi-randomly during
  MPSL/app init — blob assert, null-pointer call, precise bus fault at
  address 0, bogus index. Root cause found by diffing RRAM against the ELF
  after a failed boot: probe-rs's nRF54LM20 flash algorithm occasionally
  leaves stale words behind (observed pattern: the word at page+0x004 in a
  few 4 KiB pages keeps the previous image's content). The CPU then executes
  corrupted instructions; which addresses are bad — and whether they sit on
  the boot path — decides the signature, which is why removing dead code
  ("layout-sensitive landmine") could make it deterministic. Plain resets
  and same-image reflashes are clean because the content is already
  correct. **Always flash the LM20 with `--verify`** (read-back compare);
  on `Flash content verification failed` just retry — the reflash rewrites
  the bad words and the next boot is clean. Both LM20 examples install a
  precise HardFault handler (CFSR/HFSR/BFAR, stacked PC/R0/LR, fault-context
  and peripheral-register dump over defmt) so any recurrence is diagnosable.

## Repository layout

```
thunders/                 protocol core (modes, slot plan, one-way engine, negotiation, TDMA)
thunders-phy-nrf/         nRF PHY: bare exclusive + MPSL backends (pac-typed registers)
docs/
  architecture.md           the one-way protocol, timing arithmetic, measured numbers
  negotiation.md            addressed recall + negotiation phase design and status
examples/
  nrf52840/{bare,mpsl}    thumbv7em-none-eabihf
  nrf5340/{bare,mpsl}     thumbv8m.main-none-eabi (net core)
  nrf54lm20/{bare,mpsl}   thumbv8m.main-none-eabihf
  tdma_config.rs            the shared compile-time TDMA schedule
```
