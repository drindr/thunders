# thunders

A slot-based link layer for the Nordic 2.4 GHz RADIO, over two backends — a
raw RADIO driver and an MPSL timeslot backend (the Zephyr ESB-on-MPSL
pattern). Three heterogeneous boards interoperate on one wire format:
**nRF52840**, **nRF5340** (net core), **nRF54LM20** — any role, any backend.

```
┌─────────────┐   Data (seq)     ┌─────────────┐
│ Central     │ ───────────────▶ │ Peripheral  │
│  TX slots   │ ◀─────────────── │  RX slots    │
└─────────────┘   reverse Data   └─────────────┘
```

The link is **slot-based**: every slot is a TX or an RX (never both), and the
mix is the `Config::tx_rx_ratio` — `(8, 1)` by default (the 8 kHz one-way +
a 1 kHz reverse channel on the bare path). Full-duplex is the interleaved
slots, not the in-frame turnaround. Reliability is the **sequence window**, not
acks.

## Architecture

Three layers — see `docs/architecture.md` for the full picture:

```
postcard   Packet::to_bytes / from_bytes  (the wire format)
thunders   Central/Peripheral, Config, Scheduler, Security
phy        Phy trait → NrfRadioPhy (bare) / MpslRadioPhy (timeslots)
```

| Crate | Role |
|---|---|
| `thunders` | Protocol core: the slot-aware `Central`/`Peripheral`, the `Config` (the ratio, the channels), the `Scheduler` (the hop), the `Security` (the ChaCha/CCM), the `Phy` trait. Board-agnostic. |
| `thunders-phy-nrf` | The nRF PHY. Two backends: **bare** (the direct RADIO, the burst TX, the hardware CCM) and **mpsl** (the radio inside the MPSL timeslots). All registers via the `nrf-pac` accessors. |
| `examples/{nrf52840,nrf5340,nrf54lm20}/{bare,mpsl}` | The role-agnostic firmware, one binary per role. |

The MPSL layer is the **stock official** [`alexmoon/nrf-sdc`](https://github.com/alexmoon/nrf-sdc)
git dependency — no vendored fork. The nRF54L-specific bits (the RRAM
fetch-latency, the NVIC enables) live in `thunders-phy-nrf`.

## The protocol (see `docs/hopping-txrx-ratio-full-duplex.md`)

- **The slot model** — the **software-paced 400 µs slot** on the bare path
  (the bare PHY's slot scheduler makes every slot start equidistant, so the
  two boards share one cadence) and the 500 µs slot on the MPSL (its grant
  floor). `Central::frame` / `Peripheral::frame` run one slot each, the
  TX/RX type from the ratio.
- **The TX:RX ratio** — `Config::with_tx_rx_ratio(tx, rx)`. The central runs
  `tx` TX slots then `rx` RX slots; the peripheral mirrors.
- **The hopping** — beacon-driven: the central advances the 25-channel
  sequence after `HOP_MISS_THRESHOLD` consecutive misses, the peripheral
  follows the beacon's `channel_index`. No free-running local hop.
- **The burst** — the central's TX run uses `Phy::transmit_burst_begin/send`
  (the ramp once, the on-air per packet, ~10-12 kHz one-way on the bare). The
  MPSL falls back to the plain per-slot transmit.
- **The typed interface** — `send<T: Serialize>(&T)` / `recv<T:
  DeserializeOwned>()` — the app works with its own struct; the postcard, the
  crypto, the CRC and the radio are invisible.

## Security

`Security::new(key)` uses the software ChaCha20-Poly1305 (the default);
`Security::with_ccm(key)` uses the radio's **hardware AES-CCM** (the AEAD over
the EasyDMA; the nRF52840/nRF5340 bare backends — the nRF54L falls back to
`Unsupported`). The CRC is always the radio's hardware CRC-16 — no software CRC.

## Roles

Each example builds once; the role is a build-time feature:

```sh
cargo build --release                          # central (default)
cargo build --release --no-default-features \
  --features peripheral                        # peripheral
```

## nRF5340 host/controller split

`examples/nrf5340-app/` is the **host** on the nRF5340's application core; the
**controller** (the radio link) runs on the network core (`examples/nrf5340/mpsl`).
They share a 4 KiB mailbox at `0x2007F000` (the app core's top RAM, made
non-secure via the SPU) plus two IPC channels: `event0` = host → net (a TX
payload is ready), `event1` = net → host (an RX payload was stored). The host
`put_tx`s a payload and triggers `event0`; the net core polls the mailbox,
transmits it on its TX slot, and on each catch stores the RX and triggers
`event1`.

Role pairing (the MPSL phase-lock is peripheral-only and was verified on the
nRF5340): the **5340 net core = peripheral** (follower), the **peer = 52840
central** (time master). Build/flash with cargo-make:

```sh
cd examples/nrf5340-app
cargo make flash        # build+flash net (peripheral), then flash+run the host
cargo make run          # host only (the net core's flash persists)
```

Run the 52840 peer as the central first (`cargo build --release` in
`examples/nrf52840/mpsl`), then `cargo make run` on the 5340. The host's RTT
shows `host TX` / `host RX` pairs — the full `host TX → radio → echo → host RX`
loop.

## Building & flashing

Prereqs: `probe-rs`, the `thumbv7em-none-eabihf` (nRF52840),
`thumbv8m.main-none-eabi` (nRF5340) and `thumbv8m.main-none-eabihf` (nRF54L)
targets, and the patched embassy HAL at `../../embassy-nrf54` (the
`[patch.crates-io]` in each example's Cargo.toml).

```sh
# nRF52840, DAPLink
probe-rs run --chip nRF52840_xxAA \
  --probe "0d28:0204-3:0700000100440055360000054e534d4ca5a5a5a597969908" \
  target/thumbv7em-none-eabihf/release/thunders-<name>

# nRF5340 (net core), DAPLink
probe-rs run --chip nRF5340_xxAA \
  --probe "0d28:0204-3:13040003001100e10465599500004fca0000000097969921" \
  --allow-erase-all target/thumbv8m.main-none-eabi/release/thunders-<name>

# nRF54LM20, J-Link
probe-rs run --chip nRF54LM20A --probe 1366:1069 --speed 100 \
  target/thumbv8m.main-none-eabihf/release/thunders-<name>
```

Flash the peripheral first, then the central — the logs show the seq'd Data
and the periodic `BENCH` summary.

## Benchmarks

Every example's bench loop reports **latency, bandwidth and packet loss**
per 5 s window (the `BENCH` lines, first window dropped as the
connection-forming warmup):

| metric | who | how |
|---|---|---|
| **RTT** (latency) | central | PING TX slot → echo RX slot (`rtt_avg/min/max`). The peripheral echoes the last PING of each 8-slot TX run, so RTT ≈ 1 slot period (500 µs mpsl / 400 µs bare) + processing. |
| **bandwidth** | central | `bw` = payload bytes/s both ways (8 B per PING + 8 B per echo). `rate` = the slot rate. |
| **forward loss** | peripheral | `floss` = seq gaps / expected — the central→peripheral leg. The PING seq is a per-PING counter, so structural gaps (beacons, ratio skips) are excluded; gaps ≥ 1 M seqs are peer restarts, not loss. |
| **reverse loss** | central | `rloss` = RX slots with no echo — the peripheral→central leg. |
| **busy** | both | the app's per-slot processing time. |

The bench loop is identical across the six examples (52840/5340/LM20 ×
bare/mpsl), so the runs are comparable. The MPSL `PLL` lines carry the
phase-lock diagnostics, the bare `RADIO` lines the radio counters.

### Running the matrix

All **six directed pairs** × both backends = 12 runs, one command:

```sh
scripts/bench.sh build     # build all 12 ELFs into bench/bin (per role)
scripts/bench.sh run 30    # the full matrix, 30 s per run -> bench/logs/
scripts/bench_parse.py     # the summary table (fwd/rev loss, RTT, bw, rate)
```

`run-pair 52840 lm20 mpsl 30` runs a single directed pair. A run flashes the
peripheral first (its slot session must be open before the central boots),
then the central; both RTT streams are captured per role. The 5340 examples
build a `host` feature for the app-core mailbox integration (default ON);
the bench builds it OFF (`--no-default-features --features <role>`), so the
net core runs standalone — without the host's SPU config the mailbox RAM is
secure and any access faults. (The 5340 net core is debug-locked, so every
flash needs `--allow-erase-all`, which also wipes the app core — irrelevant
for the standalone bench.)

### Results (2025-08-15, `docs/bench-results.md`)

The latest full 30 s matrix is in `docs/bench-results.md` (fourth pass).
Summary: the working links are still the pairs with **LM20 as the
peripheral**, now on both backends:

| run | backend | fwd loss | rev loss | rtt avg | bw |
|---|---|---|---|---|---|
| 52840 → LM20 | mpsl | 12-14 % | 13 % | 633 µs | 15.7 kB/s |
| 5340 → LM20 | mpsl | 13 % | 13 % | 644 µs | 15.7 kB/s |
| 52840 → LM20 | bare | 13 % (best windows) | 13-68 % | 480 µs | 18.3 kB/s |
| 5340 → LM20 | bare | 24 % (best windows) | 17-49 % | 484 µs | 18.8 kB/s |

The bare path is no longer dead: the software slot scheduler, Fast ramp,
TX on-air alignment and empty-slot pacing give it a real link with the
LM20 peripheral. The remaining 97-100 %-loss rows are the same
pre-existing RF-level issues as the MPSL matrix — the 5340 peripheral RX
and the 52840/5340 central RX into those peripherals.

Bare diagnostics are in the `RADIO` and `BARE PLL` bench lines; MPSL RSSI
is in the `PLL` line. `scripts/bench_parse.py --rssi` prints the raw RSSI
samples. `scripts/simulate_bare_scheduler.py` models the bare phase-lock
on a host.

The per-window detail and the raw logs live in `docs/bench-results.md` and
`bench/logs/`.

## nRF54L field notes (things the SVD won't tell you)

- **TXPOWER is encoded**: 0 dBm = `0x18`. The raw `0x00` leaves the PA off
  (the "phantom TX" — the radio runs its TX state machine but emits no RF).
- **RRAM fetch latency**: the MPSL's timeslot arming needs the code-fetch RAM
  out of PowerOff (`RRAMC.LOWPOWERCONFIG = Standby`) or the session deadline
  asserts (106:179).
- **Event block at 0x200+**: the nRF54L radio's events sit at 0x200-0x220 (the
  nRF5340's at 0x100-0x110).
- **TXD/RXD DMA amounts**: no SVD registers at 0xEE8/0xED4, but the writes are
  required on silicon (0-length PDUs / no RX otherwise).
- **Errata 54L/49**: the first on-air payload bits need a hidden register
  (0x5008C58C) set.

## Repository layout

```
thunders/                 protocol core (the slot machine, the config, the security)
thunders-phy-nrf/         nRF PHY: bare + MPSL backends (the pac-typed registers)
docs/
  architecture.md          the three-layer stack
  hopping-txrx-ratio-full-duplex.md   the slot protocol
examples/
  nrf52840/{bare,mpsl}    role-agnostic (thumbv7em-none-eabihf)
  nrf5340/{bare,mpsl}     role-agnostic (thumbv8m.main-none-eabi)
  nrf5340-app/            the host/controller host (thumbv8m.main-none-eabihf)
  nrf54lm20/{bare,mpsl}   role-agnostic (thumbv8m.main-none-eabihf)
```
