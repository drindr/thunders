# thunders

A slot-based link layer for the Nordic 2.4 GHz RADIO, over two backends — a
raw RADIO driver and an MPSL timeslot backend (the Zephyr ESB-on-MPSL
pattern). Three heterogeneous boards interoperate on one wire format:
**nRF52840**, **nRF5340** (net core), **nRF54LM20** — any role, any backend.

```
┌──────────────────┐  Data (seq)       ┌──────────────────┐
│ Central          │ ────────────────▶ │ Peripheral       │
│  TX slots        │ ◀──────────────── │  TX slots         │
│  RX slots        │  reverse Data     │  RX slots         │
└──────────────────┘                   └──────────────────┘
```

The link is **slot-based**: every slot is a TX or an RX (never both), and the
mix is the `Config::tx_rx_ratio` — `(8, 2)` by default (eight TX slots
followed by two RX slots on the central; the peripheral mirrors). Full-duplex
is the interleaved slots, not the in-frame turnaround. Reliability is
**selective-repeat ARQ**:
every `Data` packet piggybacks a cumulative ACK + a variable-length
slot-position NACK bitmap (the bitmap covers the slots of the last TX run),
and the link retransmits the named slot losses. A packet that exhausts its
retry budget is announced with `Packet::Drop { seq, ack, nack }`, which
advances the receiver past the hole without stalling the stream — so each
payload is delivered exactly once, in order, or reported as a delivery
failure (never silently dropped).

> Documentation map: [`docs/README.md`](docs/README.md) lists every page
> and the recommended reading order.

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
  with the default 2 Mbit mode (the bare PHY's slot scheduler makes every
  slot start equidistant, so the two boards share one cadence), the **600 µs
  bare slot** with the `radio-1m` example feature, and the 500 µs slot on
  the MPSL (its fallback cadence). `Central::frame` / `Peripheral::frame`
  run one slot each, the TX/RX type from the ratio.
- **The TX:RX ratio** — `Config::with_tx_rx_ratio(tx, rx)`. The central runs
  `tx` TX slots then `rx` RX slots; the peripheral mirrors.
- **The hopping** — beacon-driven and gated by the connection state: while
  `Disconnected` both sides stay on `initial_channel`; once `Connected` the
  central advances the 25-channel sequence after `HOP_MISS_THRESHOLD`
  consecutive misses and the peripheral follows the beacon's
  `channel_index`. No free-running local hop.
- **The burst** — the central's TX run uses `Phy::transmit_burst_begin/send`
  (the ramp once, the on-air per packet, ~10-12 kHz one-way on the bare). The
  MPSL falls back to the plain per-slot transmit.
- **The typed interface** — `send<T: Serialize>(&T)` / `recv<T:
  DeserializeOwned>()` — the app works with its own struct; postcard, crypto,
  CRC and radio are invisible. `send` waits until a future `frame` has
  enqueued the value (backpressure); raw `frame` callers use the one-slot
  offer semantics described in the protocol doc.

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

| Example | default features | extra features |
|---|---|---|
| `nrf52840/bare` | `central` | `peripheral`, `radio-1m`, `secure`, `one-way` |
| `nrf52840/mpsl` | `central` | `peripheral`, `radio-1m` |
| `nrf5340/bare` | `central`, `host` | `peripheral`, `radio-1m`, `secure` |
| `nrf5340/mpsl` | `central`, `host` | `peripheral`, `radio-1m` |
| `nrf54lm20/bare` | `central` | `peripheral`, `radio-1m` |
| `nrf54lm20/mpsl` | `central` | `peripheral`, `radio-1m` |

`radio-1m` selects `RadioMode::Nrf1Mbit` (more link margin, lower
throughput) and must be enabled on both nodes of a run. The 5340 `host`
feature enables the mailbox integration below and must be disabled for the
standalone bench firmware.

## nRF5340 host/controller split

`examples/nrf5340-app/` is the **host** on the nRF5340's application core; the
**controller** (the radio link) runs on the network core (`examples/nrf5340/mpsl`).
They share a 4 KiB mailbox at `0x2007F000` (the app core's top RAM, made
non-secure via the SPU) plus two IPC channels: `event0` = host → net (a TX
payload is ready), `event1` = net → host (an RX payload was stored). The host
`put_tx`s a payload and triggers `event0`; the net core polls the mailbox,
transmits it on its TX slot, and on each catch stores the RX and triggers
`event1`.

The mailbox is **example-level code**, not part of the `thunders` core:
it lives in `examples/nrf5340/ipc.rs` and is included by the two net-core
examples and by `examples/nrf5340-app` via a `#[path]` module.

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
`[patch.crates-io]` in the nRF54LM20 examples' Cargo.toml).

The radio mode is selectable per example with `--features radio-1m` (both
peers must match); the bench wrapper selects it with
`THUNDERS_RADIO_MODE=1m scripts/bench.sh build && THUNDERS_RADIO_MODE=1m scripts/bench.sh run`.

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
per 5 s window (the `BENCH` lines; the parser drops the first window as the
connection-forming warmup when the capture has more than one window):

| metric | who | how |
|---|---|---|
| **RTT** (latency) | central | PING TX slot → echo RX slot (`rtt_avg/min/max`). The peripheral echoes the last PING of each 8-slot TX run, so RTT ≈ 1 slot period (500 µs mpsl / 400 µs bare 2M / 600 µs bare 1M) + processing. |
| **bandwidth** | central | `bw` = payload bytes/s both ways (8 B per PING + 8 B per echo). `rate` = the slot rate. |
| **forward loss** | peripheral | `floss` = seq gaps / expected after ARQ — the central→peripheral leg. Bench offers only when the TX window has space, so a full window is never counted as loss. |
| **reverse raw loss** | central | `rloss` = RX slots with no echo — the raw peripheral→central radio hit rate. |
| **reverse ARQ loss** | central | `rev_loss` = echo PING seq gaps after retransmit/reorder — the delivered peripheral→central loss. |
| **busy** | both | the app's per-slot processing time. |

All six examples (52840/5340/LM20 × bare/mpsl) emit the same `BENCH C` /
`BENCH P` field contract that `bench_parse.py` consumes, so the runs are
comparable. Per-board setup and diagnostics still differ: the 5340 examples
can build the `host` mailbox integration, the bare examples add `RADIO` /
`BARE PLL` lines, and the MPSL examples add the `PLL` line.

### Running the matrix

All **six directed pairs** × both backends = 12 runs, one command:

```sh
scripts/bench.sh build     # build all 12 ELFs into bench/bin (per role)
scripts/bench.sh run 30    # the full matrix, 30 s per run -> bench/logs/
scripts/bench_parse.py     # the summary table (fwd/rev loss, RTT, bw, rate)

# 1 Mbit mode (more margin, lower throughput): build and run with the same mode
THUNDERS_RADIO_MODE=1m scripts/bench.sh build
THUNDERS_RADIO_MODE=1m scripts/bench.sh run 30
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

### Results

The current standard 12-run matrix, the bench workflow, and the fixed
acquisition/synchronization practices are recorded in
[`docs/bench-stability.md`](docs/bench-stability.md). Raw captures are in
`bench/logs/`; `scripts/bench_parse.py` only summarizes the 12 canonical
runs.

Every canonical row produces both central and peripheral `BENCH` data.
Forward delivery loss is 0–18% across the matrix (`wf=0` — offers are
gated by `tx_window_full()`). The still-large `rev_raw%` is the raw reverse
slot hit rate; `rev_arq%` is the post-ARQ reverse delivery loss, which
remains high where the reverse radio path itself has little capacity.

Bare diagnostics are in the `RADIO` and `BARE PLL` bench lines; MPSL RSSI
and RX/TX phase histograms are in the `PLL` line.
`scripts/bench_parse.py --rssi` prints the raw RSSI samples.

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
  README.md                 documentation map + reading order
  architecture.md           the three-layer stack + the Phy contract
  hopping-txrx-ratio-full-duplex.md   the slot protocol + ARQ
  connection-state-machine.md         the hop gate
  timesync-slot-alignment.md          the slot grid + acquisition
  bench-stability.md        the bench workflow + current matrix (source of truth)
  bench-results.md          historical pointer to bench-stability.md
  debugging-notes.md        historical hardware-debugging log
examples/
  nrf52840/{bare,mpsl}    role-agnostic (thumbv7em-none-eabihf)
  nrf5340/{bare,mpsl}     role-agnostic (thumbv8m.main-none-eabi)
  nrf5340-app/            the host/controller host (thumbv8m.main-none-eabihf)
  nrf54lm20/{bare,mpsl}   role-agnostic (thumbv8m.main-none-eabihf)
```
