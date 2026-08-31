# Thunders one-way architecture

## 1. Scope

The active protocol is compile-time, fixed-length, and one-way. The former
symmetric Central/Peripheral link, cadence negotiation, probe/PLL servo, dynamic
payload contracts, and bidirectional selective-repeat scheduler were removed.

The mode family is:

```rust
OneWay<PAYLOAD, ACK, FEEDBACK_EVERY>
```

Public aliases:

```rust
OneWayState<PAYLOAD, DIFF_EVERY> // ACK=false; newest state wins
OneWayChanges<PAYLOAD>           // ACK=true; changes must be delivered
```

`OneWayState` does not promise delivery of every snapshot. It is appropriate for
position, orientation, sensor state, current actuator target, and similar data
where a newer packet replaces an older packet.

`OneWayChanges` is for commands and transitions that must not disappear. Its
current engine is stop-and-wait: retain one payload, retransmit until the
matching ACK, then accept the next change.

## 2. Current hardware test configuration

Both MPSL examples currently instantiate:

```rust
const PAYLOAD: usize = 6;
const HOPPING: bool = cfg!(feature = "hopping");
const FEEDBACK_EVERY: u16 = if HOPPING { 120 } else { 128 };
type Mode = OneWayState<PAYLOAD, FEEDBACK_EVERY>;
```

Meaning:

```text
application state bytes: 6
Data events per batch: 128 normally, 120 with hopping
radio mode: Nordic proprietary 2 Mbit
address: E7:E7:E7:E7:E7
CRC: CRC16-CCITT, polynomial 0x11021, init 0xFFFF
initial channel: 0 (2400 MHz mapping used by the backend)
```

The six-byte benchmark state is:

```text
byte 0: 'S'
byte 1: 'T'
byte 2..5: little-endian u32 state counter
```

The `u32` counter is benchmark application data. It is not a protocol sequence
or header.

## 3. State packet format

### 3.1 MPSL Data packet

MPSL configures a fixed packet:

```text
LFLEN=0
S0LEN=0
S1LEN=0
STATLEN=6
MAXLEN=6
PLEN=16-bit
BALEN=4 plus one prefix byte = five-byte address
CRC length=2
```

Complete on-air state packet:

```text
2-byte preamble     8 us
5-byte address     20 us
6-byte state       24 us
2-byte CRC          8 us
-------------------------
total              60 us
```

There is no transmitted length, marker, or protocol sequence.

### 3.2 TimeDiff feedback packet

When enabled, reverse feedback is exactly:

```text
i16 diff_us, little-endian
```

Its fixed `STATLEN` is two bytes. Complete on-air duration is approximately
44 us with the same preamble/address/CRC configuration.

### 3.3 Bare exclusive packet

The bare exclusive path additionally uses an eight-bit preamble, so its state
packet airtime is approximately:

```text
1-byte preamble + 5-byte address + 6-byte state + 2-byte CRC = 14 bytes
14 × 4 us at 2 Mbit = 56 us
```

## 4. Compile-time MPSL timing

The schedule is produced by:

```rust
const PLAN: OneWayMpslPlan =
    one_way_mpsl_plan::<Mode, PHASE_ALIGN>(
        AirTiming::NRF_2MBIT,
        SlotOverhead::MPSL_CONSERVATIVE,
    );
```

Current compile-time arithmetic:

```text
state airtime:                    60 us
TX/RX setup+ramp:                 50 us
radio tail reserve:               40 us
MPSL mandatory inter-slot gap:   150 us
compile-time margin:               0 us
----------------------------------------
Data event:                      300 us
```

Feedback receives an additional 10 us window:

```text
Data event:      300 us
Feedback event:  310 us
Batch size:      128 Data events
```

Cycles:

```text
phase-align off: 128 × 300                = 38400 us
phase-align on:  128 × 300 + 310          = 38710 us
```

The receiver uses one long event per cycle:

```text
phase-align off RX window: 38400 us nominal
phase-align on RX window:  38710 us nominal
```

MPSL subtracts its 150 us hand-back interval from the granted radio work time.

## 5. MPSL receiver model

The receiver follows the q305 latest-state model:

```text
one six-byte EasyDMA buffer
one RXEN/ramp at the beginning of the long event
END -> validate CRC -> increment packet count
latest state is overwritten in place
TASKS_START from RXIDLE for the next packet
```

It does not allocate or preserve 128 historical packet buffers. Application
context receives:

```text
latest six-byte state
number of CRC-valid packets observed in the completed long event
```

## 6. Optional phase alignment

Build without feedback:

```bash
cd examples/nrf52840/mpsl
cargo build --release --no-default-features

cd examples/nrf54lm20/mpsl
cargo build --release --no-default-features
```

Build with periodic TimeDiff:

```bash
cargo build --release --no-default-features --features phase-align
```

With `phase-align` disabled:

```text
transmitter cycle contains 128 Data events only
receiver never switches to TX
feedback cost is zero
```

With `phase-align` enabled:

```text
128 Data events
one 310 us feedback RX/TX event
receiver returns two-byte diff_us
```

The receiver identifies the transmitter batch boundary from the long address
interval around the reserved feedback event. It then sends TimeDiff only after
Data phase 127, so the reply lands in the transmitter feedback event.

Measured LM20 -> nRF52840 results:

| Configuration | TX packets/s | RX packets/s | Invalid | TX cost |
|---|---:|---:|---:|---:|
| phase-align off | 3333 | 3333-3334 | 0 | baseline |
| phase-align on | 3305-3306 | 3281 | 0 | about 0.81% |

## 7. Optional hopping

Enable synchronized hopping with:

```bash
cargo build --release --no-default-features --features hopping
```

Cargo defines:

```toml
hopping = ["phase-align"]
```

Hopping therefore always reserves the TimeDiff boundary. The compile-time
channel sequence is:

```text
0, 13, 29, 43, 57, 71, 89, 97
```

Startup and hop procedure:

```text
1. Both peers start on channel 0.
2. Receiver detects the feedback gap and learns Data phase 0..119.
3. Receiver sends TimeDiff after phase 119.
4. Transmitter receives TimeDiff in its feedback event.
5. Both peers switch channel at that batch boundary.
6. hop_index advances modulo eight.
```

Recovery:

```text
if a hopped receiver window contains zero valid packets:
    reset hop_index to 0
    switch to channel 0
    clear hop lock and phase history
    reacquire through the TimeDiff handshake
```

Hopping uses 120 Data packets per batch:

```text
hop cycle: 120 × 300us + 310us = 36310us
hop frequency: about 27.5 hops/s
previous 128-packet frequency: about 25.8 hops/s
increase: about 6.7%
```

Measured 120-packet hopping results after lock:

```text
TX: 3304-3305 packets/s
RX: 3229-3268 packets/s (average about 3251/s)
steady receive ratio: about 98.4%
invalid: 0 in most windows
```

A 112-packet trial occasionally lost hop lock. A 124-packet trial acquired only
the first hop and then stopped receiving feedback. A 96-packet trial remained
locked but had a larger steady receive-rate penalty. The 120-packet setting is
therefore retained as the moderate frequency increase.

A transmitter restart recovery test was performed with the original 128-packet
setting. The receiver returned to channel 0, reacquired the restarted
transmitter, re-established the hop epoch, and resumed approximately 3290
packets/s without resetting the receiver. The same channel-zero recovery state
machine is used by the 120-packet setting.

Hopping adds no slot beyond the existing phase-align feedback event.

## 8. Bare exclusive mode

The generic bare PHY path previously enabled and disabled RADIO for every packet:

```text
TX: 8812 packets/s
RX: 4406 packets/s
```

The dedicated exclusive path configures:

```text
LFLEN=0
STATLEN=6
8-bit preamble
one initial TXEN/RXEN
TXIDLE/RXIDLE + TASKS_START between packets
```

LM20 -> nRF52840 measured:

```text
TX: 16177-16178 packets/s
RX: 16180-16181 packets/s (independent timer measurement)
steady loss: approximately 0%
invalid: 0
```

With a five-byte address, six-byte state and CRC16, airtime is approximately
56 us. The theoretical airtime ceiling is about 17857 packets/s. The measured
61.8 us period includes state copy, END handling and TASKS_START.

### Bare hopping

Build both bare examples with:

```bash
cargo build --release --no-default-features --features hopping
```

The bare path uses 512 Data packets per hop. The transmitter inserts one bounded
feedback RX window, and the receiver detects that long inter-packet gap to learn
phase 0..511 before returning TimeDiff. Both then advance through the same
eight-channel sequence used by MPSL.

```text
Data packets per hop: 512
hop interval: about 32.2 ms
hop frequency: about 31.0 hops/s
feedback RX timeout: 700 us
```

LM20 -> nRF52840 measured:

```text
TX: 15892-15897 packets/s
RX: 15893-15901 packets/s
lost: typically 0-20 per ~79,500 packets
invalid: 0
hopping overhead versus no-hop bare: about 1.7-1.8%
```

If feedback is missed, the transmitter returns to channel 0. A locked receiver
that times out also returns to channel 0, clears its phase state, and reacquires.

## 9. Multiple state senders

The MPSL receiver can accept up to eight senders in one continuous RX window.
Sender identity uses Nordic RADIO logical addresses rather than adding an ID to
the six-byte state payload.

All configured addresses must share the same four-byte base. Their first byte is
the independent prefix selected by `RXMATCH`:

```rust
assert!(phy.configure_state_senders(&[
    Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]), // sender 0
    Address([0xC3, 0xE7, 0xE7, 0xE7, 0xE7]), // sender 1
]));
```

At each CRC-valid END event the callback reads `RXMATCH`, copies the six-byte
DMA state into that sender's latest-value cell, and increments that sender's
atomic cumulative packet count. Application context reads a coherent snapshot:

```rust
let mut state = [0u8; 6];
let cumulative_count = phy.sender_state(1, &mut state).unwrap();
```

The two-sender receiver example is built with:

```bash
cd examples/nrf52840/mpsl
cargo build --release --no-default-features --features multi-sender
```

The simultaneous benchmark senders are built with:

```bash
# sender 0: nRF54LM20, prefix E7, one transmission per two local slots
cd examples/nrf54lm20/mpsl
cargo build --release --no-default-features --features multi-sender-bench

# sender 1: nRF5340 network core, prefix C3, one transmission per three local slots
cd examples/nrf5340/mpsl
cargo build --release --no-default-features
```

The 2-TX/1-RX hardware setup is:

```text
sender 0: nRF54LM20 -> 1666 packets/s hardware TX
sender 1: nRF5340 net core -> 1111 packets/s hardware TX
receiver: nRF52840 multi-sender long RX
channel: 0
```

Measured steady receiver rates:

```text
sender 0: 1637-1672 packets/s
sender 1:  853-869 packets/s
aggregate: 2506-2532 packets/s
aggregate offered rate: 2777 packets/s
aggregate delivery: about 90.7%
```

Both latest states and counters advanced independently with the correct
`RXMATCH` sender index. The two senders currently use free-running local slot
grids with divisors two and three, so overlapping transmissions are expected;
the lower sender-1 delivery rate is collision loss, not address-demultiplexing
failure. Unique RADIO addresses identify packets but do not prevent overlap.
A future shared-epoch TDMA allocator can remove those collisions.

Current multi-sender RX uses one fixed channel and rejects combination with the
hopping receiver example.

### Reversed roles: LM20 receiver

The opposite hardware assignment is also available:

```bash
# receiver: nRF54LM20
cd examples/nrf54lm20/mpsl
cargo build --release --no-default-features --features multi-receiver

# sender 0: nRF52840, prefix E7, divisor two
cd examples/nrf52840/mpsl
cargo build --release --no-default-features --features multi-sender-bench

# sender 1: nRF5340 network core, prefix C3, divisor three
cd examples/nrf5340/mpsl
cargo build --release --no-default-features
```

Measured hardware TX rates:

```text
nRF52840 sender 0: 1383-1384 packets/s
nRF5340 sender 1: 1111 packets/s
aggregate offered: about 2495 packets/s
```

Measured LM20 receiver rates:

```text
sender 0: 1175-1192 packets/s (average about 1183/s)
sender 1:  831-846 packets/s  (average about 840/s)
aggregate: about 2023 packets/s
aggregate delivery: about 81.1%
```

Both sender states were independently decoded and advanced. This orientation is
less efficient than the 52840-receiver result: the free-running sender grids
still collide, and the 52840 transmitter's actual MPSL publication rate in this
configuration is 1384/s rather than the nominal 1666/s. These measurements are
for the current asynchronous divisor schedule, not synchronized TDMA.

## 10. Current limitations

- The benchmark payload embeds a state counter, but the protocol itself carries
  no sequence in `OneWayState`.
- TimeDiff is currently transported and used as the batch/hop synchronization
  boundary; a full long-term oscillator-frequency servo remains a future
  refinement.
- `OneWayChanges` has a core stop-and-wait engine but does not yet have the same
  optimized MPSL/bare hardware examples as `OneWayState`.
