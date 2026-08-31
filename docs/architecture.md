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
type Mode = OneWayState<PAYLOAD, 128>;
```

Meaning:

```text
application state bytes: 6
Data events per batch: 128
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
2. Receiver detects the feedback gap and learns Data phase 0..127.
3. Receiver sends TimeDiff after phase 127.
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

Measured hopping results after lock:

```text
TX: 3305-3306 packets/s
RX: 3280-3295 packets/s (30-second validation; average about 3286/s)
steady receive ratio: about 99.3-99.5%
invalid: normally 0
```

A transmitter restart was tested while the receiver was locked. The receiver
returned to channel 0, reacquired the restarted transmitter, re-established the
hop epoch, and resumed approximately 3290 packets/s without resetting the
receiver.

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

## 9. Current limitations

- The benchmark payload embeds a state counter, but the protocol itself carries
  no sequence in `OneWayState`.
- TimeDiff is currently transported and used as the batch/hop synchronization
  boundary; a full long-term oscillator-frequency servo remains a future
  refinement.
- `OneWayChanges` has a core stop-and-wait engine but does not yet have the same
  optimized MPSL/bare hardware examples as `OneWayState`.
