# Addressed recall and the negotiation phase

The one-way link streams fixed state packets; the only reverse traffic is one
three-byte feedback beacon per batch. This document describes how the receiver
uses that beacon to recall one sender into a reliable negotiation phase for
link configuration, without changing the slot grid.

## 1. Design rules

- The slot grid, batch length, receiver window, and feedback event position
  never change. Recall only changes packet *content*.
- All phase transitions happen at the batch boundary that already
  synchronizes hopping. There is no runtime timing renegotiation.
- The recall request is level-triggered, not edge-triggered: the receiver
  asserts it in every beacon until the sender's echo confirms the
  transition. A lost beacon only costs one batch of latency.
- The negotiation phase is exclusive: while recalled, every forward frame
  is a config frame, so streaming payloads need no discriminant byte.
- Any unrecoverable state falls back to the compile-time rendezvous:
  channel 0, default power, streaming. This reuses the hopping recovery
  path.

## 2. Feedback beacon format (3 bytes)

```text
byte 0: flags
  bit0 NEG_REQ    receiver requests negotiation with one sender
  bit1..7         reserved (version parity candidate)
byte 1..2: semantics follow the flags byte
  NEG_REQ clear:  i16 diff_us, little-endian (phase error, as before)
  NEG_REQ set:    byte 1 = target sender's on-air address prefix
                  byte 2 = 0
```

The widening from two to three bytes costs 4 us of airtime per batch
(44 -> 48 us). The compile-time feedback event was already floored at
data+10 us (310 us), so the schedule is unchanged.

The recall target is the sender's *on-air* address prefix (the bit-reversed
form stored in `prefix0`), which is also the sender identity used by
multi-sender RXMATCH demultiplexing. `MpslRadioPhy::recall_sender(prefix)`
takes the application-level prefix byte and reverses it internally.

## 3. Recall handshake

```text
batch N:   sender ── batch of data slots ──► receiver
           sender ◄── beacon: NEG_REQ + target prefix ── receiver
batch N+1: sender stops app data; every slot carries ConfigFrame echo
           receiver decodes echo -> recall confirmed
release:   receiver clears NEG_REQ; sender resumes streaming at the
           next boundary (in-flight reliable payload stays parked)
```

Sender-side rules (`timeslot_do_work` feedback RX):

```text
beacon NEG_REQ with own prefix  -> negotiation = true
beacon NEG_REQ with other prefix -> keep streaming (addressed recall)
beacon without NEG_REQ          -> negotiation = false
```

Receiver-side confirmation: the echo is a six-byte config frame; its
`value` reports the sender's own on-air prefix and current channel, so the
receiver verifies *which* sender answered:

```text
byte 0: 0xCF magic
byte 1: cfg_seq      (per-echo sequence)
byte 2: op           (0 = ECHO_STATUS)
byte 3: param        (0)
byte 4: own on-air prefix
byte 5: current channel
```

## 4. Hopping interaction

Recall pins both peers to the rendezvous channel at the same boundary:

```text
receiver: the window that sends NEG_REQ does not advance hop_index;
          cur_channel = HOP_SEQUENCE[0] while the recall is active
sender:   on accepting a recall, hop_index = 0, cur_channel = 0
release:  both advance from index 0 at the same boundary
```

Rationale: a recall is likely triggered by poor link quality, and channel 0
is the only configuration both ends are guaranteed to agree on.

## 5. Quality guarantees inside the negotiation phase

- The same echo frame repeats in every slot of each batch; CRC16 plus the
  0xCF magic make false positives negligible; `cfg_seq` allows idempotent
  dedup when config commands are added.
- Recall latency is bounded by one batch (~38 ms at the 128-packet MPSL
  plan, ~32 ms for the 512-packet bare plan) plus one batch per lost
  beacon.
- A recalled sender never leaves the phase on its own; only a beacon
  without NEG_REQ releases it, so transient receiver stalls cannot drop
  the sender back to streaming mid-negotiation.

## 6. Current implementation status

Implemented and compile-verified (host tests + all MPSL/bare feature
combos):

- three-byte beacon with NEG_REQ addressing (`thunders::negotiation`)
- sender-side addressed recall accept/release in the MPSL fast path
- exclusive negotiation phase with ConfigFrame status echo on every slot
- rendezvous-channel pinning under hopping
- nrf52840 receiver demo: foreign-prefix recall (ignored), addressed
  recall, echo confirmation, timed release
- `ACK`/`RETRANSMIT` removed from the `LinkMode` type level; the
  `OneWaySender` engine's stop-and-wait reliability is now a runtime
  property (`set_reliable`) shared with the negotiation phase

Measured on hardware (nRF54LM20 sender -> nRF52840 receiver, phase-align
build, channel 0):

```text
baseline streaming:        3306 packets/s, feedback locked
foreign-prefix recall:     sender kept streaming (neg=false), addressing OK
addressed recall -> echo:  confirmed after 232 ms (~6 batch boundaries,
                           includes app-loop detection latency)
during negotiation:        full-rate echoes, 0 CRC-bad
release:                   streaming resumed at 3306/s, state counter
                           continuous across the phase round trip
```

Not yet implemented:

- the config command loop (power up/down, frequency step, apply-at-boundary)
  — the feedback command bits and commit semantics are designed but the
  beacon currently carries only NEG_REQ
- the bare PHY path (its per-hop feedback window needs the same beacon
  parsing)
- recall on the multi-sender benchmark: its senders run free-running
  divisor schedules with no feedback listener; they would need a periodic
  feedback RX op first
- diff_us transport in the MPSL fast path (the beacon's timing bytes are
  currently zero; the aligner exists in the core engine only)
