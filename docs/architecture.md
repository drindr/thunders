# Thunders fixed-mode architecture

## Scope

The former symmetric `Central`/`Peripheral`, runtime cadence search, dynamic
payload contract, and bidirectional selective-repeat scheduler were removed.
The new core is intentionally compile-time and mode-driven.

Initial mode family is `OneWay<PAYLOAD, ACK, FEEDBACK_EVERY>`:

- `OneWayAck<PAYLOAD>` aliases `OneWay<PAYLOAD, true, 1>`: fixed forward
  payload, one reverse ACK per packet, one in-flight packet, retransmission on
  timeout.
- `OneWayNoAck<PAYLOAD, DIFF_EVERY>` aliases
  `OneWay<PAYLOAD, false, DIFF_EVERY>`: fixed forward payload, no retained
  packet and no retransmit API, one reverse `TimeDiff` after a compile-time
  packet batch.

Future duplex, multicast, burst, or low-power modes implement the same
`LinkMode` trait instead of adding runtime flags to one state machine.

### Semantic choice: state versus state change

No-ACK mode does **not** guarantee delivery. It is intended for continuously
refreshed state where a newer packet supersedes every older packet, for example
position, orientation, sensor samples, actuator target, or current UI state.
Losing an intermediate packet is acceptable because the next snapshot repairs
the receiver's view. `OneWayState<PAYLOAD, DIFF_EVERY>` is the semantic alias
for this mode.

ACK mode is intended for state changes and events that must not disappear, for
example start/stop, arming, configuration changes, counters, commands, and
transaction boundaries. The sender retains the change and retransmits until a
matching ACK. `OneWayChanges<PAYLOAD>` is the semantic alias for this mode.

Neither mode should be selected only for throughput: selection follows the
meaning of the data.

## Compile-time timing

`fixed_slot_plan::<Mode>(AirTiming, SlotOverhead)` derives:

- exact Data wire bytes: marker + sequence + fixed payload;
- exact feedback bytes: marker + sequence + signed time difference;
- on-air duration including length and CRC;
- TX/RX ramp and setup budget;
- shutdown tail, jitter margin, and mandatory MPSL gap;
- forward slot period, reverse feedback period, long receiver window, and
  complete cycle duration.

No payload length is negotiated or transmitted at runtime.

For a 2 Mbit Nordic frame and conservative measured MPSL overhead:

```text
8-byte application payload
wire = 11 bytes
airtime = 84 us
slot = airtime + 265 us = 349 us
```

A production adapter may round this mathematical floor upward, but it must not
recompute packet timing dynamically.

## Asymmetric physical schedule

One-way transmission does not require equal transmitter and receiver slot
chains.

The transmitter uses `N` short Data grants and one reverse feedback RX grant.
The receiver uses:

1. one long RX grant covering the whole `N`-packet forward batch;
2. one short reverse TX grant carrying ACK or TimeDiff.

This keeps the receiver listening continuously across forward packet starts and
removes the old requirement that two free-running symmetric slot grids first
match by chance.

## ACK mode

The first reliable implementation deliberately permits one in-flight packet:

```text
Data(seq, fixed payload)
ACK(seq, diff_us)
```

The sender retains the exact fixed payload until the matching ACK. Timeouts
rebuild the same fixed frame. Later reliable modes may add a compile-time window
size without changing `OneWayAck` semantics.

## No-ACK mode

The sender never retains or retransmits Data. The receiver returns:

```text
TimeDiff(last_seq, diff_us)
```

every `DIFF_EVERY` packets. `diff_us` is the measured difference between the
predicted and captured packet ADDRESS time. The transmitter/follower timing
adapter uses it only for clock/phase correction; it is not delivery feedback.

## Connection events

Control packets use a small postcard codec:

```text
ConnectOffer(start_after_us, window_us)
FirstEvent(challenge)
FirstResponse(challenge)
Connected(start_epoch)
```

The future first event is relative to the hardware ADDRESS timestamp of the
received offer, following the BLE model. Free-running slot-counter subtraction
is not part of the new protocol.

## Hardware smoke test

A first architecture-only smoke test used nRF54LM20 as an unpaced
`OneWayState<8, 32>` transmitter and nRF52840 as receiver at 2 Mbit. The fixed
11-byte wire frame decoded without invalid packets and sustained tens of
thousands of received state snapshots per 5-second window.

Observed loss was approximately 49–50%. This is expected from the temporary
adapter: `Phy::receive()` stops after one packet and restarts RADIO for every
call, while the LM20 test transmitter sends continuously. Therefore the
calculated `receiver_window_us=11168` is currently only a timeout, not yet one
continuous multi-packet RX grant. The next PHY adapter must keep RADIO in RX
and collect multiple fixed frames inside that one long receiver window.

This smoke result validates the fixed codec and one-way state semantics, not
the final slot plan or TimeDiff feedback loop.

## Migration status

The compile-time mode specification, fixed timing planner, exact codecs,
one-way sender/receiver engines, connection-event wire format, and RX timing
metadata are implemented. Hardware scheduling adapters for long receiver
windows and first-event timing are the next layer; old symmetric examples and
bench tooling were intentionally deleted rather than kept as a second protocol.
