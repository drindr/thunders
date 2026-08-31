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

- state Data wire bytes: exactly the fixed payload, with no marker/sequence;
- state feedback bytes: one signed `i16 diff_us`;
- reliable state-change Data: fixed payload plus two-byte sequence;
- reliable feedback: two-byte sequence plus signed `i16 diff_us`;
- on-air duration including length and CRC;
- TX/RX ramp and setup budget;
- shutdown tail, jitter margin, and mandatory MPSL gap;
- forward slot period, reverse feedback period, long receiver window, and
  complete cycle duration.

No payload length is negotiated or transmitted at runtime.

For a 2 Mbit Nordic frame and conservative measured MPSL overhead:

```text
6-byte state payload
wire = 6 bytes
airtime = 64 us
RX restart allowance = 150 us
mathematical MPSL period = 479 us
25 us quantization = 500 us
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

This smoke result validates the fixed codec and one-way state semantics. The
retained smoke programs now use `OneWayState<6, 32>`. State mode has no marker
or protocol sequence, so six application bytes produce a six-byte wire frame.

### MPSL implementation

The MPSL adapter derives its schedule entirely at compile time. State packets
contain only the six payload bytes. Runtime cadence fallback and probe logic are
not part of the schedule:

```text
2M six-byte state airtime: 64us
setup/ramp + tail + margin + MPSL gap: 265us
mathematical Data period: 329us
steady-state guard: 10us
10us scheduling quantization: 340us
feedback period: 340us
cycle: 128 × 340us + 340us = 43860us
```

The LM20 transmitter uses 128 Data events plus one feedback RX event. The 52840
receiver uses one long event and returns a two-byte signed `diff_us`; state mode
uses neither marker nor protocol sequence. The longer observation cycle
amortizes MPSL boundary and callback/application handoff costs and gives cold
acquisition substantially more phase coverage.

LM20→52840 validation after acquisition measured:

```text
receiver long slots: 22–23/s
hardware transmitter packets: 2917–2918/s
receiver delivered packets: 2897–2898/s
receiver CRC-valid packets: 2897–2898/s
invalid frames: 0
steady receive ratio: about 99.3%
```

A 330us zero-guard trial reached 3006 TX packets/s but only 1477–1504 RX
packets/s and lost TimeDiff feedback. It is therefore rejected; 340us is the
current shortest validated period.

All durations and packet-relative reply offsets come from
`one_way_mpsl_plan::<Mode>()` and compile-time PHY timing constants. The old
symmetric link, cadence probes, PLL servo, runtime alignment, and their debug
state have been removed.
