# Connection State Machine

The link layer keeps a two-state connection machine that gates channel
hopping. The rule: **the adaptive hop is disabled until a connection forms**.
Two nodes that have never exchanged a packet stay pinned to the initial
channel, so their slot schedules can align without ever landing on different
channels.

## States

```
Disconnected ── first successful RX ──────────▶ Connected
Connected    ── LINK_LOSS_THRESHOLD misses ──▶ Disconnected
```

| state          | meaning                                             | hopping |
|----------------|-----------------------------------------------------|---------|
| `Disconnected` | no packet received yet, or the link was lost        | **off** — the scheduler is pinned to `Config::initial_channel` |
| `Connected`    | a packet was received                                | on — the adaptive hop advances after `HOP_MISS_THRESHOLD` misses |

## Why the gate

Before the connection forms, a miss is **not** an interference signal — it is
the normal condition of two free-running slot schedules that have not aligned
yet. If each node advanced its channel after a few misses (the pre-gate
behaviour), the two could hop to *different* channels from the same miss
streak and never meet again. Holding the initial channel removes that failure
mode: both nodes stay where the other can find them until the first packet
lands.

The same argument applies to a lost connection: after `LINK_LOSS_THRESHOLD`
consecutive misses the node declares the link lost, drops back to
`Disconnected`, and returns to the initial channel so the two can re-align
from a known place.

## Transitions

- **Form** (`Disconnected → Connected`): the first successful RX slot — any
  valid packet (a Data frame, or the Beacon on the peripheral) — sets
  `Connected` and resets the miss streak. It does not matter whether the
  packet's seq is accepted; the channel is proven usable.
- **Hop** (while `Connected`): `HOP_MISS_THRESHOLD` (4) consecutive missed RX
  slots advance the scheduler; a healthy slot resets the streak. This is the
  existing adaptive-hop behaviour, now only active once connected.
- **Loss** (`Connected → Disconnected`): `LINK_LOSS_THRESHOLD` (16)
  consecutive misses declare the link lost; the scheduler re-syncs to
  `initial_channel` and the streak resets.

Both roles run the same machine. The peripheral additionally re-syncs its
scheduler from every received Beacon's `channel_index` (the central is the
hop authority); that is independent of — and complementary to — the status
gate.

## API

- `Central::status()` / `Peripheral::status()` → `LinkStatus`
  (`Disconnected` | `Connected`), exported as `thunders::LinkStatus`.

## Where it lives

| piece | file |
|---|---|
| the state machine + the gate | `thunders/src/link.rs` (`LinkStatus`, `LinkState::on_miss`, `LinkState::on_rx`) |
| the thresholds | `HOP_MISS_THRESHOLD`, `LINK_LOSS_THRESHOLD` in `link.rs` |
| the scheduler (the channel sequence) | `thunders/src/scheduler.rs` |

## Limitation

The miss streak is counted in *RX slots*, and the two roles have different RX
densities under an asymmetric ratio (the peripheral RXes 8 slots per
superframe at `(8,1)`, the central 1). The peripheral therefore reaches a
threshold faster than the central. This is acceptable for the hop gate — both
nodes still pin to the initial channel while disconnected — but a
role-normalized streak is a possible refinement.
