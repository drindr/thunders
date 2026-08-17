# Connection State Machine

The link layer keeps a two-state connection machine that gates channel
hopping. The rule: **the adaptive hop is disabled until a connection forms**.
Nodes that have not yet sustained the form-up streak stay pinned to the
initial channel, so their slot schedules can align without ever landing on
different channels.

## States

```
Disconnected ── CONNECT_STREAK_THRESHOLD catches ──▶ Connected
Connected    ── LINK_LOSS_THRESHOLD misses ────────▶ Disconnected
```

| state          | meaning                                             | hopping |
|----------------|-----------------------------------------------------|---------|
| `Disconnected` | no packet received yet, or the link was lost        | **off** — the scheduler is pinned to `Config::initial_channel` |
| `Connected`    | the form-up streak was reached and the link is alive | on — **only the central** advances after `HOP_MISS_THRESHOLD` misses |

## Why the gate

Before the connection forms, a miss is **not** an interference signal — it is
the normal condition of two slot schedules that have not aligned yet. Holding
the initial channel keeps both nodes where the other can find them until the
form-up streak is reached.

The same argument applies to a lost connection: after `LINK_LOSS_THRESHOLD`
consecutive misses the node declares the link lost, drops back to
`Disconnected`, and returns to the initial channel so the two can re-align
from a known place.

## Transitions

- **Form** (`Disconnected → Connected`): `CONNECT_STREAK_THRESHOLD` (8)
  consecutive successful RX slots set `Connected`. One lucky catch on a
  marginal link is not enough to enable hopping; the streak proves the
  channel can sustain traffic.
- **Hop** (while `Connected`, central only): `HOP_MISS_THRESHOLD` (16)
  consecutive missed RX slots advance the central's scheduler; a healthy
  slot resets the streak. The peripheral never advances the hop locally —
  it follows the beacon's `channel_index`.
- **Loss** (`Connected → Disconnected`): `LINK_LOSS_THRESHOLD` (16)
  consecutive misses declare the link lost; the scheduler re-syncs to
  `initial_channel` and the streak resets.

Both roles run the same form-up and loss transitions. The peripheral
additionally re-syncs its scheduler from every received Beacon's
`channel_index` (the central is the hop authority); that is independent of —
and complementary to — the status gate.

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
densities under an asymmetric ratio (at `(8,2)` the peripheral has 8 RX slots
per 10-slot period and the central has 2). The peripheral therefore reaches
`LINK_LOSS_THRESHOLD` faster than the central after the link is actually
gone. This is acceptable for the gate — both nodes pin back to the initial
channel while disconnected — but a role-normalized streak is a possible
refinement. (Hopping itself is central-only, so the asymmetry does not cause
a peripheral hop.)
