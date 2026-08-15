# Bench Results — full matrix (2025-08-14)

All six directed pairs × both backends = 12 runs, 30 s per run, ratio (8,1),
8-byte PING payloads. Measured by the firmware itself (5 s windows, the
first window dropped as the connection-forming warmup) and parsed by
`scripts/bench_parse.py` from the RTT logs in `bench/logs/`.

| metric | how |
|---|---|
| **RTT** (latency) | central, PING TX slot → echo RX slot. The peripheral echoes the last PING of each 8-slot TX run, so RTT ≈ 1 slot period (500 µs mpsl / 125 µs bare) plus processing. |
| **bandwidth** | central, payload bytes/s both ways (8 B/PING + 8 B/echo). `c-rate` = the central's slot rate. |
| **forward loss** | peripheral, seq gaps (per-PING seqs, beacons excluded, gaps ≥ 1 M treated as peer restarts). |
| **reverse loss** | central, RX slots with no echo. |

## The matrix

| run | fwd loss | rev loss | rtt avg | min | max | bw B/s | c-rate/s |
|---|---|---|---|---|---|---|---|
| 52840 → 5340 | bare | 80% | 99.98% | 282 | 0 | 305 | 46 160 | 6 594 |
| 52840 → 5340 | mpsl | 99% | 99.41% | 584 | 0 | 1 037 | 13 201 | 1 884 |
| 52840 → LM20 | bare | 99.6% | 100% | 0 | 0 | 0 | 46 111 | 6 587 |
| 52840 → LM20 | mpsl | **14.4%** | **13.1%** | **633** | **518** | **1 586** | **15 737** | 2 000 |
| 5340 → 52840 | bare | 97.6% | 99.99% | 305 | 0 | 305 | 42 492 | 6 070 |
| 5340 → 52840 | mpsl | 98.9% | 99.66% | 674 | 518 | 1 159 | 13 336 | 1 904 |
| 5340 → LM20 | bare | 99.7% | 99.88% | 336 | 0 | 457 | 42 492 | 6 070 |
| 5340 → LM20 | mpsl | **12.5%** | **12.6%** | **644** | **549** | **1 586** | **15 745** | 2 000 |
| LM20 → 52840 | bare | 62.4% | 100% | 0 | 0 | 0 | 43 308 | 6 187 |
| LM20 → 52840 | mpsl | — | 100% | 0 | 0 | 0 | 23 290 | 3 327 |
| LM20 → 5340 | bare | 61.7% | 100% | 0 | 0 | 0 | 43 307 | 6 187 |
| LM20 → 5340 | mpsl | 0%* | 100% | 0 | 0 | 0 | 23 327 | 3 332 |

\* 0% forward loss with rx = 0 (no frames at all — not a healthy link).

## What works

**mpsl with the LM20 as the peripheral** — the only healthy links:

| pair | forward | reverse | rtt | bw |
|---|---|---|---|---|
| 52840 → LM20 mpsl | 86% | 87% | 633 µs | 15.7 kB/s |
| 5340 → LM20 mpsl | 87% | 87% | 644 µs | 15.7 kB/s |

The LM20 peripheral phase-locks (PLL dist = 500, catch ~100-120 µs, `mis=0`),
adopts the central's 500 µs cadence, and decodes ~84% of address-matched
frames. RTT ≈ 1.2 slots (500 µs slot + processing), bandwidth = 2 kHz slot
rate × 8 B payload, both ways.

## What's broken (all reproducible across the run's windows)

1. **The bare backend, all 6 pairs: 97-100% loss.** The two free-running slot
   loops run at different rates (the 52840 central ~6.5 kHz, the 5340
   peripheral ~2.5 kHz — the radio ops differ per chip), and the bare path
   has no cadence-locking mechanism. The beacon phase-mirror only corrects
   *phase*, not *cadence*, and it is itself only caught ~1% of the time
   while misaligned (a chicken-and-egg). The README's old bare numbers came
   from the pre-ratio architecture (which paced the central to the echo).

2. **mpsl with the 5340 (or 52840) as the peripheral: 98-100% loss.** The
   5340 net core shows the frames arriving (`addr` events = 5-7 k/window,
   ~64% of the central's PINGs) but decodes only ~10% of them (rx ≈ 535-650
   vs addr ≈ 5 700 — the rest fail CRC). The LM20 peripheral decodes ~84%,
   so this is specific to the 5340 net-core RX path (mpsl radio).
   The 52840-as-peripheral shows the same symptom.

3. **mpsl with the LM20 as the central: 100% loss.** The LM20's TX never
   reaches the peers (`addr = 0` at the 5340/52840 peripheral), and the LM20
   central's own RX gets nothing back. The one exception: the 5340
   peripheral eventually aligned (dist 502→300, `addr` events appear in the
   third window) but still decoded nothing.

4. **A peripheral hang**: in the lm20 → 52840 mpsl run the 52840 peripheral
   stopped printing after one window (busy 811 µs > slot time — the starved
   executor failure mode; the MPSL low-priority process never returns).

## Raw logs

`bench/logs/<central>-<peripheral>-<backend>.{central,peripheral}.log` —
every BENCH window plus the PLL (mpsl) / RADIO (bare) diagnostics.

## Reproduce

```sh
scripts/bench.sh build && scripts/bench.sh run 30 && python3 scripts/bench_parse.py
```

Requires probe access to the three boards (2× DAPLink + J-Link); see
`scripts/bench.sh` for the probe map.

## Fixes applied during this run (2025-08-14, second pass)

Two mpsl-phy fixes, both hardware-verified:

1. **The PLL phase-lock now corrects on any ADDRESS event, not only on a
   successful decode.** A misaligned poll truncates the frame and fails the
   CRC; gating the correction on `rx_ok` made that failure *prevent* the
   correction, so the phase error fed back and never converged (the 5340
   peripheral swept forever, decoding ~5-10 %). The address event is a fixed
   28 µs after the frame's on-air start, so it is a valid anchor regardless
   of the decode outcome. `addr_seen` flag added; the correction moved out of
   the `rx_ok` branch.
   - A/B verified: disabling it drops the working pairs from ~12 % loss to
     67-95 % — the tighter lock helps every pair, so it stays.

2. **The RX success check now requires the END event** (`got_end && CRC`).
   On the 5340 the CRCSTATUS reads 1 (stale) on nearly every poll, so the
   bare `crc & 1` check counted misses as successes and inflated the
   diagnostics; gating on END (a completed frame) makes the success
   detection correct. `crc_ok`/`crc_bad` counters added (now consistent with
   the address-event count).

The 5340's RX remains marginal in every role (26-62 % reverse catch even
when phase-locked; ~2-9 % when it is the mpsl peripheral). The diagnosis:
its app runs at ~680-1000 µs per slot (vs the 500 µs cadence) — it falls a
slot behind, so its RX polls happen at half rate. The `mpsl.run()` vs the
polling loop made no difference (the polling was worse); the net core's
per-slot overhead itself needs work (clock config, or the mpsl library's
net-core path). This is the next debugging target, separate from the bench
tooling.

### Third pass (the 5340's app latency)

3. **The MPSL inter-slot gap must be >= 150 us, not 100.** The slot-length
   caps in the callback used `nominal - 100`, which for a 500 us slot leaves
   a 100 us gap — below the MPSL scheduler's own minimum, so the chain
   degraded and the app fell a slot behind. On the 5340 peripheral the
   symptom was half-rate RX polls (rate 1468/s, busy 680-990 us). With
   `nominal - 150` the app keeps up (rate ~2000/s, busy ~450-500 us).
   - Verified: `mpsl_low_priority_process` costs only 19-53 us/call (not
     the culprit); the wake-driven vs polling mpsl_task made no difference.

4. **Integrating PLL (tried, reverted).** An integrator (`pll_acc`) was
   tried so the phase correction could compensate the 5340's cadence drift.
   Its dist swings (up to +40 us) destabilized the healthy LM20 pairs
   (rx 1000+/window -> single digits), so it is reverted to the one-shot
   step. The 5340's residual ~2-5% RX catch (polls aligned, frames mostly
   not reaching its address match) is an RF-level marginality of the 5340
   net-core mpsl RX — the README's original mpsl table showed the same
   (~98/s) before any of this work.
