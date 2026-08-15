#!/usr/bin/env python3
"""Parse the thunders bench RTT logs into a summary table.

Usage:
  scripts/bench_parse.py [LOGS_DIR] [--raw] [--rssi]

Reads bench/logs/*.{central,peripheral}.log (produced by scripts/bench.sh),
extracts the BENCH lines - the first window of each log is dropped (the
connection-forming warmup) - and prints one row per directed pair:

  run            | wins | fwd_loss% | rev_loss% | rtt avg | min | max | bw B/s | c-rate/s
  (central)-(peripheral)-(backend)

Metrics:
  fwd_loss%  forward-link loss  = the peripheral's seq gaps / expected
  rev_loss%  reverse-link loss  = the central's RX slots with no echo
  rtt        app-level round trip: PING TX slot -> echo RX (~1 slot period)
  bw         payload throughput: 8 B per PING + 8 B per echo, both ways
  c-rate     the central's slot rate
"""

import glob
import os
import re
import sys

C_LINE = re.compile(
    r"BENCH C slots=(\d+) tx=(\d+) rx=(\d+) rloss=(\d+)% rate=(\d+)/s bw=(\d+)B/s "
    r"rtt_avg=(\d+)us rtt_min=(\d+)us rtt_max=(\d+)us busy=(\d+)us"
)
P_LINE = re.compile(
    r"BENCH P slots=(\d+) rx=(\d+) lost=(\d+) floss=(\d+)% rate=(\d+)/s busy=(\d+)us"
)
# Both the MPSL PLL line and the bare RADIO line carry rssi=<raw sample>.
RSSI_LINE = re.compile(r"rssi=(\d+)")

WINDOW_US = 5_000_000  # the firmware's report window


def parse(fn, rx):
    out = []
    try:
        with open(fn, errors="replace") as f:
            for line in f:
                m = rx.search(line)
                if m:
                    out.append(tuple(int(x) for x in m.groups()))
    except FileNotFoundError:
        pass
    return out


def parse_rssi(fn):
    """Return the raw RSSISAMPLE values found in `fn` (both PLL and RADIO lines)."""
    out = []
    try:
        with open(fn, errors="replace") as f:
            for line in f:
                m = RSSI_LINE.search(line)
                if m:
                    out.append(int(m.group(1)))
    except FileNotFoundError:
        pass
    return out


def summarize_c(windows):
    """Aggregate the per-window central lines -> (wins, rloss%, rtt_avg,
    rtt_min, rtt_max, bw, rate)."""
    if not windows:
        return None
    n = len(windows)
    slots = sum(w[0] for w in windows)
    tx = sum(w[1] for w in windows)
    rx = sum(w[2] for w in windows)
    rx_slots = slots - tx
    rev_lost = sum(max(0, (w[0] - w[1]) - w[2]) for w in windows)
    rloss = 100.0 * rev_lost / rx_slots if rx_slots else 0.0
    rtt_avg = sum(w[6] * w[2] for w in windows) / rx if rx else 0.0
    rtt_min = min(w[7] for w in windows) if rx else 0
    rtt_max = max(w[8] for w in windows) if rx else 0
    elapsed = n * WINDOW_US
    bw = (tx + rx) * 8 * 1_000_000 / elapsed if elapsed else 0.0
    rate = slots * 1_000_000 / elapsed if elapsed else 0.0
    return (n, rloss, rtt_avg, rtt_min, rtt_max, bw, rate)


def summarize_p(windows):
    """Aggregate the per-window peripheral lines -> (wins, floss%, rate)."""
    if not windows:
        return None
    n = len(windows)
    rx = sum(w[1] for w in windows)
    lost = sum(w[2] for w in windows)
    slots = sum(w[0] for w in windows)
    floss = 100.0 * lost / (lost + rx) if (lost + rx) else 0.0
    elapsed = n * WINDOW_US
    rate = slots * 1_000_000 / elapsed if elapsed else 0.0
    return (n, floss, rate)


def main():
    args = [a for a in sys.argv[1:] if a not in ("--raw", "--rssi")]
    show_rssi = "--rssi" in sys.argv[1:]
    logs = args[0] if args else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "bench", "logs"
    )
    cfiles = sorted(glob.glob(os.path.join(logs, "*.central.log")))
    if not cfiles:
        print(f"no *.central.log in {logs} - run scripts/bench.sh run first")
        return 1

    header = f"{'run':<22} {'wins':>4} {'fwd%':>6} {'rev%':>6} {'rtt_avg':>8} {'min':>6} {'max':>6} {'bw B/s':>9} {'c-rate/s':>9}"
    print(header)
    print("-" * len(header))
    rssi_rows = []
    for cf in cfiles:
        base = os.path.basename(cf)[: -len(".central.log")]
        pf = os.path.join(logs, base + ".peripheral.log")
        c = summarize_c(parse(cf, C_LINE)[1:])  # drop the warmup window
        p = summarize_p(parse(pf, P_LINE)[1:])
        if not c:
            print(f"{base:<22} {'no central data':>4}")
            continue
        wins = c[0]
        floss = p[1] if p else float("nan")
        print(
            f"{base:<22} {wins:>4} {floss:>6.2f} {c[1]:>6.2f} "
            f"{c[2]:>8.1f} {c[3]:>6} {c[4]:>6} {c[5]:>9.0f} {c[6]:>9.0f}"
        )
        rssi = parse_rssi(cf) + parse_rssi(pf)
        if rssi:
            rssi_rows.append((base, rssi))
    if show_rssi:
        print()
        if not rssi_rows:
            print("no rssi= samples found")
        else:
            rssi_header = f"{'run':<22} {'rssi_avg':>9} {'rssi_min':>9} {'rssi_max':>9} {'samples':>8}"
            print(rssi_header)
            print("-" * len(rssi_header))
            for base, vals in rssi_rows:
                avg = sum(vals) / len(vals)
                print(
                    f"{base:<22} {avg:>9.1f} {min(vals):>9} {max(vals):>9} {len(vals):>8}"
                )
    return 0


if __name__ == "__main__":
    sys.exit(main())
