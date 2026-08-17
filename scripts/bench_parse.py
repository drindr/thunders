#!/usr/bin/env python3
"""Parse the thunders bench RTT logs into a summary table.

Usage:
  scripts/bench_parse.py [LOGS_DIR] [--raw] [--rssi]

Reads bench/logs/*.{central,peripheral}.log (produced by scripts/bench.sh),
extracts the BENCH lines - the first window of each log is dropped (the
connection-forming warmup) when the log contains more than one window; a
single-window capture is used as-is so short runs still produce a row -
and prints one row per directed pair:

  run            | wins | fwd% | rev_raw% | rev_arq% | rtt avg | min | max | bw B/s | c-rate/s
  (central)-(peripheral)-(backend)

Metrics:
  fwd%      forward-link delivery loss = the peripheral's seq gaps / expected
  rev_raw%  reverse radio hit rate miss = the central's RX slots with no echo
  rev_arq%  reverse delivery loss after ARQ = the central's echo-seq gaps
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
# The ARQ-corrected reverse metric appended to BENCH C.
REV_LINE = re.compile(r"rev_lost=(\d+) rev_loss=(\d+)%")
P_LINE = re.compile(
    r"BENCH P slots=(\d+) rx=(\d+) lost=(\d+) floss=(\d+)% rate=(\d+)/s busy=(\d+)us"
)
# Both the MPSL PLL line and the bare RADIO line carry rssi=<raw sample>.
RSSI_LINE = re.compile(r"rssi=(\d+)")
# App-layer diagnostics appended by the bench firmware (absent in old logs):
# dup = echoes whose app seq did not advance (peripheral filler re-sends),
# ow  = echoes overwritten at the peripheral before ever being offered
#       (TX window stayed full) - the app-layer share of rev_lost.
DUP_LINE = re.compile(r"dup=(\d+)")
OW_LINE = re.compile(r" ow=(\d+)")
# Filler payloads received by the central (reverse saturation traffic with
# its own seq space; counts as a radio hit but is not an echo).
FILL_LINE = re.compile(r"fill=(\d+)")

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


def parse_c(fn):
    """Return BENCH C tuples plus the optional rev_lost/rev_loss suffix."""
    out = []
    try:
        with open(fn, errors="replace") as f:
            for line in f:
                m = C_LINE.search(line)
                if m:
                    base = tuple(int(x) for x in m.groups())
                    r = REV_LINE.search(line)
                    rev = tuple(int(x) for x in r.groups()) if r else None
                    d = DUP_LINE.search(line)
                    dup = int(d.group(1)) if d else None
                    fl = FILL_LINE.search(line)
                    fill = int(fl.group(1)) if fl else None
                    out.append((base, rev, dup, fill))
    except FileNotFoundError:
        pass
    return out


def parse_p(fn):
    """Return (BENCH P tuple, ow) pairs; ow is None on old firmware logs."""
    out = []
    try:
        with open(fn, errors="replace") as f:
            for line in f:
                m = P_LINE.search(line)
                if m:
                    base = tuple(int(x) for x in m.groups())
                    o = OW_LINE.search(line)
                    ow = int(o.group(1)) if o else None
                    out.append((base, ow))
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
    """Aggregate BENCH C windows -> (wins, rev_raw%, rev_arq%, rtt_avg,
    rtt_min, rtt_max, bw, rate)."""
    if not windows:
        return None
    n = len(windows)
    base = [w for w, _, _, _ in windows]
    rev = [r for _, r, _, _ in windows]
    dup_vals = [d for _, _, d, _ in windows]
    dup = sum(d for d in dup_vals if d is not None) if any(d is not None for d in dup_vals) else None
    fill_vals = [f for _, _, _, f in windows]
    fill = sum(f for f in fill_vals if f is not None) if any(f is not None for f in fill_vals) else 0
    slots = sum(w[0] for w in base)
    tx = sum(w[1] for w in base)
    rx = sum(w[2] for w in base)
    # The firmware computes rloss against the true RX-slot count (its
    # tx_frames), which the log's tx= field no longer equals once offer
    # throttling (RATE_DIV) is in play - use the firmware's per-window
    # rloss, slots-weighted.
    rev_raw = sum(w[3] * w[0] for w in base) / slots if slots else 0.0
    rev_lost = sum(r[0] for r in rev if r is not None)
    if any(r is not None for r in rev):
        rev_arq = 100.0 * rev_lost / (rx + rev_lost) if (rx + rev_lost) else 0.0
    else:
        rev_arq = float("nan")
    rtt_avg = sum(w[6] * w[2] for w in base) / rx if rx else 0.0
    rtt_min = min(w[7] for w in base) if rx else 0
    rtt_max = max(w[8] for w in base) if rx else 0
    elapsed = n * WINDOW_US
    bw = (tx + rx + fill) * 8 * 1_000_000 / elapsed if elapsed else 0.0
    rate = slots * 1_000_000 / elapsed if elapsed else 0.0
    return (n, rev_raw, rev_arq, rtt_avg, rtt_min, rtt_max, bw, rate, dup, rev_lost)


def summarize_p(windows):
    """Aggregate the per-window peripheral lines -> (wins, floss%, rate, ow)."""
    if not windows:
        return None
    n = len(windows)
    rx = sum(w[1] for w, _ in windows)
    lost = sum(w[2] for w, _ in windows)
    slots = sum(w[0] for w, _ in windows)
    ow_vals = [o for _, o in windows]
    ow = sum(o for o in ow_vals if o is not None) if any(o is not None for o in ow_vals) else None
    floss = 100.0 * lost / (lost + rx) if (lost + rx) else 0.0
    elapsed = n * WINDOW_US
    rate = slots * 1_000_000 / elapsed if elapsed else 0.0
    return (n, floss, rate, ow)


def main():
    args = [a for a in sys.argv[1:] if a not in ("--raw", "--rssi")]
    show_rssi = "--rssi" in sys.argv[1:]
    logs = args[0] if args else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "bench", "logs"
    )
    # Only the canonical directed-pair matrix belongs in the summary. Old
    # experiment logs (e.g. *-sr-*) stay on disk but must not pollute the
    # table.
    canonical = {
        f"{c}-{p}-{backend}{mode_suffix}{ratio_suffix}"
        for c in ("52840", "5340", "lm20")
        for p in ("52840", "5340", "lm20")
        for backend in ("bare", "mpsl")
        for mode_suffix in ("", "-1m")
        for ratio_suffix in ("", "-r844", "-r622", "-r422")
        if c != p
    }
    cfiles = [
        fn
        for fn in sorted(glob.glob(os.path.join(logs, "*.central.log")))
        if os.path.basename(fn)[: -len(".central.log")] in canonical
    ]
    if not cfiles:
        print(f"no canonical *.central.log in {logs} - run scripts/bench.sh run first")
        return 1

    header = f"{'run':<22} {'wins':>4} {'fwd%':>6} {'rev_raw%':>8} {'rev_arq%':>8} {'rtt_avg':>8} {'min':>6} {'max':>6} {'bw B/s':>9} {'c-rate/s':>9} {'dup':>6} {'ow':>6} {'ow/rev%':>8}"
    print(header)
    print("-" * len(header))
    rssi_rows = []
    for cf in cfiles:
        base = os.path.basename(cf)[: -len(".central.log")]
        pf = os.path.join(logs, base + ".peripheral.log")
        c_windows = parse_c(cf)
        if len(c_windows) > 1:
            c_windows = c_windows[1:]  # drop the warmup window
        p_windows = parse_p(pf)
        if len(p_windows) > 1:
            p_windows = p_windows[1:]  # drop the warmup window
        c = summarize_c(c_windows)
        p = summarize_p(p_windows)
        if not c:
            print(f"{base:<22} {'no central data':>4}")
            continue
        wins = c[0]
        floss = p[1] if p else float("nan")
        dup = c[8]
        ow = p[3] if p else None
        # The app-layer share of the ARQ-corrected reverse loss: echoes the
        # peripheral dropped before they ever reached the link layer, over
        # all reverse gaps the central observed.
        rev_lost_sum = c[9]
        if ow is None:
            ow_rev_s = "-"
        elif rev_lost_sum > 0:
            ow_rev_s = f"{min(100.0, 100.0 * ow / rev_lost_sum):>6.0f}%"
        else:
            ow_rev_s = f"{0.0:>6.0f}%"
        print(
            f"{base:<22} {wins:>4} {floss:>6.2f} {c[1]:>8.2f} {c[2]:>8.2f} "
            f"{c[3]:>8.1f} {c[4]:>6} {c[5]:>6} {c[6]:>9.0f} {c[7]:>9.0f} "
            f"{dup if dup is not None else '-':>6} {ow if ow is not None else '-':>6} {ow_rev_s:>7}"
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
