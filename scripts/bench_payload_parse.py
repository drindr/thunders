#!/usr/bin/env python3
"""Summarize fast-cadence payload sweep logs as CSV or Markdown."""

import argparse
import glob
import os
import re

C_RE = re.compile(
    r"BENCH C slots=(\d+) tx=(\d+) rx=(\d+) rloss=(\d+)% rate=(\d+)/s "
    r"bw=(\d+)B/s rtt_avg=(\d+)us rtt_min=(\d+)us rtt_max=(\d+)us busy=(\d+)us"
)
P_RE = re.compile(r"BENCH P slots=(\d+) rx=(\d+) lost=(\d+) floss=(\d+)% rate=(\d+)/s")
REV_RE = re.compile(r"rev_lost=(\d+) rev_loss=(\d+)%")
DF_RE = re.compile(r"\bdf=(\d+)")
RETX_RE = re.compile(r"\b(?:retx|rt)=(\d+)")
NAME_RE = re.compile(r"(.+)-p(1|4|8|16|32)\.central\.log$")
WINDOW_US = 5_000_000


def lines(path, regex):
    out = []
    with open(path, errors="replace") as stream:
        for line in stream:
            match = regex.search(line)
            if match:
                out.append((tuple(int(x) for x in match.groups()), line))
    return out[1:] if len(out) > 1 else out


def summarize(cpath, ppath):
    with open(cpath, errors="replace") as stream:
        if "CADENCE STABLE" not in stream.read():
            return None
    cw = lines(cpath, C_RE)
    pw = lines(ppath, P_RE)
    if not cw or not pw:
        return None
    n = len(cw)
    cvals = [x for x, _ in cw]
    pvals = [x for x, _ in pw]
    slots = sum(x[0] for x in cvals)
    tx = sum(x[1] for x in cvals)
    rx = sum(x[2] for x in cvals)
    p_rx = sum(x[1] for x in pvals)
    p_lost = sum(x[2] for x in pvals)
    c_rate = slots * 1_000_000 / (n * WINDOW_US)
    p_rate = sum(x[0] for x in pvals) * 1_000_000 / (len(pvals) * WINDOW_US)
    bw = sum(x[5] for x in cvals) / n
    raw_loss = sum(x[3] * x[0] for x in cvals) / slots if slots else 0.0
    rtt = sum(x[6] * x[2] for x in cvals) / rx if rx else 0.0
    rtt_min = min(x[7] for x in cvals) if rx else 0
    rtt_max = max(x[8] for x in cvals) if rx else 0
    fwd_loss = 100.0 * p_lost / (p_rx + p_lost) if p_rx + p_lost else 0.0
    rev_lost = 0
    for _, line in cw:
        match = REV_RE.search(line)
        if match:
            rev_lost += int(match.group(1))
    rev_loss = 100.0 * rev_lost / (rx + rev_lost) if rx + rev_lost else 0.0
    df = max((int(m.group(1)) for _, line in cw if (m := DF_RE.search(line))), default=0)
    df += max((int(m.group(1)) for _, line in pw if (m := DF_RE.search(line))), default=0)
    retx = max((int(m.group(1)) for _, line in cw if (m := RETX_RE.search(line))), default=0)
    retx += max((int(m.group(1)) for _, line in pw if (m := RETX_RE.search(line))), default=0)
    return n, c_rate, p_rate, bw, raw_loss, fwd_loss, rev_loss, rtt, rtt_min, rtt_max, df, retx, tx, rx


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="?", default="bench/logs")
    parser.add_argument("--csv", action="store_true")
    args = parser.parse_args()
    rows = []
    for cpath in sorted(glob.glob(os.path.join(args.logs, "*-p*.central.log"))):
        match = NAME_RE.search(os.path.basename(cpath))
        if not match:
            continue
        pair, payload = match.groups()
        ppath = cpath.replace(".central.log", ".peripheral.log")
        result = summarize(cpath, ppath)
        if result:
            rows.append((int(payload), pair, *result))
    rows.sort()
    if args.csv:
        print("payload,pair,wins,c_rate,p_rate,bw_Bps,raw_loss_pct,fwd_loss_pct,rev_loss_pct,rtt_avg_us,rtt_min_us,rtt_max_us,delivery_failures,retransmits,tx,rx")
        for row in rows:
            print(",".join(str(round(x, 2)) if isinstance(x, float) else str(x) for x in row))
    else:
        print("| B | pair | wins | C slot/s | P slot/s | payload B/s | raw loss | fwd loss | rev loss | RTT avg/min/max us | df | retx |")
        print("|---:|:---|---:|---:|---:|---:|---:|---:|---:|:---|---:|---:|")
        for p, pair, wins, cr, pr, bw, raw, fwd, rev, rtt, rmin, rmax, df, retx, _, _ in rows:
            fwd_text = f"{fwd:.2f}%" if p >= 8 else "n/a"
            rev_text = f"{rev:.2f}%" if p >= 8 else "n/a"
            print(f"| {p} | {pair} | {wins} | {cr:.0f} | {pr:.0f} | {bw:.0f} | {raw:.1f}% | {fwd_text} | {rev_text} | {rtt:.0f}/{rmin}/{rmax} | {df} | {retx} |")
    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
