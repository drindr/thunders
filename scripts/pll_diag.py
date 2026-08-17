#!/usr/bin/env python3
"""Diagnose thunders link loss from the PLL/RADIO lines in bench logs.

Usage:
  scripts/pll_diag.py bench/logs/52840-lm20-mpsl.central.log [more.log...]

What it does:

- MPSL `PLL ...` lines (one per 5 s window per node): the counters
  (addr/crcok/crcbad/crcbadl/mis) are CUMULATIVE, so this script diffs
  consecutive lines into per-window deltas and classifies each window:
    empty    no ADDRESS event, no long CRC-bad frame: nothing entered the
             window (timing/PLL problem, or genuinely silent channel)
    trunc    the last address-matched failure had got_end=false with
             in_flight=true: the frame was cut by the MPSL hard edge
             (slot_len - 40) - a timing artifact, not an RF problem
    corrupt  address-matched frames with a real length byte died on CRC
             (crcbadl high) - genuine RF corruption (SNR/interference/
             freq-offset/phantom address match)
- Bare `RADIO rxst=` lines: rxst is the OR of the per-op flags seen during
  the window (bit0 any END, bit1 any CRC-ok, bit2 any CRC-bad,
  bit3 any no-END); addr_ev is the window's address-event count.

Caveats this script cannot fix (read before trusting numbers):
- rssi= is the LAST RSSISAMPLE of the window, not an average (dBm = -value).
- On the 5340, CRCSTATUS reads stale-1 so crc_ok is gated on END: raw
  crcbad is inflated there - trust addr/crcbadl/got_end, not crcbad alone.
- got_end/infl/lai/len/crc describe only the LAST address-matched failure
  of the window; use them as fingerprints, not statistics.
"""

import re
import sys

PLL_RE = {
    "addr": re.compile(r"\baddr=(\d+)"),
    "ai": re.compile(r"\bai=(\d+)"),
    "mis": re.compile(r"\bmis=(\d+)"),
    "crcok": re.compile(r"\bcrcok=(\d+)"),
    "crcbad": re.compile(r"\bcrcbad=(\d+)"),
    "crcbadl": re.compile(r"\bcrcbadl=(\d+)"),
    "rssi": re.compile(r"\brssi=(\d+)"),
    "w": re.compile(r"\bw=(\d+)"),
    "dist": re.compile(r"\bdist=(\d+)"),
    "target": re.compile(r"\btarget=(\d+)"),
    "got_end": re.compile(r"\bgot_end=(true|false)"),
    "infl": re.compile(r"\binfl=(true|false)"),
    "lai": re.compile(r"\blai=(\d+)"),
    "len": re.compile(r"\blen=(\d+)"),
    "crc": re.compile(r"\bcrc=(\d+)"),
    "end": re.compile(r"\bend=(\d+)"),
    # Catch-RSSI stats (absent in logs from before the instrumentation):
    # cumulative sum/count over CRC-ok catches; rmax = weakest catch since
    # the previous PLL line (self-resetting). dBm = -value.
    "rsum": re.compile(r"\brsum=(\d+)"),
    "rcnt": re.compile(r"\brcnt=(\d+)"),
    "rmax": re.compile(r"\brmax=(\d+)"),
}
RADIO_RXST = re.compile(r"RADIO rxst=(0x[0-9a-fA-F]+|\d+).*?\brssi=(\d+)")
BARE_PLL = {
    "addr_ev": re.compile(r"\baddr_ev=(\d+)"),
    "misses": re.compile(r"\bmisses=(\d+)"),
    "sweep": re.compile(r"\bsweep=(\d+)"),
    "rx_op": re.compile(r"\brx_op=(\d+)"),
}


def parse_pll(line):
    if "PLL dist=" not in line or "BARE PLL" in line:
        return None
    row = {}
    for k, rx in PLL_RE.items():
        m = rx.search(line)
        if not m:
            if k in ("rsum", "rcnt", "rmax"):
                row[k] = None  # pre-instrumentation log
                continue
            return None
        v = m.group(1)
        row[k] = v if v in ("true", "false") else int(v)
    return row


def classify(d_addr, d_crcok, d_crcbad, d_crcbadl, last):
    """One tag per window, checked most-informative first."""
    if d_crcok + d_crcbad == 0:
        return "no-rx"
    if d_crcbadl == 0 and d_addr <= d_crcok:
        # Every address-matched frame became a catch; every miss heard
        # nothing at all. The losses are silent windows, not corruption.
        return "empty"
    if last["got_end"] == "false" and last["infl"] == "true":
        return "trunc"  # cut by the hard edge (timing artifact)
    if last["len"] >= 60:
        return "phantom"  # garbage length byte: false address match
    if d_addr and d_crcbadl / d_addr > 0.3:
        return "corrupt"  # real frames destroyed on CRC
    return "mixed"


def show_pll(fn, rows):
    print(f"== {fn} (MPSL) ==")
    have_rssi_stats = any(r["rsum"] is not None for r in rows)
    hdr = (f"{'win':>4} {'rx-slots':>8} {'crcok':>6} {'d_addr':>6} {'d_cbad':>6} "
           f"{'d_cbadl':>7} {'catch%':>7} {'corr/addr%':>10} {'rssi':>6}")
    if have_rssi_stats:
        hdr += f" {'cavg':>6} {'cweak':>6}"
    print(hdr + f" {'tag':>8}  last-fail")
    tot = {"rx": 0, "ok": 0, "addr": 0, "cbad": 0, "cbadl": 0}
    tags = {}
    prev = None
    for i, r in enumerate(rows):
        if prev is None or not have_rssi_stats and prev["rsum"] is not None:
            prev = r
            continue
        d_addr = r["addr"] - prev["addr"]
        d_ok = r["crcok"] - prev["crcok"]
        d_cbad = r["crcbad"] - prev["crcbad"]
        d_cbadl = r["crcbadl"] - prev["crcbadl"]
        rx_slots = d_ok + d_cbad
        catch = 100.0 * d_ok / rx_slots if rx_slots else 0.0
        corr = 100.0 * d_cbadl / d_addr if d_addr else 0.0
        tag = classify(d_addr, d_ok, d_cbad, d_cbadl, r)
        tags[tag] = tags.get(tag, 0) + 1
        for k, v in zip(("rx", "ok", "addr", "cbad", "cbadl"),
                        (rx_slots, d_ok, d_addr, d_cbad, d_cbadl)):
            tot[k] += v
        last = (f"got_end={r['got_end']} infl={r['infl']} lai={r['lai']}us "
                f"len={r['len']} crc={r['crc']} ai={r['ai']}us")
        line = (f"{i:>4} {rx_slots:>8} {d_ok:>6} {d_addr:>6} {d_cbad:>6} "
                f"{d_cbadl:>7} {catch:>6.1f} {corr:>9.1f} {-r['rssi']:>5}d")
        if have_rssi_stats:
            d_rsum = (r["rsum"] - prev["rsum"]) & 0xFFFFFFFF
            d_rcnt = (r["rcnt"] - prev["rcnt"]) & 0xFFFFFFFF
            cavg = f"{-d_rsum / d_rcnt:>5.0f}d" if d_rcnt else "     -"
            cweak = f"{-r['rmax']:>5}d" if r["rmax"] else "     -"
            line += f" {cavg:>6} {cweak:>6}"
        print(line + f" {tag:>8}  {last}")
        prev = r
    n = sum(tags.values()) or 1
    print(f"-- windows by tag: " +
          "  ".join(f"{k}={100.0 * v / n:.0f}%" for k, v in sorted(tags.items())))
    if tot["rx"]:
        print(f"-- totals: rx_slots={tot['rx']} catch={100.0 * tot['ok'] / tot['rx']:.1f}% "
              f"addr={tot['addr']} crcbadl={tot['cbadl']} "
              f"corr/addr={100.0 * tot['cbadl'] / tot['addr']:.1f}%" if tot["addr"] else
              f"-- totals: rx_slots={tot['rx']} catch={100.0 * tot['ok'] / tot['rx']:.1f}% addr=0")
    # Reading hints (heuristic thresholds).
    if tags.get("empty", 0) * 2 > n:
        print("=> mostly EMPTY windows: timing/PLL or silent channel, not CRC. "
              "Check dist=/target= lock and ai= vs the calibrated target.")
    elif tags.get("trunc", 0) * 4 > n:
        print("=> frequent TRUNCATION: frames arrive late in the window and hit the "
              "MPSL hard edge. This is phase placement, not SNR - TX power won't help.")
    elif tot["addr"] and tot["cbadl"] / tot["addr"] > 0.3:
        if have_rssi_stats and rows[-1]["rcnt"] and rows[0]["rcnt"] is not None:
            d_sum = (rows[-1]["rsum"] - rows[0]["rsum"]) & 0xFFFFFFFF
            d_cnt = (rows[-1]["rcnt"] - rows[0]["rcnt"]) & 0xFFFFFFFF
            weak = max((r["rmax"] for r in rows if r["rmax"]), default=0)
            if d_cnt:
                print(f"-- catch RSSI: avg {-d_sum / d_cnt:.0f} dBm, weakest "
                      f"{-weak} dBm (~{95 - weak} dB headroom at 2M)")
                rssi_dbm = -d_sum // d_cnt
            else:
                rssi_dbm = -rows[-1]["rssi"]
        else:
            rssi_dbm = -rows[-1]["rssi"]
        if rssi_dbm < -75:
            print(f"=> real corruption at weak RSSI ({rssi_dbm} dBm, last sample): "
                  "link budget. Try closer spacing, antenna orientation, radio-1m.")
        else:
            print(f"=> real corruption at decent RSSI ({rssi_dbm} dBm, last sample): "
                  "interference / freq offset / collision. A/B the channel and the "
                  "environment; check crcbadl spread across channels.")
    print()


def show_bare(fn, lines):
    print(f"== {fn} (bare) ==")
    print(f"{'win':>4} {'END':>4} {'CRCok':>6} {'CRCbad':>7} {'noEND':>6} {'addr_ev':>8} {'rssi':>6} {'sweep':>6}")
    last_rssi = None
    last_sweep = 0
    for i, line in enumerate(lines):
        m = RADIO_RXST.search(line)
        if m:
            rxst = int(m.group(1), 0)
            last_rssi = -int(m.group(2))
        b = {k: rx.search(line) for k, rx in BARE_PLL.items()}
        if "BARE PLL" in line:
            addr_ev = int(b["addr_ev"].group(1)) if b["addr_ev"] else 0
            sweep = int(b["sweep"].group(1)) if b["sweep"] else 0
            last_sweep = sweep
            continue
        if not m:
            continue
        print(f"{i:>4} {'Y' if rxst & 1 else '-':>4} {'Y' if rxst & 2 else '-':>6} "
              f"{'Y' if rxst & 4 else '-':>7} {'Y' if rxst & 8 else '-':>6} "
              f"{'?':>8} {last_rssi if last_rssi is not None else '?':>5}d {last_sweep:>6}")
    print("-- rxst bits are window-OR flags (coarse): 'Y' means seen at least once.")
    print("-- combine with the adjacent BARE PLL addr_ev= (per-window address count).\n")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    for fn in sys.argv[1:]:
        try:
            with open(fn, errors="replace") as f:
                lines = f.readlines()
        except FileNotFoundError:
            print(f"missing {fn}")
            continue
        pll = [r for r in (parse_pll(l) for l in lines) if r]
        if pll:
            show_pll(fn, pll)
        elif any("RADIO rxst=" in l for l in lines):
            show_bare(fn, [l for l in lines if "RADIO rxst=" in l or "BARE PLL" in l])
        else:
            print(f"{fn}: no PLL or RADIO lines found")
    return 0


if __name__ == "__main__":
    sys.exit(main())
