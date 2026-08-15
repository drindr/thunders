#!/usr/bin/env python3
"""Phase-lock simulation for the bare NrfRadioPhy software slot scheduler.

The model tracks the follower phase modulo the slot period, wraps the RX
window around the period boundary, and applies the same one-shot
proportional correction and sweep as the firmware. It is intentionally a
phase-only model: TX on-air offsets and RX window edges are fixed.

Run this after changing any BARE_SLOT_* / BARE_RX_* constant to make sure
the follower still converges for both per-board address targets.
"""

import random

T = 400                 # BARE_SLOT_PERIOD_US
RX_OFFSET = 30          # BARE_RX_OFFSET_US
RX_WINDOW = 200         # link-layer RX timeout
TX_RAMP = 40            # BARE_TX_RAMP_US
SWEEP = 2               # BARE_SLOT_SWEEP_US
GAIN = 1 / 4            # BARE_SLOT_GAIN_NUM / BARE_SLOT_GAIN_DEN
CLAMP = 20              # BARE_SLOT_CORR_CLAMP_US
RESWEEP = 5000          # BARE_SLOT_RESWEEP_MISSES
AIR = 84                # on-air time of the 11-byte bench PING/echo packet


def contains_wrapped(win_start, win_len, x_start, x_len):
    """True if [x_start, x_start+x_len] is fully inside
    [win_start, win_start+win_len] modulo T."""
    ws = win_start % T
    def unwrap(v):
        v %= T
        if v < ws:
            v += T
        return v
    x_s = unwrap(x_start)
    x_e = unwrap(x_end := x_start + x_len)
    return x_s >= ws and x_e <= ws + win_len


def simulate(target_addr, central_tx_offset, peripheral_tx_offset,
             slots=5000, seed=0):
    """Return (forward_catch_rate, reverse_catch_rate)."""
    random.seed(seed)
    d = random.uniform(0, T)  # follower slot start minus central slot start
    sweep = True
    rx_misses = 0
    peer_window = 0
    last_addr = None
    fwd = rev = 0
    tx_slots = rx_slots = 0

    for n in range(slots):
        central_is_tx = (n % 9) < 8  # (8,1) ratio
        if central_is_tx:
            tx_slots += 1
            if contains_wrapped(d + RX_OFFSET, RX_WINDOW,
                                central_tx_offset, AIR):
                fwd += 1
                addr = (central_tx_offset + 28 - d) % T
                err = addr - target_addr
                corr = max(-CLAMP, min(CLAMP, GAIN * err))
                d = (d + corr + (SWEEP if sweep else 0)) % T
                sweep = False
                rx_misses = 0
                peer_window = RX_WINDOW
                last_addr = addr
            else:
                rx_misses += 1
                if rx_misses >= RESWEEP:
                    sweep = True
                d = (d + (SWEEP if sweep else 0)) % T
        else:
            rx_slots += 1
            if peer_window > 0 and last_addr is not None:
                s = last_addr - 28
                target_on_air = RX_OFFSET + (peer_window - AIR) // 2
                our_tx_on_air = target_on_air + s - 50
                desired_txen = max(0, our_tx_on_air - TX_RAMP)
                txen = max(desired_txen, peripheral_tx_offset - TX_RAMP)
                on_air = d + txen + TX_RAMP
            else:
                on_air = d + peripheral_tx_offset
            if contains_wrapped(RX_OFFSET, RX_WINDOW, on_air, AIR):
                rev += 1

    return fwd / tx_slots, rev / rx_slots


def main():
    for target_addr in (78, 156):  # nRF54L vs nRF52/53 follower targets
        for central_tx, peripheral_tx in [(50, 50), (50, 70), (70, 50)]:
            fwd = rev = 0.0
            seeds = 100
            for seed in range(seeds):
                f, r = simulate(target_addr, central_tx, peripheral_tx,
                                seed=seed)
                fwd += f
                rev += r
            print(
                f"target={target_addr:3d} us central_tx={central_tx:2d} us "
                f"peripheral_tx={peripheral_tx:2d} us "
                f"-> fwd {fwd / seeds:6.1%}  rev {rev / seeds:6.1%}"
            )


if __name__ == "__main__":
    main()
