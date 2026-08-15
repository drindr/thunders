#!/usr/bin/env python3
"""Phase-lock simulation for the bare NrfRadioPhy software slot scheduler.

Validates the follower's sweep/PLL and the echo placement against the
fixed-period bare slot grid. The model is intentionally simple: phase is
tracked modulo the slot period, TX on-air offsets are fixed, and the RX
windows are the ones used by the bare PHY (BARE_RX_OFFSET_US and the 200 us
link-layer timeout). The acquisition sweep loses a few percent of slots at
the start; after lock both directions should be ~100 % for any reasonable
TX-offset combination.
"""

import random

T = 400                 # BARE_SLOT_PERIOD_US
RX_OFFSET = 30          # BARE_RX_OFFSET_US
RX_WINDOW = 200         # CENTRAL_REPLY_TIMEOUT_US / PERIPHERAL_LISTEN_TIMEOUT_US
TARGET_ON_AIR = 50      # BARE_RX_ON_AIR_TARGET_US
TARGET_ADDR = TARGET_ON_AIR + 28
TX_RAMP = 40            # BARE_TX_RAMP_US (MODECNF0.RU = Fast)
SWEEP = 2               # BARE_SLOT_SWEEP_US
GAIN = 1 / 4            # BARE_SLOT_GAIN_NUM / BARE_SLOT_GAIN_DEN
CLAMP = 20              # BARE_SLOT_CORR_CLAMP_US
RESWEEP = 500           # BARE_SLOT_RESWEEP_MISSES
AIR = 84                # on-air time of the 11-byte bench PING/echo packet


def simulate(central_tx_offset, peripheral_tx_offset, slots=3000, seed=0):
    """Return (forward_catch_rate, reverse_catch_rate)."""
    random.seed(seed)
    d = random.uniform(0, T)  # follower slot start minus central slot start
    sweep = True
    rx_misses = 0
    peer_window = 0
    fwd = rev = 0
    tx_slots = rx_slots = 0

    for n in range(slots):
        central_is_tx = (n % 9) < 8  # (8,1) ratio
        if central_is_tx:
            tx_slots += 1
            # Central frame [cto, cto+AIR] relative to the central slot;
            # follower RX window [d+RX_OFFSET, d+RX_OFFSET+RX_WINDOW].
            if d + RX_OFFSET <= central_tx_offset and \
                    central_tx_offset + AIR <= d + RX_OFFSET + RX_WINDOW:
                fwd += 1
                addr = central_tx_offset + 28 - d
                err = addr - TARGET_ADDR
                corr = max(-CLAMP, min(CLAMP, GAIN * err))
                d = (d + corr + (SWEEP if sweep else 0)) % T
                sweep = False
                rx_misses = 0
                peer_window = RX_WINDOW
            else:
                rx_misses += 1
                if rx_misses >= RESWEEP:
                    sweep = True
                d = (d + (SWEEP if sweep else 0)) % T
        else:
            rx_slots += 1
            # Peripheral echo, placed in the central's advertised window.
            if peer_window > 0:
                target_on_air = RX_OFFSET + (peer_window - AIR) // 2
                desired_txen = max(0, target_on_air - TX_RAMP)
                txen = max(desired_txen, peripheral_tx_offset - TX_RAMP)
                on_air = d + txen + TX_RAMP
            else:
                on_air = d + peripheral_tx_offset
            if RX_OFFSET <= on_air and on_air + AIR <= RX_OFFSET + RX_WINDOW:
                rev += 1

    return fwd / tx_slots, rev / rx_slots


def main():
    for central_tx, peripheral_tx in [(50, 50), (50, 70), (70, 50), (50, 40)]:
        fwd = rev = 0.0
        seeds = 100
        for seed in range(seeds):
            f, r = simulate(central_tx, peripheral_tx, seed=seed)
            fwd += f
            rev += r
        print(
            f"central_tx={central_tx:2d} us peripheral_tx={peripheral_tx:2d} us "
            f"-> fwd {fwd / seeds:6.1%}  rev {rev / seeds:6.1%}"
        )


if __name__ == "__main__":
    main()
