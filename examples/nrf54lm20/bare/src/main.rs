#![no_std]
#![no_main]

//! thunders link over the direct RADIO backend — role-agnostic.
//!
//! The same binary runs as the **central** (TX the PING, await the reply)
//! or the **peripheral** (RX, echo back) — so ANY two boards with this
//! example can talk to each other, either way around.
//!
//! Build with `--features central` (default) or `--features peripheral`.
//! Run two boards, one per role, on the same channel. Add `--features
//! radio-1m` to both nodes for the 1 Mbit mode.
//!
//! The bench measures (5 s windows, the `BENCH` lines):
//!   central  — the reverse-link loss (RX slots with no echo), the payload
//!              bandwidth, the app-level RTT (PING TX slot -> echo RX).
//!   peripheral — the forward-link loss (seq gaps), the slot rate.

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::Instant;
use {defmt_rtt as _, panic_probe as _};
use defmt::info;

use thunders::{Address, Config, Role};
use thunders_phy_nrf::{NrfRadioPhy, RadioIrqHandler, RadioMode};

#[cfg(not(feature = "peripheral"))]
use thunders::link::Central as Link;
#[cfg(feature = "peripheral")]
use thunders::link::Peripheral as Link;

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}


bind_interrupts!(struct Irqs {
    RADIO_0 => RadioIrqHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    thunders_phy_nrf::hfxo_cap_trim(); // before the HFXO starts
    let mut config = embassy_nrf::config::Config::default();
    config.flpr_reset = embassy_nrf::config::FlprReset::Leave;
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
    config.clock_speed = embassy_nrf::config::ClockSpeed::CK128;
    let p = embassy_nrf::init(config);

    #[cfg(not(feature = "peripheral"))]
    let role = Role::Central;
    #[cfg(feature = "peripheral")]
    let role = Role::Peripheral;
    #[cfg(feature = "radio-1m")]
    let mode = RadioMode::Nrf1Mbit;
    #[cfg(not(feature = "radio-1m"))]
    let mode = RadioMode::Nrf2Mbit;

    info!("thunders link role={:?}", role);

    // Follower TX early margin, per-board tunable. The PHY default is the
    // safe baseline; adjust only for a specific follower/master pair after
    // checking the bare txph/txair diagnostics.
    const FOLLOWER_TX_MARGIN_US: i32 = 10;
    let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, mode);
    phy.set_paced_role(role);
    phy.set_tx_phase_margin_us(FOLLOWER_TX_MARGIN_US);
    #[cfg(feature = "radio-1m")]
    phy.set_paced_period_us(600);
    let cfg = Config::new(
        [0xAB, 0xCD, 0xEF, 0x01],
        Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
        role,
    );
    #[cfg(feature = "ratio-8-4-4")]
    let cfg = cfg.with_tx_rx_idle(8, 4, 4);
    #[cfg(feature = "ratio-6-2-2")]
    let cfg = cfg.with_tx_rx_idle(6, 2, 2);
    #[cfg(feature = "ratio-4-2-2")]
    let cfg = cfg.with_tx_rx_idle(4, 2, 2);

    let (tx_n, rx_n) = cfg.tx_rx_ratio;
    let idle_n = cfg.idle_slots;
    #[cfg(not(feature = "peripheral"))]
    let period = tx_n as u64 + rx_n as u64 + idle_n as u64;
    let mut link = Link::new(phy, cfg).await.unwrap();
    info!("link ready ({:?})", role);

    let mut rx_buf = [0u8; 32];
    // The bench accounting (5 s windows): the central measures the
    // reverse-link loss + the app-level RTT, the peripheral the
    // forward-link loss.
    let mut frames: u64 = 0;
    let mut busy_total: u64 = 0;
    let mut report_at = Instant::now();
    #[cfg(not(feature = "peripheral"))]
    let (mut ping_tx, mut echo_rx, mut rev_lost, mut last_echo_seq, mut rtt_sum, mut rtt_min, mut rtt_max, mut t_ping_tx) =
        (0u64, 0u64, 0u64, 0u32, 0u64, u32::MAX, 0u32, Instant::now());
    // Duplicate echoes (app seq does not advance); ~0 with one-shot
    // echo semantics, kept as a regression check.
    #[cfg(not(feature = "peripheral"))]
    let mut dup = 0u64;
    // FILL payloads received (reverse saturation traffic; not echoes).
    #[cfg(not(feature = "peripheral"))]
    let mut fill_rx = 0u64;
    // Monotonic across windows: the echo gap accounting needs a stable seq
    // space (a per-window restart makes every new window's echoes look
    // like backward jumps).
    #[cfg(not(feature = "peripheral"))]
    let mut ping_seq = 0u32;
    // TX-phase slot count (the rloss denominator): with RATE_DIV the offer
    // count no longer equals the TX-slot count.
    #[cfg(not(feature = "peripheral"))]
    let mut tx_frames = 0u64;
    #[cfg(feature = "peripheral")]
    let (mut rx_ok, mut fwd_lost, mut last_seq) = (0u64, 0u64, 0u32);
    #[cfg(feature = "peripheral")]
    let mut echo = [0u8; 32];
    #[cfg(feature = "peripheral")]
    let mut echo_len = 0usize;
    // Echoes overwritten before ever reaching the TX window (app-layer
    // backpressure loss).
    #[cfg(feature = "peripheral")]
    let mut ow = 0u64;
    // Filler traffic keeps the reverse link saturated without polluting
    // the echo stream: each echo is enqueued at most once (tracked via
    // offer_taken), the FILL prefix uses an independent seq space.
    #[cfg(feature = "peripheral")]
    let mut fill = *b"FILL\0\0\0\0";
    #[cfg(feature = "peripheral")]
    let mut fill_seq = 0u32;
    // Forward-rate divisor. The peripheral has reverse_tx slots per
    // period (2 with the default 8,2 ratio), so the ping offer rate must
    // stay at or below that or echoes pile up and get overwritten (ow).
    const RATE_DIV: u64 = 4;
    info!("BENCH READY role={} ratio={},{}", if cfg!(feature = "peripheral") { "P" } else { "C" }, tx_n, rx_n);

    loop {
        // The TX:RX ratio decides the slot: TX slots carry a fresh PING
        // (seq = a per-PING counter, so no structural gaps - the every-
        // 64th-slot beacon is skipped), RX slots just listen.
        #[cfg(not(feature = "peripheral"))]
        let mut p = [0x50u8, 0x49, 0x4E, 0x47, 0, 0, 0, 0];
        #[cfg(not(feature = "peripheral"))]
        let tx_phase = (frames % period) < tx_n as u64 && frames % 64 != 0;
        #[cfg(not(feature = "peripheral"))]
        if tx_phase {
            tx_frames += 1;
        }
        #[cfg(not(feature = "peripheral"))]
        let tx: Option<&[u8]> = if tx_phase && frames % RATE_DIV == 0 && !link.tx_window_full() {
            ping_tx += 1;
            ping_seq = ping_seq.wrapping_add(1);
            t_ping_tx = Instant::now();
            p[4..].copy_from_slice(&ping_seq.to_le_bytes());
            Some(&p)
        } else {
            None
        };
        #[cfg(feature = "peripheral")]
        let offered_echo = echo_len > 0 && !link.tx_window_full();
        #[cfg(feature = "peripheral")]
        let tx: Option<&[u8]> = if offered_echo {
            Some(&echo[..echo_len])
        } else if link.tx_inflight() < 8 {
            fill[4..].copy_from_slice(&fill_seq.to_le_bytes());
            Some(&fill)
        } else {
            None
        };

        let t_frame = Instant::now();
        match link.frame(tx, &mut rx_buf).await {
            Ok(Some(n)) if n >= 8 && rx_buf[..4] == *b"PING" => {
                #[cfg(not(feature = "peripheral"))]
                {
                    // The echo of the last PING arrived on an RX slot:
                    // the RTT is measured from that PING's TX slot.
                    echo_rx += 1;
                    let seq = u32::from_le_bytes([rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]]);
                    let gap = seq.wrapping_sub(last_echo_seq);
                    // Only forward movement rebaselines: a backward jump is
                    // a resync/restart artifact, and accepting it would
                    // re-count the catch-up span as fresh loss.
                    if echo_rx == 1 || gap < 1_000_000 {
                        if echo_rx > 1 {
                            if gap == 0 {
                                dup += 1;
                            } else if gap > 1 {
                                rev_lost += (gap - 1) as u64;
                            }
                        }
                        last_echo_seq = seq;
                    }
                    let rtt = t_ping_tx.elapsed().as_micros() as u32;
                    rtt_sum += rtt as u64;
                    if rtt < rtt_min {
                        rtt_min = rtt;
                    }
                    if rtt > rtt_max {
                        rtt_max = rtt;
                    }
                }
                #[cfg(feature = "peripheral")]
                {
                    let seq =
                        u32::from_le_bytes([rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]]);
                    if rx_ok > 1 {
                        let gap = seq.wrapping_sub(last_seq);
                        // A gap beyond ~1 M seqs is a peer restart, not a
                        // loss burst (a 5 s window holds at most ~35 k).
                        if gap > 1 && gap < 1_000_000 {
                            fwd_lost += (gap - 1) as u64;
                        }
                    }
                    last_seq = seq;
                    rx_ok += 1;
                    // The previous echo never reached the TX window
                    // (persistent backpressure): an app-layer loss.
                    if echo_len > 0 {
                        ow += 1;
                    }
                    echo[..n].copy_from_slice(&rx_buf[..n]);
                    echo_len = n;
                }
            }
            Ok(Some(n)) if n >= 8 && rx_buf[..4] == *b"FILL" => {
                #[cfg(not(feature = "peripheral"))]
                {
                    fill_rx += 1;
                }
            }
            Ok(_) => {}
            Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
        }
        // One-shot echo: consumed by the link layer at most once, then the
        // slot offers filler instead.
        #[cfg(feature = "peripheral")]
        if link.offer_taken() {
            if offered_echo {
                echo_len = 0;
            } else {
                fill_seq = fill_seq.wrapping_add(1);
            }
        }
        frames += 1;
        busy_total += t_frame.elapsed().as_micros() as u64;
        let now = Instant::now();
        if now - report_at >= embassy_time::Duration::from_secs(5) {
            let elapsed = (now - report_at).as_micros() as u64;
            let rate = frames * 1_000_000 / elapsed.max(1);
            let avg_busy = busy_total / frames.max(1);
            #[cfg(not(feature = "peripheral"))]
            {
                // The reverse-link loss: RX slots that caught no echo.
                let rx_slots = frames - tx_frames;
                // A reverse hit is any payload from the peripheral (echo or
                // filler).
                let rev_rx = echo_rx + fill_rx;
                let rloss = if rx_slots > 0 {
                    rx_slots.saturating_sub(rev_rx) * 100 / rx_slots
                } else {
                    0
                };
                // Payload throughput: 8 B per PING + 8 B per reverse packet.
                let bw = (ping_tx + rev_rx) * 8 * 1_000_000 / elapsed.max(1);
                let (ra, rmin, rmax) = if echo_rx > 0 {
                    (rtt_sum / echo_rx, rtt_min, rtt_max)
                } else {
                    (0, 0, 0)
                };
                let rev_loss = if echo_rx + rev_lost > 0 {
                    rev_lost * 100 / (echo_rx + rev_lost)
                } else {
                    0
                };
                info!("BENCH C slots={} tx={} rx={} rloss={}% rate={}/s bw={}B/s rtt_avg={}us rtt_min={}us rtt_max={}us busy={}us df={} wf={} rt={} nk={} rev_lost={} rev_loss={}% dup={} fill={}", frames, ping_tx, echo_rx, rloss, rate, bw, ra, rmin, rmax, avg_busy, link.delivery_failures(), link.window_full(), link.retransmits(), link.nacks_recv(), rev_lost, rev_loss, dup, fill_rx);
            }
            #[cfg(feature = "peripheral")]
            {
                let floss = if rx_ok + fwd_lost > 0 {
                    fwd_lost * 100 / (rx_ok + fwd_lost)
                } else {
                    0
                };
                info!("BENCH P slots={} rx={} lost={} floss={}% rate={}/s busy={}us df={} wf={} rt={} ow={}", frames, rx_ok, fwd_lost, floss, rate, avg_busy, link.delivery_failures(), link.window_full(), link.retransmits(), ow);
            }
            let rxst =
                thunders_phy_nrf::radio_phy::RX_STATS.swap(0, core::sync::atomic::Ordering::Relaxed);
            let rxp = thunders_phy_nrf::radio_phy::RX_POLL.load(core::sync::atomic::Ordering::Relaxed);
            let rxp_us = thunders_phy_nrf::radio_phy::RX_POLL_US.load(core::sync::atomic::Ordering::Relaxed);
            let rssi = thunders_phy_nrf::radio_phy::RX_RSSI.load(core::sync::atomic::Ordering::Relaxed);
            let ba = thunders_phy_nrf::radio_phy::BARE_ADDR_POLL_US.load(core::sync::atomic::Ordering::Relaxed);
            let bmis = thunders_phy_nrf::radio_phy::BARE_RX_MISSES.load(core::sync::atomic::Ordering::Relaxed);
            let bsw = thunders_phy_nrf::radio_phy::BARE_SWEEP.load(core::sync::atomic::Ordering::Relaxed);
            let bpw = thunders_phy_nrf::radio_phy::BARE_PEER_WINDOW_US.load(core::sync::atomic::Ordering::Relaxed);
            let bper = thunders_phy_nrf::radio_phy::BARE_EFFECTIVE_PERIOD_US.load(core::sync::atomic::Ordering::Relaxed);
            let txair = thunders_phy_nrf::radio_phy::BARE_TX_ON_AIR_US.load(core::sync::atomic::Ordering::Relaxed);
            let txair_min = thunders_phy_nrf::radio_phy::BARE_TX_AIR_MIN.swap(u32::MAX, core::sync::atomic::Ordering::Relaxed);
            let txair_max = thunders_phy_nrf::radio_phy::BARE_TX_AIR_MAX.swap(0, core::sync::atomic::Ordering::Relaxed);
            let rx_op = thunders_phy_nrf::radio_phy::BARE_RX_OP_US.swap(0, core::sync::atomic::Ordering::Relaxed);
            let tx_op = thunders_phy_nrf::radio_phy::BARE_TX_OP_US.swap(0, core::sync::atomic::Ordering::Relaxed);
            let ba_slot = thunders_phy_nrf::radio_phy::BARE_ADDR_SLOT_US.load(core::sync::atomic::Ordering::Relaxed);
            let baddr_ev = thunders_phy_nrf::radio_phy::BARE_ADDR_EVENTS.swap(0, core::sync::atomic::Ordering::Relaxed);
            let bcorr = thunders_phy_nrf::radio_phy::BARE_PLL_CORR_US.load(core::sync::atomic::Ordering::Relaxed);
            let txp = thunders_phy_nrf::radio_phy::TX_POLL.load(core::sync::atomic::Ordering::Relaxed);
            let txs = thunders_phy_nrf::radio_phy::TX_STATS.load(core::sync::atomic::Ordering::Relaxed);
            info!("RADIO rxst={:#x} rxp={} rxp_us={} rssi={} txp={} txs={:#x}", rxst, rxp, rxp_us, rssi, txp, txs);
            let (btxph, brxph) = thunders_phy_nrf::radio_phy::bare_phase_snapshot();
            info!("BARE PLL addr_us={} addr_slot={} corr={} misses={} sweep={} peerw={} period={} txair={} txair_min={} txair_max={} rx_op={} tx_op={} addr_ev={} txph={:?} rxph={:?} phase={}", ba, ba_slot, bcorr, bmis, bsw, bpw, bper, txair, txair_min, txair_max, rx_op, tx_op, baddr_ev, btxph, brxph, link.slot_phase());
            frames = 0;
            busy_total = 0;
            report_at = now;
            #[cfg(not(feature = "peripheral"))]
            {
                ping_tx = 0;
                tx_frames = 0;
                echo_rx = 0;
                rev_lost = 0;
                dup = 0;
                fill_rx = 0;
                rtt_sum = 0;
                rtt_min = u32::MAX;
                rtt_max = 0;
            }
            #[cfg(feature = "peripheral")]
            {
                rx_ok = 0;
                fwd_lost = 0;
                ow = 0;
            }
        }
    }
}
