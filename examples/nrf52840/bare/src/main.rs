#![no_std]
#![no_main]

//! thunders link over the direct RADIO backend — role-agnostic.
//!
//! The same binary runs as the **central** (TX the PING, await the reply)
//! or the **peripheral** (RX, echo back) — so ANY two boards with this
//! example can talk to each other, either way around.
//!
//! Build with `--features central` (default) or `--features peripheral`.
//! Run two boards, one per role, on the same channel.
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

use thunders::phy::Phy;
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
    RADIO => RadioIrqHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Board-specific clocks: the 5340 net core's LFCLK XTAL is owned by the
    // app core, so use the internal RC; the nRF54LM20 uses the XTALs.
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    #[cfg(not(feature = "peripheral"))]
    let role = Role::Central;
    #[cfg(feature = "peripheral")]
    let role = Role::Peripheral;

    info!("thunders link role={:?}", role);

    #[cfg(feature = "one-way")]
    {
        // Standalone TX bench: the raw radio transmit, no link, no await
        // hop - the pure radio TX rate (the transmit_blocking is sync).
        let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
        phy.set_address(&Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7])).await;
        phy.set_channel(0).await;
        info!("one-way TX ready");
        // DWT cycle counter for the rate: the RTC/Instant time driver on the
        // 52840 skews the elapsed; the CPU cycle counter does not.
        unsafe {
            (0xE000_EDFC as *mut u32).write_volatile(1); // DEMCR.TRCENA
            (0xE000_1000 as *mut u32).write_volatile(1); // DWT.CTRL.CYCCNTENA
            (0xE000_1004 as *mut u32).write_volatile(0); // DWT.CYCCNT = 0
        }
        let mut txc: u32 = 0;
        let mut frames: u32 = 0;
        // Burst TX: the ramp once, then the packets chain (the radio stays
        // warm). The rate's measured over the host wall-clock (the run
        // duration) - the DWT CYCCNT freezes during the radio IO.
        let mut first = true;
        loop {
            let payload = [0x50, 0x49, 0x4E, 0x47, (txc & 0xFF) as u8, 0, 0, 0];
            txc = txc.wrapping_add(1);
            if first {
                phy.transmit_burst_begin(&payload).unwrap();
                first = false;
            } else {
                phy.transmit_burst_send(&payload).unwrap();
            }
            frames += 1;
            if frames % 5000 == 0 {
                let txp = thunders_phy_nrf::radio_phy::TX_POLL.load(core::sync::atomic::Ordering::Relaxed);
                info!("BURST frames={} txp={}", frames, txp);
                frames = 0;
            }
        }
    }
    #[cfg(not(feature = "one-way"))]
    {
        let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
        phy.set_paced(cfg!(feature = "peripheral"));
        let mut cfg = Config::new(
            [0xAB, 0xCD, 0xEF, 0x01],
            Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
            role,
        );
        let cfg = if cfg!(feature = "secure") {
            cfg.with_security(thunders::Security::with_ccm([0xAB; 32]))
        } else {
            cfg
        };
        // The CCM loopback: the encrypt -> the decrypt -> the MIC, on one board
        // (no radio) to isolate the AES-CCM hardware from the nonce/radio path.
        #[cfg(feature = "secure")]
        {
            let mut data = [0x50u8, 0x49, 0x4E, 0x47, 0x00, 0x00, 0x00, 0x00];
            let orig = data;
            let mut mic = [0u8; 4];
            let key = [0xABu8; 16];
            let nonce = [0u8; 13];
            match phy.ccm_crypt(&key, &nonce, &mut data, &mut mic, true) {
                Ok(()) => {
                    let mut dec = data;
                    let mut m2 = mic;
                    match phy.ccm_crypt(&key, &nonce, &mut dec, &mut m2, false) {
                        Ok(()) => info!("CCM loopback OK: dec==orig {} mic=={}", dec == orig, m2 == mic),
                        Err(e) => info!("CCM loopback decrypt err: {:?}", defmt::Debug2Format(&e)),
                    }
                }
                Err(e) => info!("CCM loopback encrypt err: {:?}", defmt::Debug2Format(&e)),
            }
        }

        let (tx_n, rx_n) = cfg.tx_rx_ratio;
        #[cfg(not(feature = "peripheral"))]
        let period = tx_n as u64 + rx_n as u64;
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
        let (mut ping_tx, mut echo_rx, mut rtt_sum, mut rtt_min, mut rtt_max, mut t_ping_tx) =
            (0u64, 0u64, 0u64, u32::MAX, 0u32, Instant::now());
        #[cfg(feature = "peripheral")]
        let (mut rx_ok, mut fwd_lost, mut last_seq) = (0u64, 0u64, 0u32);
        #[cfg(feature = "peripheral")]
        let mut echo = [0u8; 32];
        #[cfg(feature = "peripheral")]
        let mut echo_len = 0usize;
        info!("BENCH READY role={} ratio={},{}", if cfg!(feature = "peripheral") { "P" } else { "C" }, tx_n, rx_n);

        loop {
            // The TX:RX ratio decides the slot: TX slots carry a fresh PING
            // (seq = a per-PING counter, so no structural gaps - the every-
            // 64th-slot beacon is skipped), RX slots just listen.
            #[cfg(not(feature = "peripheral"))]
            let mut p = [0x50u8, 0x49, 0x4E, 0x47, 0, 0, 0, 0];
            #[cfg(not(feature = "peripheral"))]
            let tx: Option<&[u8]> = if (frames % period) < tx_n as u64 && frames % 64 != 0 {
                ping_tx += 1;
                t_ping_tx = Instant::now();
                p[4..].copy_from_slice(&(ping_tx as u32).to_le_bytes());
                Some(&p)
            } else {
                None
            };
            #[cfg(feature = "peripheral")]
            let tx: Option<&[u8]> = if echo_len > 0 {
                Some(&echo[..echo_len])
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
                        echo[..n].copy_from_slice(&rx_buf[..n]);
                        echo_len = n;
                    }
                }
                Ok(_) => {}
                Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
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
                    let rx_slots = frames - ping_tx;
                    let rloss = if rx_slots > 0 {
                        rx_slots.saturating_sub(echo_rx) * 100 / rx_slots
                    } else {
                        0
                    };
                    // Payload throughput: 8 B per PING + 8 B per echo.
                    let bw = (ping_tx + echo_rx) * 8 * 1_000_000 / elapsed.max(1);
                    let (ra, rmin, rmax) = if echo_rx > 0 {
                        (rtt_sum / echo_rx, rtt_min, rtt_max)
                    } else {
                        (0, 0, 0)
                    };
                    info!("BENCH C slots={} tx={} rx={} rloss={}% rate={}/s bw={}B/s rtt_avg={}us rtt_min={}us rtt_max={}us busy={}us", frames, ping_tx, echo_rx, rloss, rate, bw, ra, rmin, rmax, avg_busy);
                }
                #[cfg(feature = "peripheral")]
                {
                    let floss = if rx_ok + fwd_lost > 0 {
                        fwd_lost * 100 / (rx_ok + fwd_lost)
                    } else {
                        0
                    };
                    info!("BENCH P slots={} rx={} lost={} floss={}% rate={}/s busy={}us", frames, rx_ok, fwd_lost, floss, rate, avg_busy);
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
                let lup = thunders_phy_nrf::radio_phy::LOOP_US.swap(0, core::sync::atomic::Ordering::Relaxed);
                let dreads = thunders_phy_nrf::radio_phy::DISABLE_READS.swap(0, core::sync::atomic::Ordering::Relaxed);
                info!("RADIO rxst={:#x} rxp={} rxp_us={} rssi={} txp={} txs={:#x} loop={}us dis={}", rxst, rxp, rxp_us, rssi, txp, txs, lup, dreads);
                info!("BARE PLL addr_us={} addr_slot={} corr={} misses={} sweep={} peerw={} period={} txair={} txair_min={} txair_max={} rx_op={} tx_op={} addr_ev={}", ba, ba_slot, bcorr, bmis, bsw, bpw, bper, txair, txair_min, txair_max, rx_op, tx_op, baddr_ev);
                frames = 0;
                busy_total = 0;
                report_at = now;
                #[cfg(not(feature = "peripheral"))]
                {
                    ping_tx = 0;
                    echo_rx = 0;
                    rtt_sum = 0;
                    rtt_min = u32::MAX;
                    rtt_max = 0;
                }
                #[cfg(feature = "peripheral")]
                {
                    rx_ok = 0;
                    fwd_lost = 0;
                }
            }
        }
    }
}
