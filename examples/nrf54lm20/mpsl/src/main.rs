#![no_std]
#![no_main]

//! thunders over MPSL timeslots on the nRF54LM20 app core - role-agnostic.
//! Build as the central (default) or the peripheral:
//!   cargo build --release                          # central
//!   cargo build --release --no-default-features --features peripheral
//! Any thunders node can be either role; the link is full-duplex.
//!
//! The bench measures (5 s windows, the `BENCH` lines):
//!   central  — the reverse-link loss (RX slots with no echo), the payload
//!              bandwidth, the app-level RTT (PING TX slot -> echo RX).
//!   peripheral — the forward-link loss (seq gaps), the slot rate.

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::Instant;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals, raw};
use {defmt_rtt as _, panic_probe as _};
use defmt::info;
use static_cell::StaticCell;

use thunders::link::{Central, Peripheral};
use thunders::{Address, Config, Role};
use thunders_phy_nrf::mpsl::{MpslRadioPhy, MpslState};
use thunders_phy_nrf::RadioMode;

#[cfg(feature = "peripheral")]
const ROLE: Role = Role::Peripheral;
#[cfg(not(feature = "peripheral"))]
const ROLE: Role = Role::Central;

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}

bind_interrupts!(struct Irqs {
    SWI00 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO_0 => nrf_mpsl::HighPrioInterruptHandler;
    TIMER10 => nrf_mpsl::HighPrioInterruptHandler;
    GRTC_3 => nrf_mpsl::HighPrioInterruptHandler;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    let _ = mpsl;
    loop {
        unsafe { raw::mpsl_low_priority_process() };
        embassy_futures::yield_now().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    thunders_phy_nrf::hfxo_cap_trim(); // before the HFXO starts
    info!("thunders MPSL (nRF54LM20 app core, {:?})", defmt::Debug2Format(&ROLE));

    let mut config = embassy_nrf::config::Config::default();
    config.flpr_reset = embassy_nrf::config::FlprReset::Leave;
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
    config.clock_speed = embassy_nrf::config::ClockSpeed::CK128;
    let p = embassy_nrf::init(config);

    let mpsl_p = Peripherals::new(
        p.GRTC_CH7, p.GRTC_CH8, p.GRTC_CH9, p.GRTC_CH10, p.GRTC_CH11,
        p.TIMER10, p.TIMER20, p.TEMP,
        p.PPI10_CH0, p.PPI20_CH1, p.PPIB11_CH0, p.PPIB21_CH0,
    );

    let lfclk_cfg = raw::mpsl_clock_lfclk_cfg_t {
        source: raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 50,
        skip_wait_lfclk_started: false,
    };

    // The MPSL layer is initialized WITH timeslot support: the SessionMem is
    // passed to the constructor, which configures the session count internally
    // (the phy then opens its own session within that pool).
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    static MPSL_MEM: StaticCell<nrf_mpsl::SessionMem<8>> = StaticCell::new();
    let mem = MPSL_MEM.init(nrf_mpsl::SessionMem::new());
    let mpsl = MPSL.init(MultiprotocolServiceLayer::with_timeslots(mpsl_p, Irqs, lfclk_cfg, mem).unwrap());

    // The external phy opens its timeslot session and inserts the first
    // (EARLIEST) request BEFORE the mpsl_task starts processing.
    let radio = embassy_nrf::pac::RADIO_S;
    // 'static: the phy hands this pointer to the MPSL callback (ISR).
    static STATE: StaticCell<MpslState> = StaticCell::new();
    let state = STATE.init(MpslState::new(radio, cfg!(feature = "peripheral")));
    let mut phy = MpslRadioPhy::<500, 400, 1400>::new(RadioMode::Nrf2Mbit, state);
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    phy.wait_ready().await;
    info!("MPSL ready");

    let cfg = Config::new(
        [0xAB, 0xCD, 0xEF, 0x01],
        Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
        ROLE,
    );
    let (tx_n, rx_n) = cfg.tx_rx_ratio;

    match ROLE {
        Role::Central => {
            let period = tx_n as u64 + rx_n as u64;
            let mut central = Central::new(phy, cfg).await.unwrap();
            info!("link ready (Central)");

            let mut rx_buf = [0u8; 32];
            let mut frames: u64 = 0;
            let mut ping_tx: u64 = 0;
            let mut echo_rx: u64 = 0;
            let mut rtt_sum: u64 = 0;
            let mut rtt_min: u32 = u32::MAX;
            let mut rtt_max: u32 = 0;
            let mut busy_total: u64 = 0;
            let mut t_ping_tx = Instant::now();
            let mut report_at = Instant::now();
            info!("BENCH READY role=C ratio={},{}", tx_n, rx_n);

            loop {
                // TX slots carry a fresh PING (seq per PING, beacons skipped),
                // RX slots listen.
                let mut p = [0x50u8, 0x49, 0x4E, 0x47, 0, 0, 0, 0];
                let tx: Option<&[u8]> = if (frames % period) < tx_n as u64 && frames % 64 != 0 {
                    ping_tx += 1;
                    t_ping_tx = Instant::now();
                    p[4..].copy_from_slice(&(ping_tx as u32).to_le_bytes());
                    Some(&p)
                } else {
                    None
                };
                let t_start = Instant::now();
                match central.frame(tx, &mut rx_buf).await {
                    Ok(Some(n)) if n >= 8 && rx_buf[..4] == *b"PING" => {
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
                    Ok(_) => {}
                    Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
                }
                let busy = t_start.elapsed().as_micros() as u64;
                busy_total += busy;
                frames += 1;

                let now = Instant::now();
                if now - report_at >= embassy_time::Duration::from_secs(5) {
                    let elapsed = (now - report_at).as_micros() as u64;
                    let rate = frames * 1_000_000 / elapsed.max(1);
                    let avg_busy = busy_total / frames.max(1);
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
                    let pll = thunders_phy_nrf::mpsl::mpsl_pll_snapshot();
                    let rssi = thunders_phy_nrf::mpsl::mpsl_rssi();
                    info!("PLL dist={} catch={} w={} peerw={} addr={} ai={} txc={} d8={} mis={} crcok={} crcbad={} target={} calib={} rssi={} hdr={:?}", pll.distance_us, pll.catch_poll_us, pll.rx_window_us, pll.peer_rx_window_us, pll.addr_events, pll.addr_poll_us, pll.tx_count, pll.tx_delay_us, pll.rx_misses, pll.crc_ok, pll.crc_bad, pll.addr_target_us, pll.calib_count, rssi, pll.last_rx_hdr);
                    frames = 0;
                    ping_tx = 0;
                    echo_rx = 0;
                    rtt_sum = 0;
                    rtt_min = u32::MAX;
                    rtt_max = 0;
                    busy_total = 0;
                    report_at = now;
                }
            }
        }
        Role::Peripheral => {
            let mut peripheral = Peripheral::new(phy, cfg).await.unwrap();
            info!("link ready (Peripheral)");

            let mut rx_buf = [0u8; 32];
            let mut echo = [0u8; 32];
            let mut echo_len = 0usize;
            let mut frames: u64 = 0;
            let mut rx_ok: u64 = 0;
            let mut fwd_lost: u64 = 0;
            let mut last_seq: u32 = 0;
            let mut busy_total: u64 = 0;
            let mut report_at = Instant::now();
            info!("BENCH READY role=P ratio={},{}", tx_n, rx_n);

            loop {
                let t_frame = Instant::now();
                let tx: Option<&[u8]> = if echo_len > 0 {
                    Some(&echo[..echo_len])
                } else {
                    None
                };
                match peripheral.frame(tx, &mut rx_buf).await {
                    Ok(Some(n)) if n >= 8 && rx_buf[..4] == *b"PING" => {
                        let seq = u32::from_le_bytes([rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]]);
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
                    Ok(_) => {}
                    Err(e) => info!("frame err: {:?} bytes={:?}", defmt::Debug2Format(&e), &rx_buf[..8.min(rx_buf.len())]),
                }
                frames += 1;
                busy_total += t_frame.elapsed().as_micros() as u64;
                let now = Instant::now();
                if now - report_at >= embassy_time::Duration::from_secs(5) {
                    let elapsed = (now - report_at).as_micros() as u64;
                    let rate = frames * 1_000_000 / elapsed.max(1);
                    let avg_busy = busy_total / frames.max(1);
                    let floss = if rx_ok + fwd_lost > 0 {
                        fwd_lost * 100 / (rx_ok + fwd_lost)
                    } else {
                        0
                    };
                    info!("BENCH P slots={} rx={} lost={} floss={}% rate={}/s busy={}us", frames, rx_ok, fwd_lost, floss, rate, avg_busy);
                    let pll = thunders_phy_nrf::mpsl::mpsl_pll_snapshot();
                    let rssi = thunders_phy_nrf::mpsl::mpsl_rssi();
                    info!("PLL dist={} catch={} w={} peerw={} addr={} ai={} txc={} d8={} mis={} crcok={} crcbad={} target={} calib={} rssi={} hdr={:?}", pll.distance_us, pll.catch_poll_us, pll.rx_window_us, pll.peer_rx_window_us, pll.addr_events, pll.addr_poll_us, pll.tx_count, pll.tx_delay_us, pll.rx_misses, pll.crc_ok, pll.crc_bad, pll.addr_target_us, pll.calib_count, rssi, pll.last_rx_hdr);
                    frames = 0;
                    rx_ok = 0;
                    fwd_lost = 0;
                    busy_total = 0;
                    report_at = now;
                }
            }
        }
    }
}
