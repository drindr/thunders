#![no_std]
#![no_main]

//! thunders over MPSL timeslots on the nRF5340 net core - role-agnostic.
//! Build as the central (default) or the peripheral:
//!   cargo build --release                          # central
//!   cargo build --release --no-default-features --features peripheral
//! Any thunders node can be either role; the link is full-duplex.

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::Instant;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals, raw};
use {defmt_rtt as _, panic_probe as _};
use defmt::info;
use static_cell::StaticCell;

use thunders::link::{Central, Peripheral};
use thunders::{Address, Config, Role};
use thunders_phy_nrf::mpsl::MpslRadioPhy;
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
    SWI0 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("thunders MPSL (nRF5340 net core, {:?})", defmt::Debug2Format(&ROLE));

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    // The net core's LFCLK XTAL never starts here (the app core owns it);
    // the thunders timing is free-running, so the RC is fine.
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    let mpsl_p = Peripherals::new(
        p.RTC0, p.TIMER0, p.TIMER1, p.TEMP,
        p.PPI_CH0, p.PPI_CH1, p.PPI_CH2,
    );

    let lfclk_cfg = raw::mpsl_clock_lfclk_cfg_t {
        source: raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
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
    let phy = MpslRadioPhy::new(RadioMode::Nrf2Mbit).await;
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    info!("MPSL ready");

    let cfg = Config::new(
        [0xAB, 0xCD, 0xEF, 0x01],
        Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
        ROLE,
    );

    match ROLE {
        Role::Central => {
            let mut central = Central::new(phy, cfg).await.unwrap();
            info!("link ready (Central)");

            let mut rx_buf = [0u8; 32];
            let mut txc: u32 = 0;
            let mut rxc: u32 = 0;
            let mut busy_total: u64 = 0;
            let mut frame_count: u32 = 0;

            loop {
                let payload = [0x50, 0x49, 0x4E, 0x47, (txc & 0xFF) as u8, 0, 0, 0];
                let t_start = Instant::now();
                match central.frame(Some(&payload), &mut rx_buf).await {
                    Ok(Some(n)) if n >= 4 && rx_buf[..4] == *b"PING" => {
                        rxc += 1;
                        info!("central GOT reply {} bytes", n);
                    }
                    Ok(_) => {}
                    Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
                }
                let busy = t_start.elapsed().as_micros() as u64;

                txc += 1;
                busy_total += busy;
                frame_count += 1;

                if frame_count >= 1000 {
                    let avg_busy = busy_total / frame_count as u64;
                    let occ = avg_busy * 100 / 1000;
                    info!("BENCH frames={} rx={} avg_busy={}us occ={}%", frame_count, rxc, avg_busy, occ);
                    busy_total = 0;
                    frame_count = 0;
                    rxc = 0;
                }
            }
        }
        Role::Peripheral => {
            let mut peripheral = Peripheral::new(phy, cfg).await.unwrap();
            info!("link ready (Peripheral)");

            let mut rx_buf = [0u8; 32];
            let mut echo_buf = [0u8; 32];
            let mut frames: u32 = 0;
            let mut rx_ok: u32 = 0;
            let mut bad_seq: u32 = 0;
            let mut last_seq: u8 = 0;
            let mut busy_total: u64 = 0;
            let mut report_at = Instant::now();

            loop {
                let t_frame = Instant::now();
                match peripheral.frame(None, &mut rx_buf).await {
                    Ok(Some(n)) => {
                        rx_ok += 1;
                        // The PING payload is [P, I, N, G, seq, 0, 0, 0]; echo
                        // it back (the link is full-duplex).
                        if n >= 5 && rx_buf[..4] == *b"PING" {
                            let seq = rx_buf[4];
                            if seq.wrapping_sub(last_seq) > 1 {
                                bad_seq += 1;
                            }
                            last_seq = seq;
                            info!("PING seq={} len={}", seq, n);
                            let _ = peripheral.frame(Some(&rx_buf[..n]), &mut echo_buf).await;
                            continue;
                        } else {
                            info!("RX {} bytes: {:02x}", n, &rx_buf[..n]);
                        }
                    }
                    Ok(None) => {}
                    Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
                }
                frames += 1;
                busy_total += t_frame.elapsed().as_micros() as u64;
                let now = Instant::now();
                if now - report_at >= embassy_time::Duration::from_secs(2) {
                    let elapsed = (now - report_at).as_micros() as u32;
                    let rate = frames * 1_000_000 / elapsed.max(1);
                    let avg_busy = busy_total / frames.max(1) as u64;
                    info!("BENCH frames={} rx={} bad_seq={} rate={}/s avg_busy={}us", frames, rx_ok, bad_seq, rate, avg_busy);
                    frames = 0;
                    rx_ok = 0;
                    bad_seq = 0;
                    busy_total = 0;
                    report_at = now;
                }
            }
        }
    }
}
