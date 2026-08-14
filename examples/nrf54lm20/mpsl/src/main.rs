#![no_std]
#![no_main]

//! thunders over MPSL timeslots on the nRF54LM20 app core - role-agnostic.
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
    let mut state = MpslState::new(radio, cfg!(feature = "peripheral"));
    let mut phy = MpslRadioPhy::<300, 250, 1200>::new(RadioMode::Nrf2Mbit, &mut state);
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    phy.wait_ready().await;
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
                txc += 1;
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
                busy_total += busy;
                frame_count += 1;

                if frame_count >= 1000 {
                    let avg_busy = busy_total / frame_count as u64;
                    info!("BENCH frames={} rx={} avg_busy={}us", frame_count, rxc, avg_busy);
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
            let mut echo = [0u8; 32];
            let mut echo_len = 0usize;
            let mut frames: u32 = 0;
            let mut rx_ok: u32 = 0;
            let mut bad_seq: u32 = 0;
            let mut last_seq: u8 = 0;
            let mut busy_total: u64 = 0;
            let mut report_at = Instant::now();

            loop {
                let t_frame = Instant::now();
                let tx: Option<&[u8]> = if echo_len > 0 {
                    Some(&echo[..echo_len])
                } else {
                    None
                };
                match peripheral.frame(tx, &mut rx_buf).await {
                    Ok(Some(n)) => {
                        rx_ok += 1;
                        if n >= 5 && rx_buf[..4] == *b"PING" {
                            let seq = rx_buf[4];
                            if seq.wrapping_sub(last_seq) > 1 {
                                bad_seq += 1;
                            }
                            last_seq = seq;
                            echo[..n].copy_from_slice(&rx_buf[..n]);
                            echo_len = n;
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
                    let rate = (frames as u64) * 1_000_000 / elapsed.max(1) as u64;
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
