#![no_std]
#![no_main]

//! nRF54LM20 receiver for the const-generic bare TDMA schedule.

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::{Duration, Instant};
use thunders::phy::Phy;
use thunders_phy_nrf::{NrfRadioPhy, RadioIrqHandler, RadioMode};
use {defmt_rtt as _, panic_probe as _};

#[path = "../../../../tdma_config.rs"]
mod tdma_config;

bind_interrupts!(struct Irqs {
    RADIO_0 => RadioIrqHandler;
});

const PAYLOAD: usize = 6;

async fn run_receiver<const SENDERS: usize>(
    mut phy: NrfRadioPhy<'static>,
    addresses: &[thunders::Address; SENDERS],
) -> ! {
    assert!(phy.state_configure_senders(addresses));
    phy.state_rx_begin();
    let mut wire = [0u8; PAYLOAD];
    let mut counts = [0u32; SENDERS];
    let mut invalid = [0u32; SENDERS];
    let mut states = [0u32; SENDERS];
    let mut crc_bad = 0u32;
    let mut report_at = Instant::now();
    info!("RAW TDMA RX READY receiver=LM20 senders={}", SENDERS);

    loop {
        if let Some((crc_ok, _, sender)) = phy.state_rx_next_from_timeout(&mut wire, 2_000) {
            let sender = sender as usize;
            if crc_ok && sender < SENDERS {
                if wire[0] == b'S' && wire[1] == b'T' {
                    counts[sender] = counts[sender].wrapping_add(1);
                    states[sender] = u32::from_le_bytes([wire[2], wire[3], wire[4], wire[5]]);
                } else {
                    invalid[sender] = invalid[sender].wrapping_add(1);
                }
            } else {
                crc_bad = crc_bad.wrapping_add(1);
            }
        }

        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            for sender in 0..SENDERS {
                let rate = counts[sender] as u64 * 1_000_000 / elapsed_us;
                info!(
                    "RAW TDMA RX sender={} rate={}/s state={} invalid={}",
                    sender, rate, states[sender], invalid[sender]
                );
            }
            info!("RAW TDMA RX crcbad={}", crc_bad);
            counts = [0; SENDERS];
            invalid = [0; SENDERS];
            crc_bad = 0;
            report_at = Instant::now();
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    thunders_phy_nrf::hfxo_cap_trim();
    let mut config = embassy_nrf::config::Config::default();
    config.flpr_reset = embassy_nrf::config::FlprReset::Leave;
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
    config.clock_speed = embassy_nrf::config::ClockSpeed::CK128;
    let p = embassy_nrf::init(config);

    let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
    phy.set_channel(0).await;
    run_receiver(phy, tdma_config::SCHEDULE.addresses()).await;
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}
