#![no_std]
#![no_main]

//! nRF52833 sender zero for the const-generic bare TDMA schedule.

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
    RADIO => RadioIrqHandler;
});

const CPU_MHZ: u32 = 64;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    let sender = tdma_config::SCHEDULE.sender::<0>();
    let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
    phy.set_address(&sender.address()).await;
    phy.set_channel(0).await;

    let first = [b'S', b'T', 0, 0, 0, 0];
    phy.state_tx_begin(&first);
    let mut next_send_at =
        sender.deadline_after(phy.state_cycles(), sender.master_idle_us(), CPU_MHZ);
    let mut seq = 1u32;
    let mut frames = 1u32;
    let mut report_at = Instant::now();
    info!(
        "RAW TDMA TX READY sender={} senders={} frame_us={}",
        sender.index(),
        tdma_config::SCHEDULE.sender_count(),
        sender.frame_us()
    );

    loop {
        let payload = [
            b'S',
            b'T',
            seq as u8,
            (seq >> 8) as u8,
            (seq >> 16) as u8,
            (seq >> 24) as u8,
        ];
        phy.state_wait_until(next_send_at);
        phy.state_tx_begin(&payload);
        next_send_at = sender.advance_deadline(next_send_at, CPU_MHZ);
        seq = seq.wrapping_add(1);
        frames = frames.wrapping_add(1);

        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let rate = frames as u64 * 1_000_000 / elapsed_us;
            info!(
                "RAW TDMA TX sender={} rate={}/s state={}",
                sender.index(),
                rate,
                seq
            );
            frames = 0;
            report_at = Instant::now();
        }
    }
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}
