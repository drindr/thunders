#![no_std]
#![no_main]

//! nRF5340 follower in the const-generic bare TDMA schedule.

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::{Duration, Instant};
use thunders::{phy::Phy, Address};
use thunders_phy_nrf::{NrfRadioPhy, RadioIrqHandler, RadioMode};
use {defmt_rtt as _, panic_probe as _};

#[path = "../../../tdma_config.rs"]
mod tdma_config;

bind_interrupts!(struct Irqs {
    RADIO => RadioIrqHandler;
});

const CPU_MHZ: u32 = 64;
const RESYNC_EVERY: u32 = 128;

async fn lock_to_master(
    phy: &mut NrfRadioPhy<'static>,
    master: Address,
    wire: &mut [u8; 6],
) -> u32 {
    phy.flush().await;
    phy.set_address(&master).await;
    phy.state_rx_begin();
    loop {
        if let Some((crc_ok, stamp, _)) = phy.state_rx_next_from_timeout(wire, 5_000) {
            if crc_ok && wire[0] == b'S' && wire[1] == b'T' {
                return stamp;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    let clock = nrf_pac::CLOCK_NS;
    clock.events_hfclkstarted().write_value(0);
    clock.tasks_hfclkstart().write_value(1);
    while clock.events_hfclkstarted().read() == 0 {}
    nrf_pac::POWER_NS.tasks_constlat().write_value(1);

    let master = tdma_config::SCHEDULE.sender::<0>();
    let sender = tdma_config::SCHEDULE.sender::<1>();
    let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
    phy.set_channel(0).await;

    let mut wire = [0u8; 6];
    let stamp = lock_to_master(&mut phy, master.address(), &mut wire).await;
    phy.flush().await;
    phy.set_address(&sender.address()).await;
    let mut next_tx_at = sender.deadline_after(stamp, sender.after_master_end_us(), CPU_MHZ);
    phy.state_wait_until(next_tx_at);
    phy.state_tx_begin(&[b'S', b'T', 0, 0, 0, 0]);
    next_tx_at = sender.advance_deadline(next_tx_at, CPU_MHZ);

    let mut seq = 1u32;
    let mut since_sync = 1u32;
    let mut frames = 1u32;
    let mut report_at = Instant::now();
    info!(
        "RAW TDMA TX READY sender={} senders={} frame_us={} resync={}",
        sender.index(),
        tdma_config::SCHEDULE.sender_count(),
        sender.frame_us(),
        RESYNC_EVERY
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
        if since_sync >= RESYNC_EVERY {
            let stamp = lock_to_master(&mut phy, master.address(), &mut wire).await;
            phy.flush().await;
            phy.set_address(&sender.address()).await;
            next_tx_at = sender.deadline_after(stamp, sender.after_master_end_us(), CPU_MHZ);
            since_sync = 0;
        }

        phy.state_wait_until(next_tx_at);
        phy.state_tx_begin(&payload);
        next_tx_at = sender.advance_deadline(next_tx_at, CPU_MHZ);
        seq = seq.wrapping_add(1);
        frames = frames.wrapping_add(1);
        since_sync += 1;

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
