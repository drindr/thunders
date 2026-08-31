#![no_std]
#![no_main]

//! nRF54LM20 fixed one-way no-ACK transmitter.

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::{Duration, Instant};

use thunders::{Address, AirTiming, OneWayState, SlotOverhead, fixed_slot_plan, phy::Phy};
use thunders_phy_nrf::{NrfRadioPhy, RadioIrqHandler, RadioMode};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    RADIO_0 => RadioIrqHandler;
});

const PAYLOAD: usize = 6;
type Mode = OneWayState<PAYLOAD, 32>;
const PLAN: thunders::FixedSlotPlan =
    fixed_slot_plan::<Mode>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);

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
    phy.set_address(&Address([0xE7; 5])).await;
    phy.set_channel(0).await;

    let first = [b'S', b'T', 0, 0, 0, 0];
    phy.state_tx_begin(&first);
    let mut seq = 1u32;
    let mut report_frames = 1u32;
    let mut report_at = Instant::now();

    info!(
        "ONEWAY TX READY payload={} wire={} slot={} rx_window={} cycle={}",
        PAYLOAD,
        PLAN.data_wire_len,
        PLAN.data_slot_us,
        PLAN.receiver_window_us,
        PLAN.period_us()
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
        phy.state_tx_send(&payload);
        seq = seq.wrapping_add(1);
        report_frames = report_frames.wrapping_add(1);

        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let rate = report_frames as u64 * 1_000_000 / elapsed_us;
            info!(
                "ONEWAY TX frames={} rate={}/s seq={}",
                report_frames, rate, seq
            );
            report_frames = 0;
            report_at = Instant::now();
        }
    }
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}
