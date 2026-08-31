#![no_std]
#![no_main]

//! nRF52840 fixed one-way no-ACK receiver.

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::{Duration, Instant};
use thunders::{Address, AirTiming, OneWayState, SlotOverhead, fixed_slot_plan, phy::Phy};
use thunders_phy_nrf::{NrfRadioPhy, RadioIrqHandler, RadioMode};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    RADIO => RadioIrqHandler;
});

const PAYLOAD: usize = 6;
type Mode = OneWayState<PAYLOAD, 32>;
const PLAN: thunders::FixedSlotPlan =
    fixed_slot_plan::<Mode>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
    phy.set_address(&Address([0xE7; 5])).await;
    phy.set_channel(0).await;

    let mut wire = [0u8; 64];
    let mut report_at = Instant::now();
    let mut received = 0u32;
    let mut lost = 0u32;
    let mut invalid = 0u32;
    let mut have_state = false;
    let mut last_state = 0u32;
    let window = Duration::from_micros(PLAN.receiver_window_us as u64);

    info!(
        "ONEWAY RX READY payload={} wire={} slot={} long_window={} cycle={}",
        PAYLOAD,
        PLAN.data_wire_len,
        PLAN.data_slot_us,
        PLAN.receiver_window_us,
        PLAN.period_us()
    );

    loop {
        match phy.receive(&mut wire, window).await {
            Ok(Some(len)) if len == PAYLOAD && wire[0] == b'S' && wire[1] == b'T' => {
                let state = u32::from_le_bytes([wire[2], wire[3], wire[4], wire[5]]);
                if have_state {
                    lost = lost.wrapping_add(state.wrapping_sub(last_state).wrapping_sub(1));
                }
                last_state = state;
                have_state = true;
                received = received.wrapping_add(1);
            }
            Ok(Some(_)) => invalid = invalid.wrapping_add(1),
            Ok(None) => {}
            Err(_) => invalid = invalid.wrapping_add(1),
        }

        if report_at.elapsed() >= Duration::from_secs(5) {
            let total = received.wrapping_add(lost);
            let loss_pct = if total == 0 {
                0
            } else {
                lost.saturating_mul(100) / total
            };
            info!(
                "ONEWAY RX frames={} lost={} loss={}% invalid={} last={}",
                received, lost, loss_pct, invalid, last_state
            );
            received = 0;
            lost = 0;
            invalid = 0;
            report_at = Instant::now();
        }
    }
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}
