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
const HOPPING: bool = cfg!(feature = "hopping");
const HOP_EVERY: u16 = 512;
const HOP_SEQUENCE: [u8; 8] = [0, 13, 29, 43, 57, 71, 89, 97];
const CPU_MHZ: u32 = 64;
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

    let mut wire = [0u8; PAYLOAD];
    phy.state_rx_begin();
    let mut report_at = Instant::now();
    let mut received = 0u32;
    let mut lost = 0u32;
    let mut invalid = 0u32;
    let mut have_state = false;
    let mut last_state = 0u32;
    let mut last_stamp = 0u32;
    let mut data_phase = 0u16;
    let mut phase_valid = false;
    let mut hop_index = 0usize;
    let mut hop_locked = false;

    info!(
        "ONEWAY RX READY payload={} wire={} slot={} long_window={} cycle={}",
        PAYLOAD,
        PLAN.data_wire_len,
        PLAN.data_slot_us,
        PLAN.receiver_window_us,
        PLAN.period_us()
    );

    loop {
        let timeout_us = if HOPPING { 1_200 } else { u32::MAX / CPU_MHZ };
        let Some((crc_ok, stamp)) = phy.state_rx_next_timeout(&mut wire, timeout_us) else {
            if HOPPING && hop_locked {
                hop_index = 0;
                phy.state_set_channel(HOP_SEQUENCE[0]);
                phy.state_rx_begin();
                hop_locked = false;
                phase_valid = false;
                last_stamp = 0;
            }
            continue;
        };
        if crc_ok {
            if HOPPING {
                if last_stamp != 0 {
                    let delta_us = stamp.wrapping_sub(last_stamp) / CPU_MHZ;
                    if delta_us > 200 {
                        data_phase = 0;
                        phase_valid = true;
                    } else if phase_valid {
                        data_phase = (data_phase + 1) % HOP_EVERY;
                    }
                }
                last_stamp = stamp;
            }
            if wire[0] == b'S' && wire[1] == b'T' {
                let state = u32::from_le_bytes([wire[2], wire[3], wire[4], wire[5]]);
                if have_state {
                    let delta = state.wrapping_sub(last_state);
                    if delta < 0x8000_0000 {
                        lost = lost.wrapping_add(delta.saturating_sub(1));
                    }
                }
                last_state = state;
                have_state = true;
                received = received.wrapping_add(1);
            } else {
                invalid = invalid.wrapping_add(1);
            }
            if HOPPING && phase_valid && data_phase + 1 == HOP_EVERY {
                phy.state_rx_send_feedback(0);
                hop_index = (hop_index + 1) % HOP_SEQUENCE.len();
                phy.state_set_channel(HOP_SEQUENCE[hop_index]);
                phy.state_rx_begin();
                hop_locked = true;
            }
        }

        if report_at.elapsed() >= Duration::from_secs(5) {
            let total = received.wrapping_add(lost);
            let loss_pct = if total == 0 {
                0
            } else {
                lost.saturating_mul(100) / total
            };
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let rate = received as u64 * 1_000_000 / elapsed_us;
            info!(
                "ONEWAY RX frames={} rate={}/s lost={} loss={}% invalid={} last={} hop={} locked={}",
                received, rate, lost, loss_pct, invalid, last_state, hop_index, hop_locked
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
