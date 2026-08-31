#![no_std]
#![no_main]

//! nRF52840 MPSL long-window receiver for `OneWayState<6, 32>`.

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::{Duration, Instant};
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals, raw};
use static_cell::StaticCell;
use thunders::{Address, FixedOneWayFrame, OneWayReceiver, OneWayState, phy::Phy};
use thunders_phy_nrf::{
    RadioMode,
    mpsl::{MpslRadioPhy, MpslState},
};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
});

const PAYLOAD: usize = 6;
const BATCH: usize = 32;
const DATA_SLOT_US: u16 = 500;
const FEEDBACK_SLOT_US: u16 = 500;
const RX_WINDOW_US: u16 = 16_300;
type Mode = OneWayState<PAYLOAD, 32>;

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
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);
    let mpsl_p = Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = raw::mpsl_clock_lfclk_cfg_t {
        source: raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    static MPSL_MEM: StaticCell<nrf_mpsl::SessionMem<8>> = StaticCell::new();
    let mem = MPSL_MEM.init(nrf_mpsl::SessionMem::new());
    let mpsl =
        MPSL.init(MultiprotocolServiceLayer::with_timeslots(mpsl_p, Irqs, lfclk_cfg, mem).unwrap());
    static STATE: StaticCell<MpslState> = StaticCell::new();
    let state = STATE.init(MpslState::new(embassy_nrf::pac::RADIO, false));
    let mut phy = MpslRadioPhy::<500, 1400>::new(RadioMode::Nrf2Mbit, state);
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    phy.wait_ready().await;
    phy.set_address(&Address([0xE7; 5])).await;
    phy.set_channel(0).await;

    let apply = phy.slot_count().wrapping_add(6);
    assert!(phy.schedule_slot_profile(FEEDBACK_SLOT_US, RX_WINDOW_US, 2, 1, 1, apply,));

    static RECORDS0: StaticCell<[u8; BATCH * 64]> = StaticCell::new();
    static RECORDS1: StaticCell<[u8; BATCH * 64]> = StaticCell::new();
    let records0 = RECORDS0.init([0; BATCH * 64]);
    let records1 = RECORDS1.init([0; BATCH * 64]);
    let mut receiver = OneWayReceiver::<PAYLOAD, Mode>::new();
    let mut feedback_wire = [0u8; 64];
    let mut last_seq = 0u16;
    let mut received = 0u32;
    let mut rx_slots = 0u32;
    let mut invalid = 0u32;
    let mut report_at = Instant::now();

    info!(
        "MPSL ONEWAY RX READY payload=6 window={} feedback={} apply={}",
        RX_WINDOW_US, FEEDBACK_SLOT_US, apply
    );

    loop {
        let hw = phy.slot_count();
        let target = hw.wrapping_add(2);
        if (target.wrapping_sub(apply) as i32) >= 0 {
            let phase = target.wrapping_sub(apply) % 2;
            if phase == 0 {
                let records: &mut [u8] = if target & 1 == 0 {
                    &mut records0[..]
                } else {
                    &mut records1[..]
                };
                assert!(phy.publish_rx_batch(records, target));
            } else {
                let feedback = FixedOneWayFrame::TimeDiff {
                    seq: last_seq,
                    diff_us: 0,
                };
                let n = feedback.encode::<PAYLOAD>(&mut feedback_wire).unwrap();
                phy.op_publish_tx(&feedback_wire[..n], target, 0)
                    .await
                    .unwrap();
            }
        }

        let collected = hw.wrapping_add(1);
        if (collected.wrapping_sub(apply) as i32) >= 0 && collected.wrapping_sub(apply) % 2 == 0 {
            let count = phy.collect_rx_batch(collected).await;
            rx_slots = rx_slots.wrapping_add(1);
            let records: &[u8] = if collected & 1 == 0 {
                &records0[..]
            } else {
                &records1[..]
            };
            for cell in records.chunks_exact(64).take(count) {
                let len = cell[0] as usize;
                if let Ok(frame) = FixedOneWayFrame::decode::<PAYLOAD>(&cell[1..1 + len]) {
                    if let Ok((packet, _)) = receiver.receive(&frame, 0) {
                        last_seq = packet.seq;
                        received = received.wrapping_add(1);
                    }
                } else {
                    invalid = invalid.wrapping_add(1);
                }
            }
        } else {
            let _ = phy.op_collect(collected).await;
        }

        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let rx_slot_rate = rx_slots as u64 * 1_000_000 / elapsed_us;
            let frame_rate = received as u64 * 1_000_000 / elapsed_us;
            info!(
                "MPSL ONEWAY RX slots={} slot_rate={}/s frames={} frame_rate={}/s invalid={} last={}",
                rx_slots, rx_slot_rate, received, frame_rate, invalid, last_seq
            );
            received = 0;
            rx_slots = 0;
            invalid = 0;
            report_at = Instant::now();
        }
    }
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}
