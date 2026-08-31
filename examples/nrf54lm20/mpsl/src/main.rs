#![no_std]
#![no_main]

//! nRF54LM20 MPSL transmitter for `OneWayState<6, 32>`.

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::{Duration, Instant};
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals, raw};
use static_cell::StaticCell;
use thunders::{
    Address, AirTiming, FixedOneWayFrame, OneWaySender, OneWayState, SlotOverhead, phy::Phy,
};
use thunders_phy_nrf::{
    RadioMode,
    mpsl::{MpslRadioPhy, MpslState, OneWayMpslPlan, one_way_mpsl_plan},
};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SWI00 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO_0 => nrf_mpsl::HighPrioInterruptHandler;
    TIMER10 => nrf_mpsl::HighPrioInterruptHandler;
    GRTC_3 => nrf_mpsl::HighPrioInterruptHandler;
});

const PAYLOAD: usize = 6;
type Mode = OneWayState<PAYLOAD, 32>;
const PLAN: OneWayMpslPlan =
    one_way_mpsl_plan::<Mode>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);
const DATA_SLOT_US: u16 = PLAN.data_slot_us;
const FEEDBACK_SLOT_US: u16 = PLAN.feedback_slot_us;

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
    thunders_phy_nrf::hfxo_cap_trim();
    let mut config = embassy_nrf::config::Config::default();
    config.flpr_reset = embassy_nrf::config::FlprReset::Leave;
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
    config.clock_speed = embassy_nrf::config::ClockSpeed::CK128;
    let p = embassy_nrf::init(config);
    let mpsl_p = Peripherals::new(
        p.GRTC_CH7,
        p.GRTC_CH8,
        p.GRTC_CH9,
        p.GRTC_CH10,
        p.GRTC_CH11,
        p.TIMER10,
        p.TIMER20,
        p.TEMP,
        p.PPI10_CH0,
        p.PPI20_CH1,
        p.PPIB11_CH0,
        p.PPIB21_CH0,
    );
    let lfclk_cfg = raw::mpsl_clock_lfclk_cfg_t {
        source: raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 50,
        skip_wait_lfclk_started: false,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    static MPSL_MEM: StaticCell<nrf_mpsl::SessionMem<8>> = StaticCell::new();
    let mem = MPSL_MEM.init(nrf_mpsl::SessionMem::new());
    let mpsl =
        MPSL.init(MultiprotocolServiceLayer::with_timeslots(mpsl_p, Irqs, lfclk_cfg, mem).unwrap());
    static STATE: StaticCell<MpslState> = StaticCell::new();
    let state = STATE.init(MpslState::new(embassy_nrf::pac::RADIO_S, false));
    let mut phy = MpslRadioPhy::<{ DATA_SLOT_US as u32 }, 1400>::new(RadioMode::Nrf2Mbit, state);
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    phy.wait_ready().await;
    phy.set_address(&Address([0xE7; 5])).await;
    phy.set_channel(0).await;

    let apply = phy.slot_count().wrapping_add(6);
    assert!(phy.schedule_slot_profile(DATA_SLOT_US, FEEDBACK_SLOT_US, 33, 32, 0, apply,));

    let mut sender = OneWaySender::<PAYLOAD, Mode>::new();
    let mut wire = [0u8; 64];
    let mut feedback = [0u8; 64];
    let mut seq = 0u32;
    let mut sent = 0u32;
    let mut feedback_seen = false;
    let mut report_at = Instant::now();
    info!(
        "MPSL ONEWAY TX READY payload=6 data={} feedback={} apply={}",
        DATA_SLOT_US, FEEDBACK_SLOT_US, apply
    );

    loop {
        let hw = phy.slot_count();
        let target = hw.wrapping_add(2);
        if (target.wrapping_sub(apply) as i32) >= 0 {
            let phase = target.wrapping_sub(apply) % 33;
            if phase < 32 {
                let payload = [
                    b'S',
                    b'T',
                    seq as u8,
                    (seq >> 8) as u8,
                    (seq >> 16) as u8,
                    (seq >> 24) as u8,
                ];
                let frame = sender.send(payload).unwrap();
                let n = frame.encode::<PAYLOAD>(&mut wire).unwrap();
                phy.op_publish_tx(&wire[..n], target, 0).await.unwrap();
                seq = seq.wrapping_add(1);
                sent = sent.wrapping_add(1);
            } else {
                feedback.fill(0);
                phy.op_publish_rx(&mut feedback, target).await;
            }
        }
        let collected = hw.wrapping_add(1);
        if let Some(feedback_len) = phy.op_collect(collected).await {
            feedback_seen |= feedback_len < feedback.len()
                && FixedOneWayFrame::decode::<PAYLOAD>(&feedback[1..1 + feedback_len]).is_ok();
        }
        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let tx_slot_rate = sent as u64 * 1_000_000 / elapsed_us;
            info!(
                "MPSL ONEWAY TX slots={} slot_rate={}/s seq={} feedback={}",
                sent, tx_slot_rate, seq, feedback_seen
            );
            sent = 0;
            feedback_seen = false;
            report_at = Instant::now();
        }
    }
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}
