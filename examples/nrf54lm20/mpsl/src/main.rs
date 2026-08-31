#![no_std]
#![no_main]

//! nRF54LM20 MPSL transmitter for a compile-time `OneWayState` batch.

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::{Duration, Instant};
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals, raw};
use static_cell::StaticCell;
use thunders::{Address, AirTiming, OneWayState, SlotOverhead, phy::Phy};
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
const PHASE_ALIGN: bool = cfg!(feature = "phase-align");
const HOPPING: bool = cfg!(feature = "hopping");
const FEEDBACK_EVERY: u16 = if HOPPING { 120 } else { 128 };
type Mode = OneWayState<PAYLOAD, FEEDBACK_EVERY>;
const PLAN: OneWayMpslPlan =
    one_way_mpsl_plan::<Mode, PHASE_ALIGN>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);
const DATA_SLOT_US: u16 = PLAN.data_slot_us;
const FEEDBACK_SLOT_US: u16 = PLAN.feedback_slot_us;
const PERIOD_SLOTS: u16 = PLAN.transmitter_slots();

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
    let state = STATE.init(MpslState::new(embassy_nrf::pac::RADIO_S));
    let mut phy = MpslRadioPhy::<{ DATA_SLOT_US as u32 }, 1400>::new(RadioMode::Nrf2Mbit, state);
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    phy.wait_ready().await;
    phy.set_address(&Address([0xE7; 5])).await;
    phy.set_channel(0).await;
    phy.configure_one_way(
        DATA_SLOT_US,
        FEEDBACK_SLOT_US,
        if PHASE_ALIGN { PLAN.batch } else { 0 },
        HOPPING,
    );

    let apply = phy.slot_count().wrapping_add(6);
    assert!(phy.schedule_slot_profile(
        DATA_SLOT_US,
        FEEDBACK_SLOT_US,
        PERIOD_SLOTS,
        PLAN.batch,
        0,
        apply,
    ));

    let mut feedback = [0u8; 64];
    let mut state_counter = 0u32;
    let mut sent = 0u32;
    let mut last_hw_tx = thunders_phy_nrf::mpsl::mpsl_pll_snapshot().tx_count;
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
            let phase = target.wrapping_sub(apply) % PERIOD_SLOTS as u32;
            if phase < PLAN.batch as u32 {
                let payload = [
                    b'S',
                    b'T',
                    state_counter as u8,
                    (state_counter >> 8) as u8,
                    (state_counter >> 16) as u8,
                    (state_counter >> 24) as u8,
                ];
                phy.op_publish_tx(&payload, target, 0).await.unwrap();
                state_counter = state_counter.wrapping_add(1);
                sent = sent.wrapping_add(1);
            } else {
                feedback.fill(0);
                phy.op_publish_rx(&mut feedback, target).await;
            }
        }
        let collected = hw.wrapping_add(1);
        if let Some(feedback_len) = phy.op_collect(collected).await {
            feedback_seen |= feedback_len == 2;
        }
        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let tx_slot_rate = sent as u64 * 1_000_000 / elapsed_us;
            let diag = thunders_phy_nrf::mpsl::mpsl_pll_snapshot();
            let hw_tx = diag.tx_count;
            let hw_delta = hw_tx.wrapping_sub(last_hw_tx);
            let hw_rate = hw_delta as u64 * 1_000_000 / elapsed_us;
            info!(
                "MPSL ONEWAY TX published={} publish_rate={}/s hw_tx={} hw_rate={}/s state={} feedback={} hop={} locked={}",
                sent,
                tx_slot_rate,
                hw_delta,
                hw_rate,
                state_counter,
                feedback_seen,
                diag.hop_index,
                diag.hop_locked
            );
            last_hw_tx = hw_tx;
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
