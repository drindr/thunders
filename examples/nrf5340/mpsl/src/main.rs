#![no_std]
#![no_main]

//! nRF5340 network-core sender 1 for the two-TX/one-RX benchmark.

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
    SWI0 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
});

const PAYLOAD: usize = 6;
const TX_DIVISOR: u32 = 3;
type Mode = OneWayState<PAYLOAD, 128>;
const PLAN: OneWayMpslPlan =
    one_way_mpsl_plan::<Mode, false>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);
const DATA_SLOT_US: u16 = PLAN.data_slot_us;

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    let mpsl_p = Peripherals::new(
        p.RTC0, p.TIMER0, p.TIMER1, p.TEMP, p.PPI_CH0, p.PPI_CH1, p.PPI_CH2,
    );
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
    let state = STATE.init(MpslState::new(embassy_nrf::pac::RADIO_NS));
    let mut phy = MpslRadioPhy::<{ DATA_SLOT_US as u32 }, 1400>::new(RadioMode::Nrf2Mbit, state);
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    phy.wait_ready().await;
    phy.set_address(&Address([0xC3, 0xE7, 0xE7, 0xE7, 0xE7]))
        .await;
    phy.set_channel(0).await;
    phy.configure_one_way(DATA_SLOT_US, 0, 0, false);

    let apply = phy.slot_count().wrapping_add(6);
    assert!(phy.schedule_slot_profile(DATA_SLOT_US, DATA_SLOT_US, 1, 1, 0, apply));

    let mut state_counter = 0u32;
    let mut sent = 0u32;
    let mut last_hw_tx = thunders_phy_nrf::mpsl::mpsl_pll_snapshot().tx_count;
    let mut report_at = Instant::now();
    info!(
        "MPSL MULTI TX READY sender=1 divisor={} data={} apply={}",
        TX_DIVISOR, DATA_SLOT_US, apply
    );

    loop {
        let hw = phy.slot_count();
        let target = hw.wrapping_add(2);
        if (target.wrapping_sub(apply) as i32) >= 0 && target.wrapping_sub(apply) % TX_DIVISOR == 1
        {
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
        }
        let collected = hw.wrapping_add(1);
        let _ = phy.op_collect(collected).await;

        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let published_rate = sent as u64 * 1_000_000 / elapsed_us;
            let hw_tx = thunders_phy_nrf::mpsl::mpsl_pll_snapshot().tx_count;
            let hw_delta = hw_tx.wrapping_sub(last_hw_tx);
            let hw_rate = hw_delta as u64 * 1_000_000 / elapsed_us;
            info!(
                "MPSL MULTI TX sender=1 published={}/s hw={}/s state={}",
                published_rate, hw_rate, state_counter
            );
            last_hw_tx = hw_tx;
            sent = 0;
            report_at = Instant::now();
        }
    }
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}
