#![no_std]
#![no_main]

//! nRF52840 MPSL long-window receiver for a compile-time `OneWayState` batch.

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
    EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
});

const PAYLOAD: usize = 6;
type Mode = OneWayState<PAYLOAD, 128>;
const PLAN: OneWayMpslPlan =
    one_way_mpsl_plan::<Mode>(AirTiming::NRF_2MBIT, SlotOverhead::MPSL_CONSERVATIVE);
const DATA_SLOT_US: u16 = PLAN.data_slot_us;
const FEEDBACK_SLOT_US: u16 = PLAN.feedback_slot_us;
const RX_WINDOW_US: u16 = PLAN.receiver_window_us;

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
    let state = STATE.init(MpslState::new(embassy_nrf::pac::RADIO));
    let mut phy = MpslRadioPhy::<{ DATA_SLOT_US as u32 }, 1400>::new(RadioMode::Nrf2Mbit, state);
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    phy.wait_ready().await;
    phy.set_address(&Address([0xE7; 5])).await;
    phy.set_channel(0).await;

    let apply = phy.slot_count().wrapping_add(6);
    phy.configure_one_way(DATA_SLOT_US, PLAN.batch);
    assert!(phy.schedule_slot_profile(RX_WINDOW_US, RX_WINDOW_US, 1, 1, 0, apply,));

    static LATEST0: StaticCell<[u8; PAYLOAD]> = StaticCell::new();
    static LATEST1: StaticCell<[u8; PAYLOAD]> = StaticCell::new();
    let latest0 = LATEST0.init([0; PAYLOAD]);
    let latest1 = LATEST1.init([0; PAYLOAD]);
    let mut last_state = 0u32;
    let mut received = 0u32;
    let mut rx_slots = 0u32;
    let mut invalid = 0u32;
    let mut last_addr = thunders_phy_nrf::mpsl::mpsl_pll_snapshot().addr_events;
    let mut last_crc_ok = thunders_phy_nrf::mpsl::mpsl_pll_snapshot().crc_ok;
    let mut last_crc_bad = thunders_phy_nrf::mpsl::mpsl_pll_snapshot().crc_bad;
    let mut report_at = Instant::now();

    info!(
        "MPSL ONEWAY RX READY payload=6 window={} feedback={} apply={}",
        RX_WINDOW_US, FEEDBACK_SLOT_US, apply
    );

    phy.publish_state_rx(latest0, apply);
    phy.publish_state_rx(latest1, apply.wrapping_add(1));
    let mut collected = apply;
    loop {
        {
            let count = phy.collect_state_rx(collected).await;
            rx_slots = rx_slots.wrapping_add(1);
            let latest: &[u8; PAYLOAD] = if collected & 1 == 0 { latest0 } else { latest1 };
            if count > 0 {
                if latest[0] == b'S' && latest[1] == b'T' {
                    last_state = u32::from_le_bytes([latest[2], latest[3], latest[4], latest[5]]);
                    received = received.wrapping_add(count as u32);
                } else {
                    invalid = invalid.wrapping_add(count as u32);
                }
            }
        }
        let target = collected.wrapping_add(2);
        let latest: &mut [u8; PAYLOAD] = if target & 1 == 0 { latest0 } else { latest1 };
        phy.publish_state_rx(latest, target);
        collected = collected.wrapping_add(1);

        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let rx_slot_rate = rx_slots as u64 * 1_000_000 / elapsed_us;
            let frame_rate = received as u64 * 1_000_000 / elapsed_us;
            let diag = thunders_phy_nrf::mpsl::mpsl_pll_snapshot();
            let addr = diag.addr_events.wrapping_sub(last_addr);
            let crc_ok = diag.crc_ok.wrapping_sub(last_crc_ok);
            let crc_bad = diag.crc_bad.wrapping_sub(last_crc_bad);
            info!(
                "MPSL ONEWAY RX slots={} slot_rate={}/s frames={} frame_rate={}/s addr={} crcok={} crcbad={} invalid={} state={}",
                rx_slots,
                rx_slot_rate,
                received,
                frame_rate,
                addr,
                crc_ok,
                crc_bad,
                invalid,
                last_state
            );
            last_addr = diag.addr_events;
            last_crc_ok = diag.crc_ok;
            last_crc_bad = diag.crc_bad;
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
