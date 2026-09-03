#![no_std]
#![no_main]

//! nRF54LM20 MPSL transmitter for a compile-time `OneWayState` batch.

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::{Duration, Instant};
use nrf_mpsl::{raw, MultiprotocolServiceLayer, Peripherals};
use static_cell::StaticCell;
use thunders::{phy::Phy, Address, AirTiming, OneWayState, SlotOverhead};
use thunders_phy_nrf::{
    mpsl::{one_way_mpsl_plan, MpslRadioPhy, MpslState, OneWayMpslPlan},
    RadioMode,
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
const SENDER_1: bool = cfg!(feature = "sender-1");
const MULTI_SENDER_BENCH: bool = cfg!(feature = "multi-sender-bench");
const MULTI_RECEIVER: bool = cfg!(feature = "multi-receiver");
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

async fn run_multi_receiver(mut phy: MpslRadioPhy<'static, { DATA_SLOT_US as u32 }, 1400>) -> ! {
    assert!(!PHASE_ALIGN && !HOPPING);
    assert!(phy.configure_state_senders(&[
        Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
        Address([0xC3, 0xE7, 0xE7, 0xE7, 0xE7]),
    ]));
    phy.set_channel(0).await;
    phy.configure_one_way(DATA_SLOT_US, 0, 0, false);
    let apply = phy.slot_count().wrapping_add(6);
    assert!(phy.schedule_slot_profile(
        PLAN.receiver_window_us,
        PLAN.receiver_window_us,
        1,
        1,
        0,
        apply,
    ));
    static LATEST0: StaticCell<[u8; PAYLOAD]> = StaticCell::new();
    static LATEST1: StaticCell<[u8; PAYLOAD]> = StaticCell::new();
    let latest0 = LATEST0.init([0; PAYLOAD]);
    let latest1 = LATEST1.init([0; PAYLOAD]);
    phy.publish_state_rx(latest0, apply);
    phy.publish_state_rx(latest1, apply.wrapping_add(1));
    let mut collected = apply;
    let mut last_counts = [0u32; 2];
    let mut report_at = Instant::now();
    info!(
        "MPSL MULTI RX READY receiver=LM20 window={} apply={}",
        PLAN.receiver_window_us, apply
    );
    loop {
        let _ = phy.collect_state_rx(collected).await;
        let target = collected.wrapping_add(2);
        let latest: &mut [u8; PAYLOAD] = if target & 1 == 0 { latest0 } else { latest1 };
        phy.publish_state_rx(latest, target);
        collected = collected.wrapping_add(1);
        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let mut sender0 = [0u8; PAYLOAD];
            let mut sender1 = [0u8; PAYLOAD];
            let count0 = phy.sender_state(0, &mut sender0).unwrap_or(0);
            let count1 = phy.sender_state(1, &mut sender1).unwrap_or(0);
            let rate0 = count0.wrapping_sub(last_counts[0]) as u64 * 1_000_000 / elapsed_us;
            let rate1 = count1.wrapping_sub(last_counts[1]) as u64 * 1_000_000 / elapsed_us;
            let state0 = u32::from_le_bytes([sender0[2], sender0[3], sender0[4], sender0[5]]);
            let state1 = u32::from_le_bytes([sender1[2], sender1[3], sender1[4], sender1[5]]);
            info!(
                "MPSL MULTI RX sender0={}/s state={} sender1={}/s state={}",
                rate0, state0, rate1, state1
            );
            last_counts = [count0, count1];
            report_at = Instant::now();
        }
    }
}

/// TEMP DIAG: dump the residue-suspect register basket (same dump runs at
/// boot and in the HardFault handler so the two can be diffed).
fn dump_regs(tag: &str) {
    let read = |a: u32| unsafe { (a as *const u32).read_volatile() };
    defmt::info!(
        "DUMP[{}] RRAMC 500={=u32:08x} 504={=u32:08x} 508={=u32:08x} 50c={=u32:08x} 510={=u32:08x} 514={=u32:08x} 518={=u32:08x}",
        tag,
        read(0x5004_e500),
        read(0x5004_e504),
        read(0x5004_e508),
        read(0x5004_e50c),
        read(0x5004_e510),
        read(0x5004_e514),
        read(0x5004_e518),
    );
    defmt::info!(
        "DUMP[{}] CPWR 00={=u32:08x} 04={=u32:08x} 08={=u32:08x} 0c={=u32:08x} 10={=u32:08x} 14={=u32:08x} 400={=u32:08x} 404={=u32:08x}",
        tag,
        read(0x5010_e000),
        read(0x5010_e004),
        read(0x5010_e008),
        read(0x5010_e00c),
        read(0x5010_e010),
        read(0x5010_e014),
        read(0x5010_e400),
        read(0x5010_e404),
    );
    defmt::info!(
        "DUMP[{}] GRTC 00={=u32:08x} 04={=u32:08x} 500={=u32:08x} 504={=u32:08x} 508={=u32:08x} 50c={=u32:08x} 510={=u32:08x} 514={=u32:08x}",
        tag,
        read(0x500e_2000),
        read(0x500e_2004),
        read(0x500e_2500),
        read(0x500e_2504),
        read(0x500e_2508),
        read(0x500e_250c),
        read(0x500e_2510),
        read(0x500e_2514),
    );
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
    if MULTI_RECEIVER {
        run_multi_receiver(phy).await;
    }
    assert!(
        !MULTI_SENDER_BENCH || (!PHASE_ALIGN && !HOPPING && !SENDER_1),
        "multi-sender-bench uses sender 0 without feedback/hopping"
    );
    let address = if SENDER_1 {
        Address([0xC3, 0xE7, 0xE7, 0xE7, 0xE7])
    } else {
        Address([0xE7; 5])
    };
    phy.set_address(&address).await;
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
    let mut negotiation = false;
    let mut report_at = Instant::now();
    info!(
        "MPSL ONEWAY TX READY sender={} payload=6 data={} feedback={} apply={}",
        SENDER_1 as u8, DATA_SLOT_US, FEEDBACK_SLOT_US, apply
    );

    loop {
        let hw = phy.slot_count();
        let target = hw.wrapping_add(2);
        if (target.wrapping_sub(apply) as i32) >= 0 {
            let phase = target.wrapping_sub(apply) % PERIOD_SLOTS as u32;
            let sender_slot = target.wrapping_sub(apply);
            let transmit_this_slot = !MULTI_SENDER_BENCH || sender_slot % 2 == 0;
            if phase < PLAN.batch as u32 && transmit_this_slot {
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
            } else if phase >= PLAN.batch as u32 {
                feedback.fill(0);
                phy.op_publish_rx(&mut feedback, target).await;
            }
        }
        let collected = hw.wrapping_add(1);
        if let Some(feedback_len) = phy.op_collect(collected).await {
            feedback_seen |= feedback_len == 3;
        }
        let diag_now = thunders_phy_nrf::mpsl::mpsl_pll_snapshot();
        if diag_now.negotiation != negotiation {
            negotiation = diag_now.negotiation;
            if negotiation {
                info!("MPSL ONEWAY TX recalled into negotiation; echoing ConfigFrame status");
            } else {
                info!("MPSL ONEWAY TX recall released; resuming stream");
            }
        }
        if report_at.elapsed() >= Duration::from_secs(5) {
            let elapsed_us = report_at.elapsed().as_micros().max(1);
            let tx_slot_rate = sent as u64 * 1_000_000 / elapsed_us;
            let diag = thunders_phy_nrf::mpsl::mpsl_pll_snapshot();
            let hw_tx = diag.tx_count;
            let hw_delta = hw_tx.wrapping_sub(last_hw_tx);
            let hw_rate = hw_delta as u64 * 1_000_000 / elapsed_us;
            info!(
                "MPSL ONEWAY TX published={} publish_rate={}/s hw_tx={} hw_rate={}/s state={} feedback={} hop={} locked={} neg={}",
                sent,
                tx_slot_rate,
                hw_delta,
                hw_rate,
                state_counter,
                feedback_seen,
                diag.hop_index,
                diag.hop_locked,
                diag.negotiation
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

/// Precise HardFault dump: this board intermittently escalates a UsageFault
/// during exception entry and the MPSL blob can assert at boot in a
/// layout-sensitive way; capture CFSR/BFAR and the stacked PC/registers
/// instead of the opaque default trampoline.
#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    let cfsr = unsafe { (0xE000_ED28 as *const u32).read_volatile() };
    let hfsr = unsafe { (0xE000_ED2C as *const u32).read_volatile() };
    let bfar = unsafe { (0xE000_ED38 as *const u32).read_volatile() };
    defmt::error!(
        "HARDFAULT cfsr={=u32:08x} hfsr={=u32:08x} bfar={=u32:08x} pc={=u32:08x} r0={=u32:08x} lr={=u32:08x}",
        cfsr,
        hfsr,
        bfar,
        frame.pc(),
        frame.r0(),
        frame.lr()
    );
    // TEMP DIAG: dump the null-callback struct the blob called through
    // (r0 == context pointer at the fault) and the register basket.
    let r0 = frame.r0() as usize;
    if (0x2000_0000..0x2008_0000).contains(&r0) {
        let p = r0 as *const u32;
        defmt::error!(
            "FAULTCTX @{=u32:08x}: {=u32:08x} {=u32:08x} {=u32:08x} {=u32:08x} {=u32:08x} {=u32:08x} {=u32:08x} {=u32:08x}",
            r0 as u32,
            unsafe { p.read_volatile() },
            unsafe { p.add(1).read_volatile() },
            unsafe { p.add(2).read_volatile() },
            unsafe { p.add(3).read_volatile() },
            unsafe { p.add(4).read_volatile() },
            unsafe { p.add(5).read_volatile() },
            unsafe { p.add(6).read_volatile() },
            unsafe { p.add(7).read_volatile() },
        );
    }
    dump_regs("fault");
    cortex_m::asm::bkpt();
    loop {
        cortex_m::asm::wfi();
    }
}
