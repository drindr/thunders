#![no_std]
#![no_main]

//! thunders over MPSL timeslots on the nRF5340 net core - role-agnostic.
//! Build as the central (default) or the peripheral:
//!   cargo build --release                          # central (2 Mbit)
//!   cargo build --release --no-default-features --features peripheral
//! Add `--features radio-1m` to both nodes for the 1 Mbit mode.
//! Any thunders node can be either role; the link is full-duplex.
//!
//! The bench measures (5 s windows, the `BENCH` lines):
//!   central  — the reverse-link loss (RX slots with no echo), the payload
//!              bandwidth, the app-level RTT (PING TX slot -> echo RX).
//!   peripheral — the forward-link loss (seq gaps), the slot rate.

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::Instant;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals, raw};
use {defmt_rtt as _, panic_probe as _};
use defmt::info;
use static_cell::StaticCell;

use thunders::link::{Central, Peripheral};
use thunders::{Address, Config, Role};
#[cfg(feature = "host")]
#[path = "../../ipc.rs"]
mod ipc_mailbox;

#[cfg(feature = "host")]
use crate::ipc_mailbox::mailbox;
#[cfg(feature = "host")]
use thunders::MAX_PAYLOAD;
#[cfg(feature = "host")]
use embassy_nrf::ipc::{Ipc, IpcChannel};
use thunders_phy_nrf::mpsl::{MpslRadioPhy, MpslState};
use thunders_phy_nrf::RadioMode;

#[cfg(feature = "peripheral")]
const ROLE: Role = Role::Peripheral;
#[cfg(not(feature = "peripheral"))]
const ROLE: Role = Role::Central;

#[cfg(feature = "payload-1")]
const BENCH_PAYLOAD_LEN: usize = 1;
#[cfg(feature = "payload-4")]
const BENCH_PAYLOAD_LEN: usize = 4;
#[cfg(feature = "payload-16")]
const BENCH_PAYLOAD_LEN: usize = 16;
#[cfg(feature = "payload-32")]
const BENCH_PAYLOAD_LEN: usize = 32;
#[cfg(not(any(feature = "payload-1", feature = "payload-4", feature = "payload-16", feature = "payload-32")))]
const BENCH_PAYLOAD_LEN: usize = 8;

#[cfg(all(feature = "cadence-step-5", feature = "cadence-step-10"))]
compile_error!("select only one cadence step feature");
#[cfg(feature = "cadence-step-5")]
const CADENCE_STEP_US: u16 = 5;
#[cfg(all(not(feature = "cadence-step-5"), feature = "cadence-step-10"))]
const CADENCE_STEP_US: u16 = 10;
#[cfg(not(any(feature = "cadence-step-5", feature = "cadence-step-10")))]
const CADENCE_STEP_US: u16 = 25;

fn is_ping(payload: &[u8]) -> bool {
    payload.len() == BENCH_PAYLOAD_LEN
        && if BENCH_PAYLOAD_LEN >= 4 {
            payload[..4] == *b"PING"
        } else {
            payload[0] == b'P'
        }
}

fn is_fill(payload: &[u8]) -> bool {
    payload.len() == BENCH_PAYLOAD_LEN
        && if BENCH_PAYLOAD_LEN >= 4 {
            payload[..4] == *b"FILL"
        } else {
            payload[0] == b'F'
        }
}

/// Frame counter readable from the debugger (RTT-independent health check).
static FRAME_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}


bind_interrupts!(struct Irqs {
    SWI0 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    IPC => embassy_nrf::ipc::InterruptHandler<embassy_nrf::peripherals::IPC>;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    // Wake-driven: the low-prio IRQ wakes this task when there is work.
    // (Measured: mpsl_low_priority_process costs ~19-53 us/call - not the
    // app-latency culprit; the polling loop was also tried and no better.)
    mpsl.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("thunders MPSL (nRF5340 net core, {:?})", defmt::Debug2Format(&ROLE));

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    // The net core's LFCLK XTAL never starts here (the app core owns it);
    // the thunders timing is free-running, so the RC is fine.
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    // IPC to the app core (channel 1 = net -> app "RX ready"). The net core
    // polls the shared mailbox for the app -> net direction, so no IPC wait.
    // Without the host (the standalone bench) the mailbox region is secure
    // and any access faults, so this is host-only.
    #[cfg(feature = "host")]
    let mut ipc = Ipc::new(p.IPC, Irqs);
    #[cfg(feature = "host")]
    {
        ipc.event1.configure_trigger([IpcChannel::Channel1]);
        mailbox().tx.valid = 0;
        mailbox().rx.valid = 0;
    }

    let mpsl_p = Peripherals::new(
        p.RTC0, p.TIMER0, p.TIMER1, p.TEMP,
        p.PPI_CH0, p.PPI_CH1, p.PPI_CH2,
    );

    let lfclk_cfg = raw::mpsl_clock_lfclk_cfg_t {
        source: raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    // The MPSL layer is initialized WITH timeslot support: the SessionMem is
    // passed to the constructor, which configures the session count internally
    // (the phy then opens its own session within that pool).
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    static MPSL_MEM: StaticCell<nrf_mpsl::SessionMem<8>> = StaticCell::new();
    let mem = MPSL_MEM.init(nrf_mpsl::SessionMem::new());
    let mpsl = MPSL.init(MultiprotocolServiceLayer::with_timeslots(mpsl_p, Irqs, lfclk_cfg, mem).unwrap());

    // The MPSL phy opens its timeslot session and inserts the first
    // (EARLIEST) request BEFORE the mpsl_task starts processing.
    let radio = embassy_nrf::pac::RADIO_NS;
    // 'static: the phy hands this pointer to the MPSL callback (ISR).
    static STATE: StaticCell<MpslState> = StaticCell::new();
    let state = STATE.init(MpslState::new(radio, cfg!(feature = "peripheral")));
    #[cfg(feature = "radio-1m")]
    let mode = RadioMode::Nrf1Mbit;
    #[cfg(not(feature = "radio-1m"))]
    let mode = RadioMode::Nrf2Mbit;
    let phy = MpslRadioPhy::<500, 1400>::new(mode, state);
    let _ = spawner.spawn(mpsl_task(mpsl).expect("spawn"));
    phy.wait_ready().await;
    info!("MPSL ready");

    let cfg = Config::new(
        [0xAB, 0xCD, 0xEF, 0x01],
        Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
        ROLE,
    );
    #[cfg(feature = "ratio-8-4-4")]
    let cfg = cfg.with_tx_rx_idle(8, 4, 4);
    #[cfg(feature = "ratio-6-2-2")]
    let cfg = cfg.with_tx_rx_idle(6, 2, 2);
    #[cfg(feature = "ratio-4-2-2")]
    let cfg = cfg.with_tx_rx_idle(4, 2, 2);

    let (tx_n, rx_n) = cfg.tx_rx_ratio;
    let idle_n = cfg.idle_slots;

    match ROLE {
        Role::Central => {
            let period = 1 + tx_n as u64 + rx_n as u64 + idle_n as u64;
            let mut central = Central::new(phy, cfg).await.unwrap();
            info!("link ready (Central)");
            #[cfg(feature = "cadence-probe")]
            let mut cadence_requested = false;
            #[cfg(feature = "cadence-probe")]
            let mut cadence_attempts: u8 = 0;
            #[cfg(feature = "cadence-probe")]
            let mut cadence_retry_at: Option<Instant> = None;
            let mut cadence_draining = false;
            #[cfg(feature = "cadence-probe")]
            let mut cadence_reported = false;
            #[cfg(feature = "cadence-probe")]
            let mut cadence_stable_at: Option<Instant> = None;
            #[cfg(feature = "cadence-probe")]
            let mut cadence_exit_requested = false;
            #[cfg(feature = "cadence-probe")]
            let mut cadence_released = false;
            #[cfg(feature = "cadence-probe")]
            let mut cadence_last_bounds: Option<thunders::CadenceProbeBounds> = None;
            #[cfg(feature = "cadence-probe")]
            let mut cadence_last_candidate: Option<thunders::CadenceProfile> = None;

            let mut rx_buf = [0u8; 32];
            let mut frames: u64 = 0;
            let mut ping_tx: u64 = 0;
            let mut echo_rx: u64 = 0;
            let mut rev_lost: u64 = 0;
            // Duplicate echoes (app seq does not advance); ~0 with
            // one-shot echo semantics, kept as a regression check.
            let mut dup: u64 = 0;
            // FILL payloads received (reverse saturation traffic; not
            // echoes).
            let mut fill_rx: u64 = 0;
            // Monotonic across windows: the echo gap accounting needs a
            // stable seq space (a per-window restart makes every new
            // window's echoes look like backward jumps).
            let mut ping_seq: u32 = 0;
            // TX-phase slot count (the rloss denominator).
            let mut tx_frames: u64 = 0;
            // NOTE: offer-rate thinning (slot % N) was tried here and
            // starved the MPSL link (A/B verified) - the central offers at
            // full rate.
            let mut last_echo_seq: u32 = 0;
            let mut rtt_sum: u64 = 0;
            let mut rtt_min: u32 = u32::MAX;
            let mut rtt_max: u32 = 0;
            let mut busy_total: u64 = 0;
            let mut busy_max: u64 = 0;
            let mut t_ping_tx = Instant::now();
            // delivery_failures() at the window start, for the ping-pong pacing.
            let mut df_base: u32 = 0;
            let mut report_at = Instant::now();
            info!("BENCH READY role=C ratio={},{} payload={}B", tx_n, rx_n, BENCH_PAYLOAD_LEN);

            loop {
                #[cfg(feature = "cadence-probe")]
                if cadence_requested
                    && !cadence_reported
                    && matches!(
                        central.cadence_status(),
                        thunders::CadenceNegotiationStatus::Failed(
                            thunders::CadenceError::ControlTimeout
                        )
                    )
                    && cadence_attempts < 4
                {
                    let since = cadence_retry_at.get_or_insert_with(Instant::now);
                    if since.elapsed() >= embassy_time::Duration::from_secs(2) {
                        info!("CADENCE RETRY after attempt={}", cadence_attempts);
                        cadence_requested = false;
                        cadence_draining = true;
                        cadence_retry_at = None;
                        cadence_last_bounds = None;
                        cadence_last_candidate = None;
                    }
                }
                #[cfg(feature = "cadence-probe")]
                if !cadence_requested && central.status() == thunders::link::LinkStatus::Connected {
                    cadence_draining = true;
                    let policy = thunders::CadenceProbePolicy::new(300, CADENCE_STEP_US, 32, 0);
                    if let Ok(generation) = central.negotiate_cadence(
                        thunders::TrafficContract::new(BENCH_PAYLOAD_LEN as u16, BENCH_PAYLOAD_LEN as u16),
                        policy,
                    ) {
                        cadence_requested = true;
                        cadence_attempts = cadence_attempts.saturating_add(1);
                        cadence_retry_at = None;
                        cadence_draining = false;
                        info!("CADENCE REQUEST gen={} attempt={}", generation, cadence_attempts);
                    }
                }
                #[cfg(feature = "cadence-probe")]
                if let Some(bounds) = central.cadence_probe_bounds() {
                    if cadence_last_bounds != Some(bounds) {
                        cadence_last_bounds = Some(bounds);
                        let peer = bounds.peer.unwrap_or((0, 0));
                        info!(
                            "CADENCE BOUNDS local={}/{} peer={}/{} effective={}/{}",
                            bounds.local.0,
                            bounds.local.1,
                            peer.0,
                            peer.1,
                            bounds.effective.0,
                            bounds.effective.1
                        );
                    }
                }
                #[cfg(feature = "cadence-probe")]
                if let Some(candidate) = central.cadence_candidate() {
                    if cadence_last_candidate != Some(candidate) {
                        cadence_last_candidate = Some(candidate);
                        info!(
                            "CADENCE CANDIDATE short={} long={}",
                            candidate.short_slot_us, candidate.long_slot_us
                        );
                    }
                }
                #[cfg(feature = "cadence-probe")]
                if cadence_requested && !cadence_reported {
                    if let thunders::CadenceNegotiationStatus::Stable(profile) =
                        central.cadence_status()
                    {
                        cadence_reported = true;
                        cadence_stable_at = Some(Instant::now());
                        info!("CADENCE STABLE short={} long={}", profile.short_slot_us, profile.long_slot_us);
                    }
                }
                #[cfg(feature = "cadence-probe")]
                if !cfg!(feature = "cadence-hold") && cadence_reported && !cadence_exit_requested {
                    if let Some(at) = cadence_stable_at {
                        if at.elapsed() >= embassy_time::Duration::from_secs(3) {
                            if let Ok(generation) = central.exit_cadence() {
                                cadence_exit_requested = true;
                                info!("CADENCE EXIT gen={}", generation);
                            }
                        }
                    }
                }
                #[cfg(feature = "cadence-probe")]
                if cadence_exit_requested
                    && !cadence_released
                    && matches!(central.cadence_status(), thunders::CadenceNegotiationStatus::Idle)
                {
                    cadence_released = true;
                    info!("CADENCE RELEASED");
                }
                // TX slots carry a fresh PING (seq per PING, beacons skipped),
                // RX slots listen.
                let mut p = [b'P'; 32];
                if BENCH_PAYLOAD_LEN >= 4 {
                    p[..4].copy_from_slice(b"PING");
                }
                let slot = thunders_phy_nrf::mpsl::mpsl_slot_count().wrapping_add(1);
                let phase = slot % period as u32;
                let tx_phase = (1..=tx_n as u32).contains(&phase) && slot % 64 != 0;
                if tx_phase {
                    tx_frames += 1;
                }
                // Ping-pong pacing: one outstanding PING at a time. The
                // peripheral's single echo buffer is consumed once per
                // period; offering faster overwrites pending echoes (ow)
                // and rev_loss measures the 8:2 capacity mismatch, not
                // the radio. A dropped PING (df) frees its slot.
                let echo_pending =
                    ping_tx > echo_rx + central.delivery_failures().saturating_sub(df_base) as u64;
                let tx: Option<&[u8]> =
                    if tx_phase && !cadence_draining && !echo_pending && !central.tx_window_full() {
                    ping_tx += 1;
                    ping_seq = ping_seq.wrapping_add(1);
                    t_ping_tx = Instant::now();
                    if BENCH_PAYLOAD_LEN >= 8 {
                        p[4..8].copy_from_slice(&ping_seq.to_le_bytes());
                    }
                    Some(&p[..BENCH_PAYLOAD_LEN])
                } else {
                    None
                };
                let t_start = Instant::now();
                match central.frame(tx, &mut rx_buf).await {
                    Ok(Some(n)) => {
                        // Forward the received payload to the app core (host only).
                        #[cfg(feature = "host")]
                        {
                            mailbox().put_rx(&rx_buf[..n]);
                            ipc.event1.trigger();
                        }
                        if is_fill(&rx_buf[..n]) {
                            fill_rx += 1;
                        }
                        if is_ping(&rx_buf[..n]) {
                            // The echo of the last PING arrived on an RX slot:
                            // the RTT is measured from that PING's TX slot.
                            echo_rx += 1;
                            if BENCH_PAYLOAD_LEN >= 8 {
                                let seq = u32::from_le_bytes([
                                    rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7],
                                ]);
                                let gap = seq.wrapping_sub(last_echo_seq);
                                if echo_rx == 1 || gap < 1_000_000 {
                                    if echo_rx > 1 {
                                        if gap == 0 {
                                            dup += 1;
                                        } else if gap > 1 {
                                            rev_lost += (gap - 1) as u64;
                                        }
                                    }
                                    last_echo_seq = seq;
                                }
                            }
                            let rtt = t_ping_tx.elapsed().as_micros() as u32;
                            rtt_sum += rtt as u64;
                            if rtt < rtt_min {
                                rtt_min = rtt;
                            }
                            if rtt > rtt_max {
                                rtt_max = rtt;
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(thunders::Error::WindowFull) => {}
                Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
                }
                let busy = t_start.elapsed().as_micros() as u64;
                busy_total += busy;
                if busy > busy_max {
                    busy_max = busy;
                }
                frames += 1;
                FRAME_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if frames % 5000 == 0 {
                    info!("HB f={}", frames);
                }

                let now = Instant::now();
                if now - report_at >= embassy_time::Duration::from_secs(5) {
                    let elapsed = (now - report_at).as_micros() as u64;
                    let rate = frames * 1_000_000 / elapsed.max(1);
                    let avg_busy = busy_total / frames.max(1);
                    // The reverse-link loss: RX slots that caught no echo.
                    let rx_slots = frames - tx_frames;
                    // A reverse hit is any payload from the peripheral
                    // (echo or filler).
                    let rev_rx = echo_rx + fill_rx;
                    let rloss = if rx_slots > 0 {
                        rx_slots.saturating_sub(rev_rx) * 100 / rx_slots
                    } else {
                        0
                    };
                    // Payload throughput: configured bytes per PING plus
                    // configured bytes per reverse packet.
                    let bw = (ping_tx + rev_rx) * BENCH_PAYLOAD_LEN as u64 * 1_000_000
                        / elapsed.max(1);
                    let (ra, rmin, rmax) = if echo_rx > 0 {
                        (rtt_sum / echo_rx, rtt_min, rtt_max)
                    } else {
                        (0, 0, 0)
                    };
                    let rev_loss = if echo_rx + rev_lost > 0 {
                        rev_lost * 100 / (echo_rx + rev_lost)
                    } else {
                        0
                    };
                    info!("BENCH C slots={} tx={} rx={} rloss={}% rate={}/s bw={}B/s rtt_avg={}us rtt_min={}us rtt_max={}us busy={}us busy_max={}us df={} wf={} rt={} rev_lost={} rev_loss={}% dup={} fill={} rxd={} txd={}", frames, ping_tx, echo_rx, rloss, rate, bw, ra, rmin, rmax, avg_busy, busy_max, central.delivery_failures(), central.window_full(), central.retransmits(), rev_lost, rev_loss, dup, fill_rx, central.rx_data(), central.tx_data());
                    let pll = thunders_phy_nrf::mpsl::mpsl_pll_snapshot();
                    let rssi = thunders_phy_nrf::mpsl::mpsl_rssi();
                    let trace = thunders_phy_nrf::mpsl::mpsl_probe_trace();
                    if trace.slot[1] != 0 {
                        info!("PROBE TRACE slot={:?} phase={:?} nominal={:?} kind={:?} target={:?} event={:?}", trace.slot, trace.phase, trace.nominal, trace.exec_kind, trace.exec_target, trace.event_us);
                    }
                    info!("PLL dist={} catch={} w={} peerw={} addr={} ai={} txc={} d8={} mis={} crcok={} crcbad={} target={} calib={} sfq8={} sanc={} rssi={} rxo={} rxr={} txo={} txr={} prxo={} prxr={} ptxo={} ptxr={} hdr={:?} end={} got_end={} sl={} crc={} infl={} lai={} len={} crcbadl={} txs={} txl={} txp={:?} txph={:?} rxph={:?} rsum={} rcnt={} rmax={} late={} grc={} pub={} cn={} cl={} cc={} ce={} off={} hwp={}", pll.distance_us, pll.catch_poll_us, pll.rx_window_us, pll.peer_rx_window_us, pll.addr_events, pll.addr_poll_us, pll.tx_count, pll.tx_delay_us, pll.rx_misses, pll.crc_ok, pll.crc_bad, pll.addr_target_us, pll.calib_count, pll.sync_freq_q8, pll.sync_anchor_count, rssi, pll.rx_en_offset_us, pll.rx_ramp_us, pll.tx_en_offset_us, pll.tx_ramp_us, pll.peer_rx_en_offset_us, pll.peer_rx_ramp_us, pll.peer_tx_en_offset_us, pll.peer_tx_ramp_us, pll.last_rx_hdr, pll.last_rx_end_us, pll.last_rx_got_end, pll.last_rx_slot_len, pll.last_rx_crc, pll.last_rx_in_flight, pll.last_rx_addr_us, pll.last_rx_len, pll.crc_bad_long, pll.tx_short, pll.tx_long, pll.tx_long_phase, pll.tx_phase_all, pll.rx_phase_all, pll.rssi_catch_sum, pll.rssi_catch_cnt, pll.rssi_catch_max, pll.op_late, pll.op_grace_used, pll.op_publish_max_us, pll.coll_noop, pll.coll_late, pll.coll_catch, pll.coll_empty, central.slot_offset(), central.hw_phase());
                    frames = 0;
                    ping_tx = 0;
                    tx_frames = 0;
                    echo_rx = 0;
                    rev_lost = 0;
                    dup = 0;
                    fill_rx = 0;
                    rtt_sum = 0;
                    rtt_min = u32::MAX;
                    rtt_max = 0;
                    df_base = central.delivery_failures();
                    busy_total = 0;
                    busy_max = 0;
                    report_at = now;
                }
            }
        }
        Role::Peripheral => {
            let mut peripheral = Peripheral::new(phy, cfg).await.unwrap();
            info!("link ready (Peripheral)");

            let mut rx_buf = [0u8; 32];
            let mut echo = [0u8; 32];
            let mut echo_len = 0usize;
            // Echoes overwritten before ever reaching the TX window
            // (app-layer backpressure loss).
            let mut ow: u64 = 0;
            // Filler traffic keeps the reverse link saturated without
            // polluting the echo stream: each echo is enqueued at most
            // once (tracked via offer_taken), the FILL prefix uses an
            // independent seq space.
            #[cfg(not(feature = "host"))]
            let mut fill = [b'F'; 32];
            if BENCH_PAYLOAD_LEN >= 4 {
                fill[..4].copy_from_slice(b"FILL");
            }
            #[cfg(not(feature = "host"))]
            let mut fill_seq = 0u32;
            let mut frames: u64 = 0;
            let mut rx_ok: u64 = 0;
            let mut fwd_lost: u64 = 0;
            let mut last_seq: u32 = 0;
            let mut busy_total: u64 = 0;
            let mut report_at = Instant::now();
            #[cfg(feature = "host")]
            let mut ipc_tx = [0u8; MAX_PAYLOAD];
            info!("BENCH READY role=P ratio={},{} payload={}B", tx_n, rx_n, BENCH_PAYLOAD_LEN);

            loop {
                let t_frame = Instant::now();
                // The app core's payload takes priority, else the echo
                // (host only).
                let mut offered_echo = false;
                #[cfg(feature = "host")]
                let tx: Option<&[u8]> = {
                    if !peripheral.tx_window_full() {
                        if let Some(m) = mailbox().take_tx(&mut ipc_tx) {
                            Some(&ipc_tx[..m])
                        } else if echo_len > 0 {
                            offered_echo = true;
                            Some(&echo[..echo_len])
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                };
                #[cfg(not(feature = "host"))]
                let tx: Option<&[u8]> = {
                    offered_echo = echo_len > 0 && !peripheral.tx_window_full();
                    if offered_echo {
                        Some(&echo[..echo_len])
                    } else if peripheral.tx_inflight() < 8 {
                        if BENCH_PAYLOAD_LEN >= 8 {
                            fill[4..8].copy_from_slice(&fill_seq.to_le_bytes());
                        }
                        Some(&fill[..BENCH_PAYLOAD_LEN])
                    } else {
                        None
                    }
                };
                match peripheral.frame(tx, &mut rx_buf).await {
                    Ok(Some(n)) => {
                        // Forward the received payload to the app core (host only).
                        #[cfg(feature = "host")]
                        {
                            mailbox().put_rx(&rx_buf[..n]);
                            ipc.event1.trigger();
                        }
                        if is_ping(&rx_buf[..n]) {
                            if BENCH_PAYLOAD_LEN >= 8 {
                                let seq = u32::from_le_bytes([
                                    rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7],
                                ]);
                                if rx_ok > 1 {
                                    let gap = seq.wrapping_sub(last_seq);
                                    if gap > 1 && gap < 1_000_000 {
                                        fwd_lost += (gap - 1) as u64;
                                    }
                                }
                                last_seq = seq;
                            }
                            rx_ok += 1;
                            // The previous echo never reached the TX window
                            // (persistent backpressure): an app-layer loss.
                            if echo_len > 0 {
                                ow += 1;
                            }
                            echo[..n].copy_from_slice(&rx_buf[..n]);
                            echo_len = n;
                        }
                    }
                    Ok(None) => {}
                    Err(thunders::Error::WindowFull) => {}
                Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
                }
                // One-shot echo: consumed by the link layer at most once,
                // then the slot offers filler instead.
                if peripheral.offer_taken() {
                    #[cfg(not(feature = "host"))]
                    if !offered_echo {
                        fill_seq = fill_seq.wrapping_add(1);
                    }
                    if offered_echo {
                        echo_len = 0;
                    }
                }
                frames += 1;
                busy_total += t_frame.elapsed().as_micros() as u64;
                FRAME_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                let now = Instant::now();
                if now - report_at >= embassy_time::Duration::from_secs(5) {
                    let elapsed = (now - report_at).as_micros() as u64;
                    let rate = frames * 1_000_000 / elapsed.max(1);
                    let avg_busy = busy_total / frames.max(1);
                    let floss = if rx_ok + fwd_lost > 0 {
                        fwd_lost * 100 / (rx_ok + fwd_lost)
                    } else {
                        0
                    };
                    info!("BENCH P slots={} rx={} lost={} floss={}% rate={}/s busy={}us ow={} rxd={} txd={} df={} retx={}", frames, rx_ok, fwd_lost, floss, rate, avg_busy, ow, peripheral.rx_data(), peripheral.tx_data(), peripheral.delivery_failures(), peripheral.retransmits());
                    let pll = thunders_phy_nrf::mpsl::mpsl_pll_snapshot();
                    let rssi = thunders_phy_nrf::mpsl::mpsl_rssi();
                    let trace = thunders_phy_nrf::mpsl::mpsl_probe_trace();
                    if trace.slot[1] != 0 {
                        info!("PROBE TRACE slot={:?} phase={:?} nominal={:?} kind={:?} target={:?} event={:?}", trace.slot, trace.phase, trace.nominal, trace.exec_kind, trace.exec_target, trace.event_us);
                    }
                    info!("PLL dist={} catch={} w={} peerw={} addr={} ai={} txc={} d8={} mis={} crcok={} crcbad={} target={} calib={} sfq8={} sanc={} rssi={} rxo={} rxr={} txo={} txr={} prxo={} prxr={} ptxo={} ptxr={} hdr={:?} end={} got_end={} sl={} crc={} infl={} lai={} len={} crcbadl={} txs={} txl={} txp={:?} txph={:?} rxph={:?} rsum={} rcnt={} rmax={} late={} grc={} pub={} cn={} cl={} cc={} ce={} off={} hwp={}", pll.distance_us, pll.catch_poll_us, pll.rx_window_us, pll.peer_rx_window_us, pll.addr_events, pll.addr_poll_us, pll.tx_count, pll.tx_delay_us, pll.rx_misses, pll.crc_ok, pll.crc_bad, pll.addr_target_us, pll.calib_count, pll.sync_freq_q8, pll.sync_anchor_count, rssi, pll.rx_en_offset_us, pll.rx_ramp_us, pll.tx_en_offset_us, pll.tx_ramp_us, pll.peer_rx_en_offset_us, pll.peer_rx_ramp_us, pll.peer_tx_en_offset_us, pll.peer_tx_ramp_us, pll.last_rx_hdr, pll.last_rx_end_us, pll.last_rx_got_end, pll.last_rx_slot_len, pll.last_rx_crc, pll.last_rx_in_flight, pll.last_rx_addr_us, pll.last_rx_len, pll.crc_bad_long, pll.tx_short, pll.tx_long, pll.tx_long_phase, pll.tx_phase_all, pll.rx_phase_all, pll.rssi_catch_sum, pll.rssi_catch_cnt, pll.rssi_catch_max, pll.op_late, pll.op_grace_used, pll.op_publish_max_us, pll.coll_noop, pll.coll_late, pll.coll_catch, pll.coll_empty, peripheral.slot_offset(), peripheral.hw_phase());
                    frames = 0;
                    rx_ok = 0;
                    fwd_lost = 0;
                    ow = 0;
                    busy_total = 0;
                    report_at = now;
                }
            }
        }
    }
}
