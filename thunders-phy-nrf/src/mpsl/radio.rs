//! The radio work performed inside a granted timeslot.

use nrf_pac::radio::regs;
use nrf_pac::radio::vals;

use super::state::{MpslState, OpKind};

type Radio = nrf_pac::radio::Radio;

// RX completion event: EVENTS_END on the 5340/52840, EVENTS_PHYEND on the LM20.
#[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
fn end_ev_set(r: Radio) -> bool {
    r.events_end().read() != 0
}
#[cfg(feature = "_nrf54")]
fn end_ev_set(r: Radio) -> bool {
    r.events_phyend().read() != 0
}
fn end_ev_clear(r: Radio) {
    #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
    r.events_end().write_value(0);
    #[cfg(feature = "_nrf54")]
    r.events_phyend().write_value(0);
}

#[cfg(feature = "nrf5340-net")]
const CPU_MHZ: u32 = 64;
#[cfg(feature = "nrf52840")]
const CPU_MHZ: u32 = 64;
#[cfg(feature = "_nrf54")]
const CPU_MHZ: u32 = 128;

/// Cycle-accurate busy wait on the DWT cycle counter (embassy_time's tick
/// is 30 us - far too coarse for echo placement). Enabled once by the phy.
#[inline(always)]
fn cyc() -> u32 {
    unsafe { (0xE000_1004 as *const u32).read_volatile() }
}

fn delay_us(us: u32) {
    let start = cyc();
    let cycles = us * CPU_MHZ;
    while cyc().wrapping_sub(start) < cycles {}
}

/// Errata 20 (54L): preamble transmits but the payload does not when the
/// MCU power domain sleeps around the radio start. Force constant latency
/// before RXEN/TXEN.
#[cfg(feature = "_nrf54")]
fn power_constlat() {
    unsafe { (0x5010_E030 as *mut u32).write_volatile(1) }; // POWER.TASKS_CONSTLAT
}

/// Bounded wait for the radio to finish disabling. ponytail: ~2000 reads
/// is tens of us; raise only if a chip's disable path proves slower.
fn disable_wait(r: Radio) {
    let mut d = 0;
    while r.events_disabled().read() == 0 && d < 2000 {
        d += 1;
    }
    r.events_disabled().write_value(0);
}

/// The RX shortcuts: the radio auto-starts once ramped (READY->START) and
/// auto-disables at the frame end (END/PHYEND->DISABLE).
fn shorts_rx() -> regs::Shorts {
    let mut s = regs::Shorts(0);
    #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
    {
        s.set_rxready_start(true);
        s.set_end_disable(true);
    }
    #[cfg(feature = "_nrf54")]
    {
        s.set_ready_start(true);
        s.set_phyend_disable(true);
    }
    s
}

/// The TX shortcuts (same idea, keyed on TXREADY on the 5340/52840).
fn shorts_tx() -> regs::Shorts {
    let mut s = regs::Shorts(0);
    #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
    {
        s.set_txready_start(true);
        s.set_end_disable(true);
    }
    #[cfg(feature = "_nrf54")]
    {
        s.set_ready_start(true);
        s.set_phyend_disable(true);
    }
    s
}

/// Mark the timeslot operation complete (called from the callback).
pub unsafe fn signal_done(state: &mut MpslState) {
    state.done_count.fetch_add(1, core::sync::atomic::Ordering::Release);
    state.done.store(true, core::sync::atomic::Ordering::Release);
    state.done_signal.signal(());
}

/// Configure the radio (inside a granted timeslot) with the current state.
/// The BARE-path wire format (CRC-16, balen 4, crcinc-exclude) is used: it is
/// the verified-working config on this 5340 (the MPSL format was deaf in RX).
unsafe fn radio_configure(state: &MpslState) {
    let r = state.radio;

    // Errata 20/49 (54L): latch constant-latency for the whole session —
    // the MCU domain must never sleep around a radio start.
    #[cfg(feature = "_nrf54")]
    power_constlat();

    // MODECNF0.RU = Fast: the radio ramp drops from 129 us (Legacy) to 40 us.
    #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
    r.modecnf0().modify(|w| w.set_ru(vals::Ru::Fast));

    // 2 Mbps (the two families name the mode register differently).
    #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
    r.mode().write_value(regs::Mode(vals::Mode::Nrf2mbit as u32));
    #[cfg(feature = "_nrf54")]
    r.mode().write_value(regs::RadioMode(vals::Mode::Nrf2mbit as u32));

    // The ESB-compatible frame: 8-bit length field, 16-bit payload length,
    // the CRC excludes the length field.
    let mut pcnf0 = regs::Pcnf0(0);
    pcnf0.set_lflen(8);
    pcnf0.set_plen(vals::Plen::_16bit);
    pcnf0.set_crcinc(vals::Crcinc::Exclude);
    r.pcnf0().write_value(pcnf0);

    // 255-byte max payload, 4-byte base address, big-endian, no whitening.
    let mut pcnf1 = regs::Pcnf1(0);
    pcnf1.set_maxlen(255);
    pcnf1.set_balen(4);
    pcnf1.set_endian(vals::Endian::Big);
    pcnf1.set_whiteen(false);
    r.pcnf1().write_value(pcnf1);

    // CRC-16 (two bytes), the address is included in the CRC.
    let mut crccnf = regs::Crccnf(0);
    crccnf.set_len(vals::Len::Two);
    crccnf.set_skipaddr(vals::Skipaddr::Include);
    r.crccnf().write_value(crccnf);

    // The CRC-16-CCITT polynomial and its init value.
    let mut crcpoly = regs::Crcpoly(0);
    crcpoly.set_crcpoly(0x11021);
    r.crcpoly().write_value(crcpoly);
    let mut crcinit = regs::Crcinit(0);
    crcinit.set_crcinit(0xFFFF);
    r.crcinit().write_value(crcinit);

    r.frequency().write_value(regs::Frequency(state.cur_channel as u32));

    // 0 dBm; the 54L runs +8: its reverse link into the 52/53 is marginal
    // at 0 dBm (address matches but CRC fails) even on-frequency.
    let mut txpower = regs::Txpower(0);
    #[cfg(feature = "_nrf54")]
    txpower.set_txpower(vals::Txpower::Pos8dBm);
    #[cfg(not(feature = "_nrf54"))]
    txpower.set_txpower(vals::Txpower::_0dBm);
    r.txpower().write_value(txpower);

    r.base0().write_value(state.cur_base0);
    r.prefix0().write_value(regs::Prefix0(state.cur_prefix));
    r.txaddress().write_value(regs::Txaddress(0));
    r.rxaddresses().write_value(regs::Rxaddresses(0x01));
}

fn pll_enable(r: Radio) {
    let _ = r;
    #[cfg(feature = "_nrf54")]
    {
        r.events_pllready().write_value(0);
        r.tasks_pllen().write_value(1);
        let mut i = 0;
        while r.events_pllready().read() == 0 {
            i += 1;
            if i > 100_000 {
                break;
            }
        }
        r.events_pllready().write_value(0);
    }
}

/// Perform the pending TX/RX inside the timeslot.
pub unsafe fn timeslot_do_work(state: &mut MpslState) {
    let r = state.radio;
    radio_configure(state);
    match state.op_kind {
        x if x == OpKind::Tx as u8 => {
            // The pending TX buffer: [0] = len, [1..=len] = payload.
            let buf: &mut [u8] = if state.tx_ptr.is_null() {
                &mut state.tx_buf
            } else {
                core::slice::from_raw_parts_mut(state.tx_ptr as *mut u8, 64)
            };
            r.packetptr().write_value(buf.as_ptr() as u32);
            // The follower's echo placement: TXEN delayed into the slot so
            // the echo lands mid-window at the peer (see callback.rs).
            if state.tx_delay_us > 0 {
                delay_us(state.tx_delay_us);
            }
            state.tx_count += 1;
            pll_enable(r);

            r.shorts().write_value(shorts_tx());
            r.events_phyend().write_value(0);
            r.events_end().write_value(0);
            #[cfg(feature = "_nrf54")]
            power_constlat();
            r.tasks_txen().write_value(1);
            let mut i = 0;
            while !end_ev_set(r) {
                i += 1;
                if i > 1_000_000 {
                    break;
                }
            }
            end_ev_clear(r);
            r.events_phyend().write_value(0);
            r.tasks_disable().write_value(1);
            disable_wait(r);
        }
        x if x == OpKind::Rx as u8 => {
            // The caller's RX slice (the radio writes into it in place).
            let buf = core::slice::from_raw_parts_mut(state.rx_ptr, state.rx_cap);
            buf.fill(0);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            pll_enable(r);
            r.shorts().write_value(shorts_rx());
            r.packetptr().write_value(state.rx_ptr as u32);
            r.events_ready().write_value(0);
            r.events_address().write_value(0);
            r.events_payload().write_value(0);
            r.events_end().write_value(0);
            r.events_disabled().write_value(0);
            r.events_phyend().write_value(0);
            #[cfg(feature = "nrf5340-net")]
            r.tasks_disable().write_value(1);
            #[cfg(feature = "_nrf54")]
            power_constlat();
            state.addr_seen = false;
            let t_rx = cyc();
            r.tasks_rxen().write_value(1);
            // The poll MUST end inside the grant: an overrun leaves the
            // callback perpetually a slot behind, the granted chain goes
            // contiguous, mpsl_low_priority_process() never returns and the
            // executor starves (the app freezes on its first RX slot).
            // Cap by time, not just count: iteration cost is chip-dependent.
            // Cycle-exact (DWT) - the embassy tick (30 us) would quantize
            // every alignment measurement past usefulness.
            let budget_cyc = state.slot_len.saturating_sub(100) * CPU_MHZ;
            // In-flight frames run past the listen budget (up to the grant's
            // own edge): breaking mid-packet truncates the END/CRC and the
            // catch is lost even though the radio is decoding fine.
            let hard_cyc = state.slot_len.saturating_sub(40) * CPU_MHZ;
            let mut i = 0;
            let mut in_flight = false;
            let mut got_end = false;
            loop {
                if end_ev_set(r) {
                    got_end = true;
                    break;
                }
                i += 1;
                if !in_flight && r.events_address().read() != 0 {
                    in_flight = true;
                    state.addr_seen = true;
                    // The phase anchor: the address event is a fixed 28 us
                    // after the frame's on-air start (16-bit preamble +
                    // 5-byte address at 2 Mbit).
                    state.addr_poll_us = (cyc() - t_rx) / CPU_MHZ;
                }
                if !in_flight && i > state.rx_poll {
                    break;
                }
                if i & 15 == 0 {
                    let el = cyc() - t_rx;
                    if el > if in_flight { hard_cyc } else { budget_cyc } {
                        break;
                    }
                }
            }
            end_ev_clear(r);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            if r.events_address().read() != 0 {
                state.addr_events += 1;
                state.last_rx_hdr.copy_from_slice(&buf[..14.min(state.rx_cap)]);
            }
            let crc = r.crcstatus().read().0;
            state.rssi_last = r.rssisample().read().rssisample() as u32;
            r.tasks_disable().write_value(1);
            // Quiesce before slot end: an unfinished disable leaks a live
            // receiver into the next slot (its END/CRC land in that slot's
            // poll setup -> instant exits, torn cross-boundary catches).
            disable_wait(r);
            // A frame is real only if the END event fired (a completed
            // frame) AND the CRC passed. The CRCSTATUS alone is not
            // trustworthy on the 5340: it reads 1 (stale) on nearly every
            // poll, so gate on END - a miss or a cap-break has no END.
            if got_end && crc & 0x1 == 0x1 {
                state.crc_ok += 1;
                let len = buf[0] as usize;
                // Valid only if the phy frame [len | payload] fits the
                // caller's buffer (receive() shifts left by one).
                state.rx_ok = len > 0 && len + 1 <= state.rx_cap;
                state.rx_result = len.min(63);
                state.catch_poll_us = (cyc() - t_rx) / CPU_MHZ; // END stamp
            } else {
                state.crc_bad += 1;
                // No catch: the poll ran to a bound, so its duration is
                // the listen window (advertised in the beacon). Only an
                // idle poll measures it (an in-flight frame that failed
                // CRC ran past the window on the in-flight extension).
                if r.events_address().read() == 0 {
                    state.rx_window_us = ((cyc() - t_rx) / CPU_MHZ).saturating_sub(40);
                }
                state.rx_ok = false;
                state.rx_result = 0;
            }
        }
        _ => {}
    }
    signal_done(state);
}
