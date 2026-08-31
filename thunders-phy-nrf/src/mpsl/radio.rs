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
pub(crate) const CPU_MHZ: u32 = 64;
#[cfg(feature = "nrf52840")]
pub(crate) const CPU_MHZ: u32 = 64;
#[cfg(feature = "_nrf54")]
pub(crate) const CPU_MHZ: u32 = 128;

/// Cycle-accurate busy wait on the DWT cycle counter (embassy_time's tick
/// is 30 us - far too coarse for echo placement). Enabled once by the phy.
#[inline(always)]
pub(crate) fn cyc() -> u32 {
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
        // RSSI is not started by default on the 52840/5340; without these
        // the RSSISAMPLE register reads 0.
        s.set_address_rssistart(true);
        s.set_disabled_rssistop(true);
    }
    #[cfg(feature = "_nrf54")]
    {
        s.set_ready_start(true);
        s.set_phyend_disable(true);
    }
    s
}

/// State-window RX ramps once and returns to RXIDLE after each packet.
fn shorts_rx_state() -> regs::Shorts {
    let mut s = regs::Shorts(0);
    #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
    {
        s.set_rxready_start(true);
    }
    #[cfg(feature = "_nrf54")]
    {
        s.set_ready_start(true);
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
    state
        .done_count
        .fetch_add(1, core::sync::atomic::Ordering::Release);
    state.done_signal.signal(());
}

/// Configure the radio (inside a granted timeslot) with the current state.
/// The BARE-path wire format (CRC-16, balen 4, crcinc-exclude) is used: it is
/// the verified-working config on this 5340 (the MPSL format was deaf in RX).
unsafe fn radio_configure(state: &MpslState, statlen: u8) {
    let r = state.radio;

    // Errata 20/49 (54L): latch constant-latency for the whole session —
    // the MCU domain must never sleep around a radio start.
    #[cfg(feature = "_nrf54")]
    power_constlat();

    // MODECNF0.RU = Fast: the radio ramp drops from 129 us (Legacy) to 40 us.
    #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
    r.modecnf0().modify(|w| w.set_ru(vals::Ru::Fast));

    // Configure the selected on-air mode (the two families name the mode
    // register differently). 1M uses an 8-bit preamble; 2M needs 16 bits.
    let (mode_val, plen) = match state.radio_mode {
        crate::radio_phy::RadioMode::Nrf1Mbit => (vals::Mode::Nrf1mbit as u32, vals::Plen::_8bit),
        crate::radio_phy::RadioMode::Nrf2Mbit => (vals::Mode::Nrf2mbit as u32, vals::Plen::_16bit),
        crate::radio_phy::RadioMode::Ble1Mbit => (vals::Mode::Ble1mbit as u32, vals::Plen::_8bit),
        crate::radio_phy::RadioMode::Ble2Mbit => (vals::Mode::Ble2mbit as u32, vals::Plen::_16bit),
    };
    #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
    r.mode().write_value(regs::Mode(mode_val));
    #[cfg(feature = "_nrf54")]
    r.mode().write_value(regs::RadioMode(mode_val));

    // The mode determines the exact packet size at compile time. No S0/S1 or
    // length field is transmitted.
    let mut pcnf0 = regs::Pcnf0(0);
    pcnf0.set_lflen(0);
    pcnf0.set_plen(plen);
    pcnf0.set_crcinc(vals::Crcinc::Exclude);
    r.pcnf0().write_value(pcnf0);

    let mut pcnf1 = regs::Pcnf1(0);
    pcnf1.set_maxlen(statlen);
    pcnf1.set_statlen(statlen);
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

    r.frequency()
        .write_value(regs::Frequency(state.cur_channel as u32));

    // Board TX power ceiling: LM20 and 52840 run +8 dBm, the 5340 net core
    // is limited to 0 dBm by its RADIO frontend.
    let mut txpower = regs::Txpower(0);
    #[cfg(any(feature = "_nrf54", feature = "nrf52840"))]
    txpower.set_txpower(vals::Txpower::Pos8dBm);
    #[cfg(feature = "nrf5340-net")]
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

unsafe fn receive_state(state: &mut MpslState, ei: usize, slot_start_cyc: u32) {
    let r = state.radio;
    let latest = state.ops[ei].rx_ptr;
    let deadline = state.slot_len.saturating_sub(40) * CPU_MHZ;
    let mut count = 0usize;
    state.ops[ei].rx_ok = false;
    state.ops[ei].rx_result = 0;

    pll_enable(r);
    r.shorts().write_value(shorts_rx_state());
    r.packetptr().write_value(latest as u32);
    r.events_address().write_value(0);
    r.events_end().write_value(0);
    r.events_phyend().write_value(0);
    r.events_disabled().write_value(0);
    r.tasks_rxen().write_value(1);

    while cyc().wrapping_sub(slot_start_cyc) < deadline {
        let mut address_cyc = 0u32;
        let mut got_end = false;
        while cyc().wrapping_sub(slot_start_cyc) < deadline {
            if end_ev_set(r) {
                got_end = true;
                break;
            }
            if address_cyc == 0 && r.events_address().read() != 0 {
                address_cyc = cyc();
            }
        }
        if !got_end {
            break;
        }
        end_ev_clear(r);
        r.events_address().write_value(0);
        if r.crcstatus().read().0 & 1 == 1 {
            count += 1;
            state.crc_ok = state.crc_ok.wrapping_add(1);
            state.one_way_rx_since_feedback = state.one_way_rx_since_feedback.wrapping_add(1);
        } else {
            state.crc_bad = state.crc_bad.wrapping_add(1);
        }

        let feedback_due = state.one_way_rx_since_feedback >= state.one_way_feedback_every
            && address_cyc != 0
            && state.one_way_data_slot_us > 28;
        if feedback_due {
            let tx_at = address_cyc.wrapping_add((state.one_way_data_slot_us - 28) * CPU_MHZ);
            let wait = tx_at.wrapping_sub(cyc());
            if (wait as i32) > 0 && cyc().wrapping_sub(slot_start_cyc).wrapping_add(wait) < deadline
            {
                state.one_way_rx_since_feedback = 0;
                r.tasks_disable().write_value(1);
                disable_wait(r);
                delay_us(wait / CPU_MHZ);
                let feedback = [0u8; 2];
                radio_configure(state, 2);
                r.packetptr().write_value(feedback.as_ptr() as u32);
                r.shorts().write_value(shorts_tx());
                r.events_end().write_value(0);
                r.events_phyend().write_value(0);
                r.events_disabled().write_value(0);
                r.tasks_txen().write_value(1);
                while !end_ev_set(r) && cyc().wrapping_sub(slot_start_cyc) < deadline {}
                end_ev_clear(r);
                r.tasks_disable().write_value(1);
                disable_wait(r);
                if cyc().wrapping_sub(slot_start_cyc) >= deadline {
                    break;
                }
                radio_configure(state, 6);
                r.shorts().write_value(shorts_rx_state());
                r.packetptr().write_value(latest as u32);
                r.events_address().write_value(0);
                r.events_end().write_value(0);
                r.events_phyend().write_value(0);
                r.events_disabled().write_value(0);
                r.tasks_rxen().write_value(1);
                continue;
            }
        }

        // RADIO remains RXIDLE. Re-arm without DISABLE/PLL/RXEN/ramp.
        r.events_end().write_value(0);
        r.events_phyend().write_value(0);
        r.tasks_start().write_value(1);
    }

    r.tasks_disable().write_value(1);
    disable_wait(r);
    state.ops[ei].rx_ok = count > 0;
    state.ops[ei].rx_result = count;
}

/// Perform the pending TX/RX inside the timeslot. `exec` is the ops-ring
/// entry the callback latched at START (None for a stale/late/unpublished
/// op) - the radio never runs an op outside its target slot.
pub unsafe fn timeslot_do_work(state: &mut MpslState, exec: Option<usize>) {
    let slot_start_cyc = cyc();
    let r = state.radio;
    let kind = match exec {
        Some(i) => state.ops[i].kind,
        None => OpKind::Idle as u8,
    };
    let statlen = match kind {
        x if x == OpKind::Tx as u8 => state.ops[exec.unwrap_or(0)].tx_buf[0],
        x if x == OpKind::Rx as u8 => 2,
        x if x == OpKind::RxState as u8 => 6,
        _ => 6,
    };
    radio_configure(state, statlen);
    match kind {
        x if x == OpKind::Tx as u8 => {
            let ei = exec.unwrap_or(0);
            let buf: &mut [u8] = &mut state.ops[ei].tx_buf;
            r.packetptr().write_value(buf.as_ptr().add(1) as u32);
            state.tx_count = state.tx_count.wrapping_add(1);
            pll_enable(r);

            r.shorts().write_value(shorts_tx());
            r.events_phyend().write_value(0);
            r.events_end().write_value(0);
            r.events_ready().write_value(0);
            #[cfg(feature = "_nrf54")]
            power_constlat();
            r.tasks_txen().write_value(1);
            while !end_ev_set(r) {}
            end_ev_clear(r);
            r.events_phyend().write_value(0);
            r.tasks_disable().write_value(1);
            disable_wait(r);
        }
        x if x == OpKind::RxState as u8 => {
            receive_state(state, exec.unwrap_or(0), slot_start_cyc);
        }
        x if x == OpKind::Rx as u8 => {
            let ei = exec.unwrap_or(0);
            let rx_ptr = state.ops[ei].rx_ptr;
            let rx_cap = state.ops[ei].rx_cap;
            let buf = core::slice::from_raw_parts_mut(rx_ptr, rx_cap);
            buf[0] = 0;
            pll_enable(r);
            r.shorts().write_value(shorts_rx());
            r.packetptr().write_value(rx_ptr as u32);
            r.events_address().write_value(0);
            r.events_end().write_value(0);
            r.events_phyend().write_value(0);
            r.events_disabled().write_value(0);
            let started = cyc();
            r.tasks_rxen().write_value(1);
            let budget = state.slot_len.saturating_sub(40) * CPU_MHZ;
            let mut got_end = false;
            while cyc().wrapping_sub(started) < budget {
                if end_ev_set(r) {
                    got_end = true;
                    break;
                }
            }
            end_ev_clear(r);
            if r.events_address().read() != 0 {
                state.addr_events = state.addr_events.wrapping_add(1);
            }
            let crc_ok = r.crcstatus().read().0 & 1 == 1;
            r.tasks_disable().write_value(1);
            disable_wait(r);
            if got_end && crc_ok {
                state.crc_ok = state.crc_ok.wrapping_add(1);
                state.ops[ei].rx_ok = rx_cap >= 2;
                state.ops[ei].rx_result = 2;
            } else {
                if got_end {
                    state.crc_bad = state.crc_bad.wrapping_add(1);
                }
                state.ops[ei].rx_ok = false;
                state.ops[ei].rx_result = 0;
            }
        }
        _ => {}
    }
    signal_done(state);
}
