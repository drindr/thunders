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
unsafe fn radio_configure(state: &MpslState) {
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

    // The ESB-compatible frame: 8-bit length field, mode-sized preamble,
    // the CRC excludes the length field.
    let mut pcnf0 = regs::Pcnf0(0);
    pcnf0.set_lflen(8);
    pcnf0.set_plen(plen);
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

unsafe fn receive_batch(state: &mut MpslState, ei: usize, slot_start_cyc: u32) {
    let r = state.radio;
    let base = state.ops[ei].rx_ptr;
    let cells = state.ops[ei].rx_cap / 64;
    let deadline_cyc = state.slot_len.saturating_sub(40) * CPU_MHZ;
    let mut count = 0usize;
    state.ops[ei].rx_ok = false;
    state.ops[ei].rx_result = 0;
    state.addr_seen = false;

    while count < cells && cyc().wrapping_sub(slot_start_cyc) < deadline_cyc {
        let cell = base.add(count * 64);
        core::slice::from_raw_parts_mut(cell, 64).fill(0);
        pll_enable(r);
        r.shorts().write_value(shorts_rx());
        r.packetptr().write_value(cell as u32);
        r.events_ready().write_value(0);
        r.events_address().write_value(0);
        r.events_end().write_value(0);
        r.events_phyend().write_value(0);
        r.events_disabled().write_value(0);
        let t_rx = cyc();
        r.tasks_rxen().write_value(1);

        let mut got_end = false;
        let mut address_cyc = 0u32;
        while cyc().wrapping_sub(slot_start_cyc) < deadline_cyc {
            if end_ev_set(r) {
                got_end = true;
                break;
            }
            if address_cyc == 0 && r.events_address().read() != 0 {
                address_cyc = cyc();
                state.addr_seen = true;
                state.addr_poll_us = address_cyc.wrapping_sub(t_rx) / CPU_MHZ;
            }
        }
        end_ev_clear(r);
        let crc_ok = r.crcstatus().read().0 & 1 == 1;
        r.tasks_disable().write_value(1);
        disable_wait(r);
        if !got_end {
            break;
        }
        if crc_ok {
            let len = *cell as usize;
            if len > 0 && len < 64 {
                count += 1;
                state.crc_ok = state.crc_ok.wrapping_add(1);
                // No-ACK alignment feedback is tied to the packet itself,
                // not to an independently phased receiver slot chain. After
                // every 32nd fixed Data frame, transmit TimeDiff at the next
                // configured sender event boundary while still in this grant.
                if len >= 3
                    && *cell.add(1) == 0xF0
                    && address_cyc != 0
                    && state.one_way_data_slot_us > 28
                {
                    let seq = u16::from_le_bytes([*cell.add(2), *cell.add(3)]);
                    if seq & 31 == 31 {
                        let tx_at =
                            address_cyc.wrapping_add((state.one_way_data_slot_us - 28) * CPU_MHZ);
                        let wait_cyc = tx_at.wrapping_sub(cyc());
                        if (wait_cyc as i32) > 0
                            && cyc().wrapping_sub(slot_start_cyc).wrapping_add(wait_cyc)
                                < deadline_cyc
                        {
                            delay_us(wait_cyc / CPU_MHZ);
                            let feedback = [5u8, 0xF2, seq as u8, (seq >> 8) as u8, 0, 0];
                            r.packetptr().write_value(feedback.as_ptr() as u32);
                            r.shorts().write_value(shorts_tx());
                            r.events_end().write_value(0);
                            r.events_phyend().write_value(0);
                            r.events_disabled().write_value(0);
                            r.tasks_txen().write_value(1);
                            while !end_ev_set(r)
                                && cyc().wrapping_sub(slot_start_cyc) < deadline_cyc
                            {
                            }
                            end_ev_clear(r);
                            r.tasks_disable().write_value(1);
                            disable_wait(r);
                        }
                    }
                }
            }
        } else {
            state.crc_bad = state.crc_bad.wrapping_add(1);
        }
    }
    state.ops[ei].rx_ok = count > 0;
    state.ops[ei].rx_result = count;
    state.catch_poll_us = cyc().wrapping_sub(slot_start_cyc) / CPU_MHZ;
}

/// Perform the pending TX/RX inside the timeslot. `exec` is the ops-ring
/// entry the callback latched at START (None for a stale/late/unpublished
/// op) - the radio never runs an op outside its target slot.
pub unsafe fn timeslot_do_work(state: &mut MpslState, exec: Option<usize>) {
    let slot_start_cyc = cyc();
    let r = state.radio;
    radio_configure(state);
    let kind = match exec {
        Some(i) => state.ops[i].kind,
        None => OpKind::Idle as u8,
    };
    match kind {
        x if x == OpKind::Tx as u8 => {
            // The pending TX buffer: [0] = len, [1..=len] = payload.
            let ei = exec.unwrap_or(0);
            let air = state.airtime_us(state.ops[ei].tx_buf[0] as usize);
            let buf: &mut [u8] = &mut state.ops[ei].tx_buf;
            r.packetptr().write_value(buf.as_ptr() as u32);
            // The follower's echo placement: TXEN delayed into the slot so
            // the echo lands mid-window at the peer (see callback.rs).
            state.tx_pre_delay_us = (cyc() - slot_start_cyc) / CPU_MHZ;
            let mut delay = state.tx_delay_us;
            if state.tx_delay_sweep {
                // The follower is still trying to deliver SlotRequest. Walk
                // the TX delay across the peer's RX window; one of these
                // positions will land regardless of the initial slot phase.
                const SWEEP: [u32; 8] = [0, 30, 60, 90, 120, 150, 180, 210];
                let setup = state.tx_pre_delay_us;
                let ramp = if state.tx_ramp_us > 0 {
                    state.tx_ramp_us
                } else {
                    40
                };
                let max_delay = state.slot_len.saturating_sub(setup + ramp + air + 40);
                delay = SWEEP[(state.tx_delay_sweep_step as usize) % SWEEP.len()].min(max_delay);
                state.tx_delay_sweep_step =
                    state.tx_delay_sweep_step.wrapping_add(1) % SWEEP.len() as u8;
            }
            if delay > 0 {
                delay_us(delay);
            }
            state.tx_en_offset_us = (cyc() - slot_start_cyc) / CPU_MHZ;
            state.tx_count += 1;
            {
                let phase = (state.slot_count % 10) as usize;
                state.tx_phase_all[phase] += 1;
            }
            if buf[0] >= 11 {
                state.tx_long += 1;
                let phase = (state.slot_count % 10) as usize;
                state.tx_long_phase[phase] += 1;
            } else {
                state.tx_short += 1;
            }
            pll_enable(r);

            r.shorts().write_value(shorts_tx());
            r.events_phyend().write_value(0);
            r.events_end().write_value(0);
            r.events_ready().write_value(0);
            #[cfg(feature = "_nrf54")]
            power_constlat();
            let t_txen = cyc();
            r.tasks_txen().write_value(1);
            let mut i = 0;
            let mut ready_us = 0u32;
            while !end_ev_set(r) {
                i += 1;
                if ready_us == 0 && r.events_ready().read() != 0 {
                    ready_us = (cyc() - t_txen) / CPU_MHZ;
                }
                if i > 1_000_000 {
                    break;
                }
            }
            state.tx_ramp_us = ready_us;
            end_ev_clear(r);
            r.events_phyend().write_value(0);
            r.tasks_disable().write_value(1);
            disable_wait(r);
        }
        x if x == OpKind::RxBatch as u8 => {
            receive_batch(state, exec.unwrap_or(0), slot_start_cyc);
        }
        x if x == OpKind::Rx as u8 => {
            {
                let phase = (state.slot_count % 10) as usize;
                state.rx_phase_all[phase] += 1;
            }
            // The caller's RX slice (the radio writes into it in place).
            let ei = exec.unwrap_or(0);
            let rx_ptr = state.ops[ei].rx_ptr;
            let rx_cap = state.ops[ei].rx_cap;
            let buf = core::slice::from_raw_parts_mut(rx_ptr, rx_cap);
            buf.fill(0);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            pll_enable(r);
            r.shorts().write_value(shorts_rx());
            r.packetptr().write_value(rx_ptr as u32);
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
            state.rx_en_offset_us = t_rx.wrapping_sub(slot_start_cyc) / CPU_MHZ;
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
            let mut ready_us = 0u32;
            loop {
                if end_ev_set(r) {
                    got_end = true;
                    break;
                }
                i += 1;
                if ready_us == 0 && r.events_ready().read() != 0 {
                    ready_us = (cyc() - t_rx) / CPU_MHZ;
                }
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
            state.rx_ramp_us = ready_us;
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            if r.events_address().read() != 0 {
                state.addr_events += 1;
                state.last_rx_hdr.copy_from_slice(&buf[..14.min(rx_cap)]);
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
                // RSSI of catches only: the link-budget probe. Empty
                // windows sample the noise floor (useless); a catch's
                // sample is the frame's own strength.
                state.rssi_catch_sum = state.rssi_catch_sum.wrapping_add(state.rssi_last);
                state.rssi_catch_cnt = state.rssi_catch_cnt.wrapping_add(1);
                if state.rssi_last > state.rssi_catch_max {
                    state.rssi_catch_max = state.rssi_last;
                }
                let len = buf[0] as usize;
                // Valid only if the phy frame [len | payload] fits the
                // caller's buffer (op_collect shifts left by one).
                state.ops[ei].rx_ok = len > 0 && len + 1 <= rx_cap;
                state.ops[ei].rx_result = len.min(63);
                state.catch_poll_us = (cyc() - t_rx) / CPU_MHZ; // END stamp
            } else {
                state.crc_bad += 1;
                if buf[0] >= 11 {
                    state.crc_bad_long += 1;
                }
                // No catch: the poll ran to a bound, so its duration is
                // the listen window (advertised in the beacon). Only an
                // idle poll measures it (an in-flight frame that failed
                // CRC ran past the window on the in-flight extension).
                if r.events_address().read() == 0 {
                    state.rx_window_us = ((cyc() - t_rx) / CPU_MHZ).saturating_sub(40);
                }
                // Diagnostics for the short-packet-passes/long-packet-fails
                // split: remember the last address-matched CRC failure.
                if state.addr_seen {
                    state.last_rx_got_end = got_end;
                    state.last_rx_addr_us = state.addr_poll_us;
                    state.last_rx_end_us = if got_end { (cyc() - t_rx) / CPU_MHZ } else { 0 };
                    state.last_rx_slot_len = state.slot_len;
                    state.last_rx_crc = crc;
                    state.last_rx_in_flight = in_flight;
                    state.last_rx_len = buf[0] as u32;
                }
                state.ops[ei].rx_ok = false;
                state.ops[ei].rx_result = 0;
            }
        }
        _ => {}
    }
    signal_done(state);
}
