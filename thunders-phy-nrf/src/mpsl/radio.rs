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
pub unsafe fn signal_done(state: &MpslState) {
    state.done.store(true, core::sync::atomic::Ordering::Release);
    state.done_signal.signal(());
}

/// Configure the radio (inside a granted timeslot) with the current state.
/// The BARE-path wire format (CRC-16, balen 4, crcinc-exclude) is used: it is
/// the verified-working config on this 5340 (the MPSL format was deaf in RX).
unsafe fn radio_configure(state: &MpslState) {
    let r = state.radio;

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

    // 0 dBm (the two families encode the PA level differently).
    let mut txpower = regs::Txpower(0);
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
            let buf = &mut state.tx_buf;
            r.packetptr().write_value(buf.as_ptr() as u32);
            pll_enable(r);
            #[cfg(feature = "_nrf54")]
            unsafe {
                let len = buf[0] as usize;
                (r.as_ptr() as *mut u32).add(0xEE8 / 4).write_volatile(1 + len as u32);
            }
            r.shorts().write_value(shorts_tx());
            r.events_phyend().write_value(0);
            r.events_end().write_value(0);
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
            r.tasks_rxen().write_value(1);
            let mut i = 0;
            while !end_ev_set(r) {
                i += 1;
                if i > state.rx_poll {
                    break;
                }
            }
            end_ev_clear(r);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            let crc = r.crcstatus().read().0;
            r.tasks_disable().write_value(1);
            r.tasks_disable().write_value(1);
            if crc & 0x1 == 0x1 {
                let len = buf[0] as usize;
                state.rx_ok = len <= 63;
                state.rx_result = len.min(63);
                state.rx_catch_iter = i; // the poll count at the catch
            } else {
                state.rx_ok = false;
                state.rx_result = 0;
            }
        }
        _ => {}
    }
    signal_done(state);
}
