//! Nordic RADIO PHY implementation for `thunders`.
//!
//! Supports nRF52, nRF53, and nRF54 series chips.

use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_nrf::{peripherals, Peri};
use embassy_nrf::interrupt::typelevel::{self, Binding, Handler, Interrupt};

/// nRF54L HFXO load-cap trim (call once at boot). The 32 MHz crystal's
/// internal capacitors come from FICR.XOSC32MTRIM + the DK's 15 pF target;
/// untrimmed (INTCAP=0) the carrier sits off-frequency and old-IP receivers
/// (nRF52/53) barely decode it, while the 54L's own CFO-tracking RX copes -
/// a deaf-TX/marginal-link pattern. Zephyr does this in soc.c; nobody does
/// it for us.
#[cfg(feature = "_nrf54")]
pub fn hfxo_cap_trim() {
    unsafe {
        let trim = (0x00FF_C620 as *const u32).read_volatile(); // FICR.XOSC32MTRIM
        let slope_field = trim & 0x1FF;
        let slope = (slope_field ^ 0x100) as i32 - 0x100; // 9-bit two's complement
        let offset = ((trim >> 16) & 0x3FF) as i32;
        let cap_ff = 15000i32; // the DK's internal load capacitance (15 pF)
        let mid = (((cap_ff - 5500) * (slope + 791)) + ((offset << 2) * 1000)) >> 8;
        let mut cap = mid / 1000;
        if mid % 1000 >= 500 {
            cap += 1;
        }
        (0x5012_071C as *mut u32).write_volatile(cap as u32); // OSCILLATORS.XO32M.CONFIG.INTCAP
    }
}
#[cfg(feature = "_nrf54")]
use nrf_pac::radio::vals::{
    Crcinc, Crcstatus, Endian, Len, Mode, Plen, Skipaddr, State, Txpower,
};
#[cfg(not(feature = "_nrf54"))]
use nrf_pac::radio::vals::{
    Crcinc, Crcstatus, Endian, Len, Map, Mode, Plen, Skipaddr, State, Txpower,
};
use nrf_pac::radio::vals::S1incl;
#[cfg(feature = "nrf5340-net")]
use nrf_pac::RADIO_NS as PAC_RADIO;
#[cfg(feature = "_nrf54")]
use nrf_pac::RADIO_S as PAC_RADIO;
#[cfg(not(any(feature = "nrf5340-net", feature = "_nrf54")))]
use nrf_pac::RADIO as PAC_RADIO;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Duration;
use thunders::{config::Address, error::Error, phy::Phy};

/// RADIO mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioMode {
    /// 1 Mbps Nordic proprietary.
    Nrf1Mbit,
    /// 2 Mbps Nordic proprietary.
    Nrf2Mbit,
    /// 1 Mbps BLE.
    Ble1Mbit,
    /// 2 Mbps BLE.
    Ble2Mbit,
}

/// PHY-specific error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioError {
    /// Hardware CCM operation failed (the MIC mismatch or the timeout).
    Crypto,
    /// Provided buffer exceeded the RADIO FIFO.
    BufferTooLong,
    /// CRC check failed on a received packet.
    CrcFailed,
}

static WAKER: AtomicWaker = AtomicWaker::new();

/// Interrupt handler for the custom RADIO driver.
///
/// On nRF52/nRF53, binds to `RADIO`.  On nRF54, binds to `RADIO_0`.
pub struct RadioIrqHandler;

#[cfg(not(feature = "_nrf54"))]
impl Handler<typelevel::RADIO> for RadioIrqHandler {
    unsafe fn on_interrupt() {
        let r = PAC_RADIO;
        r.intenclr().write(|w| w.0 = 0xffff_ffff);
        WAKER.wake();
    }
}

#[cfg(feature = "_nrf54")]
impl Handler<typelevel::RADIO_0> for RadioIrqHandler {
    unsafe fn on_interrupt() {
        let r = PAC_RADIO;
        r.intenclr0(0).write(|w| w.0 = 0xffff_ffff);
        WAKER.wake();
    }
}

/// Nordic RADIO PHY.
pub struct NrfRadioPhy<'d> {
    r: nrf_pac::radio::Radio,
    _irq: PhantomData<&'d ()>,
}

// Static packet buffers: the radio DMA must reach them. Stack-allocated
// buffers (the old struct fields) were not DMA-visible on the nRF5340 net core.
static mut TX_BUF: [u8; 256] = [0; 256];
static mut RX_BUF: [u8; 256] = [0; 256];

/// Debug counters for the nRF54 receive path (read from the app).
pub static RX_STATS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// 1 = TX END fired within the poll bound, 2 = TX poll timed out.
pub static TX_STATS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Max TX poll iterations (diagnostics).
pub static TX_POLL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Max RX poll iterations (diagnostics).
pub static RX_POLL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Main-loop time outside the frame (the between-frame overhead; diagnostic).
pub static LOOP_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static RXOK_LOG: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// TX sub-phase cycle counters (the DWT CYCCNT deltas; diagnostic).
pub static TX_PHASE_DIS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static TX_PHASE_SETUP: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static TX_PHASE_POLL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Read the ARM DWT cycle counter (the CPU cycles; the ground-truth timing).
#[inline(always)]
fn dwt_cycles() -> u32 {
    unsafe { (0xE000_1004 as *mut u32).read_volatile() }
}

/// Cumulative RADIO STATE reads inside disable() (the disable latency proxy).
pub static DISABLE_READS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static TX_T0: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static RX_T0: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
// bit0: phyend fired, bit1: crc ok, bit2: crc fail, bit3: timeout

impl<'d> NrfRadioPhy<'d> {
    /// Create a new RADIO PHY (nRF52 / nRF53 variant).
    #[cfg(not(feature = "_nrf54"))]
    pub fn new(
        _radio: Peri<'d, peripherals::RADIO>,
        _irq: impl Binding<typelevel::RADIO, RadioIrqHandler> + 'd,
        mode: RadioMode,
    ) -> Self {
        let r = PAC_RADIO;

        // Reset the peripheral.
        r.power().write(|w| w.set_power(false));
        r.power().write(|w| w.set_power(true));

        let (mode_val, plen) = match mode {
            RadioMode::Nrf1Mbit => (Mode::Nrf1mbit, Plen::_8bit),
            // Nordic ESB uses a 16-bit preamble at 2 Mbps (2 Mbit sync needs it).
            RadioMode::Nrf2Mbit => (Mode::Nrf2mbit, Plen::_16bit),
            RadioMode::Ble1Mbit => (Mode::Ble1mbit, Plen::_8bit),
            RadioMode::Ble2Mbit => (Mode::Ble2mbit, Plen::_16bit),
        };

        r.mode().write(|w| w.set_mode(mode_val));

        // CRC config varies by mode. Nordic ESB includes the address in CRC.
        let crc_len = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => Len::Three,
            _ => Len::Two,
        };
        r.crccnf().write(|w| {
            w.set_len(crc_len);
            w.set_skipaddr(Skipaddr::Include);
        });
        let crc_poly = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 0x0000_065B,
            _ => 0x0001_1021,
        };
        r.crcpoly().write(|w| w.set_crcpoly(crc_poly));
        let crc_init = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 0x55_5555,
            _ => 0xFFFF,
        };
        r.crcinit().write(|w| w.set_crcinit(crc_init));

        // Packet configuration.
        r.pcnf0().write(|w| {
            // lflen=8: [length byte | payload]; the length byte lands in
            // RX_BUF[0].
            w.set_lflen(8);
            w.set_s0len(false);
            w.set_s1len(0);
            w.set_s1incl(S1incl::Automatic);
            w.set_cilen(0);
            w.set_plen(plen);
            w.set_crcinc(Crcinc::Exclude);
            w.set_termlen(0);
        });

        let balen = match mode { RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 3u8, _ => 4u8 };
        // Whitening disabled: Nordic ESB does not whiten, and keeping it off
        // removes any cross-generation whitening-algorithm mismatch.
        let whiteen = false;
        r.pcnf1().write(|w| {
            w.set_maxlen(255);
            w.set_statlen(0);
            w.set_balen(balen);
            w.set_endian(Endian::Big);
            w.set_whiteen(whiteen);
        });

        // Default power and channel.
        r.txpower().write(|w| w.set_txpower(Txpower::_0dBm));
        // Whitening IV kept for completeness (unused while whiteen=false).
        #[cfg(not(feature = "_nrf54"))]
        r.datawhiteiv().write(|w| w.set_datawhiteiv(25));
        r.frequency().write(|w| {
            w.set_frequency(2);
            w.set_map(Map::Default);
        });

        // Base address and prefix default.
        r.base0().write_value(0);
        r.prefix0().write(|w| w.set_ap0(0));
        r.txaddress().write(|w| w.set_txaddress(0));
        r.rxaddresses().write(|w| w.set_addr0(true));

        // Enable the NVIC interrupt.
        typelevel::RADIO::unpend();
        unsafe { typelevel::RADIO::enable() };

        Self {
            r,
            _irq: PhantomData,
        }
    }

    /// Create a new RADIO PHY (nRF54 variant).
    #[cfg(feature = "_nrf54")]
    pub fn new(
        _radio: Peri<'d, peripherals::RADIO>,
        _irq: impl Binding<typelevel::RADIO_0, RadioIrqHandler> + 'd,
        mode: RadioMode,
    ) -> Self {
        let r = PAC_RADIO;

        // nRF54 RADIO does not have a power register; the peripheral
        // is always powered when the system is on.

        // NOTE: hfxo_cap_trim() must run BEFORE embassy_nrf::init() starts the
        // HFXO; the examples call it at the top of main().

        // nRF54L errata workarounds required for correct radio operation
        // (mirrors Nordic's ESB driver, esb_glue.c):
        //  - 54L/39: the CLOCK PLL must be started (radio reference clock).
        //  - 54L/20: constant-latency mode (radio timing vs power manager).
        {
            let c = nrf_pac::CLOCK_S;
            c.events_pllstarted().write_value(0);
            c.tasks_pllstart().write_value(1);
            let mut ok = false;
            for _ in 0..1_000_000 {
                if c.events_pllstarted().read() != 0 {
                    ok = true;
                    break;
                }
            }
            nrf_pac::POWER_S.tasks_constlat().write_value(1);
            // Errata 54L/49 workaround (first on-air payload bits): hidden radio reg.
            unsafe { (0x5008C58C as *mut u32).write_volatile(1); }
            // Report PLL state via a scratch RAM word (readable post-run).
            unsafe { (0x2007F000 as *mut u32).write_volatile(if ok { 0x504C4C4F } else { 0x504C4C58 }); }
        }

        let (mode_val, plen) = match mode {
            RadioMode::Nrf1Mbit => (Mode::Nrf1mbit, Plen::_8bit),
            // Nordic ESB uses a 16-bit preamble at 2 Mbps (2 Mbit sync needs it).
            RadioMode::Nrf2Mbit => (Mode::Nrf2mbit, Plen::_16bit),
            RadioMode::Ble1Mbit => (Mode::Ble1mbit, Plen::_8bit),
            RadioMode::Ble2Mbit => (Mode::Ble2mbit, Plen::_16bit),
        };

        r.mode().write(|w| w.set_mode(mode_val));

        let crc_len = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => Len::Three,
            _ => Len::Two,
        };
        r.crccnf().write(|w| {
            w.set_len(crc_len);
            w.set_skipaddr(Skipaddr::Include);
        });
        let crc_poly = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 0x0000_065B,
            _ => 0x0001_1021,
        };
        r.crcpoly().write(|w| w.set_crcpoly(crc_poly));
        let crc_init = match mode {
            RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 0x55_5555,
            _ => 0xFFFF,
        };
        r.crcinit().write(|w| w.set_crcinit(crc_init));

        // Packet configuration.
        r.pcnf0().write(|w| {
            // lflen=8: [length byte | payload]; the length byte lands in
            // RX_BUF[0].
            w.set_lflen(8);
            w.set_s0len(false);
            w.set_s1len(0);
            w.set_s1incl(S1incl::Automatic);
            w.set_cilen(0);
            w.set_plen(plen);
            w.set_crcinc(Crcinc::Exclude);
            w.set_termlen(0);
        });

        let balen = match mode { RadioMode::Ble1Mbit | RadioMode::Ble2Mbit => 3u8, _ => 4u8 };
        // Whitening disabled: Nordic ESB does not whiten, and keeping it off
        // removes any cross-generation whitening-algorithm mismatch.
        let whiteen = false;
        r.pcnf1().write(|w| {
            w.set_maxlen(255);
            w.set_statlen(0);
            w.set_balen(balen);
            w.set_endian(Endian::Big);
            w.set_whiteen(whiteen);
        });

        // Default power and channel.
        r.txpower().write(|w| w.set_txpower(Txpower::_0dBm));
        // Whitening IV kept for completeness (unused while whiteen=false).
        #[cfg(feature = "_nrf54")]
        r.datawhite().write(|w| w.set_iv(25));
        r.frequency().write(|w| {
            w.set_frequency(2);
            w.set_map(false);
        });

        // Base address and prefix default.
        r.base0().write_value(0);
        r.prefix0().write(|w| w.set_ap0(0));
        r.txaddress().write(|w| w.set_txaddress(0));
        r.rxaddresses().write(|w| w.set_addr0(true));

        // Shorts: READY_START + PHYEND_DISABLE work for both TX and RX on
        // nRF54 (see Nordic's ESB driver). Set once, never rewrite (rewriting
        // shorts per-operation was observed to break RX).
        r.shorts().write(|w| {
            w.set_ready_start(true);
            w.set_phyend_disable(true);
        });

        // Enable the NVIC interrupt.
        typelevel::RADIO_0::unpend();
        unsafe { typelevel::RADIO_0::enable() };

        Self {
            r,
            _irq: PhantomData,
        }
    }

    /// Read current radio state.
    fn state(&self) -> State {
        self.r.state().read().state()
    }

    /// Move the RADIO to the Disabled state from any state.
    /// Returns the number of STATE register reads (the disable latency
    /// proxy - the 5340's time driver does not tick, so iteration counts
    /// are the reliable timing here).
    fn disable(&self) -> u32 {
        let r = self.r;
        let mut reads = 0u32;
        loop {
            reads += 1;
            match self.state() {
                State::Disabled => {
                    DISABLE_READS.fetch_add(reads, core::sync::atomic::Ordering::Relaxed);
                    return reads;
                }
                State::RxRu | State::RxIdle | State::TxRu | State::TxIdle => {
                    r.tasks_disable().write_value(1);
                    while self.state() != State::Disabled {
                        reads += 1;
                    }
                    DISABLE_READS.fetch_add(reads, core::sync::atomic::Ordering::Relaxed);
                    return reads;
                }
                State::RxDisable | State::TxDisable => {
                    while self.state() != State::Disabled {
                        reads += 1;
                    }
                    DISABLE_READS.fetch_add(reads, core::sync::atomic::Ordering::Relaxed);
                    return reads;
                }
                State::Rx => {
                    r.tasks_stop().write_value(1);
                    while self.state() != State::RxIdle {}
                }
                State::Tx => {
                    r.tasks_stop().write_value(1);
                    while self.state() != State::TxIdle {}
                }
                _ => {
                    // Unknown/substates: wait a bit and retry.
                    r.tasks_disable().write_value(1);
                    while self.state() != State::Disabled {
                        reads += 1;
                    }
                    DISABLE_READS.fetch_add(reads, core::sync::atomic::Ordering::Relaxed);
                    return reads;
                }
            }
        }
    }

    /// Set the RAM pointer for DMA.
    fn set_packet_ptr(&mut self, ptr: *const u8) {
        let base = self.r.as_ptr() as usize;
        self.r.packetptr().write_value(ptr as u32);
        // nRF54: the RXD.MAXCNT (0xED4) must match the buffer capacity or
        // the RX transfers nothing.
        #[cfg(feature = "_nrf54")]
        unsafe {
            ((base + 0xED4) as *mut u32).write_volatile(64);
        }
    }

    /// Wait for the END event with interrupt-driven async.
    async fn wait_end(&self) {
        let r = self.r;
        core::future::poll_fn(|cx| {
            WAKER.register(cx.waker());
            if r.events_end().read() != 0 {
                r.events_end().write_value(0);
                return Poll::Ready(());
            }
            #[cfg(not(feature = "_nrf54"))]
            r.intenset().write(|w| w.set_end(true));
            #[cfg(feature = "_nrf54")]
            r.intenset0(0).write(|w| w.set_end(true));
            Poll::Pending
        })
        .await;
    }

    /// Bit-reverse each byte of a 4-byte array (ESB nRF24L01+ compatible).
    fn bytewise_bit_swap(v: u32) -> u32 {
        let mut out = 0u32;
        for i in 0..4 {
            let byte = ((v >> (i * 8)) & 0xFF) as u8;
            let swapped = byte.reverse_bits();
            out |= (swapped as u32) << (i * 8);
        }
        out
    }

    /// Convert a raw 5-byte address into BASE0/PREFIX0 configuration.
    ///
    /// Matches the ESB (Enhanced ShockBurst) nRF24L01+ compatible format:
    /// `base0 = __REV(bytewise_bit_swap(base_bytes))` and
    /// `prefix0 = bytewise_bit_swap(prefix_byte)`.
    fn write_address(&self, addr: &Address) {
        // ESB address: [prefix, base0_0, base0_1, base0_2, base0_3]
        // base0 in little-endian bytes is [addr.0[1], addr.0[2], addr.0[3], addr.0[4]]
        let base_raw = u32::from_le_bytes([addr.0[1], addr.0[2], addr.0[3], addr.0[4]]);
        let base = Self::bytewise_bit_swap(base_raw).swap_bytes();
        self.r.base0().write_value(base);
        // Prefix byte: bit-swap the first address byte.
        let prefix = addr.0[0].reverse_bits();
        self.r.prefix0().write(|w| w.set_ap0(prefix));
    }

    /// Synchronous transmit - no await hop, no executor. Measures the raw
    /// radio TX rate (the async wrapper alone costs the executor hop).
    pub fn transmit_blocking(
        &mut self,
        pkt: &[u8],
    ) -> Result<(), Error<RadioError>> {
        if pkt.len() > 255 - 1 {
            return Err(Error::Phy(RadioError::BufferTooLong));
        }
        let c0 = dwt_cycles();
        self.disable();
        let c1 = dwt_cycles();

        // nRF54 requires PLL enable after the radio is disabled.
        #[cfg(feature = "_nrf54")]
        self.pll_enable();

        // ESB format: [S0 = length][payload].
        let tx_buf = unsafe { &mut TX_BUF };
        tx_buf[0] = pkt.len() as u8;
        tx_buf[1..1 + pkt.len()].copy_from_slice(pkt);

        compiler_fence(Ordering::Release);
        self.set_packet_ptr(tx_buf.as_ptr());
        // nRF54: the TX DMA amount (TXD at 0xEE8) must match the PDU length.
        // nRF52/53: the lflen drives the transfer - nothing to write.
        // nRF54: the TX DMA amount (TXD.AMOUNT at 0xEE8) must match the PDU
        // length or the radio transmits nothing.
        #[cfg(feature = "_nrf54")]
        unsafe {
            ((self.r.as_ptr() as usize + 0xEE8) as *mut u32).write_volatile(1 + pkt.len() as u32);
        }

        // Shortcuts: ramp-up TX then start packet automatically; disable at END.
        #[cfg(not(feature = "_nrf54"))]
        self.r.shorts().write(|w| {
            w.set_txready_start(true);
            w.set_end_disable(true);
        });

        self.r.events_end().write_value(0);
        let c2 = dwt_cycles();
        self.r.tasks_txen().write_value(1);

        // Poll for completion (interrupt-driven waits are unreliable here).
        #[cfg(not(feature = "_nrf54"))]
        {
            // 2 Mbps short packet TX completes in ~60 us; bound the poll so a
            // missed END (radio not ready) cannot stall the 1 kHz link.
            let mut t = 0u32;
            while self.r.events_end().read() == 0 {
                t += 1;
                if t > 40_000 {
                    break;
                }
            }
            if t < 40_000 {
                TX_STATS.fetch_or(1, core::sync::atomic::Ordering::Relaxed);
            } else {
                TX_STATS.fetch_or(2, core::sync::atomic::Ordering::Relaxed);
            }
            TX_POLL.store(t, core::sync::atomic::Ordering::Relaxed);
            self.r.events_end().write_value(0);
        }
        let c3 = dwt_cycles();
        TX_PHASE_DIS.fetch_add(c1 - c0, core::sync::atomic::Ordering::Relaxed);
        TX_PHASE_SETUP.fetch_add(c2 - c1, core::sync::atomic::Ordering::Relaxed);
        TX_PHASE_POLL.fetch_add(c3 - c2, core::sync::atomic::Ordering::Relaxed);
        #[cfg(feature = "_nrf54")]
        {
            let mut t = 0u32;
            while self.r.events_phyend().read() == 0 {
                t += 1;
                if t > 40_000 {
                    break;
                }
            }
            self.r.events_phyend().write_value(0);
        }
        compiler_fence(Ordering::Acquire);
        // The END_DISABLE short already ramps the radio down; skip the
        // explicit disable() wait here (the next op's disable() is
        // state-aware and returns instantly when Disabled).
        Ok(())
    }

    /// Begin a TX burst: ramp the radio once and leave it on across packets
    /// (no END_DISABLE). The ramp is amortized - the subsequent
    /// [`transmit_burst_send`] packets skip the TXEN ramp entirely.
    ///
    /// The nRF54L burst is not implemented (its SHORTS auto-disable and the
    /// per-packet PLL/TXD.AMOUNT differ) - return Unsupported so the link
    /// falls back to the plain per-packet transmit.
    pub fn transmit_burst_begin(
        &mut self,
        pkt: &[u8],
    ) -> Result<(), Error<RadioError>> {
        #[cfg(feature = "_nrf54")]
        return Err(Error::Unsupported);
        if pkt.len() > 255 - 1 {
            return Err(Error::Phy(RadioError::BufferTooLong));
        }
        self.disable();
        let tx_buf = unsafe { &mut TX_BUF };
        tx_buf[0] = pkt.len() as u8;
        tx_buf[1..1 + pkt.len()].copy_from_slice(pkt);
        compiler_fence(Ordering::Release);
        self.set_packet_ptr(tx_buf.as_ptr());
        #[cfg(not(feature = "_nrf54"))]
        self.r.shorts().write(|w| {
            w.set_txready_start(true);
            // No end_disable: the radio stays warm (TXIDLE) after the END.
        });
        self.r.events_end().write_value(0);
        self.r.tasks_txen().write_value(1);
        self.poll_tx_end();
        Ok(())
    }

    /// Send the next packet in a burst: the radio is already ramped, so only
    /// the packetptr + the START - no TXEN, no ramp (the ~on-air time).
    pub fn transmit_burst_send(
        &mut self,
        pkt: &[u8],
    ) -> Result<(), Error<RadioError>> {
        #[cfg(feature = "_nrf54")]
        return Err(Error::Unsupported);
        if pkt.len() > 255 - 1 {
            return Err(Error::Phy(RadioError::BufferTooLong));
        }
        let tx_buf = unsafe { &mut TX_BUF };
        tx_buf[0] = pkt.len() as u8;
        tx_buf[1..1 + pkt.len()].copy_from_slice(pkt);
        compiler_fence(Ordering::Release);
        self.set_packet_ptr(tx_buf.as_ptr());
        self.r.events_end().write_value(0);
        self.r.tasks_start().write_value(1);
        self.poll_tx_end();
        Ok(())
    }

    /// Poll the TX END event (the packet's on-air completion).
    #[inline(always)]
    fn poll_tx_end(&mut self) {
        #[cfg(not(feature = "_nrf54"))]
        {
            let mut t = 0u32;
            while self.r.events_end().read() == 0 {
                t += 1;
                if t > 40_000 {
                    break;
                }
            }
            if t < 40_000 {
                TX_STATS.fetch_or(1, core::sync::atomic::Ordering::Relaxed);
            } else {
                TX_STATS.fetch_or(2, core::sync::atomic::Ordering::Relaxed);
            }
            TX_POLL.store(t, core::sync::atomic::Ordering::Relaxed);
            self.r.events_end().write_value(0);
        }
        #[cfg(feature = "_nrf54")]
        {
            let mut t = 0u32;
            while self.r.events_phyend().read() == 0 {
                t += 1;
                if t > 40_000 {
                    break;
                }
            }
            self.r.events_phyend().write_value(0);
        }
    }
}

impl<'d> Phy for NrfRadioPhy<'d> {
    type Error = RadioError;

    async fn set_channel(&mut self, ch: u8) {
        let freq = ch % 101;
        #[cfg(not(feature = "_nrf54"))]
        self.r.frequency().write(|w| {
            w.set_frequency(freq);
            w.set_map(Map::Default);
        });
        #[cfg(feature = "_nrf54")]
        self.r.frequency().write(|w| {
            w.set_frequency(freq);
            w.set_map(false);
        });
    }

    async fn set_address(&mut self, addr: &Address) {
        self.write_address(addr);
    }

    async fn transmit(
        &mut self,
        pkt: &[u8],
    ) -> Result<(), Error<RadioError>> {
        // The body is fully synchronous (busy-polls, no awaits) - route
        // through the blocking twin so both paths share one implementation.
        self.transmit_blocking(pkt)
    }

    async fn receive(
        &mut self,
        buf: &mut [u8],
        timeout: Duration,
    ) -> Result<Option<usize>, Error<RadioError>> {
        // nRF54: the radio auto-disables via the PHYEND_DISABLE short; skip the
        // explicit disable before RX (verified: explicit disable broke RX).
        #[cfg(not(feature = "_nrf54"))]
        self.disable();

        // nRF54 requires PLL enable after the radio is disabled.
        #[cfg(feature = "_nrf54")]
        self.pll_enable();

        let rx_buf = unsafe { &mut RX_BUF };
        rx_buf.fill(0);
        let ptr = rx_buf.as_mut_ptr();
        self.set_packet_ptr(ptr);

        // Shortcut: ramp-up RX then start listening automatically.
        #[cfg(not(feature = "_nrf54"))]
        self.r.shorts().write(|w| {
            w.set_rxready_start(true);
            w.set_end_disable(true);
        });

        self.r.events_ready().write_value(0);
        self.r.events_phyend().write_value(0);
        self.r.tasks_rxen().write_value(1);



        #[cfg(not(feature = "_nrf54"))]
        let result: Result<(), embassy_time::TimeoutError> = {
            // The net core's time driver does not tick here, so bound the
            // poll by raw iteration count (the nRF54 path does the same).
            let limit = (timeout.as_micros() as u64 * 12).min(4_000_000) as u32;
            let mut t = 0u32;
            while self.r.events_end().read() == 0 {
                t += 1;
                if t > limit {
                    break;
                }
            }
            RX_POLL.store(t, core::sync::atomic::Ordering::Relaxed);
            if t < limit {
                self.r.events_end().write_value(0);
                RX_STATS.fetch_or(1, core::sync::atomic::Ordering::Relaxed);
                Ok(())
            } else {
                RX_STATS.fetch_or(8, core::sync::atomic::Ordering::Relaxed);
                Err(embassy_time::TimeoutError)
            }
        };
        #[cfg(feature = "_nrf54")]
        let result: Result<(), embassy_time::TimeoutError> = {
            // Busy-poll PHYEND (matches the verified working raw sequence),
            // bounded by the caller's timeout (~40 iterations/us at 128 MHz).
            let limit = (timeout.as_micros() as u64 * 12).min(4_000_000) as u32;
            let mut t = 0u32;
            while self.r.events_phyend().read() == 0 {
                t += 1;
                if t > limit {
                    break;
                }
            }
            RX_POLL.store(t, core::sync::atomic::Ordering::Relaxed);
            if t < limit {
                self.r.events_phyend().write_value(0);
                RX_STATS.fetch_or(1, core::sync::atomic::Ordering::Relaxed);
                Ok(())
            } else {
                RX_STATS.fetch_or(8, core::sync::atomic::Ordering::Relaxed);
                Err(embassy_time::TimeoutError)
            }
        };

        if result.is_err() {
            // Timeout: stop and return None.
            self.disable();
            return Ok(None);
        }

        compiler_fence(Ordering::Acquire);

        let crc_ok = self.r.crcstatus().read().crcstatus() == Crcstatus::CrcOk;
        if crc_ok {
            RX_STATS.fetch_or(2, core::sync::atomic::Ordering::Relaxed);
        } else {
            RX_STATS.fetch_or(4, core::sync::atomic::Ordering::Relaxed);
        }
        if !crc_ok {
            self.disable();
            return Ok(None);
        }

        // lflen=8: the length byte lands in RX_BUF[0]; the full PDU is in
        // the buffer (the AMOUNT register reads inconsistently on nRF54L,
        // so use the length byte, not the AMOUNT).
        let payload_len = unsafe { RX_BUF[0] } as usize;
        let rx_buf = unsafe { &RX_BUF };
        #[cfg(feature = "defmt")]
        if RXOK_LOG.fetch_add(1, core::sync::atomic::Ordering::Relaxed) % 1000 == 0 {
            defmt::info!("RXOK lenbyte={} buf={:02x}", payload_len, &rx_buf[..16]);
        }
        if payload_len == 0 || payload_len > buf.len() {
            self.disable();
            return Err(Error::BufferTooSmall);
        }
        buf[..payload_len].copy_from_slice(&rx_buf[1..1 + payload_len]);
        self.disable();
        Ok(Some(payload_len))
    }

    async fn flush(&mut self) {
        self.disable();
        self.r.events_end().write_value(0);
        self.r.events_crcok().write_value(0);
        self.r.events_crcerror().write_value(0);
        self.r.events_ready().write_value(0);
        self.r.events_address().write_value(0);
        self.r.events_payload().write_value(0);
        // nRF54 requires explicit PLL enable before TX/RX.
        #[cfg(feature = "_nrf54")]
        self.pll_enable();
    }

    fn transmit_burst_begin(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        NrfRadioPhy::transmit_burst_begin(self, pkt)
    }

    fn transmit_burst_send(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        NrfRadioPhy::transmit_burst_send(self, pkt)
    }

    #[cfg(any(feature = "nrf52840", feature = "nrf5340-net", feature = "nrf52833"))]
    fn ccm_crypt(
        &mut self,
        key: &[u8; 16],
        nonce: &[u8; 13],
        payload: &mut [u8],
        mic: &mut [u8; 4],
        encrypt: bool,
    ) -> Result<(), Error<Self::Error>> {
        NrfRadioPhy::ccm_crypt(key, nonce, payload, mic, encrypt)
            .map_err(|_| Error::Phy(RadioError::Crypto))
    }
}

impl<'d> NrfRadioPhy<'d> {
    /// Hardware AES-CCM over the EasyDMA: the config [key(16) | nonce(13) |
    /// iv(8)] at CNFPTR, the payload at INPTR/OUTPTR, the CBC-MAC scratch at
    /// SCRATCHPTR. KSGEN (the key schedule) then CRYPT (the AEAD); MICSTATUS
    /// gates the decrypt. The MIC (4 bytes) is appended after the payload.
    #[cfg(any(feature = "nrf52840", feature = "nrf5340-net", feature = "nrf52833"))]
    fn ccm_crypt(
        key: &[u8; 16],
        nonce: &[u8; 13],
        payload: &mut [u8],
        mic: &mut [u8; 4],
        encrypt: bool,
    ) -> Result<(), ()> {
        // The nrfx nrf_ccm_cnf_t: key[16] | pktctr[9] | iv[8] (33 packed bytes).
        #[repr(C, packed)]
        struct CcmCnf {
            key: [u8; 16],
            pktctr: [u8; 9],
            iv: [u8; 8],
        }
        // The scratch must hold (16 + MAXPACKETSIZE) bytes when MODE.LENGTH
        // is Extended (the keystream + the AES state); MAXPACKETSIZE resets
        // to 251.
        static mut CCM_CNF: CcmCnf =
            CcmCnf { key: [0; 16], pktctr: [0; 9], iv: [0; 8] };
        static mut CCM_SCRATCH: [u8; 267] = [0; 267];
        // The EasyDMA packet: [HEADER(S0) | LENGTH | RFU | payload | MIC].
        static mut CCM_BUF: [u8; 3 + 40] = [0; 3 + 40];
        // Separate output buffer (the in-place INPTR==OUTPTR read-back
        // hazards the MAC for the streaming EasyDMA).
        static mut CCM_OUT: [u8; 3 + 40 + 4] = [0; 3 + 40 + 4];

        // The nonce -> the CNF mapping (PS Table 56): the pktctr field is
        // the 5-byte packet counter + 3 reserved bytes + the direction byte;
        // the IV carries the channel. make_nonce_13 = [seq | 0*4 | channel
        // | direction | 0*6]; the channel is dropped so the IV is zeroed.
        let cnf = unsafe { &mut CCM_CNF };
        cnf.key.copy_from_slice(key);
        cnf.pktctr[..5].copy_from_slice(&nonce[..5]);
        cnf.pktctr[5..8].fill(0);
        cnf.pktctr[8] = nonce[6]; // the direction bit
        cnf.iv[0] = nonce[5]; // the channel
        cnf.iv[1..].fill(0);

        let buf = unsafe { &mut CCM_BUF };
        let len = payload.len();
        buf[0] = 0; // the S0 header
        // The LENGTH byte is the packet length the CCM processes: the
        // plaintext alone on encrypt (the hardware appends the 4-byte MIC),
        // the ciphertext + MIC on decrypt (the hardware strips the MIC).
        buf[1] = if encrypt { len } else { len + 4 } as u8;
        buf[2] = 0; // the RFU
        buf[3..3 + len].copy_from_slice(payload);
        // On decrypt, the received MIC is the input after the ciphertext.
        if !encrypt {
            buf[3 + len..3 + len + 4].copy_from_slice(mic);
        }

        // The 5-bit (Default) LENGTH field tops out at 31. Match nrf-hal: the
        // encrypt's plaintext must fit so the +4 MIC output stays <= 31, the
        // decrypt's ciphertext+MIC is the read value. Larger packets switch
        // to the 8-bit (Extended) field and need MAXPACKETSIZE set.
        let extended = if encrypt { len > 27 } else { len + 4 > 31 };

        #[cfg(feature = "nrf52840")]
        let ccm = unsafe { &nrf_pac::CCM };
        #[cfg(feature = "nrf52833")]
        let ccm = unsafe { &nrf_pac::CCM };
        #[cfg(feature = "nrf5340-net")]
        let ccm = unsafe { &nrf_pac::CCM_NS };
        // The MODE register (the datarate) must be written while DISABLED;
        // write the whole thing before enabling.
        ccm.enable().write_value(nrf_pac::ccm::regs::Enable(0));
        if extended {
            ccm.maxpacketsize()
                .write_value(nrf_pac::ccm::regs::Maxpacketsize(buf[1] as u32));
        }
        // MODE: bit0 = encrypt(0)/decrypt(1), bits 16-17 = the datarate
        // (2 Mbit = 1), bit 24 = LENGTH (0 = 5-bit Default, 1 = 8-bit Extended).
        ccm.mode().write_value(nrf_pac::ccm::regs::Mode(
            ((extended as u32) << 24) | (1u32 << 16) | (if encrypt { 0 } else { 1 }),
        ));
        ccm.enable().write_value(nrf_pac::ccm::regs::Enable(2));
        ccm.cnfptr().write_value(cnf as *const _ as u32);
        ccm.inptr().write_value(buf.as_ptr() as u32);
        let out = unsafe { &mut CCM_OUT };
        ccm.outptr().write_value(out.as_ptr() as u32);
        ccm.scratchptr().write_value(unsafe { CCM_SCRATCH.as_ptr() } as u32);

        ccm.events_endksgen().write_value(0);
        ccm.tasks_ksgen().write_value(1);
        let mut t = 0u32;
        while ccm.events_endksgen().read() == 0 {
            t += 1;
            if t > 1_000_000 {
                return Err(());
            }
        }
        ccm.events_endcrypt().write_value(0);
        ccm.tasks_crypt().write_value(1);
        t = 0;
        while ccm.events_endcrypt().read() == 0 {
            t += 1;
            if t > 1_000_000 {
                return Err(());
            }
        }
        if encrypt {
            payload.copy_from_slice(&out[3..3 + len]);
            mic.copy_from_slice(&out[3 + len..3 + len + 4]);
            Ok(())
        } else {
            // MICSTATUS bit 0: 0 = CheckFailed, 1 = CheckPassed.
            if ccm.micstatus().read().0 & 0x1 == 0 {
                return Err(());
            }
            payload.copy_from_slice(&out[3..3 + len]);
            Ok(())
        }
    }

    /// Enable the RADIO PLL (nRF54 only). On nRF52/53 the PLL
    /// auto-enables on TXEN/RXEN, but RADIO3 requires an explicit
    /// TASKS_PLLEN and waits for EVENTS_PLLREADY before it can
    /// ramp to TX/RX.
    #[cfg(feature = "_nrf54")]
    fn pll_enable(&mut self) {
        self.r.events_pllready().write_value(0);
        self.r.tasks_pllen().write_value(1);
        // Busy-wait for the PLL to settle.
        for _ in 0..100_000 {
            if self.r.events_pllready().read() != 0 {
                self.r.events_pllready().write_value(0);
                return;
            }
        }
    }
}
