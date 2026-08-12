//! Nordic RADIO PHY implementation for `thunders`.
//!
//! Supports nRF52, nRF53, and nRF54 series chips.

use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_nrf::{peripherals, Peri};
use embassy_nrf::interrupt::typelevel::{self, Binding, Handler, Interrupt};
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
    fn disable(&self) {
        let r = self.r;
        loop {
            match self.state() {
                State::Disabled => return,
                State::RxRu | State::RxIdle | State::TxRu | State::TxIdle => {
                    r.tasks_disable().write_value(1);
                    while self.state() != State::Disabled {}
                    return;
                }
                State::RxDisable | State::TxDisable => {
                    while self.state() != State::Disabled {}
                    return;
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
                    while self.state() != State::Disabled {}
                    return;
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
        if pkt.len() > 255 - 1 {
            return Err(Error::Phy(RadioError::BufferTooLong));
        }

        self.disable();

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
        #[cfg(feature = "_nrf54")]
        unsafe {
            let base = self.r.as_ptr() as usize;
            ((base + 0xEE8) as *mut u32).write_volatile(1 + pkt.len() as u32);
        }
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
        self.disable();
        Ok(())
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
        defmt::info!("RXOK lenbyte={} buf={:02x}", payload_len, &rx_buf[..16]);
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
}

impl<'d> NrfRadioPhy<'d> {
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
