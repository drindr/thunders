//! The MPSL-backed PHY (the Nordic Multiprotocol Service Layer's radio timeslots).
//!
//! The callback runs the chained slots at full cadence and exchanges packets
//! with the link layer through the caller-provided rings in [`MpslState`].
//! The slot cadence (`SLOT_US`), the slot length (`SLOT_LEN_US`) and the RX
//! poll bound (`RX_POLL`) are const generics chosen by the caller.

pub mod callback;
pub mod radio;
pub mod state;

pub use state::{MpslState, Pkt};

use embassy_time::Duration;
use thunders::config::Address;
use thunders::error::Error;
use thunders::phy::Phy;

use crate::radio_phy::RadioMode;

// The phase-lock (the proportional controller on the peripheral).
pub(crate) const PLL_SWEEP_US: u32 = 2;
pub(crate) const PLL_SWEEP_MISSES: u32 = 8;
pub(crate) const PLL_GAIN_NUM: i32 = 21;
pub(crate) const PLL_GAIN_DEN: i32 = 1000;

// --- the callback's context (the only global the C callback needs) ---
pub(crate) static mut STATE: *mut () = core::ptr::null_mut();

/// The MPSL radio PHY. The caller provides the [`MpslState`] (the rings, the
/// schedule) and the slot constants (as const generics); the callback drives
/// the chained slots through the state.
pub struct MpslRadioPhy<'d, const SLOT_US: u32, const SLOT_LEN_US: u32, const RX_POLL: u32> {
    state: &'d mut MpslState,
    _mode: RadioMode,
}

impl<'d, const SLOT_US: u32, const SLOT_LEN_US: u32, const RX_POLL: u32>
    MpslRadioPhy<'d, SLOT_US, SLOT_LEN_US, RX_POLL>
{
    /// Open the timeslot session and start the chained slots.
    ///
    /// MPSL must already be initialized; `state` must outlive the phy.
    pub fn new(_radio_mode: RadioMode, state: &'d mut MpslState) -> Self {
        // Fill the runtime slot constants from the const generics.
        state.slot_nominal = SLOT_US;
        state.slot_len = SLOT_LEN_US;
        state.rx_poll = RX_POLL;
        state.slot_distance = SLOT_US;

        unsafe {
            STATE = state as *mut MpslState as *mut ();
        }

        // The bare path power-cycles the radio before first use; the MPSL's
        // init does not. On the nRF5340 the RX frontend needs that reset.
        #[cfg(feature = "nrf5340-net")]
        {
            use nrf_pac::radio::regs;
            state.radio.power().write_value(regs::Power(0));
            state.radio.power().write_value(regs::Power(1));
        }

        // MODECNF0.RU = Fast BEFORE the session opens: the grant machinery
        // sizes the radio ramp from this register.
        #[cfg(any(feature = "nrf5340-net", feature = "nrf52840"))]
        {
            state
                .radio
                .modecnf0()
                .modify(|w| w.set_ru(nrf_pac::radio::vals::Ru::Fast));
            let ru = state.radio.modecnf0().read().ru().to_bits();
            #[cfg(feature = "defmt")]
            defmt::info!("modecnf0 ru={} (1=Fast)", ru);
        }

        let mut session_id: u8 = 0;
        let ret = unsafe {
            nrf_mpsl::raw::mpsl_timeslot_session_open(Some(callback::timeslot_cb), &mut session_id)
        };
        if ret != 0 {
            #[cfg(feature = "defmt")]
            defmt::error!("mpsl_timeslot_session_open failed: {}", ret);
        } else {
            state.session_id = session_id;
            state.first_request = true;
            let ret = callback::mpsl_request_timeslot(state);
            #[cfg(feature = "defmt")]
            defmt::info!("first request ret={}", ret);
        }

        // The nRF54L needs its RADIO/TIMER NVIC lines + the RRAM out of
        // PowerOff (the chained arming asserts otherwise).
        #[cfg(feature = "_nrf54")]
        unsafe {
            use embassy_nrf::interrupt::typelevel::{Interrupt as _, RADIO_0, TIMER10};
            RADIO_0::enable();
            TIMER10::enable();
            let lowpower = nrf_pac::RRAMC_S.power().lowpowerconfig();
            lowpower.write(|w| w.set_mode(nrf_pac::rramc::vals::Mode::Standby));
        }
        // The MPSL's radio processing runs on the RADIO IRQ.
        #[cfg(feature = "nrf5340-net")]
        unsafe {
            use embassy_nrf::interrupt::typelevel::{Interrupt as _, RADIO};
            RADIO::enable();
        }

        Self {
            state,
            _mode: _radio_mode,
        }
    }

    /// Wait until the first timeslot is granted and the chain is flowing
    /// (the callback signals the ready gate). Spawn the MPSL's low-priority
    /// task before awaiting this.
    pub async fn wait_ready(&self) {
        self.state.done_signal.wait().await;
    }
}

impl<'d, const SLOT_US: u32, const SLOT_LEN_US: u32, const RX_POLL: u32> Phy
    for MpslRadioPhy<'d, SLOT_US, SLOT_LEN_US, RX_POLL>
{
    type Error = ();

    async fn set_channel(&mut self, ch: u8) {
        self.state.cur_channel = ch;
    }

    async fn set_address(&mut self, addr: &Address) {
        // ESB-style: base0 = __REV(bytewise_bit_swap(base_bytes)), prefix = bit-swap.
        let base_raw = u32::from_le_bytes([addr.0[1], addr.0[2], addr.0[3], addr.0[4]]);
        let mut swapped = 0u32;
        for i in 0..4 {
            let byte = ((base_raw >> (i * 8)) & 0xFF) as u8;
            swapped |= (byte.reverse_bits() as u32) << (i * 8);
        }
        self.state.cur_base0 = swapped.swap_bytes();
        self.state.cur_prefix = addr.0[0].reverse_bits() as u32;
    }

    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        if pkt.len() > 63 {
            return Err(Error::BufferTooSmall);
        }
        // The on-air frame is [length byte | payload].
        self.state.tx_buf[0] = pkt.len() as u8;
        self.state.tx_buf[1..1 + pkt.len()].copy_from_slice(pkt);
        self.state.op_kind = state::OpKind::Tx as u8;
        // Await this slot's completion (the callback sets the done flag).
        self.state.done.store(false, core::sync::atomic::Ordering::Release);
        for _ in 0..400 {
            if self.state.done.load(core::sync::atomic::Ordering::Acquire) {
                break;
            }
            embassy_futures::yield_now().await;
        }
        Ok(())
    }

    async fn receive(
        &mut self,
        buf: &mut [u8],
        _timeout: Duration,
    ) -> Result<Option<usize>, Error<Self::Error>> {
        self.state.rx_ptr = buf.as_mut_ptr();
        self.state.rx_cap = buf.len();
        self.state.op_kind = state::OpKind::Rx as u8;
        self.state.done.store(false, core::sync::atomic::Ordering::Release);
        for _ in 0..400 {
            if self.state.done.load(core::sync::atomic::Ordering::Acquire) {
                break;
            }
            embassy_futures::yield_now().await;
        }
        if self.state.rx_ok {
            // The radio left `[len, payload..]`; the Phy contract is the
            // payload only, so shift it.
            let n = self.state.rx_result;
            buf.copy_within(1..1 + n, 0);
            Ok(Some(n))
        } else {
            Ok(None)
        }
    }

    async fn flush(&mut self) {}
}
