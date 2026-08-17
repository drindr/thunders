#![no_std]
#![no_main]

//! thunders host on the nRF5340 application core.
//!
//! The application core exchanges payloads with the network core (which owns
//! the radio and runs the link) through the shared mailbox + the IPC
//! peripheral. This is the BLE host/controller split, without the HCI/rpmsg
//! stack: a fixed-address mailbox plus two IPC channels.

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use {defmt_rtt as _, panic_probe as _};
use defmt::info;

use embassy_nrf::ipc::{Ipc, IpcChannel};
#[path = "../../nrf5340/ipc.rs"]
mod ipc_mailbox;

use crate::ipc_mailbox::mailbox;
use thunders::MAX_PAYLOAD;

bind_interrupts!(struct Irqs {
    IPC => embassy_nrf::ipc::InterruptHandler<embassy_nrf::peripherals::IPC>;
});

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    info!("thunders host (app core)");

    // Mark the shared mailbox (the top 4 KiB of the app RAM, 0x2007F000) as
    // non-secure so the net core (non-secure domain) can read/write it. The
    // app RAM is 512 KiB in 8 KiB regions; 0x2007F000 is in RAM region 63.
    nrf_pac::SPU_S.ramregion(63).perm().write(|w| {
        w.set_secattr(false); // non-secure
        w.set_read(true);
        w.set_write(true);
        w.set_execute(true);
    });
    info!("SPU mailbox region non-secure");

    // Release the network core (it is held in reset until FORCEOFF is cleared).
    nrf_pac::RESET_S
        .network()
        .forceoff()
        .write_value(nrf_pac::reset::regs::Forceoff(0));
    info!("network core released");

    // Give the network core's power domain time to come up.
    embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;

    // IPC: channel 0 = app -> net ("TX ready"), channel 1 = net -> app ("RX ready").
    let mut ipc = Ipc::new(p.IPC, Irqs);
    ipc.event0.configure_trigger([IpcChannel::Channel0]);
    ipc.event1.configure_wait([IpcChannel::Channel1]);

    // Both mailbox slots start invalid.
    mailbox().tx.valid = 0;
    mailbox().rx.valid = 0;

    info!("host ready");

    let mut txc = 0u32;
    let mut rxc = 0u32;
    let mut report_at = embassy_time::Instant::now();
    loop {
        // Send a PING to the net core, which transmits it over the radio.
        let payload = [0x50, 0x49, 0x4E, 0x47, (txc & 0xFF) as u8, 0, 0, 0];
        txc = txc.wrapping_add(1);
        mailbox().put_tx(&payload);
        ipc.event0.trigger();

        // Wait for the net core to deliver a received payload.
        ipc.event1.wait().await;
        let mut rx = [0u8; MAX_PAYLOAD];
        if let Some(n) = mailbox().take_rx(&mut rx) {
            rxc = rxc.wrapping_add(1);
            let _ = (n, rx);
        }

        // Summary every 2 s (the per-iteration RTT logging is the host's
        // rate limiter).
        let now = embassy_time::Instant::now();
        if now - report_at >= embassy_time::Duration::from_secs(2) {
            info!("HOST tx={} rx={}", txc, rxc);
            txc = 0;
            rxc = 0;
            report_at = now;
        }
    }
}
