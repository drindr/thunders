#![no_std]
#![no_main]

//! thunders link over the direct RADIO backend — role-agnostic.
//!
//! The same binary runs as the **central** (TX the PING, await the reply)
//! or the **peripheral** (RX, echo back) — so ANY two boards with this
//! example can talk to each other, either way around.
//!
//! Build with `--features central` (default) or `--features peripheral`.
//! Run two boards, one per role, on the same channel.

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_time::Instant;
use {defmt_rtt as _, panic_probe as _};
use defmt::info;

use thunders::{Address, Config, Role};
use thunders_phy_nrf::{NrfRadioPhy, RadioIrqHandler, RadioMode};

#[cfg(not(feature = "peripheral"))]
use thunders::link::Central as Link;
#[cfg(feature = "peripheral")]
use thunders::link::Peripheral as Link;

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}

bind_interrupts!(struct Irqs {
    RADIO => RadioIrqHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Board-specific clocks: the 5340 net core's LFCLK XTAL is owned by the
    // app core, so use the internal RC; the nRF54LM20 uses the XTALs.
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);

    #[cfg(not(feature = "peripheral"))]
    let role = Role::Central;
    #[cfg(feature = "peripheral")]
    let role = Role::Peripheral;

    info!("thunders link role={:?}", role);

    let phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
    let cfg = Config::new(
        [0xAB, 0xCD, 0xEF, 0x01],
        Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
        role,
    );
    let mut link = Link::new(phy, cfg).await.unwrap();
    info!("link ready ({:?})", role);

    let mut rx_buf = [0u8; 32];
    let mut frames: u32 = 0;
    let mut ok: u32 = 0;
    let mut txc: u32 = 0;
    let mut echo = [0u8; 32];
    let mut echo_len = 0usize;
    let mut busy_total: u64 = 0;
    let mut report_at = Instant::now();

    loop {
        // PING payload: [P,I,N,G,seq,0,0,0].
        #[cfg(not(feature = "peripheral"))]
        let payload = [0x50, 0x49, 0x4E, 0x47, (txc & 0xFF) as u8, 0, 0, 0];
        #[cfg(not(feature = "peripheral"))]
        let tx: Option<&[u8]> = {
            txc += 1;
            Some(&payload)
        };
        #[cfg(feature = "peripheral")]
        let tx: Option<&[u8]> = if echo_len > 0 {
            Some(&echo[..echo_len])
        } else {
            None
        };

        let t_frame = Instant::now();
        match link.frame(tx, &mut rx_buf).await {
            Ok(Some(n)) => {
                ok += 1;
                #[cfg(feature = "peripheral")]
                {
                    // Echo the last received payload back on the next frame.
                    echo[..n].copy_from_slice(&rx_buf[..n]);
                    echo_len = n;
                }
                info!("RX {} bytes: {:02x}", n, &rx_buf[..n]);
            }
            Ok(None) => {}
            Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
        }
        frames += 1;
        busy_total += t_frame.elapsed().as_micros() as u64;
        let now = Instant::now();
        if now - report_at >= embassy_time::Duration::from_secs(2) {
            let elapsed = (now - report_at).as_micros() as u32;
            let rate = frames * 1_000_000 / elapsed.max(1);
            let avg_busy = busy_total / frames.max(1) as u64;
            let rxst =
                thunders_phy_nrf::radio_phy::RX_STATS.load(core::sync::atomic::Ordering::Relaxed);
            info!("BENCH frames={} ok={} rate={}/s avg_busy={}us rxst={:#x}", frames, ok, rate, avg_busy, rxst);
            frames = 0;
            ok = 0;
            busy_total = 0;
            report_at = now;
        }
    }
}
