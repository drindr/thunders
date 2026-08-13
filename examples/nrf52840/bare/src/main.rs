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

static FRAME_LOG: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

use thunders::phy::Phy;
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

    #[cfg(feature = "one-way")]
    {
        // Standalone TX bench: the raw radio transmit, no link, no await
        // hop - the pure radio TX rate (the transmit_blocking is sync).
        let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
        phy.set_address(&Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7])).await;
        phy.set_channel(0).await;
        info!("one-way TX ready");
        // DWT cycle counter for the rate: the RTC/Instant time driver on the
        // 52840 skews the elapsed; the CPU cycle counter does not.
        unsafe {
            (0xE000_EDFC as *mut u32).write_volatile(1); // DEMCR.TRCENA
            (0xE000_1000 as *mut u32).write_volatile(1); // DWT.CTRL.CYCCNTENA
            (0xE000_1004 as *mut u32).write_volatile(0); // DWT.CYCCNT = 0
        }
        let mut txc: u32 = 0;
        let mut frames: u32 = 0;
        // Burst TX: the ramp once, then the packets chain (the radio stays
        // warm). The rate's measured over the host wall-clock (the run
        // duration) - the DWT CYCCNT freezes during the radio IO.
        let mut first = true;
        loop {
            let payload = [0x50, 0x49, 0x4E, 0x47, (txc & 0xFF) as u8, 0, 0, 0];
            txc = txc.wrapping_add(1);
            if first {
                phy.transmit_burst_begin(&payload).unwrap();
                first = false;
            } else {
                phy.transmit_burst_send(&payload).unwrap();
            }
            frames += 1;
            if frames % 5000 == 0 {
                let txp = thunders_phy_nrf::radio_phy::TX_POLL.load(core::sync::atomic::Ordering::Relaxed);
                info!("BURST frames={} txp={}", frames, txp);
                frames = 0;
            }
        }
    }
    #[cfg(not(feature = "one-way"))]
    {
        let mut phy = NrfRadioPhy::new(p.RADIO, Irqs, RadioMode::Nrf2Mbit);
        let mut cfg = Config::new(
            [0xAB, 0xCD, 0xEF, 0x01],
            Address([0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
            role,
        );
        let cfg = if cfg!(feature = "secure") {
            cfg.with_security(thunders::Security::with_ccm([0xAB; 32]))
        } else {
            cfg
        };
        // The CCM loopback: the encrypt -> the decrypt -> the MIC, on one board
        // (no radio) to isolate the AES-CCM hardware from the nonce/radio path.
        #[cfg(feature = "secure")]
        {
            let mut data = [0x50u8, 0x49, 0x4E, 0x47, 0x00, 0x00, 0x00, 0x00];
            let orig = data;
            let mut mic = [0u8; 4];
            let key = [0xABu8; 16];
            let nonce = [0u8; 13];
            match phy.ccm_crypt(&key, &nonce, &mut data, &mut mic, true) {
                Ok(()) => {
                    let mut dec = data;
                    let mut m2 = mic;
                    match phy.ccm_crypt(&key, &nonce, &mut dec, &mut m2, false) {
                        Ok(()) => info!("CCM loopback OK: dec==orig {} mic=={}", dec == orig, m2 == mic),
                        Err(e) => info!("CCM loopback decrypt err: {:?}", defmt::Debug2Format(&e)),
                    }
                }
                Err(e) => info!("CCM loopback encrypt err: {:?}", defmt::Debug2Format(&e)),
            }
        }

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

        let t_loop = Instant::now();
        let t_frame = Instant::now();
        #[cfg(feature = "one-way")]
        match phy.transmit_blocking(payload.as_ref()) {
            Ok(()) => {}
            Err(e) => info!("tx err: {:?}", defmt::Debug2Format(&e)),
        }
        #[cfg(not(feature = "one-way"))]
        match link.frame(tx, &mut rx_buf).await {
            Ok(Some(n)) => {
                ok += 1;
                #[cfg(feature = "peripheral")]
                {
                    // Echo the last received payload back on the next frame.
                    echo[..n].copy_from_slice(&rx_buf[..n]);
                    echo_len = n;
                }
                // Throttled: the per-frame RTT log is the hot-path cost.
                if FRAME_LOG.fetch_add(1, core::sync::atomic::Ordering::Relaxed) % 1000 == 0 {
                    info!("RX {} bytes: {:02x}", n, &rx_buf[..n]);
                }
            }
            Ok(None) => {}
            Err(e) => info!("frame err: {:?}", defmt::Debug2Format(&e)),
        }
        thunders_phy_nrf::radio_phy::LOOP_US.fetch_add((t_loop.elapsed().as_micros() as i64 - t_frame.elapsed().as_micros() as i64).max(0) as u32, core::sync::atomic::Ordering::Relaxed);
        frames += 1;
        busy_total += t_frame.elapsed().as_micros() as u64;
        let now = Instant::now();
        if now - report_at >= embassy_time::Duration::from_secs(2) {
            let elapsed = (now - report_at).as_micros() as u32;
            let rate = (frames as u64) * 1_000_000 / elapsed.max(1) as u64;
            let avg_busy = busy_total / frames.max(1) as u64;
            let rxst =
                thunders_phy_nrf::radio_phy::RX_STATS.load(core::sync::atomic::Ordering::Relaxed);
            let rxp = thunders_phy_nrf::radio_phy::RX_POLL.load(core::sync::atomic::Ordering::Relaxed);
            let txp = thunders_phy_nrf::radio_phy::TX_POLL.load(core::sync::atomic::Ordering::Relaxed);
            let txs = thunders_phy_nrf::radio_phy::TX_STATS.load(core::sync::atomic::Ordering::Relaxed);
            let lup = thunders_phy_nrf::radio_phy::LOOP_US.swap(0, core::sync::atomic::Ordering::Relaxed);
            let dreads = thunders_phy_nrf::radio_phy::DISABLE_READS.swap(0, core::sync::atomic::Ordering::Relaxed);
            info!("BENCH frames={} ok={} rate={}/s avg_busy={}us rxst={:#x} rxp={} txp={} txs={:#x} loop={}us dis={}", frames, ok, rate, avg_busy, rxst, rxp, txp, txs, lup, dreads);
            frames = 0;
            ok = 0;
            busy_total = 0;
            report_at = now;
        }
    }
    }
}
