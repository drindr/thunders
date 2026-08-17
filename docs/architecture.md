# thunders Architecture

Three layers, top to bottom:

```
┌─────────────────────────────────────────────────────────────┐
│  postcard                                                    │
│  Packet::to_bytes / from_bytes — the wire (de)serialization  │
├─────────────────────────────────────────────────────────────┤
│  thunders                                                    │
│  Central / Peripheral (the link state machines)              │
│  Config (the ratio, the channels), Scheduler (the hop),      │
│  Security (the ChaCha / CCM)                                 │
├─────────────────────────────────────────────────────────────┤
│  Phy (the trait)                                             │
│  NrfRadioPhy (the bare RADIO) / MpslRadioPhy (the timeslots) │
└─────────────────────────────────────────────────────────────┘
```

The rule: **each layer only talks downward through the interface below it**.
The `Phy` trait is the seam — `thunders` is board-agnostic; everything
chip-specific lives behind `Phy`.

## 1. postcard (the top)

`Packet` is the only type that crosses the air. It is the postcard wire format:

```rust
pub enum Packet {
    Beacon       { epoch: u32, channel_index: u8, flags: u8, slot_us: u16,
                   slot_phase: u16, rx_en_offset: u8, tx_en_offset: u8,
                   rx_ramp: u8, tx_ramp: u8 },
    Data         { seq: u16, ack: u16, nack: Vec<u8, NACK_BYTES>, payload: Vec<u8, MAX_PAYLOAD> },
    Ack          { ack: u16, nack: Vec<u8, NACK_BYTES> },   // the pure ACK/NACK
    Drop         { seq: u16, ack: u16, nack: Vec<u8, NACK_BYTES> },  // the delivery-failure resync (piggybacked ACK/NACK)
    SlotRequest  { min_slot_us: u16 },
    PairingRequest  { id: DeviceId },
    PairingResponse { id: DeviceId, key: [u8; 16] },
}
```

The `Pairing*` variants are reserved placeholders: they round-trip through
postcard but are not wired into `Central::frame` / `Peripheral::frame` yet.

- `Packet::to_bytes` / `Packet::from_bytes` are thin wrappers over
  `postcard::to_slice` / `postcard::from_bytes`.
- `thunders` builds a `Packet`, serializes it, and hands the bytes to the
  phy's `transmit`; on receive it deserializes back into `Packet`.
- postcard has no framing of its own — the phy provides the length (the
  radio's S0/length byte), so postcard only needs to round-trip the enum.

## 2. thunders (the middle)

The protocol, all board-agnostic.

| module | role |
|---|---|
| `link` | `LinkCore` — the symmetric slot data plane; `Central` / `Peripheral` are thin wrappers that only add the master/follower sync hooks |
| `config` | `Config` (the network, the address, the role, the TX:RX ratio), `Address`, `Role`, the hop sequence |
| `scheduler` | `Scheduler` — the hop sequence (the network-seeded LCG permutation) |
| `packet` | the `Packet` wire format (see above) |
| `security` | `Security` + `CipherMode` — the ChaCha20-Poly1305 (software) or the AES-CCM (the phy's hardware) |
| `link_mgmt` | the link-management layer used by `frame`: the TX/RX windows, slot-NACK bookkeeping, cumulative ACK, reorder delivery, and the drop/resync path. The fragment/reassembly primitives in the same module are still pending a wire marker and are not called by `frame`. |
| `error` | `Error<P>` — the PHY-typed error |

### The link frame

`Central::frame` and `Peripheral::frame` are **one slot** each, driven by
`Config::tx_rx_ratio`:

- The phase is `(slot_count % (tx + rx)) < tx` on the central. When the PHY
  has its own cadence (`Phy::slot_count() != 0`, the MPSL chain) the hardware
  slot count is the authority; otherwise the link's own `slot_step` counter
  paces the bare radio.
- The central runs `tx` TX slots (Data, ACK/NACK, or Beacon when there is no
  payload) then `rx` RX slots (the reverse listen).
- The peripheral mirrors: RX on the central's TX slots, TX on the central's
  RX slots (the reverse Data). Until it has actually received Data it uses an
  acquisition duty cycle (RX on even phases, `SlotRequest` on odd phases on
  MPSL) so the first contact does not depend on guessing the phase.
- Cadence is negotiated once: the peripheral advertises
  `Packet::SlotRequest { min_slot_us }`, the central adopts
  `max(own_min, peer_min)`, and beacons advertise the chosen `slot_us`.
- Reliability is **selective-repeat ARQ**: each `Data` carries a cumulative
  ACK + a variable-length NACK bitmap (`Vec<u8, NACK_BYTES>`). The NACK
  bitmap is slot-position based: it covers the slots of the last TX run
  (e.g. the 8 forward slots in an 8:2 ratio), and the sender maps each bit
  back to the seq it sent in that slot.
  The cumulative ACK is still sequence based and drains the in-flight window;
  the receiver buffers out-of-order packets and delivers in order
  (exactly-once).

### The TX run

The central's consecutive TX slots form a burst: `Phy::transmit_burst_begin`
ramps the radio once, `Phy::transmit_burst_send` sends each subsequent packet
(the on-air only). Backends without the burst (the MPSL) return
`Error::Unsupported` and the link falls back to the plain `transmit`.

### The hopping

The `Scheduler` holds the 25-channel sequence (the network-seeded
permutation). Hopping is **beacon-driven** and **gated by the connection state
machine** (`connection-state-machine.md`):

- While `Disconnected`, both sides pin the scheduler to
  `Config::initial_channel`; a miss is the normal not-yet-aligned condition,
  not an interference signal.
- Once `Connected`, the central advances the index after
  `HOP_MISS_THRESHOLD` consecutive missed RX slots. The peripheral never
  advances the hop locally: it re-syncs from each beacon's `channel_index`.
- No free-running local hop — the beacon is the shared clock.

### The security

`Security` carries the key + the `CipherMode`:

- `Security::new` → `CipherMode::ChaCha` (the software ChaCha20-Poly1305, the
  256-bit key, the default).
- `Security::with_ccm` → `CipherMode::Ccm` (the hardware AES-CCM, the 128-bit
  key — the phy's `ccm_crypt`, the 4-byte MIC).

The link dispatches on `cipher.mode` in `encrypt_payload` / `decrypt_payload`;
the CCM appends/verifies the MIC, the ChaCha carries its own tag.

## 3. Phy (the bottom)

The trait is the entire contract, and **`thunders` is generic over it**
(`Central<P: Phy>` / `Peripheral<P: Phy>`) — the link layer is constrained by
this trait and cannot see past it:

```rust
pub trait Phy {
    type Error;

    // Transport
    async fn set_channel(&mut self, ch: u8);
    async fn set_address(&mut self, addr: &Address);
    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>>;
    async fn receive(&mut self, buf: &mut [u8], timeout: Duration)
        -> Result<Option<usize>, Error<Self::Error>>;
    async fn flush(&mut self);
    async fn wait_slot(&mut self) {}                   // bare: pace an empty slot; MPSL: no-op

    // Timing measurements + peer alignment (defaults shown are the no-op fallbacks)
    fn rx_window_us(&self) -> u16 { 0 }
    fn set_peer_rx_window(&mut self, us: u16) {}
    fn rx_en_offset_us(&self) -> u8 { 0 }
    fn tx_en_offset_us(&self) -> u8 { 0 }
    fn rx_ramp_us(&self) -> u8 { 0 }
    fn tx_ramp_us(&self) -> u8 { 0 }
    fn set_peer_rx_en_offset(&mut self, us: u8) {}
    fn set_peer_tx_en_offset(&mut self, us: u8) {}
    fn set_peer_rx_ramp(&mut self, us: u8) {}
    fn set_peer_tx_ramp(&mut self, us: u8) {}
    fn set_tx_delay_sweep(&mut self, sweep: bool) {}

    // Slot cadence
    fn slot_count(&self) -> u32 { 0 }                  // 0 = software-paced (bare)
    fn slot_period_us(&self) -> u16 { 0 }              // current cadence, advertised in beacons
    fn min_slot_period_us(&self) -> u16 { 0 }          // this PHY's floor for cadence negotiation
    fn fallback_slot_period_us(&self) -> u16 { 0 }     // pre-negotiation period
    fn align_slot_period(&mut self, us: u16) {}        // adopt the negotiated cadence

    // Optional hardware acceleration (both default to Error::Unsupported)
    fn transmit_burst_begin(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        Err(Error::Unsupported)
    }
    fn transmit_burst_send(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>> {
        Err(Error::Unsupported)
    }
    fn ccm_crypt(
        &mut self,
        key: &[u8; 16],
        nonce: &[u8; 13],
        payload: &mut [u8],
        mic: &mut [u8; 4],
        encrypt: bool,
    ) -> Result<(), Error<Self::Error>> {
        Err(Error::Unsupported)
    }
}
```

**The boundary (the wire format):** `transmit(pkt)` / `receive(buf)` carry the
**payload only** — the postcard bytes of a `Packet`. The on-air `[length byte |
payload]` framing (the radio's S0/length byte) is the phy's concern: the phy
prepends the length on TX and strips it on RX. `thunders` never sees the
length byte; postcard never frames itself.

**What the phy owns:** the channel, the address, the CRC (the radio's
hardware CRC-16 via `crcstatus`), the length byte, the TX/RX turnaround, the
slot pacing, and the timing measurements used by slot alignment. Optional
hardware accelerators (`transmit_burst_*`, `ccm_crypt`) are behind the same
trait; protocol state (seq, hop, ARQ windows) never lives in the phy.

Two backends:

- **`NrfRadioPhy`** (the bare RADIO) — the direct register access via the
  nrf-pac, the DWT-capped polled RX, the burst TX (the ramp amortized,
  ~10-12 kHz one-way), the hardware CCM (the radio's AES-CCM over the
  EasyDMA); the bare pacing is the link's software slot grid.
- **`MpslRadioPhy`** (the MPSL timeslots) — the radio inside the granted
  timeslots (coexists with BLE); it implements the same `Phy` contract (the
  burst and the CCM return `Unsupported`). The slot chain and the follower
  PLL live in the callback; the link layer sees only the `Phy` trait.

## The data flow

```
TX:  thunders builds Packet → postcard::to_slice → Phy::transmit (or the burst)
RX:  Phy::receive returns the bytes → postcard::from_bytes → thunders parses + accepts (the seq)
```

## Feature flags

`thunders`:

| flag | effect |
|---|---|
| `secure` | enables the `Security` layer (the `encrypt_payload`/`decrypt_payload` dispatch) |
| `defmt` | the format impls + the logging |

`thunders-phy-nrf`:

| flag | effect |
|---|---|
| `nrf52833` / `nrf52840` / `nrf5340` / `nrf5340-net` / `nrf54l15` / `nrf54l20` | chip feature forwarded to `embassy-nrf` and `nrf-pac` |
| `mpsl` | compiles the MPSL timeslot backend (`dep:nrf-mpsl`, `dep:rtt-target`) |
| `defmt` | logging + forwarded `thunders/defmt` |
| `_nrf54` | internal marker enabling nRF54-specific register paths |

The six example binaries additionally declare `radio-1m` (select
`RadioMode::Nrf1Mbit`; the bare examples also raise the software slot period
to 600 µs) and `central`/`peripheral`; the 5340 examples declare `host`.
