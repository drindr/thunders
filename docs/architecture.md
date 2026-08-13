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
    Beacon  { epoch, channel_index, flags },
    Data    { seq, payload: Vec<u8, MAX_PAYLOAD> },
    PairingRequest  { id },
    PairingResponse { id, key: [u8; 16] },
}
```

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
| `link` | `Central` / `Peripheral` — the slot-aware frame state machines |
| `config` | `Config` (the network, the address, the role, the TX:RX ratio), `Address`, `Role`, the hop sequence |
| `scheduler` | `Scheduler` — the hop sequence (the network-seeded LCG permutation) |
| `packet` | the `Packet` wire format (see above) |
| `security` | `Security` + `CipherMode` — the ChaCha20-Poly1305 (software) or the AES-CCM (the phy's hardware) |
| `error` | `Error<P>` — the PHY-typed error |

### The link frame

`Central::frame` and `Peripheral::frame` are **one slot** each, driven by
`Config::tx_rx_ratio`:

- The central runs `tx` TX slots (the PING/Data, or the Beacon when there is
  no payload — the beacon carries the hop `channel_index`) then `rx` RX slots
  (the reverse listen).
- The peripheral mirrors: RX on the central's TX slots, TX on the central's
  RX slots (the reverse Data).
- Reliability is the **seq window** (`accept_seq`), not acks — the link is
  fire-and-forget full-duplex.

### The TX run

The central's consecutive TX slots form a burst: `Phy::transmit_burst_begin`
ramps the radio once, `Phy::transmit_burst_send` sends each subsequent packet
(the on-air only). Backends without the burst (the MPSL) return
`Error::Unsupported` and the link falls back to the plain `transmit`.

### The hopping

The `Scheduler` holds the 25-channel sequence (the network-seeded
permutation). Hopping is **beacon-driven**: the central advances the index
after `HOP_MISS_THRESHOLD` consecutive missed RX slots (the interference
signal); the peripheral follows the beacon's `channel_index`. No free-running
local hop — the beacon is the shared clock.

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
    async fn set_channel(&mut self, ch: u8);
    async fn set_address(&mut self, addr: &Address);
    async fn transmit(&mut self, pkt: &[u8]) -> Result<(), Error<Self::Error>>;
    async fn receive(&mut self, buf: &mut [u8], timeout: Duration) -> Result<Option<usize>, _>;
    async fn transmit_receive(...) -> ...;      // the TX+RX in one await (the default)
    async fn flush(&mut self);
    async fn adjust_period(&mut self, corr: i32) {}   // the sync PLL hook
    fn transmit_burst_begin(&mut self, pkt) -> ...;   // the burst (the default Unsupported)
    fn transmit_burst_send(&mut self, pkt) -> ...;
    fn ccm_crypt(&mut self, key, nonce, payload, mic, encrypt) -> ...;  // the AES-CCM (the default Unsupported)
}
```

**The boundary (the wire format):** `transmit(pkt)` / `receive(buf)` carry the
**payload only** — the postcard bytes of a `Packet`. The on-air `[length byte |
payload]` framing (the radio's S0/length byte) is the phy's concern: the phy
prepends the length on TX and strips it on RX. `thunders` never sees the
length byte; postcard never frames itself.

**What the phy owns:** the channel, the address, the CRC (the radio's
hardware CRC-16 via `crcstatus`), the length byte, and the TX/RX turnaround.
Nothing protocol-level (no seq, no hop, no crypto) lives in the phy.

Two backends:

- **`NrfRadioPhy`** (the bare RADIO) — the direct register access via the
  nrf-pac, the interrupt-driven RX, the burst TX (the ramp amortized,
  ~10-12 kHz one-way), the hardware CCM (the radio's AES-CCM over the
  EasyDMA), the `adjust_period` no-op (the pacing is the link's).
- **`MpslRadioPhy`** (the MPSL timeslots) — the radio inside the granted
  timeslots (coexists with BLE); it implements the same `Phy` contract (the
  burst and the CCM return `Unsupported`). It **also** exposes a raw
  zero-copy surface (`tx_send(closure)` / `rx_receive(buf)`) for direct use
  without the link layer — but the `Phy` impl is what keeps it usable from
  `thunders`.

## The data flow

```
TX:  thunders builds Packet → postcard::to_slice → Phy::transmit (or the burst)
RX:  Phy::receive returns the bytes → postcard::from_bytes → thunders parses + accepts (the seq)
```

## Feature flags

| flag | effect |
|---|---|
| `secure` | enables the `Security` layer (the `encrypt_payload`/`decrypt_payload` dispatch) |
| `defmt` | the format impls + the logging |
