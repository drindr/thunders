//! Optional security layer for `thunders`.
//!
//! Enabled by the `secure` Cargo feature.  When enabled, `Data` packet
//! payloads are encrypted and authenticated in-place. Two backends:
//!
//! - `CipherMode::ChaCha` (the default): the software ChaCha20-Poly1305
//!   with a 256-bit key - the portable, no-phy-dependency option.
//! - `CipherMode::Ccm`: the radio's hardware AES-CCM (the AEAD) with a
//!   128-bit key (the first 16 bytes of the key) - the phy provides
//!   `Phy::ccm_crypt`.

/// The cipher backend.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CipherMode {
    /// Software ChaCha20-Poly1305 (the 256-bit key).
    ChaCha,
    /// Hardware AES-CCM (the 128-bit key; the phy's `ccm_crypt`).
    Ccm,
}

/// Pre-shared security material.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Security {
    /// 256-bit pre-shared key (the CCM uses the first 16 bytes).
    pub key: [u8; 32],
    /// The cipher backend.
    pub mode: CipherMode,
}

impl Security {
    /// Create a new security context (the software ChaCha20-Poly1305).
    pub const fn new(key: [u8; 32]) -> Self {
        Self {
            key,
            mode: CipherMode::ChaCha,
        }
    }

    /// Create a security context using the hardware AES-CCM.
    pub const fn with_ccm(key: [u8; 32]) -> Self {
        Self {
            key,
            mode: CipherMode::Ccm,
        }
    }
}

/// Security operation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CryptoError {
    /// Encryption failed (usually plaintext too long for the buffer).
    Encrypt,
    /// Decryption or authentication failed (bad key, tampered data, wrong nonce).
    Decrypt,
}

/// Build a 96-bit ChaCha20-Poly1305 nonce from frame context.
///
/// The nonce mixes the epoch, sequence number, hop index, and the
/// sender role so that central->peripheral and peripheral->central
/// packets in the same frame never share a nonce.
pub fn make_nonce(epoch: u32, seq: u8, channel_index: u8, sender_is_central: bool) -> [u8; 12] {
    let mut n = [0u8; 12];
    n[0..4].copy_from_slice(&epoch.to_le_bytes());
    n[4] = seq;
    n[5] = channel_index;
    n[6] = if sender_is_central { 0 } else { 1 };
    n
}

/// Build a 13-byte AES-CCM nonce from the frame's seq + the direction.
///
/// The epoch/channel were dropped: they are local counters that drift between
/// the free-running chains, so they broke the nonce match between the sender
/// and the receiver (the MIC then failed). The seq is carried in the packet,
/// so both sides derive the same nonce. The known limitation: the 8-bit seq
/// wraps after 256 frames per direction, so the nonce repeats — the proper
/// long-term fix is the phase-lock (a shared epoch), which is the deferred
/// slot-phase work.
pub fn make_nonce_13(_epoch: u32, seq: u8, _channel_index: u8, sender_is_central: bool) -> [u8; 13] {
    // Layout (matched by `ccm_crypt`): [seq | 0*4 | channel(dropped) |
    // direction | 0*6]. The seq is the packet counter; the direction goes
    // in its own byte (nonce[6]), NOT inside the counter.
    let mut n = [0u8; 13];
    n[0] = seq;
    n[6] = if sender_is_central { 0 } else { 1 };
    n
}

#[cfg(feature = "secure")]
mod secure_impl {
    use crate::MAX_PAYLOAD;
    use super::{CryptoError, Security};
    use aead::{AeadInOut, Buffer, KeyInit};
    use chacha20poly1305::{ChaCha20Poly1305, Key, Nonce};
    use heapless::Vec as HeaplessVec;

    /// The cipher (the mode + the backend state).
    pub struct Cipher {
        pub mode: super::CipherMode,
        pub key: [u8; 32],
        chacha: ChaCha20Poly1305,
    }

    impl Cipher {
        /// Create a cipher from a pre-shared key + the mode.
        pub fn new(sec: &Security) -> Self {
            let key = Key::from(sec.key);
            Self {
                mode: sec.mode,
                key: sec.key,
                chacha: ChaCha20Poly1305::new(&key),
            }
        }

        /// Encrypt and authenticate `payload` in place with the ChaCha backend.
        pub fn encrypt(
            &self,
            payload: &mut HeaplessVec<u8, MAX_PAYLOAD>,
            nonce: &[u8; 12],
        ) -> Result<(), CryptoError> {
            let nonce = Nonce::from(*nonce);
            let mut buf = CryptoBuf(payload);
            self.chacha
                .encrypt_in_place(&nonce, b"", &mut buf)
                .map_err(|_| CryptoError::Encrypt)
        }

        /// Decrypt and verify `payload` in place with the ChaCha backend.
        pub fn decrypt(
            &self,
            payload: &mut HeaplessVec<u8, MAX_PAYLOAD>,
            nonce: &[u8; 12],
        ) -> Result<(), CryptoError> {
            let nonce = Nonce::from(*nonce);
            let mut buf = CryptoBuf(payload);
            self.chacha
                .decrypt_in_place(&nonce, b"", &mut buf)
                .map_err(|_| CryptoError::Decrypt)
        }
    }

    /// Newtype wrapper so we can implement the foreign `aead::Buffer` trait.
    struct CryptoBuf<'a>(&'a mut HeaplessVec<u8, MAX_PAYLOAD>,
    );

    impl AsRef<[u8]> for CryptoBuf<'_> {
        fn as_ref(&self) -> &[u8] {
            self.0
        }
    }

    impl AsMut<[u8]> for CryptoBuf<'_> {
        fn as_mut(&mut self) -> &mut [u8] {
            self.0
        }
    }

    impl Buffer for CryptoBuf<'_> {
        fn extend_from_slice(
            &mut self,
            other: &[u8],
        ) -> aead::Result<()> {
            self.0
                .extend_from_slice(other)
                .map_err(|_| aead::Error)
        }

        fn truncate(&mut self, len: usize) {
            self.0.resize(len, 0).ok();
        }
    }
}

#[cfg(feature = "secure")]
pub use secure_impl::Cipher;

#[cfg(feature = "secure")]
impl Security {
    /// Build a ChaCha20-Poly1305 cipher from this pre-shared key.
    pub fn cipher(&self) -> Cipher {
        Cipher::new(self)
    }
}

#[cfg(all(test, feature = "secure"))]
mod tests {
    use super::*;
    use crate::MAX_PAYLOAD;
    use heapless::Vec;

    #[test]
    fn round_trip_crypto() {
        let sec = Security::new([0xAB; 32]);
        let cipher = sec.cipher();
        let nonce = make_nonce(42, 7, 13, true);

        let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
        payload.extend_from_slice(b"hello mouse").unwrap();

        cipher.encrypt(&mut payload, &nonce).unwrap();
        assert_ne!(&payload[..], b"hello mouse");
        assert_eq!(payload.len(), b"hello mouse".len() + 16);

        cipher.decrypt(&mut payload, &nonce).unwrap();
        assert_eq!(&payload[..], b"hello mouse");
    }

    #[test]
    fn tamper_fails() {
        let sec = Security::new([0xCD; 32]);
        let cipher = sec.cipher();
        let nonce = make_nonce(1, 2, 3, false);

        let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
        payload.extend_from_slice(b"secret").unwrap();
        cipher.encrypt(&mut payload, &nonce).unwrap();

        payload[0] ^= 0xFF;
        assert!(cipher.decrypt(&mut payload, &nonce).is_err());
    }
}
