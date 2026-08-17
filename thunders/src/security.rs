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
    #[must_use]
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

/// Build a 96-bit ChaCha20-Poly1305 nonce from the packet seq + direction.
///
/// The nonce binds only to what travels *inside* the packet (the seq) plus
/// the direction, so a retransmission of the same seq derives the same
/// nonce and the ciphertext round-trips. The epoch/channel were dropped:
/// they are local slot counters that drift between the free-running chains,
/// so binding them broke the nonce match between the sender and the receiver
/// (the MIC then failed). The u16 seq wraps after 65536 frames per direction.
pub fn make_nonce(seq: u16, sender_is_central: bool) -> [u8; 12] {
    let mut n = [0u8; 12];
    n[0..2].copy_from_slice(&seq.to_le_bytes());
    n[2] = if sender_is_central { 0 } else { 1 };
    n
}

/// Build a 13-byte AES-CCM nonce from the packet seq + the direction.
///
/// Same rationale as [`make_nonce`]: only the seq (carried in the packet,
/// stable across retransmissions) and the direction feed the nonce, so the
/// sender and receiver derive the same value for any send/retransmit of the
/// same seq.
pub fn make_nonce_13(seq: u16, sender_is_central: bool) -> [u8; 13] {
    // Layout (matched by `ccm_crypt`): [seq(2) | 0*4 | direction | 0*6].
    // The direction lives in its own byte (nonce[6]), NOT inside the counter.
    let mut n = [0u8; 13];
    n[0..2].copy_from_slice(&seq.to_le_bytes());
    n[6] = if sender_is_central { 0 } else { 1 };
    n
}

#[cfg(feature = "secure")]
mod secure_impl {
    use super::{CryptoError, Security};
    use crate::MAX_PAYLOAD;
    use aead::{AeadInOut, Buffer, KeyInit};
    use chacha20poly1305::{ChaCha20Poly1305, Key, Nonce};
    use heapless::Vec as HeaplessVec;

    /// The cipher (the mode + the backend state).
    pub struct Cipher {
        /// The selected cipher backend.
        pub mode: super::CipherMode,
        /// The 256-bit pre-shared key (CCM uses the first 16 bytes).
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
    struct CryptoBuf<'a>(&'a mut HeaplessVec<u8, MAX_PAYLOAD>);

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
        fn extend_from_slice(&mut self, other: &[u8]) -> aead::Result<()> {
            self.0.extend_from_slice(other).map_err(|_| aead::Error)
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
        let nonce = make_nonce(7, true);

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
        let nonce = make_nonce(2, false);

        let mut payload = Vec::<u8, MAX_PAYLOAD>::new();
        payload.extend_from_slice(b"secret").unwrap();
        cipher.encrypt(&mut payload, &nonce).unwrap();

        payload[0] ^= 0xFF;
        assert!(cipher.decrypt(&mut payload, &nonce).is_err());
    }
}
