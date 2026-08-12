//! Optional security layer for `thunders`.
//!
//! Enabled by the `secure` Cargo feature.  When enabled, `Data` packet
//! payloads are encrypted and authenticated in-place with
//! ChaCha20-Poly1305 using a pre-shared 256-bit key.

/// Pre-shared security material.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Security {
    /// 256-bit pre-shared key.
    pub key: [u8; 32],
}

impl Security {
    /// Create a new security context.
    pub const fn new(key: [u8; 32]) -> Self {
        Self { key }
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

#[cfg(feature = "secure")]
mod secure_impl {
    use super::{CryptoError, Security, MAX_PAYLOAD};
    use aead::{AeadInOut, Buffer, KeyInit};
    use chacha20poly1305::{ChaCha20Poly1305, Key, Nonce};
    use heapless::Vec as HeaplessVec;

    /// In-place ChaCha20-Poly1305 wrapper.
    pub struct Cipher(ChaCha20Poly1305);

    impl Cipher {
        /// Create a cipher from a pre-shared key.
        pub fn new(sec: &Security) -> Self {
            let key = Key::from(sec.key);
            Self(ChaCha20Poly1305::new(&key))
        }

        /// Encrypt and authenticate `payload` in place.
        pub fn encrypt(
            &self,
            payload: &mut HeaplessVec<u8, MAX_PAYLOAD>,
            nonce: &[u8; 12],
        ) -> Result<(), CryptoError> {
            let nonce = Nonce::from(*nonce);
            let mut buf = CryptoBuf(payload);
            self.0
                .encrypt_in_place(&nonce, b"", &mut buf)
                .map_err(|_| CryptoError::Encrypt)
        }

        /// Decrypt and verify `payload` in place.
        pub fn decrypt(
            &self,
            payload: &mut HeaplessVec<u8, MAX_PAYLOAD>,
            nonce: &[u8; 12],
        ) -> Result<(), CryptoError> {
            let nonce = Nonce::from(*nonce);
            let mut buf = CryptoBuf(payload);
            self.0
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
