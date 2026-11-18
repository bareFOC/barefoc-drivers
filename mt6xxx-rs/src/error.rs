//! # error — Error types for the MT6xxx encoder family

use core::fmt;

/// MT6xxx driver error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<CommErr> {
    /// Low-level SPI bus communication error.
    Communication(CommErr),
    /// Received frame failed CRC-6 verification.
    CrcError {
        /// CRC-6 checksum received in the SPI frame.
        received: u8,
        /// Expected CRC-6 checksum computed from the 18 data bits.
        expected: u8,
    },
    /// All retry attempts exhausted due to persistent CRC failures or bus faults.
    CrcExhausted,
}

impl<CommErr: fmt::Display> fmt::Display for Error<CommErr> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Communication(err) => write!(f, "SPI communication error: {err}"),
            Self::CrcError { received, expected } => write!(
                f,
                "CRC-6 mismatch: received 0x{received:02X}, expected 0x{expected:02X}"
            ),
            Self::CrcExhausted => write!(f, "All CRC retry attempts exhausted"),
        }
    }
}
