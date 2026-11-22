//! # error — Error Types for TMAG5273 Driver

use crate::types::DeviceStatus;

/// Errors produced by the TMAG5273 driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// Low-level I2C communication error.
    I2c(E),
    /// Manufacturer ID did not match expected `0x5449` ('TI').
    InvalidManufacturerId(u16),
    /// Device version did not match expected variant.
    VersionMismatch {
        /// Expected silicon version.
        expected: u8,
        /// Actual version read from device.
        got: u8,
    },
    /// CRC verification failed on received transaction.
    CrcMismatch {
        /// Expected CRC received from device.
        expected: u8,
        /// Computed CRC over the data payload.
        computed: u8,
    },
    /// CORDIC angle engine was not enabled in configuration.
    AngleNotEnabled,
    /// Register returned an unrecognized or invalid value.
    InvalidRegisterValue {
        /// Register address.
        register: u8,
        /// Raw value read.
        value: u8,
    },
    /// Hardware diagnostic fault detected.
    Fault(DeviceStatus),
    /// No supported TMAG5273 device found on candidate addresses.
    DeviceNotFound,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Self {
        Error::I2c(err)
    }
}

impl<E: core::fmt::Display> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::I2c(e) => write!(f, "I2C error: {}", e),
            Self::InvalidManufacturerId(id) => {
                write!(f, "invalid manufacturer ID: 0x{:04X} (expected 0x5449)", id)
            }
            Self::VersionMismatch { expected, got } => {
                write!(
                    f,
                    "version mismatch: expected 0x{:02X}, got 0x{:02X}",
                    expected, got
                )
            }
            Self::CrcMismatch { expected, computed } => {
                write!(
                    f,
                    "CRC mismatch: expected 0x{:02X}, computed 0x{:02X}",
                    expected, computed
                )
            }
            Self::AngleNotEnabled => write!(f, "CORDIC angle calculation is not enabled"),
            Self::InvalidRegisterValue { register, value } => {
                write!(
                    f,
                    "invalid register value 0x{:02X} at address 0x{:02X}",
                    value, register
                )
            }
            Self::Fault(_) => write!(f, "hardware diagnostic fault active"),
            Self::DeviceNotFound => write!(f, "no TMAG5273 device found on candidate addresses"),
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::error::Error + 'static> std::error::Error for Error<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::I2c(e) => Some(e),
            _ => None,
        }
    }
}
