//! # barefoc-encoders
//!
//! Pure-Rust, `#![no_std]` drivers for rotary encoders commonly used in
//! Field Oriented Control (FOC):
//!
//! - **AS5047P / AS5047D**: 14-bit high-speed SPI magnetic angle sensor with Dynamic
//!   Angle Error Compensation (DAEC), parity validation, and diagnostic decoding.
//! - **AS5048A**: 14-bit SPI magnetic angle sensor with even-parity verification
//!   and error diagnostics.
//! - **AS5600**: 12-bit I²C magnetic angle sensor with automatic gain control (AGC)
//!   and magnet placement detection (`MD`, `ML`, `MH`).
//! - **MT6701**: 14-bit high-speed SSI Mode 2 magnetic encoder with 24-bit frame
//!   unpacking, status decoding (`MGH`, `MGL`, `PUSH`, `LOS`), and 6-bit CRC checking.
//! - **Generic SSI**: Configurable 12..24 bit industrial absolute SSI encoder reader
//!   with optional Gray code decoding.
//!
//! All drivers implement [`barefoc::traits::sensor::Sensor`], enabling 1-line linking
//! into BareFOC motors (`motor.link_sensor(&mut sensor)`).
//!
//! ## Upstream References
//! - [`MagneticSensorAS5047`](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/as5047)
//! - [`MagneticSensorAS5048A`](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/as5048a)
//! - [`MagneticSensorAS5600`](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/as5600)
//! - [`MagneticSensorMT6701SSI`](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/mt6701)

pub mod i2c;
pub mod spi;
pub mod ssi;

pub use i2c::as5600::{As5600, As5600Status, AS5600_DEFAULT_I2C_ADDR};
pub use spi::as5047p::{As5047p, As5047pDiagnostics, As5047pErrorFlags};
pub use spi::as5048a::{As5048a, As5048aDiagnostics, As5048aErrorFlags};
pub use ssi::generic_ssi::GenericSsi;
pub use ssi::mt6701::{Mt6701Ssi, Mt6701Status};
#[cfg(feature = "mt6xxx")]
pub use mt6xxx_rs as mt6xxx;
#[cfg(feature = "mt6xxx")]
pub use mt6xxx_rs::{Mt6701, Mt6701Frame, Mt6701Telemetry, SpiMode};

#[cfg(feature = "tmag")]
pub use tmag_rs as tmag;
#[cfg(feature = "tmag")]
pub use tmag_rs::{Tmag5273, Tmag5273Sensor, Tmag5273Sensor2D};

/// Common error type for encoder drivers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EncoderError<CommErr> {
    /// Low-level bus communication error (SPI or I²C).
    Communication(CommErr),
    /// Parity verification error in received data frame.
    ParityError,
    /// CRC verification error in received data frame.
    CrcError,
    /// Device-level error flag indicated in the protocol response.
    DeviceError,
    /// Magnetic field is too weak.
    MagnetTooWeak,
    /// Magnetic field is too strong.
    MagnetTooStrong,
    /// Magnet is not detected.
    MagnetNotDetected,
    /// Tracking loss / loss of signal.
    LossOfSignal,
    /// Invalid parameter or configuration value.
    InvalidParameter,
}

impl<CommErr> From<CommErr> for EncoderError<CommErr> {
    fn from(err: CommErr) -> Self {
        EncoderError::Communication(err)
    }
}

#[cfg(feature = "mt6xxx")]
impl<CommErr> From<mt6xxx_rs::Error<CommErr>> for EncoderError<CommErr> {
    fn from(err: mt6xxx_rs::Error<CommErr>) -> Self {
        match err {
            mt6xxx_rs::Error::Communication(e) => EncoderError::Communication(e),
            mt6xxx_rs::Error::CrcError { .. } | mt6xxx_rs::Error::CrcExhausted => {
                EncoderError::CrcError
            }
        }
    }
}

impl<CommErr: core::fmt::Display> core::fmt::Display for EncoderError<CommErr> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Communication(e) => write!(f, "communication error: {}", e),
            Self::ParityError => write!(f, "parity verification error"),
            Self::CrcError => write!(f, "CRC verification error"),
            Self::DeviceError => write!(f, "device reported error"),
            Self::MagnetTooWeak => write!(f, "magnet magnetic field too weak"),
            Self::MagnetTooStrong => write!(f, "magnet magnetic field too strong"),
            Self::MagnetNotDetected => write!(f, "magnet not detected"),
            Self::LossOfSignal => write!(f, "loss of signal"),
            Self::InvalidParameter => write!(f, "invalid parameter"),
        }
    }
}

#[cfg(feature = "std")]
impl<CommErr: std::error::Error + 'static> std::error::Error for EncoderError<CommErr> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Communication(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(test)]
pub(crate) mod test_utils {
    use barefoc::common::time::Clock;
    use core::cell::Cell;

    pub struct MockClock(Cell<u32>);

    impl MockClock {
        pub fn new() -> Self {
            Self(Cell::new(0))
        }

        #[allow(dead_code)]
        pub fn advance_us(&self, us: u32) {
            self.0.set(self.0.get().wrapping_add(us));
        }

        #[allow(dead_code)]
        pub fn set_us(&self, us: u32) {
            self.0.set(us);
        }
    }

    impl Clock for MockClock {
        fn now_us(&self) -> u32 {
            self.0.get()
        }
    }
}
