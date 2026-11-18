//! # ssi::generic_ssi — Generic Industrial SSI Absolute Encoder Driver
//!
//! Provides a configurable Synchronous Serial Interface (SSI) driver over SPI
//! supporting industrial absolute rotary encoders from 12 to 24 bits
//! (e.g. Renishaw, AksIM, Broadcom AEAT-8800, Baumer, Kübler, Pepperl+Fuchs).
//!
//! Features configurable resolution (12..24 bits) and optional Gray code decoding.

use barefoc::common::time::Clock;
use barefoc::math::_2PI;
use barefoc::traits::sensor::{Sensor, SensorBase};
use core::cell::RefCell;
use embedded_hal::spi::SpiDevice;

use crate::EncoderError;

/// Converts standard reflected binary Gray code to standard binary integer.
#[inline]
pub fn gray_to_binary(mut gray: u32) -> u32 {
    let mut mask = gray >> 1;
    while mask != 0 {
        gray ^= mask;
        mask >>= 1;
    }
    gray
}

/// Generic industrial SSI absolute encoder driver over SPI.
pub struct GenericSsi<SPI> {
    spi: RefCell<SPI>,
    base: SensorBase,
    resolution_bits: u8,
    gray_code: bool,
    cpr: f32,
}

impl<SPI: SpiDevice> GenericSsi<SPI> {
    /// Creates a new generic SSI driver with the given resolution in bits ($12 \dots 24$).
    ///
    /// # Errors
    /// Returns `Err(EncoderError::InvalidParameter)` if `resolution_bits` is not between 12 and 24.
    pub fn new(spi: SPI, resolution_bits: u8) -> Result<Self, EncoderError<SPI::Error>> {
        if !(12..=24).contains(&resolution_bits) {
            return Err(EncoderError::InvalidParameter);
        }

        let cpr = (1u32 << resolution_bits) as f32;
        Ok(Self {
            spi: RefCell::new(spi),
            base: SensorBase::new(),
            resolution_bits,
            gray_code: false,
            cpr,
        })
    }

    /// Configures whether incoming SSI data is Gray code encoded.
    pub fn with_gray_code(mut self, enabled: bool) -> Self {
        self.gray_code = enabled;
        self
    }

    /// Sets whether incoming SSI data is Gray code encoded.
    pub fn set_gray_code(&mut self, enabled: bool) {
        self.gray_code = enabled;
    }

    /// Returns the active resolution in bits.
    pub fn resolution_bits(&self) -> u8 {
        self.resolution_bits
    }

    /// Returns the counts per revolution ($2^{\text{resolution\_bits}}$).
    pub fn cpr(&self) -> f32 {
        self.cpr
    }

    /// Reads a 24-bit frame over SPI, aligns the most significant $N$ bits,
    /// applies Gray code decoding if enabled, and returns the raw count.
    pub fn read_raw_count(&self) -> Result<u32, EncoderError<SPI::Error>> {
        let tx = [0u8; 3];
        let mut rx = [0u8; 3];
        self.spi
            .borrow_mut()
            .transfer(&mut rx, &tx)
            .map_err(EncoderError::Communication)?;

        let word = ((rx[0] as u32) << 16) | ((rx[1] as u32) << 8) | (rx[2] as u32);
        let shift = 24 - self.resolution_bits;
        let mask = if self.resolution_bits == 32 {
            0xFFFF_FFFF
        } else {
            (1u32 << self.resolution_bits) - 1
        };
        let raw = (word >> shift) & mask;

        if self.gray_code {
            Ok(gray_to_binary(raw))
        } else {
            Ok(raw)
        }
    }

    /// Reads the angle in radians in `[0, 2π)`.
    pub fn read_angle_radians(&self) -> Result<f32, EncoderError<SPI::Error>> {
        let count = self.read_raw_count()?;
        Ok((count as f32) * (_2PI / self.cpr))
    }

    /// Returns a reference to the embedded [`SensorBase`] state.
    pub fn base(&self) -> &SensorBase {
        &self.base
    }

    /// Returns a mutable reference to the embedded [`SensorBase`] state.
    pub fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    /// Returns the shaft angle from the most recent update in radians `[0, 2π)`.
    pub fn get_mechanical_angle(&self) -> f32 {
        self.base.angle_prev
    }
}

impl<C: Clock, SPI: SpiDevice> Sensor<C> for GenericSsi<SPI> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        match self.read_raw_count() {
            Ok(count) => (count as f32) * (_2PI / self.cpr),
            Err(_) => -1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec;

    use super::*;
    use crate::test_utils::MockClock;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

    #[test]
    fn test_gray_code_conversion() {
        // Binary 0b1010 (10) -> Gray 0b1111 (15)
        assert_eq!(gray_to_binary(0b1111), 10);
        // Gray 0 -> Binary 0
        assert_eq!(gray_to_binary(0), 0);
        // Binary 0b0110 (6) -> Gray 0b0101 (5)
        assert_eq!(gray_to_binary(0b0101), 6);
    }

    #[test]
    fn test_generic_ssi_12bit_scaling() {
        // 12-bit encoder (CPR = 4096).
        // If angle is 1024 (quarter rotation = 0x400):
        // In 24-bit frame, 12 bits are in bits 23..12 -> 0x400 << 12 = 0x400000 -> bytes [0x40, 0x00, 0x00]
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x00, 0x00, 0x00], vec![0x40, 0x00, 0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x00, 0x00, 0x00], vec![0x40, 0x00, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let sensor = GenericSsi::new(spi.clone(), 12).unwrap();

        let raw = sensor.read_raw_count().unwrap();
        assert_eq!(raw, 1024);
        let rad = sensor.read_angle_radians().unwrap();
        assert!((rad - (_2PI * 0.25)).abs() < 1e-4);
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_generic_ssi_16bit_scaling() {
        // 16-bit encoder (CPR = 65536).
        // Angle 32768 (half rotation = 0x8000).
        // In 24-bit frame, 16 bits are in bits 23..8 -> 0x8000 << 8 = 0x800000 -> bytes [0x80, 0x00, 0x00]
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x00, 0x00, 0x00], vec![0x80, 0x00, 0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x00, 0x00, 0x00], vec![0x80, 0x00, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let sensor = GenericSsi::new(spi.clone(), 16).unwrap();

        let raw = sensor.read_raw_count().unwrap();
        assert_eq!(raw, 32768);
        let rad = sensor.read_angle_radians().unwrap();
        assert!((rad - barefoc::math::_PI).abs() < 1e-4);
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_generic_ssi_24bit_scaling() {
        // 24-bit encoder (CPR = 16777216).
        // Angle = 0x400000 (quarter rotation).
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x00, 0x00, 0x00], vec![0x40, 0x00, 0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x00, 0x00, 0x00], vec![0x40, 0x00, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let sensor = GenericSsi::new(spi.clone(), 24).unwrap();

        let raw = sensor.read_raw_count().unwrap();
        assert_eq!(raw, 0x400000);
        let rad = sensor.read_angle_radians().unwrap();
        assert!((rad - (_2PI * 0.25)).abs() < 1e-4);
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_generic_ssi_sensor_trait() {
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x00, 0x00, 0x00], vec![0x80, 0x00, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut sensor = GenericSsi::new(spi.clone(), 16).unwrap();
        let clock = MockClock::new();

        sensor.update(&clock);
        let angle = sensor.get_mechanical_angle();
        assert!((angle - barefoc::math::_PI).abs() < 1e-4);
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_invalid_resolution() {
        let mut spi = SpiMock::<u8>::new(&[]);
        assert_eq!(
            GenericSsi::new(spi.clone(), 8).map(|_| ()),
            Err(EncoderError::InvalidParameter)
        );
        spi.done();
    }
}
