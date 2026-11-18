//! # i2c::as5600 — AS5600 12-Bit I²C Magnetic Angle Sensor
//!
//! Ports [`AS5600.h`](source/Arduino-FOC-drivers/src/encoders/as5600/AS5600.h)
//! and [`AS5600.cpp`](source/Arduino-FOC-drivers/src/encoders/as5600/AS5600.cpp)
//! from Arduino-FOC-drivers.
//!
//! The AS5600 is an easy-to-use 12-bit programmable magnetic rotary position
//! sensor communicating over I²C. It features Automatic Gain Control (AGC) and
//! magnetic field status indicators (`MD`, `ML`, `MH`).

use barefoc::common::time::Clock;
use barefoc::math::_2PI;
use barefoc::traits::sensor::{Sensor, SensorBase};
use core::cell::RefCell;
use embedded_hal::i2c::I2c;

use crate::encoders::EncoderError;

/// Standard 7-bit I²C address for the AS5600 sensor (`0x36`).
pub const AS5600_DEFAULT_I2C_ADDR: u8 = 0x36;

/// Counts per revolution for the 12-bit AS5600 sensor ($2^{12} = 4096$).
pub const AS5600_CPR: f32 = 4096.0;

/// Register addresses for AS5600.
pub mod reg {
    /// Number of programming burns remaining.
    pub const ZMCO: u8 = 0x00;
    /// Zero position upper/lower bytes (2 bytes).
    pub const ZPOS: u8 = 0x01;
    /// Maximum position upper/lower bytes (2 bytes).
    pub const MPOS: u8 = 0x03;
    /// Maximum angle upper/lower bytes (2 bytes).
    pub const MANG: u8 = 0x05;
    /// Configuration register (2 bytes).
    pub const CONF: u8 = 0x07;
    /// Status register (`MD`, `ML`, `MH`).
    pub const STATUS: u8 = 0x0B;
    /// Raw unscaled 12-bit angle without filtering/hysteresis (2 bytes).
    pub const RAW_ANGLE: u8 = 0x0C;
    /// Scaled 12-bit angle with hysteresis applied (2 bytes).
    pub const ANGLE: u8 = 0x0E;
    /// Automatic Gain Control value (0..255).
    pub const AGC: u8 = 0x1A;
    /// Magnitude of the magnetic field (2 bytes).
    pub const MAGNITUDE: u8 = 0x1B;
    /// Non-volatile burn command register.
    pub const BURN: u8 = 0xFF;
}

/// Decoded status register bits from AS5600 register 0x0B.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct As5600Status {
    /// Magnet detected: a suitable magnetic field is present.
    pub magnet_detected: bool,
    /// Magnet too weak (ML): the magnetic field is below the recommended threshold.
    pub magnet_too_weak: bool,
    /// Magnet too strong (MH): the magnetic field exceeds the recommended threshold.
    pub magnet_too_strong: bool,
}

/// AS5600 12-bit I²C magnetic angle sensor driver.
pub struct As5600<I2C> {
    i2c: RefCell<I2C>,
    address: u8,
    base: SensorBase,
    use_hysteresis: bool,
}

impl<I2C: I2c> As5600<I2C> {
    /// Creates a new AS5600 driver at default I²C address `0x36`.
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_address(i2c, AS5600_DEFAULT_I2C_ADDR)
    }

    /// Creates a new AS5600 driver with a custom I²C address.
    pub fn new_with_address(i2c: I2C, address: u8) -> Self {
        Self {
            i2c: RefCell::new(i2c),
            address,
            base: SensorBase::new(),
            use_hysteresis: false,
        }
    }

    /// Configures whether to read filtered angle (`ANGLE`, 0x0E) or raw angle (`RAW_ANGLE`, 0x0C).
    pub fn set_use_hysteresis(&mut self, enable: bool) {
        self.use_hysteresis = enable;
    }

    /// Reads an 8-bit register over I²C.
    fn read_u8(&self, reg: u8) -> Result<u8, EncoderError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[reg], &mut buf)
            .map_err(EncoderError::Communication)?;
        Ok(buf[0])
    }

    /// Reads a 16-bit big-endian register over I²C.
    fn read_u16(&self, reg: u8) -> Result<u16, EncoderError<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[reg], &mut buf)
            .map_err(EncoderError::Communication)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Reads the raw 12-bit angle (0..4095) without hysteresis from register `0x0C`.
    pub fn read_raw_angle(&self) -> Result<u16, EncoderError<I2C::Error>> {
        let word = self.read_u16(reg::RAW_ANGLE)?;
        Ok(word & 0x0FFF)
    }

    /// Reads the 12-bit angle (0..4095) with hysteresis from register `0x0E`.
    pub fn read_angle(&self) -> Result<u16, EncoderError<I2C::Error>> {
        let word = self.read_u16(reg::ANGLE)?;
        Ok(word & 0x0FFF)
    }

    /// Reads the active angle counts according to `use_hysteresis`.
    pub fn read_angle_counts(&self) -> Result<u16, EncoderError<I2C::Error>> {
        if self.use_hysteresis {
            self.read_angle()
        } else {
            self.read_raw_angle()
        }
    }

    /// Reads the current shaft angle in radians in `[0, 2π)`.
    pub fn read_angle_radians(&self) -> Result<f32, EncoderError<I2C::Error>> {
        let raw = self.read_angle_counts()?;
        Ok((raw as f32) * (_2PI / AS5600_CPR))
    }

    /// Reads the `STATUS` register (`0x0B`) and decodes magnet indicators.
    pub fn read_status(&self) -> Result<As5600Status, EncoderError<I2C::Error>> {
        let val = self.read_u8(reg::STATUS)?;
        Ok(As5600Status {
            magnet_detected: (val & (1 << 5)) != 0,
            magnet_too_weak: (val & (1 << 4)) != 0,
            magnet_too_strong: (val & (1 << 3)) != 0,
        })
    }

    /// Checks if a magnet is detected (`MD` bit 5 of `STATUS` register).
    pub fn is_magnet_detected(&self) -> Result<bool, EncoderError<I2C::Error>> {
        let status = self.read_status()?;
        Ok(status.magnet_detected)
    }

    /// Checks if the magnetic field is too weak (`ML` bit 4 of `STATUS` register).
    pub fn is_magnet_too_weak(&self) -> Result<bool, EncoderError<I2C::Error>> {
        let status = self.read_status()?;
        Ok(status.magnet_too_weak)
    }

    /// Checks if the magnetic field is too strong (`MH` bit 3 of `STATUS` register).
    pub fn is_magnet_too_strong(&self) -> Result<bool, EncoderError<I2C::Error>> {
        let status = self.read_status()?;
        Ok(status.magnet_too_strong)
    }

    /// Reads the Automatic Gain Control (AGC) value (0..255).
    pub fn read_agc(&self) -> Result<u8, EncoderError<I2C::Error>> {
        self.read_u8(reg::AGC)
    }

    /// Reads the magnetic magnitude value from register `0x1B`.
    pub fn read_magnitude(&self) -> Result<u16, EncoderError<I2C::Error>> {
        let word = self.read_u16(reg::MAGNITUDE)?;
        Ok(word & 0x0FFF)
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

impl<C: Clock, I2C: I2c> Sensor<C> for As5600<I2C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        match self.read_angle_counts() {
            Ok(raw) => (raw as f32) * (_2PI / AS5600_CPR),
            Err(_) => -1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec;

    use super::*;
    use crate::encoders::test_utils::MockClock;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[test]
    fn test_as5600_read_status_decoding() {
        // Bit 5 = MD (1), Bit 4 = ML (0), Bit 3 = MH (1) -> 0b0010_1000 = 0x28
        let expectations = [
            I2cTransaction::write_read(AS5600_DEFAULT_I2C_ADDR, vec![reg::STATUS], vec![0x28]),
            I2cTransaction::write_read(AS5600_DEFAULT_I2C_ADDR, vec![reg::STATUS], vec![0x28]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let sensor = As5600::new(i2c.clone());

        let status = sensor.read_status().unwrap();
        assert!(status.magnet_detected);
        assert!(!status.magnet_too_weak);
        assert!(status.magnet_too_strong);
        assert!(sensor.is_magnet_detected().unwrap_or(false));
        drop(sensor);
        i2c.done();
    }

    #[test]
    fn test_as5600_read_raw_angle() {
        // 12-bit angle 2048 (half revolution) -> 0x0800
        let expectations = [
            I2cTransaction::write_read(
                AS5600_DEFAULT_I2C_ADDR,
                vec![reg::RAW_ANGLE],
                vec![0x08, 0x00],
            ),
            I2cTransaction::write_read(
                AS5600_DEFAULT_I2C_ADDR,
                vec![reg::RAW_ANGLE],
                vec![0x08, 0x00],
            ),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let sensor = As5600::new(i2c.clone());

        let raw = sensor.read_raw_angle().unwrap();
        assert_eq!(raw, 2048);
        let rad = sensor.read_angle_radians().unwrap();
        assert!((rad - barefoc::math::_PI).abs() < 1e-4);
        drop(sensor);
        i2c.done();
    }

    #[test]
    fn test_as5600_sensor_trait() {
        let expectations = [I2cTransaction::write_read(
            AS5600_DEFAULT_I2C_ADDR,
            vec![reg::RAW_ANGLE],
            vec![0x04, 0x00], // 1024 -> quarter revolution
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = As5600::new(i2c.clone());
        let clock = MockClock::new();

        sensor.update(&clock);
        let angle = sensor.get_mechanical_angle();
        let expected = _2PI * 0.25;
        assert!((angle - expected).abs() < 1e-4);
        drop(sensor);
        i2c.done();
    }
}
