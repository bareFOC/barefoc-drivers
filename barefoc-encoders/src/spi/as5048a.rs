//! # spi::as5048a — AS5048A 14-Bit SPI Magnetic Angle Sensor
//!
//! Ports [`AS5048A.h`](source/Arduino-FOC-drivers/src/encoders/as5048a/AS5048A.h)
//! and [`AS5048A.cpp`](source/Arduino-FOC-drivers/src/encoders/as5048a/AS5048A.cpp)
//! from Arduino-FOC-drivers.
//!
//! The AS5048A is a 14-bit magnetic rotary encoder providing absolute angle
//! measurement over SPI Mode 1 with parity verification and silicon diagnostic
//! reporting (AGC, OCF, COF, COMP_LOW, COMP_HIGH).

use barefoc::common::time::Clock;
use barefoc::math::_2PI;
use barefoc::traits::sensor::{Sensor, SensorBase};
use core::cell::RefCell;
use embedded_hal::spi::SpiDevice;

use super::as5047p::{build_command, verify_even_parity};
use crate::EncoderError;

/// Counts per revolution for the 14-bit AS5048A sensor ($2^{14} = 16384$).
pub const AS5048A_CPR: f32 = 16384.0;

/// Register addresses for AS5048A.
pub mod reg {
    /// No operation / dummy read.
    pub const NOP: u16 = 0x0000;
    /// Clear error flag and read error register.
    pub const CLEAR_ERROR_FLAG: u16 = 0x0001;
    /// Programming control register.
    pub const PROGRAMMING_CONTROL: u16 = 0x0003;
    /// OTP zero position high register.
    pub const OTP_ZERO_HIGH: u16 = 0x0016;
    /// OTP zero position low register.
    pub const OTP_ZERO_LOW: u16 = 0x0017;
    /// Diagnostic and AGC register.
    pub const DIAGNOSTICS: u16 = 0x3FFD;
    /// Angle register (or magnitude depending on OTP mapping; default read target).
    pub const ANGLE: u16 = 0x3FFE;
    /// Full-range angle register.
    pub const ANGLE_FULL: u16 = 0x3FFF;
}

/// Decoded error flags from AS5048A register 0x0001.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct As5048aErrorFlags {
    /// Framing error occurred.
    pub framing_error: bool,
    /// Invalid command transmitted.
    pub command_invalid: bool,
    /// Parity error detected by the IC.
    pub parity_error: bool,
}

/// Decoded diagnostics from AS5048A register 0x3FFD.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct As5048aDiagnostics {
    /// Automatic Gain Control value (0..255).
    pub agc: u8,
    /// Offset compensation finished (OCF).
    pub ocf: bool,
    /// CORDIC overflow (COF).
    pub cof: bool,
    /// High magnetic field / too weak (COMP_LOW).
    pub comp_low: bool,
    /// Low magnetic field / too strong (COMP_HIGH).
    pub comp_high: bool,
}

/// AS5048A 14-bit SPI magnetic angle sensor driver.
pub struct As5048a<SPI> {
    spi: RefCell<SPI>,
    base: SensorBase,
    angle_register: u16,
}

impl<SPI: SpiDevice> As5048a<SPI> {
    /// Creates a new AS5048A driver targeting register `0x3FFE`.
    pub fn new(spi: SPI) -> Self {
        Self {
            spi: RefCell::new(spi),
            base: SensorBase::new(),
            angle_register: reg::ANGLE,
        }
    }

    /// Creates a new AS5048A driver with a custom angle register address.
    pub fn with_angle_register(spi: SPI, angle_register: u16) -> Self {
        Self {
            spi: RefCell::new(spi),
            base: SensorBase::new(),
            angle_register,
        }
    }

    /// Performs a 16-bit SPI transfer, checks parity and error flag, and returns
    /// the 14-bit payload.
    pub fn transfer_word(&self, command: u16) -> Result<u16, EncoderError<SPI::Error>> {
        let tx = command.to_be_bytes();
        let mut rx = [0u8; 2];
        self.spi
            .borrow_mut()
            .transfer(&mut rx, &tx)
            .map_err(EncoderError::Communication)?;

        let raw = u16::from_be_bytes(rx);
        if !verify_even_parity(raw) {
            return Err(EncoderError::ParityError);
        }
        if (raw & 0x4000) != 0 {
            return Err(EncoderError::DeviceError);
        }
        Ok(raw & 0x3FFF)
    }

    /// Reads the raw 14-bit angle counts (0..16383).
    pub fn read_angle_raw(&self) -> Result<u16, EncoderError<SPI::Error>> {
        let cmd = build_command(self.angle_register, true);
        self.transfer_word(cmd)
    }

    /// Reads the angle in radians in `[0, 2π)`.
    pub fn read_angle_radians(&self) -> Result<f32, EncoderError<SPI::Error>> {
        let raw = self.read_angle_raw()?;
        Ok((raw as f32) * (_2PI / AS5048A_CPR))
    }

    /// Reads the error register and returns decoded error flags.
    pub fn clear_error_flag(&self) -> Result<As5048aErrorFlags, EncoderError<SPI::Error>> {
        let cmd = build_command(reg::CLEAR_ERROR_FLAG, true);
        let _ = self.transfer_word(cmd)?;
        let nop_cmd = build_command(reg::NOP, true);
        let result = self.transfer_word(nop_cmd)?;

        Ok(As5048aErrorFlags {
            framing_error: (result & 0x0001) != 0,
            command_invalid: (result & 0x0002) != 0,
            parity_error: (result & 0x0004) != 0,
        })
    }

    /// Reads the diagnostics register `0x3FFD`.
    pub fn read_diagnostics(&self) -> Result<As5048aDiagnostics, EncoderError<SPI::Error>> {
        let cmd = build_command(reg::DIAGNOSTICS, true);
        let _ = self.transfer_word(cmd)?;
        let nop_cmd = build_command(reg::NOP, true);
        let result = self.transfer_word(nop_cmd)?;

        Ok(As5048aDiagnostics {
            agc: (result & 0x00FF) as u8,
            ocf: (result & 0x0100) != 0,
            cof: (result & 0x0200) != 0,
            comp_low: (result & 0x0400) != 0,
            comp_high: (result & 0x0800) != 0,
        })
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

impl<C: Clock, SPI: SpiDevice> Sensor<C> for As5048a<SPI> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        match self.read_angle_raw() {
            Ok(raw) => (raw as f32) * (_2PI / AS5048A_CPR),
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
    fn test_as5048a_angle_command_and_read() {
        // Command for 0x3FFE read: 0x7FFE
        // Response: count 8192 (half-turn, 0x2000, 1 one).
        // Parity requires bit 15 set: 0xA000.
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x7F, 0xFE], vec![0xA0, 0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x7F, 0xFE], vec![0xA0, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let sensor = As5048a::new(spi.clone());

        let raw = sensor.read_angle_raw().unwrap();
        assert_eq!(raw, 8192);
        let rad = sensor.read_angle_radians().unwrap();
        assert!((rad - barefoc::math::_PI).abs() < 1e-4);
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_as5048a_clear_error_flag() {
        // 1st transfer: command 0x4001 (CLEAR_ERROR_FLAG)
        // 2nd transfer: NOP (0xC000), response has parity_error bit set (0x0004 -> 1 one, with parity bit 15 = 0x8004)
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x40, 0x01], vec![0x00, 0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0xC0, 0x00], vec![0x80, 0x04]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let sensor = As5048a::new(spi.clone());

        let flags = sensor.clear_error_flag().unwrap();
        assert!(flags.parity_error);
        assert!(!flags.framing_error);
        assert!(!flags.command_invalid);
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_as5048a_sensor_trait() {
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x7F, 0xFE], vec![0xA0, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut sensor = As5048a::new(spi.clone());
        let clock = MockClock::new();

        sensor.update(&clock);
        let angle = sensor.get_mechanical_angle();
        assert!((angle - barefoc::math::_PI).abs() < 1e-4);
        drop(sensor);
        spi.done();
    }
}
