//! # spi::as5047p — AS5047P / AS5047D SPI Magnetic Angle Sensor
//!
//! Ports [`AS5047.h`](source/Arduino-FOC-drivers/src/encoders/as5047/AS5047.h)
//! and [`AS5047.cpp`](source/Arduino-FOC-drivers/src/encoders/as5047/AS5047.cpp)
//! from Arduino-FOC-drivers.
//!
//! The AS5047P is a 14-bit high-resolution rotary position sensor featuring
//! Dynamic Angle Error Compensation (DAEC™) for low angle delay at high rotation
//! speeds. Communication is performed over a standard 16-bit SPI transaction
//! (SPI Mode 1) with an even-parity bit and error flag verification.

use barefoc::common::time::Clock;
use barefoc::math::_2PI;
use barefoc::traits::sensor::{Sensor, SensorBase};
use core::cell::RefCell;
use embedded_hal::spi::SpiDevice;

use crate::EncoderError;

/// Counts per revolution for the 14-bit AS5047P sensor ($2^{14} = 16384$).
pub const AS5047P_CPR: f32 = 16384.0;

/// Register addresses for AS5047P.
pub mod reg {
    /// No operation / dummy read.
    pub const NOP: u16 = 0x0000;
    /// Error register (`ERRFL`).
    pub const ERRFL: u16 = 0x0001;
    /// Programming register.
    pub const PROG: u16 = 0x0003;
    /// Zero position upper bits.
    pub const ZPOSM: u16 = 0x0016;
    /// Zero position lower bits.
    pub const ZPOSL: u16 = 0x0017;
    /// Settings register 1.
    pub const SETTINGS1: u16 = 0x0018;
    /// Settings register 2.
    pub const SETTINGS2: u16 = 0x0019;
    /// Diagnostic and AGC register (`DIAAGC`).
    pub const DIAAGC: u16 = 0x3FFC;
    /// CORDIC magnitude register (`MAG`).
    pub const MAG: u16 = 0x3FFD;
    /// Uncompensated angle register (`ANGLEUNC`).
    pub const ANGLEUNC: u16 = 0x3FFE;
    /// Dynamic angle error compensated angle register (`ANGLECOM`).
    pub const ANGLECOM: u16 = 0x3FFF;
}

/// Decoded error flags from AS5047P `ERRFL` register (0x0001).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct As5047pErrorFlags {
    /// Framing error occurred.
    pub framing_error: bool,
    /// Invalid command transmitted.
    pub command_invalid: bool,
    /// Parity error detected by the IC.
    pub parity_error: bool,
}

/// Decoded diagnostics from AS5047P `DIAAGC` register (0x3FFC).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct As5047pDiagnostics {
    /// Automatic Gain Control value (0..255).
    pub agc: u8,
    /// CORDIC overflow flag (LF).
    pub lf: bool,
    /// CORDIC overflow flag (COF).
    pub cof: bool,
    /// Magnetic field too strong flag (MAGH).
    pub magh: bool,
    /// Magnetic field too weak flag (MAGL).
    pub magl: bool,
}

/// Builds a 16-bit command word with even parity for AS5047P.
///
/// - Bit 15: PAR (Even parity over bits 14..0)
/// - Bit 14: R/W (1 = Read, 0 = Write)
/// - Bits 13..0: Register address
#[inline]
pub const fn build_command(addr: u16, read: bool) -> u16 {
    let mut cmd = addr & 0x3FFF;
    if read {
        cmd |= 0x4000;
    }
    // Even parity: if the count of set bits in bits 14..0 is odd, bit 15 must be set.
    if (cmd.count_ones() & 1) == 1 {
        cmd |= 0x8000;
    }
    cmd
}

/// Verifies even parity of a 16-bit word received from the sensor.
///
/// Returns `true` if the total count of set bits (including bit 15) is even.
#[inline]
pub const fn verify_even_parity(raw: u16) -> bool {
    (raw.count_ones() & 1) == 0
}

/// AS5047P / AS5047D 14-bit magnetic angle sensor driver.
pub struct As5047p<SPI> {
    spi: RefCell<SPI>,
    base: SensorBase,
    use_daec: bool,
}

impl<SPI: SpiDevice> As5047p<SPI> {
    /// Creates a new AS5047P driver instance.
    ///
    /// By default, Dynamic Angle Error Compensation (`use_daec = true`) is enabled,
    /// reading from `ANGLECOM` (0x3FFF).
    pub fn new(spi: SPI) -> Self {
        Self {
            spi: RefCell::new(spi),
            base: SensorBase::new(),
            use_daec: true,
        }
    }

    /// Creates a new AS5047P driver with explicit DAEC configuration.
    pub fn with_daec(spi: SPI, use_daec: bool) -> Self {
        Self {
            spi: RefCell::new(spi),
            base: SensorBase::new(),
            use_daec,
        }
    }

    /// Enables or disables Dynamic Angle Error Compensation (DAEC).
    ///
    /// When `true`, reads register `ANGLECOM` (`0x3FFF`).
    /// When `false`, reads register `ANGLEUNC` (`0x3FFE`).
    pub fn set_use_daec(&mut self, enable: bool) {
        self.use_daec = enable;
    }

    /// Returns whether DAEC is currently enabled.
    pub fn use_daec(&self) -> bool {
        self.use_daec
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
    ///
    /// Uses `ANGLECOM` if DAEC is enabled, or `ANGLEUNC` if disabled.
    pub fn read_angle_raw(&self) -> Result<u16, EncoderError<SPI::Error>> {
        let cmd = if self.use_daec {
            build_command(reg::ANGLECOM, true)
        } else {
            build_command(reg::ANGLEUNC, true)
        };
        self.transfer_word(cmd)
    }

    /// Reads the angle in radians in `[0, 2π)`.
    pub fn read_angle_radians(&self) -> Result<f32, EncoderError<SPI::Error>> {
        let raw = self.read_angle_raw()?;
        Ok((raw as f32) * (_2PI / AS5047P_CPR))
    }

    /// Reads the `ERRFL` register and returns decoded error flags.
    ///
    /// Sends a read command for `0x0001` followed by a dummy read (`NOP`) to clock out
    /// the register contents.
    pub fn read_errfl(&self) -> Result<As5047pErrorFlags, EncoderError<SPI::Error>> {
        let cmd = build_command(reg::ERRFL, true);
        let _ = self.transfer_word(cmd)?;
        let nop_cmd = build_command(reg::NOP, true);
        let result = self.transfer_word(nop_cmd)?;

        Ok(As5047pErrorFlags {
            framing_error: (result & 0x0001) != 0,
            command_invalid: (result & 0x0002) != 0,
            parity_error: (result & 0x0004) != 0,
        })
    }

    /// Reads the `DIAAGC` register and returns decoded diagnostics.
    pub fn read_diagnostics(&self) -> Result<As5047pDiagnostics, EncoderError<SPI::Error>> {
        let cmd = build_command(reg::DIAAGC, true);
        let _ = self.transfer_word(cmd)?;
        let nop_cmd = build_command(reg::NOP, true);
        let result = self.transfer_word(nop_cmd)?;

        Ok(As5047pDiagnostics {
            agc: (result & 0x00FF) as u8,
            lf: (result & 0x0100) != 0,
            cof: (result & 0x0200) != 0,
            magh: (result & 0x0400) != 0,
            magl: (result & 0x0800) != 0,
        })
    }

    /// Reads the CORDIC magnitude register `MAG` (0x3FFD).
    pub fn read_magnitude(&self) -> Result<u16, EncoderError<SPI::Error>> {
        let cmd = build_command(reg::MAG, true);
        let _ = self.transfer_word(cmd)?;
        let nop_cmd = build_command(reg::NOP, true);
        self.transfer_word(nop_cmd)
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

impl<C: Clock, SPI: SpiDevice> Sensor<C> for As5047p<SPI> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        match self.read_angle_raw() {
            Ok(raw) => (raw as f32) * (_2PI / AS5047P_CPR),
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
    fn test_build_command_parity() {
        // ANGLECOM (0x3FFF): 14 ones + read (bit 14 = 1) = 15 ones -> parity bit 15 = 1 -> 0xFFFF
        assert_eq!(build_command(reg::ANGLECOM, true), 0xFFFF);

        // ANGLEUNC (0x3FFE): 13 ones + read (bit 14 = 1) = 14 ones -> parity bit 15 = 0 -> 0x7FFE
        assert_eq!(build_command(reg::ANGLEUNC, true), 0x7FFE);

        // ERRFL (0x0001): 1 one + read (bit 14 = 1) = 2 ones -> parity bit 15 = 0 -> 0x4001
        assert_eq!(build_command(reg::ERRFL, true), 0x4001);

        // DIAAGC (0x3FFC): 12 ones + read (bit 14 = 1) = 13 ones -> parity bit 15 = 1 -> 0xFFFC
        assert_eq!(build_command(reg::DIAAGC, true), 0xFFFC);
    }

    #[test]
    fn test_verify_even_parity() {
        assert!(verify_even_parity(0xFFFF));
        assert!(verify_even_parity(0x7FFE));
        assert!(verify_even_parity(0x0000));
        assert!(!verify_even_parity(0x0001));
        assert!(!verify_even_parity(0x7FFF));
    }

    #[test]
    fn test_read_angle_raw_success() {
        // Angle 0x1234 (4660) with even parity:
        // 0x1234 has 5 ones. EF=0, so bits 14..0 have 5 ones.
        // Even parity requires bit 15 = 1 -> 0x9234.
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0xFF, 0xFF], vec![0x92, 0x34]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let sensor = As5047p::new(spi.clone());

        let angle = sensor.read_angle_raw().unwrap();
        assert_eq!(angle, 0x1234);
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_read_angle_parity_error() {
        // Bad parity: 0x1234 without bit 15 set has 5 ones -> odd -> ParityError
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0xFF, 0xFF], vec![0x12, 0x34]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let sensor = As5047p::new(spi.clone());

        let res = sensor.read_angle_raw();
        assert_eq!(res, Err(EncoderError::ParityError));
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_read_angle_device_error() {
        // Error flag bit 14 set: 0x4000 | 0x1000 = 0x5000 (2 ones, valid even parity)
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0xFF, 0xFF], vec![0x50, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let sensor = As5047p::new(spi.clone());

        let res = sensor.read_angle_raw();
        assert_eq!(res, Err(EncoderError::DeviceError));
        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_sensor_trait_with_mock_clock() {
        // Quarter rotation: count = 4096 (0x1000, 1 one).
        // For even parity, bit 15 must be set: 0x9000.
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0xFF, 0xFF], vec![0x90, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut sensor = As5047p::new(spi.clone());
        let clock = MockClock::new();

        sensor.update(&clock);
        let angle = sensor.get_mechanical_angle();
        let expected = _2PI * (4096.0 / 16384.0);
        assert!((angle - expected).abs() < 1e-4);
        drop(sensor);
        spi.done();
    }
}
