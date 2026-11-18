//! # mt6xxx-rs
//!
//! High-performance, pure-Rust, `#![no_std]` driver crate for MagnTek magnetic rotary
//! angle encoders (focusing on the 14-bit MT6701 SSI sensor, extensible to MT6816 / MT6835).
//!
//! ## Key Features
//! - **RX-Only Simplex SPI**: Interfaces via `spi.read(&mut buf)`, fully compatible with
//!   microcontrollers where MOSI is unrouted, left floating, or assigned to other peripheral roles
//!   (such as the Miniata dual-encoder actuator).
//! - **Dual SPI Framing**: Supports both [`SpiMode::Mode1`] (24 clock cycles, bits `[23:0]`, idle-low clock)
//!   and [`SpiMode::Mode2`] (32 clock cycles, bits `[24:1]`, idle-high clock).
//! - **Hardware CRC-6 Verification**: Fully verified polynomial $X^6 + X + 1$ (0x03 residual)
//!   tested against official reference vectors.
//! - **Bounded Retries & Continuous Angle Holding**: Up to $N$ retries (default 3) on CRC mismatch.
//!   On persistent error, holds the last known good angle and raises a sticky fault flag — never
//!   injecting discontinuous velocity spikes (such as $-1.0$ rad) into the FOC motor control loop.
//! - **Async & Blocking**: Supports both `embedded-hal` 1.0 (blocking) and `embedded-hal-async` 1.0 (DMA).
//! - **BareFOC Sensor Integration**: Direct implementation of [`barefoc::traits::sensor::Sensor`].
//!
//! ## Upstream References
//! - [`MagneticSensorMT6701SSI.h`](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/mt6701/MagneticSensorMT6701SSI.h)
//! - [`MagneticSensorMT6701SSI.cpp`](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/mt6701/MagneticSensorMT6701SSI.cpp)
//! - `ref/Miniata/hardware/ENCODER_BRINGUP.md`

#![no_std]

#[cfg(test)]
extern crate std;

pub mod crc;
pub mod error;
pub mod mt6701;

#[cfg(feature = "barefoc")]
pub mod barefoc_adapter;

pub use crc::{compute_crc6, compute_crc6_bitwise, TABLE_CRC6};
pub use error::Error;
pub use mt6701::{Config, Mt6701, Mt6701Frame, Mt6701Status, Mt6701Telemetry, SpiMode};

#[cfg(feature = "async")]
pub use mt6701::Mt6701Async;

/// Counts per revolution for the 14-bit MT6701 sensor ($2^{14} = 16384$).
pub const MT6701_CPR: f32 = 16384.0;

/// Native resolution in bits (14 bits).
pub const MT6701_RESOLUTION_BITS: u8 = 14;

/// Maximum raw counter value ($2^{14} - 1 = 16383$).
pub const MT6701_MAX_RAW: u16 = 16383;

/// $2\pi$ constant in single-precision floating point.
pub const _2PI: f32 = core::f32::consts::TAU;

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec;
    use std::vec::Vec;

    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

    fn make_read_tx(bytes: Vec<u8>) -> [SpiTransaction<u8>; 3] {
        [
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(bytes),
            SpiTransaction::transaction_end(),
        ]
    }

    /// Helper to synthesize a valid 24-bit MT6701 frame.
    fn make_frame(angle: u16, status: u8, crc_init: u8) -> (u32, u8, u8, u8) {
        let angle_14 = (angle & 0x3FFF) as u32;
        let status_4 = (status & 0x0F) as u32;
        let data_18 = (angle_14 << 4) | status_4;
        let crc = compute_crc6(data_18, crc_init) as u32;
        let raw_24 = (data_18 << 6) | crc;

        let b0 = ((raw_24 >> 16) & 0xFF) as u8;
        let b1 = ((raw_24 >> 8) & 0xFF) as u8;
        let b2 = (raw_24 & 0xFF) as u8;

        (raw_24, b0, b1, b2)
    }

    #[test]
    fn test_mode1_read_success() {
        // Angle = 4096 (quarter revolution), Status = 0 (clean), CRC init = 0
        let (_frame, b0, b1, b2) = make_frame(4096, 0, 0);

        let expectations = make_read_tx(vec![b0, b1, b2]);
        let mut spi = SpiMock::new(&expectations);
        let sensor = Mt6701::new(spi.clone()).with_spi_mode(SpiMode::Mode1);

        let res = sensor.read_raw();
        assert_eq!(res, Ok(4096));
        assert_eq!(sensor.last_status(), Mt6701Status::from_raw(0));
        assert_eq!(sensor.telemetry().crc_ok, 1);
        assert_eq!(sensor.telemetry().crc_failed, 0);
        assert_eq!(sensor.telemetry().crc_exhausted, 0);
        assert!(!sensor.has_fault());

        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_mode2_read_success() {
        // Mode 2: 32 bits, 4 bytes read.
        // Frame is shifted to bits [24:1].
        let (raw_24, _, _, _) = make_frame(8192, 0b0010 /* PUSH */, 0);
        let raw_32 = (raw_24 << 1) | 0x8000_0000; // Bit 31 is stale leading bit

        let b0 = ((raw_32 >> 24) & 0xFF) as u8;
        let b1 = ((raw_32 >> 16) & 0xFF) as u8;
        let b2 = ((raw_32 >> 8) & 0xFF) as u8;
        let b3 = (raw_32 & 0xFF) as u8;

        let expectations = make_read_tx(vec![b0, b1, b2, b3]);
        let mut spi = SpiMock::new(&expectations);
        let sensor = Mt6701::new(spi.clone()).with_spi_mode(SpiMode::Mode2);

        let res = sensor.read_raw();
        assert_eq!(res, Ok(8192));
        let status = sensor.last_status();
        assert!(status.push);
        assert!(!status.mgh);
        assert!(!status.mgl);
        assert!(!status.los);
        assert_eq!(sensor.telemetry().crc_ok, 1);
        assert!(!sensor.has_fault());

        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_retry_recovery_on_second_attempt() {
        // Attempt 1: Corrupted CRC byte
        let (_frame, b0, b1, b2) = make_frame(2048, 0, 0);
        let corrupted_b2 = b2 ^ 0x01; // Corrupt CRC bit 0

        // Attempt 2: Valid frame
        let (_frame2, vb0, vb1, vb2) = make_frame(2048, 0, 0);

        let mut expectations = Vec::new();
        expectations.extend(make_read_tx(vec![b0, b1, corrupted_b2]));
        expectations.extend(make_read_tx(vec![vb0, vb1, vb2]));

        let mut spi = SpiMock::new(&expectations);
        let sensor = Mt6701::new(spi.clone()).with_spi_mode(SpiMode::Mode1);

        let res = sensor.read_raw();
        assert_eq!(res, Ok(2048));
        assert_eq!(sensor.telemetry().crc_failed, 1);
        assert_eq!(sensor.telemetry().crc_ok, 1);
        assert_eq!(sensor.telemetry().crc_exhausted, 0);
        assert!(!sensor.has_fault());

        drop(sensor);
        spi.done();
    }

    #[test]
    fn test_exhausted_retries_holds_last_good_angle() {
        // First query: succeeds with angle = 8192 (half revolution = PI rad)
        let (_, b0, b1, b2) = make_frame(8192, 0, 0);

        // Second query: 3 attempts all fail with corrupted CRC
        let corrupted_b2 = b2 ^ 0x07;

        let mut expectations = Vec::new();
        // Query 1
        expectations.extend(make_read_tx(vec![b0, b1, b2]));
        // Query 2: 3 failed attempts
        expectations.extend(make_read_tx(vec![b0, b1, corrupted_b2]));
        expectations.extend(make_read_tx(vec![b0, b1, corrupted_b2]));
        expectations.extend(make_read_tx(vec![b0, b1, corrupted_b2]));

        let mut spi = SpiMock::new(&expectations);
        let sensor = Mt6701::new(spi.clone()).with_spi_mode(SpiMode::Mode1);

        // Query 1: success
        assert_eq!(sensor.read_raw(), Ok(8192));
        assert_eq!(sensor.last_good_raw(), 8192);

        // Query 2: all retries fail
        let angle_raw = sensor.read_raw_angle();
        // MUST hold last-known-good angle: 8192
        assert_eq!(angle_raw, 8192);
        assert_eq!(sensor.last_good_raw(), 8192);

        let angle_rad = (angle_raw as f32 / MT6701_CPR) * _2PI;
        let expected_rad = (8192.0 / MT6701_CPR) * _2PI;
        assert!((angle_rad - expected_rad).abs() < 1e-5);
        assert_eq!(sensor.last_good_angle(), expected_rad);
        // CRITICAL: NEVER -1.0
        assert!(angle_rad >= 0.0);

        // Telemetry check
        let telem = sensor.telemetry();
        assert_eq!(telem.crc_ok, 1);
        assert_eq!(telem.crc_failed, 3);
        assert_eq!(telem.crc_exhausted, 1);
        assert!(sensor.has_fault());

        // Clear sticky fault
        sensor.clear_fault();
        assert!(!sensor.has_fault());

        drop(sensor);
        spi.done();
    }

    #[cfg(feature = "barefoc")]
    #[test]
    fn test_barefoc_sensor_trait_integration() {
        use barefoc::traits::sensor::Sensor;

        struct TestClock(core::cell::Cell<u32>);
        impl barefoc::common::time::Clock for TestClock {
            fn now_us(&self) -> u32 {
                let val = self.0.get();
                self.0.set(val + 1000);
                val
            }
        }

        let clock = TestClock(core::cell::Cell::new(0));

        let (_, b0, b1, b2) = make_frame(4096, 0, 0);

        let mut expectations = Vec::new();
        // init() calls get_sensor_angle() 4 times
        for _ in 0..4 {
            expectations.extend(make_read_tx(vec![b0, b1, b2]));
        }
        // update() calls get_sensor_angle() 1 time
        expectations.extend(make_read_tx(vec![b0, b1, b2]));
        // explicit get_sensor_angle() call 1 time
        expectations.extend(make_read_tx(vec![b0, b1, b2]));

        let mut spi = SpiMock::new(&expectations);
        let mut sensor = Mt6701::new(spi.clone()).with_spi_mode(SpiMode::Mode1);

        sensor.init(&clock);
        sensor.update(&clock);

        let angle = <Mt6701<_> as Sensor<TestClock>>::get_sensor_angle(&sensor);
        let expected = _2PI * 0.25;
        assert!((angle - expected).abs() < 1e-4);

        let mech_angle = <Mt6701<_> as Sensor<TestClock>>::get_mechanical_angle(&sensor);
        assert!((mech_angle - expected).abs() < 1e-4);

        drop(sensor);
        spi.done();
    }
}
