//! # ssi::mt6701 — MagnTek MT6701 14-Bit SSI Magnetic Angle Sensor
//!
//! Re-exports the standalone [`mt6xxx_rs`] driver crate and provides compatibility aliases.
//!
//! The MT6701 is a high-speed magnetic encoder IC. It supports:
//! - Mode 1 (24 clock cycles, bits `[23:0]`, idle-low clock)
//! - Mode 2 (32 clock cycles, bits `[24:1]`, idle-high clock)
//! - Simplex `spi.read()` (works on RX-only buses with no MOSI)
//! - Hardware CRC-6 verification ($X^6 + X + 1$)
//! - Bounded retry and last-known-good angle holding on persistent faults

pub use mt6xxx_rs::{
    compute_crc6, compute_crc6_bitwise, Config as Mt6701Config, Mt6701, Mt6701Frame, Mt6701Status,
    Mt6701Telemetry, SpiMode, MT6701_CPR, MT6701_MAX_RAW, MT6701_RESOLUTION_BITS, TABLE_CRC6,
    _2PI,
};

/// Type alias for [`Mt6701`] preserving backwards compatibility with previous SimpleFOC naming.
pub type Mt6701Ssi<SPI> = mt6xxx_rs::Mt6701<SPI>;

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec;
    use std::vec::Vec;

    use super::*;
    use crate::test_utils::MockClock;
    use barefoc::traits::sensor::Sensor;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

    fn make_read_tx(bytes: Vec<u8>) -> [SpiTransaction<u8>; 3] {
        [
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(bytes),
            SpiTransaction::transaction_end(),
        ]
    }

    #[test]
    fn test_mt6701_crc_calculation() {
        assert_eq!(compute_crc6(0x00000, 0), 0x00);
        assert_eq!(compute_crc6(0x3FFFF, 0), 0x0E);
        assert_eq!(compute_crc6(0x2AAAA, 0), 0x35);
        assert_eq!(compute_crc6(0x15555, 0), 0x3B);
        assert_eq!(compute_crc6(0x12345, 0), 0x1D);
        assert_eq!(compute_crc6(0x3FFC0, 0), 0x0C);
    }

    #[test]
    fn test_mt6701_ssi_mode1_valid_frame_and_sensor_trait() {
        let angle: u32 = 4096; // quarter rotation
        let status: u32 = 0b0000;
        let data_18 = (angle << 4) | status;
        let crc = compute_crc6(data_18, 0) as u32;
        let frame = (data_18 << 6) | crc;

        let b0 = ((frame >> 16) & 0xFF) as u8;
        let b1 = ((frame >> 8) & 0xFF) as u8;
        let b2 = (frame & 0xFF) as u8;

        let mut expectations = Vec::new();
        // init() calls get_sensor_angle() 4 times
        for _ in 0..4 {
            expectations.extend(make_read_tx(vec![b0, b1, b2]));
        }
        // update() calls get_sensor_angle() 1 time
        expectations.extend(make_read_tx(vec![b0, b1, b2]));

        let mut spi = SpiMock::new(&expectations);
        let mut sensor = Mt6701Ssi::new(spi.clone()).with_spi_mode(SpiMode::Mode1);
        let clock = MockClock::new();

        sensor.init(&clock);
        sensor.update(&clock);

        let out_angle = <Mt6701Ssi<_> as Sensor<MockClock>>::get_mechanical_angle(&sensor);
        let expected = _2PI * 0.25;
        assert!((out_angle - expected).abs() < 1e-4);

        drop(sensor);
        spi.done();
    }
}
