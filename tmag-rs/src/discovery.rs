//! # discovery — Zero-config TMAG5273 I2C address and silicon variant auto-discovery
//!
//! Provides automated candidate address probing and silicon variant detection
//! across multi-frequency I2C configurations.

use embedded_hal::i2c::I2c;

use crate::error::Error;
use crate::registers;
use crate::types::DeviceVariant;

/// Standard factory candidate I2C addresses for TMAG5273 (A1..D2).
pub const CANDIDATE_ADDRESSES: [u8; 4] = [0x35, 0x22, 0x78, 0x44];

/// Standard probing bus frequencies in Hz (20 kHz, 50 kHz, 100 kHz).
pub const CANDIDATE_FREQUENCIES_HZ: [u32; 3] = [20_000, 50_000, 100_000];

/// Expected Texas Instruments 16-bit manufacturer ID in little-endian (`0x5449` = 'TI').
pub const TI_MANUFACTURER_ID: u16 = 0x5449;

/// Probe a specific I2C address for a valid TMAG5273 device and identify its silicon variant.
pub fn probe_device<I2C: I2c>(i2c: &mut I2C, addr: u8) -> Result<DeviceVariant, Error<I2C::Error>> {
    let mut mfg = [0u8; 2];
    i2c.write_read(addr, &[registers::MANUFACTURER_ID_LSB], &mut mfg)
        .map_err(Error::I2c)?;

    let mfg_id = u16::from_le_bytes(mfg);
    if mfg_id != TI_MANUFACTURER_ID {
        return Err(Error::InvalidManufacturerId(mfg_id));
    }

    let mut dev_id = [0u8; 1];
    i2c.write_read(addr, &[registers::DEVICE_ID], &mut dev_id)
        .map_err(Error::I2c)?;

    let is_v2 = (dev_id[0] & 0x03) == 2;
    let variant = match (addr, is_v2) {
        (0x22, false) => DeviceVariant::B1,
        (0x22, true) => DeviceVariant::B2,
        (0x78, false) => DeviceVariant::C1,
        (0x78, true) => DeviceVariant::C2,
        (0x44, false) => DeviceVariant::D1,
        (0x44, true) => DeviceVariant::D2,
        (_, true) => DeviceVariant::A2,
        (_, false) => DeviceVariant::A1,
    };

    Ok(variant)
}

/// Automatically discover an attached TMAG5273 on standard candidate addresses (0x35, 0x22, 0x78, 0x44).
///
/// Returns the resolved I2C 7-bit address and silicon variant on success.
pub fn discover<I2C: I2c>(i2c: &mut I2C) -> Result<(u8, DeviceVariant), Error<I2C::Error>> {
    for &addr in &CANDIDATE_ADDRESSES {
        if let Ok(variant) = probe_device(i2c, addr) {
            return Ok((addr, variant));
        }
    }
    Err(Error::DeviceNotFound)
}

/// Multi-frequency auto-discovery scanning candidate addresses across provided clock rates.
///
/// Invokes `set_frequency` callback before scanning each frequency stage, stopping at the
/// first successful detection.
pub fn discover_with_frequencies<I2C: I2c, F: FnMut(u32)>(
    i2c: &mut I2C,
    frequencies: &[u32],
    mut set_frequency: F,
) -> Result<(u8, DeviceVariant), Error<I2C::Error>> {
    for &freq in frequencies {
        set_frequency(freq);
        if let Ok(result) = discover(i2c) {
            return Ok(result);
        }
    }
    Err(Error::DeviceNotFound)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockI2cDevice {
        target_addr: u8,
        mfg_id: u16,
        dev_id: u8,
    }

    impl embedded_hal::i2c::ErrorType for MockI2cDevice {
        type Error = core::convert::Infallible;
    }

    impl I2c for MockI2cDevice {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            if address != self.target_addr {
                // Device NACKs address
                return Ok(());
            }

            let mut last_reg = 0u8;
            for op in operations {
                match op {
                    embedded_hal::i2c::Operation::Write(buf) => {
                        if !buf.is_empty() {
                            last_reg = buf[0];
                        }
                    }
                    embedded_hal::i2c::Operation::Read(buf) => {
                        if last_reg == registers::MANUFACTURER_ID_LSB && buf.len() >= 2 {
                            let le = self.mfg_id.to_le_bytes();
                            buf[0] = le[0];
                            buf[1] = le[1];
                        } else if last_reg == registers::DEVICE_ID && !buf.is_empty() {
                            buf[0] = self.dev_id;
                        }
                    }
                }
            }
            Ok(())
        }
    }

    #[test]
    fn test_discover_tmag5273_a1() {
        let mut mock = MockI2cDevice {
            target_addr: 0x35,
            mfg_id: TI_MANUFACTURER_ID,
            dev_id: 0x01, // v1
        };
        let (addr, variant) = discover(&mut mock).expect("discovery should succeed");
        assert_eq!(addr, 0x35);
        assert_eq!(variant, DeviceVariant::A1);
    }

    #[test]
    fn test_discover_tmag5273_b2() {
        let mut mock = MockI2cDevice {
            target_addr: 0x22,
            mfg_id: TI_MANUFACTURER_ID,
            dev_id: 0x02, // v2
        };
        let (addr, variant) = discover(&mut mock).expect("discovery should succeed");
        assert_eq!(addr, 0x22);
        assert_eq!(variant, DeviceVariant::B2);
    }

    #[test]
    fn test_discover_tmag5273_c1() {
        let mut mock = MockI2cDevice {
            target_addr: 0x78,
            mfg_id: TI_MANUFACTURER_ID,
            dev_id: 0x01, // v1
        };
        let (addr, variant) = discover(&mut mock).expect("discovery should succeed");
        assert_eq!(addr, 0x78);
        assert_eq!(variant, DeviceVariant::C1);
    }

    #[test]
    fn test_discover_tmag5273_d2() {
        let mut mock = MockI2cDevice {
            target_addr: 0x44,
            mfg_id: TI_MANUFACTURER_ID,
            dev_id: 0x02, // v2
        };
        let (addr, variant) = discover(&mut mock).expect("discovery should succeed");
        assert_eq!(addr, 0x44);
        assert_eq!(variant, DeviceVariant::D2);
    }

    #[test]
    fn test_discover_with_frequencies() {
        let mut mock = MockI2cDevice {
            target_addr: 0x22,
            mfg_id: TI_MANUFACTURER_ID,
            dev_id: 0x01, // B1
        };
        let mut freqs_tried = [0u32; 3];
        let mut idx = 0;
        let (addr, variant) = discover_with_frequencies(
            &mut mock,
            &CANDIDATE_FREQUENCIES_HZ,
            |freq| {
                if idx < 3 {
                    freqs_tried[idx] = freq;
                    idx += 1;
                }
            },
        )
        .expect("should find sensor");

        assert_eq!(addr, 0x22);
        assert_eq!(variant, DeviceVariant::B1);
        assert_eq!(freqs_tried[0], 20_000);
    }
}
