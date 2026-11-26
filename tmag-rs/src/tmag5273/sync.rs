//! # sync — Synchronous I2C Driver Implementation
//!
//! Provides blocking driver operations over `embedded_hal::i2c::I2c`.

use embedded_hal::i2c::I2c;

use super::{Tmag5273, Tmag5273Builder};
use crate::angle::Angle;
use crate::error::Error;
use crate::magnetic::MagneticField3D;
use crate::registers::*;
use crate::types::*;

impl<I2C: I2c> Tmag5273Builder<I2C> {
    /// Build and initialize the sensor synchronously.
    ///
    /// Verifies manufacturer ID `0x5449`, verifies device variant version,
    /// clears power-on-reset and latched fault flags, and writes configuration registers.
    pub fn build(mut self) -> Result<Tmag5273<I2C>, Error<I2C::Error>> {
        // 1. Verify manufacturer ID (0x5449)
        let mut mfg_buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[MANUFACTURER_ID_LSB], &mut mfg_buf)
            .map_err(Error::I2c)?;
        let mfg_id = u16::from_le_bytes(mfg_buf);
        if mfg_id != 0x5449 {
            return Err(Error::InvalidManufacturerId(mfg_id));
        }

        // 2. Verify device ID version
        let mut dev_id = [0u8; 1];
        self.i2c
            .write_read(self.address, &[DEVICE_ID], &mut dev_id)
            .map_err(Error::I2c)?;
        let ver = dev_id[0] & VER_MASK;
        let expected_ver = self.config.variant.version();
        if ver != expected_ver {
            return Err(Error::VersionMismatch {
                expected: expected_ver,
                got: ver,
            });
        }

        // 3. Clear initial POR and fault flags
        self.i2c
            .write(self.address, &[CONV_STATUS, POR_MASK])
            .map_err(Error::I2c)?;
        self.i2c
            .write(self.address, &[DEVICE_STATUS, FAULT_CLEAR_MASK])
            .map_err(Error::I2c)?;

        // 4. Configure device registers via shared reconfigure()
        // Silicon starts with CRC_EN = 0 at POR; start dev with crc_enabled = false
        // so DEVICE_CONFIG_1 is written without CRC before CRC takes effect.
        let config = self.config;
        let mut init_config = config;
        init_config.crc_enabled = false;
        let mut dev = Tmag5273 {
            i2c: self.i2c,
            address: self.address,
            config: init_config,
            read_mode: config.read_mode,
            variant: config.variant,
        };
        dev.reconfigure(config)?;
        Ok(dev)
    }
}

impl<I2C: I2c> Tmag5273<I2C> {
    /// Initialize a new TMAG5273 sensor using default configuration.
    pub fn new(i2c: I2C, address: u8) -> Result<Self, Error<I2C::Error>> {
        Self::builder(i2c, address).build()
    }

    /// Write a single register byte.
    ///
    /// Per TI TMAG5273 Datasheet Section 6.5.1.3.1, I2C register writes do not
    /// carry a CRC byte even when `CRC_EN` is set; only I2C reads append CRC-8.
    #[inline]
    pub fn write_register(&mut self, register: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, value])
            .map_err(Error::I2c)
    }

    /// Read a single register byte.
    #[inline]
    pub fn read_register(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.read_registers(register, &mut buf)?;
        Ok(buf[0])
    }

    /// Read multiple contiguous register bytes starting at `register`.
    ///
    /// In TMAG5273 standard 3-byte read mode with `CRC_EN = 1`, the sensor
    /// appends the CRC-8 byte after 4 consecutive data bytes (`[d0, d1, d2, d3, crc]`),
    /// computed over those 4 data bytes (`0x07` polynomial, init `0xFF`).
    #[inline]
    pub fn read_registers(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        if buf.is_empty() {
            return Ok(());
        }
        if self.config.crc_enabled && buf.len() <= 4 {
            let mut tmp = [0u8; 5];
            self.i2c
                .write_read(self.address, &[register], &mut tmp)
                .map_err(Error::I2c)?;
            let computed = crate::crc::crc8(&tmp[..4]);
            let rx_crc = tmp[4];
            if rx_crc != computed {
                return Err(Error::CrcMismatch {
                    expected: rx_crc,
                    computed,
                });
            }
            buf.copy_from_slice(&tmp[..buf.len()]);
            Ok(())
        } else {
            self.i2c
                .write_read(self.address, &[register], buf)
                .map_err(Error::I2c)
        }
    }

    /// Execute a 1-byte fast stream read directly without sending a register address byte.
    ///
    /// Unlocks 20 kSPS high-speed feedback in continuous conversion mode.
    #[inline]
    pub fn read_stream(&mut self, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.read(self.address, buf).map_err(Error::I2c)
    }

    /// Clear the latched Power-On-Reset (POR) flag in `CONV_STATUS` (0x18).
    #[inline]
    pub fn clear_por(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(CONV_STATUS, POR_MASK)
    }

    /// Clear all latched fault flags in `DEVICE_STATUS` (0x1C).
    #[inline]
    pub fn clear_faults(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(DEVICE_STATUS, FAULT_CLEAR_MASK)
    }

    /// Read and decode the device status register (`DEVICE_STATUS`, 0x1C).
    pub fn read_status(&mut self) -> Result<DeviceStatus, Error<I2C::Error>> {
        let val = self.read_register(DEVICE_STATUS)?;
        Ok(DeviceStatus::from_raw(val))
    }

    /// Read and decode the conversion status register (`CONV_STATUS`, 0x18).
    pub fn read_conv_status(&mut self) -> Result<ConversionStatus, Error<I2C::Error>> {
        let val = self.read_register(CONV_STATUS)?;
        Ok(ConversionStatus::from_raw(val))
    }

    /// Read the hardware CORDIC angle from `ANGLE_RESULT_MSB` and `ANGLE_RESULT_LSB` (0x19..0x1A).
    pub fn read_angle(&mut self) -> Result<Angle, Error<I2C::Error>> {
        if self.config.angle_enabled == AngleEnable::None {
            return Err(Error::AngleNotEnabled);
        }
        let mut buf = [0u8; 2];
        self.read_registers(ANGLE_RESULT_MSB, &mut buf)?;
        let raw = u16::from_be_bytes(buf);
        Angle::from_raw(raw).ok_or(Error::InvalidRegisterValue {
            register: ANGLE_RESULT_MSB,
            value: buf[0],
        })
    }

    /// Read synchronous 3D magnetic flux density vector $[B_x, B_y, B_z]$ via a 6-byte burst read.
    pub fn read_magnetic(&mut self) -> Result<MagneticField3D, Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_registers(X_MSB_RESULT, &mut buf)?;
        let raw_x = i16::from_be_bytes([buf[0], buf[1]]);
        let raw_y = i16::from_be_bytes([buf[2], buf[3]]);
        let raw_z = i16::from_be_bytes([buf[4], buf[5]]);

        let range_xy = self.variant.range_value(self.config.xy_range);
        let range_z = self.variant.range_value(self.config.z_range);

        Ok(MagneticField3D::from_raw_16bit(
            raw_x, raw_y, raw_z, range_xy, range_z,
        ))
    }

    /// Read magnetic field using fast 8-bit MSB reading mode.
    ///
    /// Halves I2C bus transaction time for low-latency FOC velocity loops.
    pub fn read_magnetic_8bit(&mut self) -> Result<MagneticField3D, Error<I2C::Error>> {
        let range_xy = self.variant.range_value(self.config.xy_range);
        let range_z = self.variant.range_value(self.config.z_range);

        if self.read_mode == I2cReadMode::OneByte8Bit {
            let mut buf = [0u8; 4];
            self.read_stream(&mut buf)?;
            Ok(MagneticField3D::from_raw_8bit(
                buf[0] as i8,
                buf[1] as i8,
                buf[2] as i8,
                range_xy,
                range_z,
            ))
        } else {
            let raw_x = self.read_register(X_MSB_RESULT)? as i8;
            let raw_y = self.read_register(Y_MSB_RESULT)? as i8;
            let raw_z = self.read_register(Z_MSB_RESULT)? as i8;
            Ok(MagneticField3D::from_raw_8bit(
                raw_x, raw_y, raw_z, range_xy, range_z,
            ))
        }
    }

    /// Read temperature in degrees Celsius from `T_MSB_RESULT` and `T_LSB_RESULT` (0x10..0x11).
    pub fn read_temperature(&mut self) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_registers(T_MSB_RESULT, &mut buf)?;
        let raw = i16::from_be_bytes(buf);
        let temp = 25.0 + ((raw as f32) - 17508.0) / 60.1;
        Ok(temp)
    }

    /// Update I2C read mode in `DEVICE_CONFIG_1` (0x00).
    pub fn set_read_mode(&mut self, mode: I2cReadMode) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(DEVICE_CONFIG_1)?;
        let updated = insert(current, I2C_RD_MASK, I2C_RD_SHIFT, mode as u8);
        self.write_register(DEVICE_CONFIG_1, updated)?;
        self.read_mode = mode;
        self.config.read_mode = mode;
        Ok(())
    }

    /// Update device operating mode in `DEVICE_CONFIG_2` (0x01).
    pub fn set_operating_mode(&mut self, mode: OperatingMode) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(DEVICE_CONFIG_2)?;
        let updated = insert(current, OPERATING_MODE_MASK, OPERATING_MODE_SHIFT, mode as u8);
        self.write_register(DEVICE_CONFIG_2, updated)?;
        self.config.operating_mode = mode;
        Ok(())
    }

    /// Update X and Y axis magnetic measurement range in `SENSOR_CONFIG_2` (0x03).
    pub fn set_xy_range(&mut self, range: Range) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(SENSOR_CONFIG_2)?;
        let updated = insert(current, X_Y_RANGE_MASK, X_Y_RANGE_SHIFT, range as u8);
        self.write_register(SENSOR_CONFIG_2, updated)?;
        self.config.xy_range = range;
        Ok(())
    }

    /// Update Z axis magnetic measurement range in `SENSOR_CONFIG_2` (0x03).
    pub fn set_z_range(&mut self, range: Range) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(SENSOR_CONFIG_2)?;
        let updated = insert(current, Z_RANGE_MASK, Z_RANGE_SHIFT, range as u8);
        self.write_register(SENSOR_CONFIG_2, updated)?;
        self.config.z_range = range;
        Ok(())
    }

    /// Update conversion averaging filter in `DEVICE_CONFIG_1` (0x00).
    pub fn set_averaging(&mut self, avg: ConversionAverage) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(DEVICE_CONFIG_1)?;
        let updated = insert(current, CONV_AVG_MASK, CONV_AVG_SHIFT, avg as u8);
        self.write_register(DEVICE_CONFIG_1, updated)?;
        self.config.averaging = avg;
        Ok(())
    }

    /// Update power vs. noise mode in `DEVICE_CONFIG_2` (0x01).
    pub fn set_power_noise(&mut self, mode: PowerNoiseMode) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(DEVICE_CONFIG_2)?;
        let updated = insert(current, LP_LN_MASK, LP_LN_SHIFT, mode as u8);
        self.write_register(DEVICE_CONFIG_2, updated)?;
        self.config.power_noise = mode;
        Ok(())
    }

    /// Enable or disable the I2C glitch filter in `DEVICE_CONFIG_2` (0x01).
    pub fn set_glitch_filter(&mut self, enabled: bool) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(DEVICE_CONFIG_2)?;
        let updated = if enabled {
            current | I2C_GLITCH_FILTER_MASK
        } else {
            current & !I2C_GLITCH_FILTER_MASK
        };
        self.write_register(DEVICE_CONFIG_2, updated)?;
        self.config.glitch_filter = enabled;
        Ok(())
    }

    /// Enable or disable CRC-8 on I2C transactions in `DEVICE_CONFIG_1` (0x00).
    pub fn set_crc_enabled(&mut self, enabled: bool) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(DEVICE_CONFIG_1)?;
        let updated = if enabled {
            current | CRC_EN_MASK
        } else {
            current & !CRC_EN_MASK
        };
        self.write_register(DEVICE_CONFIG_1, updated)?;
        self.config.crc_enabled = enabled;
        Ok(())
    }

    /// Reconfigure all device configuration registers in-place without dropping the I2C peripheral.
    pub fn reconfigure(&mut self, cfg: Config) -> Result<(), Error<I2C::Error>> {
        // DEVICE_CONFIG_1
        let mut dc1 = 0u8;
        if cfg.crc_enabled {
            dc1 |= CRC_EN_MASK;
        }
        dc1 = insert(dc1, CONV_AVG_MASK, CONV_AVG_SHIFT, cfg.averaging as u8);
        dc1 = insert(dc1, I2C_RD_MASK, I2C_RD_SHIFT, cfg.read_mode as u8);
        self.write_register(DEVICE_CONFIG_1, dc1)?;
        // Silicon updates CRC_EN immediately on DEVICE_CONFIG_1 write; sync local flag
        // before writing subsequent registers.
        self.config.crc_enabled = cfg.crc_enabled;

        // DEVICE_CONFIG_2
        let mut dc2 = 0u8;
        dc2 = insert(dc2, LP_LN_MASK, LP_LN_SHIFT, cfg.power_noise as u8);
        if cfg.glitch_filter {
            dc2 |= I2C_GLITCH_FILTER_MASK;
        }
        dc2 = insert(
            dc2,
            OPERATING_MODE_MASK,
            OPERATING_MODE_SHIFT,
            cfg.operating_mode as u8,
        );
        self.write_register(DEVICE_CONFIG_2, dc2)?;

        // SENSOR_CONFIG_1
        let sc1 = insert(0, MAG_CH_EN_MASK, MAG_CH_EN_SHIFT, cfg.channels as u8);
        self.write_register(SENSOR_CONFIG_1, sc1)?;

        // SENSOR_CONFIG_2
        let mut sc2 = 0u8;
        sc2 = insert(sc2, ANGLE_EN_MASK, ANGLE_EN_SHIFT, cfg.angle_enabled as u8);
        sc2 = insert(sc2, X_Y_RANGE_MASK, X_Y_RANGE_SHIFT, cfg.xy_range as u8);
        sc2 = insert(sc2, Z_RANGE_MASK, Z_RANGE_SHIFT, cfg.z_range as u8);
        self.write_register(SENSOR_CONFIG_2, sc2)?;

        // T_CONFIG
        let tc = if cfg.temp_enabled { T_CH_EN_MASK } else { 0 };
        self.write_register(T_CONFIG, tc)?;

        self.config = cfg;
        self.read_mode = cfg.read_mode;
        self.variant = cfg.variant;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[test]
    fn live_setters_update_registers_and_cached_config() {
        let addr = 0x35;
        let expectations = [
            // set_xy_range(Range::Low) -> read SENSOR_CONFIG_2 (0x03), write updated
            I2cTransaction::write_read(addr, vec![SENSOR_CONFIG_2], vec![0x06]),
            I2cTransaction::write(addr, vec![SENSOR_CONFIG_2, 0x04]),
            // set_averaging(ConversionAverage::Avg8x) -> read DEVICE_CONFIG_1 (0x00), write updated
            I2cTransaction::write_read(addr, vec![DEVICE_CONFIG_1], vec![0x00]),
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_1, 0x0C]),
            // set_crc_enabled(true) -> read DEVICE_CONFIG_1 (0x00), write updated
            I2cTransaction::write_read(addr, vec![DEVICE_CONFIG_1], vec![0x0C]),
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_1, 0x8C]),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Tmag5273::new_uninit(i2c, addr);

        dev.set_xy_range(Range::Low).unwrap();
        assert_eq!(dev.config().xy_range, Range::Low);

        dev.set_averaging(ConversionAverage::Avg8x).unwrap();
        assert_eq!(dev.config().averaging, ConversionAverage::Avg8x);

        dev.set_crc_enabled(true).unwrap();
        assert!(dev.config().crc_enabled);

        dev.into_inner().done();
    }

    #[test]
    fn reconfigure_writes_all_config_registers_in_place() {
        let addr = 0x35;
        let cfg = Config {
            xy_range: Range::Low,
            z_range: Range::Low,
            averaging: ConversionAverage::Avg4x,
            power_noise: PowerNoiseMode::LowNoise,
            glitch_filter: true,
            crc_enabled: true,
            operating_mode: OperatingMode::ContinuousMeasure,
            channels: MagneticChannel::XYZ,
            angle_enabled: AngleEnable::XY,
            ..Config::default()
        };
        let expectations = [
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_1, 0x88]),
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_2, 0x1A]),
            I2cTransaction::write(addr, vec![SENSOR_CONFIG_1, 0x70]),
            I2cTransaction::write(addr, vec![SENSOR_CONFIG_2, 0x04]),
            I2cTransaction::write(addr, vec![T_CONFIG, 0x00]),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Tmag5273::new_uninit(i2c, addr);

        dev.reconfigure(cfg).unwrap();
        assert_eq!(dev.config().xy_range, Range::Low);
        assert_eq!(dev.config().averaging, ConversionAverage::Avg4x);
        assert!(dev.config().crc_enabled);
        assert!(dev.config().glitch_filter);

        dev.into_inner().done();
    }

    #[test]
    fn crc8_read_write_and_angle_verification() {
        let addr = 0x35;

        let reg_read_crc = crate::crc::crc8(&[0x00, 0x01, 0x02, 0x03]);
        // Angle raw = 0x0B40 (180.0 deg = (0x0B40 >> 4) + (0x0B40 & 0xF)/16)
        let angle_msb = 0x0Bu8;
        let angle_lsb = 0x40u8;
        let angle_crc = crate::crc::crc8(&[angle_msb, angle_lsb, 0x10, 0x20]);

        let expectations = [
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_2, 0x02]),
            I2cTransaction::write_read(
                addr,
                vec![DEVICE_STATUS],
                vec![0x00, 0x01, 0x02, 0x03, reg_read_crc],
            ),
            I2cTransaction::write_read(
                addr,
                vec![ANGLE_RESULT_MSB],
                vec![angle_msb, angle_lsb, 0x10, 0x20, angle_crc],
            ),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Tmag5273::new_uninit(i2c, addr);
        dev.config.crc_enabled = true;
        dev.config.angle_enabled = AngleEnable::XY;

        dev.write_register(DEVICE_CONFIG_2, 0x02).unwrap();
        assert_eq!(dev.read_register(DEVICE_STATUS).unwrap(), 0x00);
        let angle = dev.read_angle().unwrap();
        assert!((angle.degrees() - 180.0).abs() < 0.1);

        dev.into_inner().done();
    }

    #[test]
    fn crc8_mismatch_returns_error_on_corrupted_read() {
        let addr = 0x35;

        let good_reg_crc = crate::crc::crc8(&[0x00, 0x00, 0x00, 0x00]);
        let bad_reg_crc = good_reg_crc ^ 0xFF;

        let angle_msb = 0x0Bu8;
        let angle_lsb = 0x40u8;
        let good_angle_crc = crate::crc::crc8(&[angle_msb, angle_lsb, 0x00, 0x00]);
        let bad_angle_crc = good_angle_crc ^ 0x01;

        let expectations = [
            I2cTransaction::write_read(
                addr,
                vec![DEVICE_STATUS],
                vec![0x00, 0x00, 0x00, 0x00, bad_reg_crc],
            ),
            I2cTransaction::write_read(
                addr,
                vec![ANGLE_RESULT_MSB],
                vec![angle_msb, angle_lsb, 0x00, 0x00, bad_angle_crc],
            ),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Tmag5273::new_uninit(i2c, addr);
        dev.config.crc_enabled = true;
        dev.config.angle_enabled = AngleEnable::XY;

        assert_eq!(
            dev.read_register(DEVICE_STATUS),
            Err(Error::CrcMismatch {
                expected: bad_reg_crc,
                computed: good_reg_crc,
            })
        );
        assert_eq!(
            dev.read_angle(),
            Err(Error::CrcMismatch {
                expected: bad_angle_crc,
                computed: good_angle_crc,
            })
        );

        dev.into_inner().done();
    }

    #[test]
    fn crc_enable_disable_transitions_match_silicon_state() {
        let addr = 0x35;

        let read_dc1_with_crc = crate::crc::crc8(&[0x8C, 0x00, 0x00, 0x00]);

        let expectations = [
            // false -> true: read without CRC, write without CRC
            I2cTransaction::write_read(addr, vec![DEVICE_CONFIG_1], vec![0x0C]),
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_1, 0x8C]),
            // true -> false: read WITH CRC (4 data + 1 CRC), write without CRC
            I2cTransaction::write_read(
                addr,
                vec![DEVICE_CONFIG_1],
                vec![0x8C, 0x00, 0x00, 0x00, read_dc1_with_crc],
            ),
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_1, 0x0C]),
            // Now CRC is disabled again: read without CRC
            I2cTransaction::write_read(addr, vec![DEVICE_CONFIG_1], vec![0x0C]),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Tmag5273::new_uninit(i2c, addr);

        dev.set_crc_enabled(true).unwrap();
        assert!(dev.config().crc_enabled);

        dev.set_crc_enabled(false).unwrap();
        assert!(!dev.config().crc_enabled);

        assert_eq!(dev.read_register(DEVICE_CONFIG_1).unwrap(), 0x0C);

        dev.into_inner().done();
    }
}
