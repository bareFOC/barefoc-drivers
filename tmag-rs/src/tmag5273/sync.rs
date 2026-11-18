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

        // 4. Configure device registers
        // DEVICE_CONFIG_1
        let mut dc1 = 0u8;
        if self.config.crc_enabled {
            dc1 |= CRC_EN_MASK;
        }
        dc1 = insert(dc1, CONV_AVG_MASK, CONV_AVG_SHIFT, self.config.averaging as u8);
        dc1 = insert(dc1, I2C_RD_MASK, I2C_RD_SHIFT, self.config.read_mode as u8);
        self.i2c
            .write(self.address, &[DEVICE_CONFIG_1, dc1])
            .map_err(Error::I2c)?;

        // DEVICE_CONFIG_2
        let mut dc2 = 0u8;
        dc2 = insert(dc2, LP_LN_MASK, LP_LN_SHIFT, self.config.power_noise as u8);
        if self.config.glitch_filter {
            dc2 |= I2C_GLITCH_FILTER_MASK;
        }
        dc2 = insert(
            dc2,
            OPERATING_MODE_MASK,
            OPERATING_MODE_SHIFT,
            self.config.operating_mode as u8,
        );
        self.i2c
            .write(self.address, &[DEVICE_CONFIG_2, dc2])
            .map_err(Error::I2c)?;

        // SENSOR_CONFIG_1
        let sc1 = insert(0, MAG_CH_EN_MASK, MAG_CH_EN_SHIFT, self.config.channels as u8);
        self.i2c
            .write(self.address, &[SENSOR_CONFIG_1, sc1])
            .map_err(Error::I2c)?;

        // SENSOR_CONFIG_2
        let mut sc2 = 0u8;
        sc2 = insert(
            sc2,
            ANGLE_EN_MASK,
            ANGLE_EN_SHIFT,
            self.config.angle_enabled as u8,
        );
        sc2 = insert(
            sc2,
            X_Y_RANGE_MASK,
            X_Y_RANGE_SHIFT,
            self.config.xy_range as u8,
        );
        sc2 = insert(sc2, Z_RANGE_MASK, Z_RANGE_SHIFT, self.config.z_range as u8);
        self.i2c
            .write(self.address, &[SENSOR_CONFIG_2, sc2])
            .map_err(Error::I2c)?;

        // T_CONFIG
        let tc = if self.config.temp_enabled {
            T_CH_EN_MASK
        } else {
            0
        };
        self.i2c
            .write(self.address, &[T_CONFIG, tc])
            .map_err(Error::I2c)?;

        Ok(Tmag5273 {
            i2c: self.i2c,
            address: self.address,
            config: self.config,
            read_mode: self.config.read_mode,
            variant: self.config.variant,
        })
    }
}

impl<I2C: I2c> Tmag5273<I2C> {
    /// Initialize a new TMAG5273 sensor using default configuration.
    pub fn new(i2c: I2C, address: u8) -> Result<Self, Error<I2C::Error>> {
        Self::builder(i2c, address).build()
    }

    /// Write a single register byte.
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
        self.i2c
            .write_read(self.address, &[register], &mut buf)
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Read multiple contiguous register bytes starting at `register`.
    #[inline]
    pub fn read_registers(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[register], buf)
            .map_err(Error::I2c)
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
}
