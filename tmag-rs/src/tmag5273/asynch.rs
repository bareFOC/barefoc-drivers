//! # asynch — Asynchronous I2C Driver Implementation
//!
//! Provides non-blocking driver operations over `embedded_hal_async::i2c::I2c`.

use embedded_hal_async::i2c::I2c;

use super::{Tmag5273, Tmag5273Builder};
use crate::angle::Angle;
use crate::error::Error;
use crate::magnetic::MagneticField3D;
use crate::registers::*;
use crate::types::*;

impl<I2C: I2c> Tmag5273Builder<I2C> {
    /// Build and initialize the sensor asynchronously.
    ///
    /// Verifies manufacturer ID `0x5449`, verifies device variant version,
    /// clears power-on-reset and latched fault flags, and writes configuration registers.
    pub async fn build_async(mut self) -> Result<Tmag5273<I2C>, Error<I2C::Error>> {
        // 1. Verify manufacturer ID (0x5449)
        let mut mfg_buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[MANUFACTURER_ID_LSB], &mut mfg_buf)
            .await
            .map_err(Error::I2c)?;
        let mfg_id = u16::from_le_bytes(mfg_buf);
        if mfg_id != 0x5449 {
            return Err(Error::InvalidManufacturerId(mfg_id));
        }

        // 2. Verify device ID version
        let mut dev_id = [0u8; 1];
        self.i2c
            .write_read(self.address, &[DEVICE_ID], &mut dev_id)
            .await
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
            .await
            .map_err(Error::I2c)?;
        self.i2c
            .write(self.address, &[DEVICE_STATUS, FAULT_CLEAR_MASK])
            .await
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
            .await
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
            .await
            .map_err(Error::I2c)?;

        // SENSOR_CONFIG_1
        let sc1 = insert(0, MAG_CH_EN_MASK, MAG_CH_EN_SHIFT, self.config.channels as u8);
        self.i2c
            .write(self.address, &[SENSOR_CONFIG_1, sc1])
            .await
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
            .await
            .map_err(Error::I2c)?;

        // T_CONFIG
        let tc = if self.config.temp_enabled {
            T_CH_EN_MASK
        } else {
            0
        };
        self.i2c
            .write(self.address, &[T_CONFIG, tc])
            .await
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
    /// Initialize a new TMAG5273 sensor asynchronously using default configuration.
    pub async fn new_async(i2c: I2C, address: u8) -> Result<Self, Error<I2C::Error>> {
        Self::builder(i2c, address).build_async().await
    }

    /// Write a single register byte asynchronously.
    #[inline]
    pub async fn write_register_async(
        &mut self,
        register: u8,
        value: u8,
    ) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, value])
            .await
            .map_err(Error::I2c)
    }

    /// Read a single register byte asynchronously.
    #[inline]
    pub async fn read_register_async(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Read multiple contiguous register bytes starting at `register` asynchronously.
    #[inline]
    pub async fn read_registers_async(
        &mut self,
        register: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[register], buf)
            .await
            .map_err(Error::I2c)
    }

    /// Execute an asynchronous 1-byte fast stream read directly without sending a register address byte.
    #[inline]
    pub async fn read_stream_async(&mut self, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.read(self.address, buf).await.map_err(Error::I2c)
    }

    /// Clear the latched Power-On-Reset (POR) flag in `CONV_STATUS` (0x18) asynchronously.
    #[inline]
    pub async fn clear_por_async(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register_async(CONV_STATUS, POR_MASK).await
    }

    /// Clear all latched fault flags in `DEVICE_STATUS` (0x1C) asynchronously.
    #[inline]
    pub async fn clear_faults_async(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register_async(DEVICE_STATUS, FAULT_CLEAR_MASK)
            .await
    }

    /// Read and decode device status asynchronously (`DEVICE_STATUS`, 0x1C).
    pub async fn read_status_async(&mut self) -> Result<DeviceStatus, Error<I2C::Error>> {
        let val = self.read_register_async(DEVICE_STATUS).await?;
        Ok(DeviceStatus::from_raw(val))
    }

    /// Read and decode conversion status asynchronously (`CONV_STATUS`, 0x18).
    pub async fn read_conv_status_async(&mut self) -> Result<ConversionStatus, Error<I2C::Error>> {
        let val = self.read_register_async(CONV_STATUS).await?;
        Ok(ConversionStatus::from_raw(val))
    }

    /// Read the hardware CORDIC angle asynchronously.
    pub async fn read_angle_async(&mut self) -> Result<Angle, Error<I2C::Error>> {
        if self.config.angle_enabled == AngleEnable::None {
            return Err(Error::AngleNotEnabled);
        }
        let mut buf = [0u8; 2];
        self.read_registers_async(ANGLE_RESULT_MSB, &mut buf).await?;
        let raw = u16::from_be_bytes(buf);
        Angle::from_raw(raw).ok_or(Error::InvalidRegisterValue {
            register: ANGLE_RESULT_MSB,
            value: buf[0],
        })
    }

    /// Read synchronous 3D magnetic flux density vector $[B_x, B_y, B_z]$ via a 6-byte burst read asynchronously.
    pub async fn read_magnetic_async(&mut self) -> Result<MagneticField3D, Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_registers_async(X_MSB_RESULT, &mut buf).await?;
        let raw_x = i16::from_be_bytes([buf[0], buf[1]]);
        let raw_y = i16::from_be_bytes([buf[2], buf[3]]);
        let raw_z = i16::from_be_bytes([buf[4], buf[5]]);

        let range_xy = self.variant.range_value(self.config.xy_range);
        let range_z = self.variant.range_value(self.config.z_range);

        Ok(MagneticField3D::from_raw_16bit(
            raw_x, raw_y, raw_z, range_xy, range_z,
        ))
    }

    /// Read magnetic field using fast 8-bit MSB reading mode asynchronously.
    pub async fn read_magnetic_8bit_async(&mut self) -> Result<MagneticField3D, Error<I2C::Error>> {
        let range_xy = self.variant.range_value(self.config.xy_range);
        let range_z = self.variant.range_value(self.config.z_range);

        if self.read_mode == I2cReadMode::OneByte8Bit {
            let mut buf = [0u8; 4];
            self.read_stream_async(&mut buf).await?;
            Ok(MagneticField3D::from_raw_8bit(
                buf[0] as i8,
                buf[1] as i8,
                buf[2] as i8,
                range_xy,
                range_z,
            ))
        } else {
            let raw_x = self.read_register_async(X_MSB_RESULT).await? as i8;
            let raw_y = self.read_register_async(Y_MSB_RESULT).await? as i8;
            let raw_z = self.read_register_async(Z_MSB_RESULT).await? as i8;
            Ok(MagneticField3D::from_raw_8bit(
                raw_x, raw_y, raw_z, range_xy, range_z,
            ))
        }
    }

    /// Read temperature in degrees Celsius asynchronously.
    pub async fn read_temperature_async(&mut self) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_registers_async(T_MSB_RESULT, &mut buf).await?;
        let raw = i16::from_be_bytes(buf);
        let temp = 25.0 + ((raw as f32) - 17508.0) / 60.1;
        Ok(temp)
    }

    /// Update I2C read mode in `DEVICE_CONFIG_1` asynchronously.
    pub async fn set_read_mode_async(&mut self, mode: I2cReadMode) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register_async(DEVICE_CONFIG_1).await?;
        let updated = insert(current, I2C_RD_MASK, I2C_RD_SHIFT, mode as u8);
        self.write_register_async(DEVICE_CONFIG_1, updated).await?;
        self.read_mode = mode;
        self.config.read_mode = mode;
        Ok(())
    }

    /// Update device operating mode in `DEVICE_CONFIG_2` asynchronously.
    pub async fn set_operating_mode_async(&mut self, mode: OperatingMode) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register_async(DEVICE_CONFIG_2).await?;
        let updated = insert(current, OPERATING_MODE_MASK, OPERATING_MODE_SHIFT, mode as u8);
        self.write_register_async(DEVICE_CONFIG_2, updated).await?;
        self.config.operating_mode = mode;
        Ok(())
    }
}
