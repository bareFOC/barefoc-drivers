//! # tmag5273 — TMAG5273 Driver Core and Builder
//!
//! Provides the generic [`Tmag5273<I2C>`] device struct and the fluent [`Tmag5273Builder<I2C>`]
//! for configuring and initializing the sensor.

pub mod sync;

#[cfg(feature = "async")]
pub mod asynch;

use crate::types::*;

/// Texas Instruments TMAG5273 3D linear Hall-effect sensor driver.
#[derive(Debug)]
pub struct Tmag5273<I2C> {
    pub(crate) i2c: I2C,
    pub(crate) address: u8,
    pub(crate) config: Config,
    pub(crate) read_mode: I2cReadMode,
    pub(crate) variant: DeviceVariant,
}

impl<I2C> Tmag5273<I2C> {
    /// Create a new uninitialized TMAG5273 driver with default configuration without performing bus I/O.
    pub fn new_uninit(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            config: Config::default(),
            read_mode: I2cReadMode::Standard,
            variant: DeviceVariant::A1,
        }
    }

    /// Create a fluent builder for configuring and initializing the driver.
    pub fn builder(i2c: I2C, address: u8) -> Tmag5273Builder<I2C> {
        Tmag5273Builder::new(i2c, address)
    }

    /// Return the active 7-bit I2C slave address.
    #[inline]
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Return a reference to the active device configuration.
    #[inline]
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Return the configured device variant.
    #[inline]
    pub fn variant(&self) -> DeviceVariant {
        self.variant
    }

    /// Return the currently active I2C read mode.
    #[inline]
    pub fn read_mode(&self) -> I2cReadMode {
        self.read_mode
    }

    /// Consume the driver and reclaim the underlying I2C peripheral.
    #[inline]
    pub fn into_inner(self) -> I2C {
        self.i2c
    }
}

/// Fluent builder for [`Tmag5273`].
#[derive(Debug)]
pub struct Tmag5273Builder<I2C> {
    pub(crate) i2c: I2C,
    pub(crate) address: u8,
    pub(crate) config: Config,
}

impl<I2C> Tmag5273Builder<I2C> {
    /// Create a new builder with the given I2C peripheral and address.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            config: Config::default(),
        }
    }

    /// Set the device variant (determines expected version and magnetic range).
    pub fn variant(mut self, variant: DeviceVariant) -> Self {
        self.config.variant = variant;
        self
    }

    /// Set operating mode (Standby, Sleep, Continuous, Wake-and-Sleep).
    pub fn operating_mode(mut self, mode: OperatingMode) -> Self {
        self.config.operating_mode = mode;
        self
    }

    /// Set magnetic channels to enable.
    pub fn channels(mut self, channels: MagneticChannel) -> Self {
        self.config.channels = channels;
        self
    }

    /// Set conversion averaging filter.
    pub fn averaging(mut self, avg: ConversionAverage) -> Self {
        self.config.averaging = avg;
        self
    }

    /// Enable hardware CORDIC angle calculation on selected axis plane.
    pub fn angle_enabled(mut self, angle: AngleEnable) -> Self {
        self.config.angle_enabled = angle;
        self
    }

    /// Set measurement range for X and Y axes.
    pub fn xy_range(mut self, range: Range) -> Self {
        self.config.xy_range = range;
        self
    }

    /// Set measurement range for Z axis.
    pub fn z_range(mut self, range: Range) -> Self {
        self.config.z_range = range;
        self
    }

    /// Set power vs noise mode.
    pub fn power_noise(mut self, mode: PowerNoiseMode) -> Self {
        self.config.power_noise = mode;
        self
    }

    /// Enable or disable I2C glitch filter.
    pub fn glitch_filter(mut self, enabled: bool) -> Self {
        self.config.glitch_filter = enabled;
        self
    }

    /// Enable or disable temperature sensor channel.
    pub fn temp_enabled(mut self, enabled: bool) -> Self {
        self.config.temp_enabled = enabled;
        self
    }

    /// Set I2C read mode (Standard, OneByte16Bit, OneByte8Bit).
    pub fn read_mode(mut self, mode: I2cReadMode) -> Self {
        self.config.read_mode = mode;
        self
    }

    /// Enable or disable CRC-8 on I2C transactions.
    pub fn crc_enabled(mut self, enabled: bool) -> Self {
        self.config.crc_enabled = enabled;
        self
    }
}
