//! # barefoc::sensor_1d — 1-DOF Joint Sensor Adapter for BareFOC
//!
//! Wraps [`Tmag5273`] to implement [`barefoc::traits::sensor::Sensor`] using the internal
//! hardware CORDIC angle calculation for 1-DOF actuators.

use barefoc::common::time::Clock;
use barefoc::traits::sensor::{
    Sensor, SensorBase, SensorConfig, SensorConfigError, SensorRuntimeConfig, SensorStatus,
};
use core::cell::{Cell, RefCell};
use embedded_hal::i2c::I2c;

use crate::error::Error;
use crate::tmag5273::Tmag5273;
use crate::types::{ConversionAverage, PowerNoiseMode, Range};

/// 1-DOF joint position sensor adapter implementing BareFOC [`Sensor`].
pub struct Tmag5273Sensor<I2C> {
    /// Internal TMAG5273 driver wrapped in `RefCell` for interior mutability during `get_sensor_angle`.
    pub driver: RefCell<Tmag5273<I2C>>,
    base: SensorBase,
    last_status: Cell<SensorStatus>,
    last_angle: Cell<f32>,
    consecutive_errors: Cell<u8>,
}

const SENSOR_ERR_DEBOUNCE: u8 = 50;

impl<I2C> Tmag5273Sensor<I2C> {
    /// Create a new 1-DOF sensor adapter wrapping a configured [`Tmag5273`] driver.
    pub fn new(driver: Tmag5273<I2C>) -> Self {
        Self {
            driver: RefCell::new(driver),
            base: SensorBase::new(),
            last_status: Cell::new(SensorStatus::Ok),
            last_angle: Cell::new(0.0),
            consecutive_errors: Cell::new(0),
        }
    }
}

impl<I2C: I2c, C: Clock> Sensor<C> for Tmag5273Sensor<I2C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        if let Ok(mut driver) = self.driver.try_borrow_mut() {
            match driver.read_angle() {
                Ok(angle) => {
                    let rad = angle.radians();
                    self.consecutive_errors.set(0);
                    self.last_status.set(SensorStatus::Ok);
                    self.last_angle.set(rad);
                    rad
                }
                Err(err) => {
                    let count = self.consecutive_errors.get().saturating_add(1);
                    self.consecutive_errors.set(count);
                    if count >= SENSOR_ERR_DEBOUNCE {
                        let st = match err {
                            Error::I2c(_) | Error::DeviceNotFound => SensorStatus::Disconnected,
                            Error::CrcMismatch { .. } => SensorStatus::CrcError,
                            _ => SensorStatus::MagFieldFault,
                        };
                        self.last_status.set(st);
                    }
                    self.last_angle.get()
                }
            }
        } else {
            let count = self.consecutive_errors.get().saturating_add(1);
            self.consecutive_errors.set(count);
            if count >= SENSOR_ERR_DEBOUNCE {
                self.last_status.set(SensorStatus::Disconnected);
            }
            self.last_angle.get()
        }
    }

    fn status(&self) -> SensorStatus {
        self.last_status.get()
    }
}

impl<I2C: I2c> SensorRuntimeConfig for Tmag5273Sensor<I2C> {
    fn sensor_config(&self) -> SensorConfig {
        let driver = self.driver.borrow();
        let cfg = driver.config();
        SensorConfig {
            range: cfg.xy_range as u8,
            averaging: cfg.averaging as u8,
            power_mode: match cfg.power_noise {
                PowerNoiseMode::LowNoise => 0,
                PowerNoiseMode::LowPower => 1,
            },
            crc_enabled: cfg.crc_enabled,
            glitch_filter: cfg.glitch_filter,
        }
    }

    fn apply_sensor_config(&mut self, cfg: SensorConfig) -> Result<(), SensorConfigError> {
        let range = Range::try_from(cfg.range).map_err(|_| SensorConfigError::InvalidParameter)?;
        let averaging = ConversionAverage::try_from(cfg.averaging)
            .map_err(|_| SensorConfigError::InvalidParameter)?;
        let power_noise = match cfg.power_mode {
            0 => PowerNoiseMode::LowNoise,
            1 => PowerNoiseMode::LowPower,
            _ => return Err(SensorConfigError::InvalidParameter),
        };

        let mut driver = self
            .driver
            .try_borrow_mut()
            .map_err(|_| SensorConfigError::BusError)?;
        let mut new_cfg = *driver.config();
        new_cfg.xy_range = range;
        new_cfg.z_range = range;
        new_cfg.averaging = averaging;
        new_cfg.power_noise = power_noise;
        new_cfg.crc_enabled = cfg.crc_enabled;
        new_cfg.glitch_filter = cfg.glitch_filter;

        driver
            .reconfigure(new_cfg)
            .map_err(|_| SensorConfigError::BusError)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registers::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[test]
    fn tmag5273_sensor_runtime_config_roundtrip() {
        let addr = 0x35;
        let expectations = [
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_1, 0x8C]),
            I2cTransaction::write(addr, vec![DEVICE_CONFIG_2, 0x1A]),
            I2cTransaction::write(addr, vec![SENSOR_CONFIG_1, 0x70]),
            I2cTransaction::write(addr, vec![SENSOR_CONFIG_2, 0x00]),
            I2cTransaction::write(addr, vec![T_CONFIG, 0x00]),
        ];
        let i2c = I2cMock::new(&expectations);
        let dev = Tmag5273::new_uninit(i2c, addr);
        let mut sensor = Tmag5273Sensor::new(dev);

        let new_cfg = SensorConfig {
            range: 0,
            averaging: 3, // Avg8x
            power_mode: 0, // LowNoise
            crc_enabled: true,
            glitch_filter: true,
        };
        sensor.apply_sensor_config(new_cfg).unwrap();
        assert_eq!(sensor.sensor_config(), new_cfg);

        // Invalid averaging should return InvalidParameter without bus I/O
        assert_eq!(
            sensor.apply_sensor_config(SensorConfig {
                averaging: 99,
                ..new_cfg
            }),
            Err(SensorConfigError::InvalidParameter)
        );

        sensor.driver.into_inner().into_inner().done();
    }
}
