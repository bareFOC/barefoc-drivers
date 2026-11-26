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

    /// Return the number of consecutive read errors since the last valid sample.
    pub fn consecutive_errors(&self) -> u8 {
        self.consecutive_errors.get()
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
                    if rad.is_finite() && rad >= 0.0 && rad < barefoc::math::_2PI {
                        self.consecutive_errors.set(0);
                        self.last_status.set(SensorStatus::Ok);
                        self.last_angle.set(rad);
                        rad
                    } else {
                        let count = self.consecutive_errors.get().saturating_add(1);
                        self.consecutive_errors.set(count);
                        if count >= SENSOR_ERR_DEBOUNCE {
                            self.last_status.set(SensorStatus::MagFieldFault);
                        }
                        self.last_angle.get()
                    }
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

    #[test]
    fn tmag5273_sensor_validates_angle_and_preserves_last_good() {
        use crate::types::AngleEnable;
        use barefoc::motor::base::NullClock;

        let addr = 0x35;
        // 1st read: 90.0 deg -> raw = 90 * 16 = 1440 = 0x05A0
        // 2nd read: out-of-range (>= 360 deg, e.g. 0x1680 = 360.0 deg = 2*pi)
        let expectations = [
            I2cTransaction::write_read(addr, vec![ANGLE_RESULT_MSB], vec![0x05, 0xA0]),
            I2cTransaction::write_read(addr, vec![ANGLE_RESULT_MSB], vec![0x16, 0x80]),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Tmag5273::new_uninit(i2c, addr);
        dev.config.angle_enabled = AngleEnable::XY;
        let sensor = Tmag5273Sensor::new(dev);

        let clock = NullClock;
        let _ = &clock;
        let a1 = <Tmag5273Sensor<I2cMock> as Sensor<NullClock>>::get_sensor_angle(&sensor);
        assert!((a1 - core::f32::consts::FRAC_PI_2).abs() < 1e-3);
        assert_eq!(sensor.consecutive_errors(), 0);

        // 360.0 deg == _2PI -> rejected, returns last good angle and increments error count
        let a2 = <Tmag5273Sensor<I2cMock> as Sensor<NullClock>>::get_sensor_angle(&sensor);
        assert!((a2 - a1).abs() < 1e-6);
        assert_eq!(sensor.consecutive_errors(), 1);

        sensor.driver.into_inner().into_inner().done();
    }
}
