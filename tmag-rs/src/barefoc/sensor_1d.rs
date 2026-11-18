//! # barefoc::sensor_1d — 1-DOF Joint Sensor Adapter for BareFOC
//!
//! Wraps [`Tmag5273`] to implement [`barefoc::traits::sensor::Sensor`] using the internal
//! hardware CORDIC angle calculation for 1-DOF actuators.

use barefoc::common::time::Clock;
use barefoc::traits::sensor::{Sensor, SensorBase};
use core::cell::RefCell;
use embedded_hal::i2c::I2c;

use crate::tmag5273::Tmag5273;

/// 1-DOF joint position sensor adapter implementing BareFOC [`Sensor`].
pub struct Tmag5273Sensor<I2C> {
    /// Internal TMAG5273 driver wrapped in `RefCell` for interior mutability during `get_sensor_angle`.
    pub driver: RefCell<Tmag5273<I2C>>,
    base: SensorBase,
}

impl<I2C> Tmag5273Sensor<I2C> {
    /// Create a new 1-DOF sensor adapter wrapping a configured [`Tmag5273`] driver.
    pub fn new(driver: Tmag5273<I2C>) -> Self {
        Self {
            driver: RefCell::new(driver),
            base: SensorBase::new(),
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
                Ok(angle) => angle.radians(),
                Err(_) => -1.0, // Negative return value signals an error to SensorBase::update
            }
        } else {
            -1.0
        }
    }
}
