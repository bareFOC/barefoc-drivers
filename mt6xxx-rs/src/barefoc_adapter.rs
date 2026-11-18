//! # barefoc_adapter — BareFOC Sensor trait implementation for MT6701
//!
//! Upstream reference:
//! - [`MagneticSensorMT6701SSI.h`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.h)
//! - [`MagneticSensorMT6701SSI.cpp`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.cpp)

use barefoc::common::time::Clock;
use barefoc::traits::sensor::{Sensor, SensorBase};
use embedded_hal::spi::SpiDevice;

use crate::mt6701::blocking::Mt6701;

impl<SPI: SpiDevice, C: Clock> Sensor<C> for Mt6701<SPI> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        self.read_angle_radians()
    }
}
