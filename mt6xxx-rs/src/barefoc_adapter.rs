//! # barefoc_adapter — BareFOC Sensor trait implementation for MT6701
//!
//! Upstream reference:
//! - [`MagneticSensorMT6701SSI.h`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.h)
//! - [`MagneticSensorMT6701SSI.cpp`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.cpp)

use barefoc::common::time::Clock;
use barefoc::traits::sensor::{
    Sensor, SensorBase, SensorConfig, SensorConfigError, SensorRuntimeConfig, SensorStatus,
};
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

    fn status(&self) -> SensorStatus {
        if self.has_fault() {
            return SensorStatus::CrcError;
        }
        let st = self.last_status();
        if st.los || st.mgh || st.mgl {
            return SensorStatus::MagFieldFault;
        }
        SensorStatus::Ok
    }
}

impl<SPI: SpiDevice> SensorRuntimeConfig for Mt6701<SPI> {
    fn sensor_config(&self) -> SensorConfig {
        SensorConfig {
            range: 1,
            averaging: 0,
            power_mode: 0,
            crc_enabled: true,
            glitch_filter: false,
        }
    }

    fn apply_sensor_config(&mut self, cfg: SensorConfig) -> Result<(), SensorConfigError> {
        // MT6701 SSI frames have fixed hardware CRC-6 and fixed range/averaging.
        if cfg.range > 1 || cfg.averaging > 0 || cfg.power_mode > 1 || !cfg.crc_enabled {
            return Err(SensorConfigError::Unsupported);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::spi::{ErrorType, Operation};

    struct DummySpi;
    #[derive(Debug)]
    struct DummyErr;
    impl embedded_hal::spi::Error for DummyErr {
        fn kind(&self) -> embedded_hal::spi::ErrorKind {
            embedded_hal::spi::ErrorKind::Other
        }
    }
    impl ErrorType for DummySpi {
        type Error = DummyErr;
    }
    impl SpiDevice for DummySpi {
        fn transaction(&mut self, _operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn mt6701_runtime_config_reports_and_validates() {
        let mut sensor = Mt6701::new(DummySpi);
        let cfg = sensor.sensor_config();
        assert!(cfg.crc_enabled);
        assert_eq!(cfg.range, 1);
        assert_eq!(cfg.averaging, 0);

        assert!(sensor.apply_sensor_config(cfg).is_ok());
        assert_eq!(
            sensor.apply_sensor_config(SensorConfig {
                crc_enabled: false,
                ..cfg
            }),
            Err(SensorConfigError::Unsupported)
        );
    }
}
