//! # drv8316::pwm_variant — PWM-over-SPI variant (DRV8316P)
//!
//! Controls the DRV8316P where phase outputs can be commanded over high-speed SPI.

use crate::{CsaGainProvider, Drv83xxDriver, DrvError};
use super::config::Drv8316PwmMode;
use super::spi_variant::Drv8316S;
use super::status::Drv8316Status;

/// PWM-over-SPI variant of the DRV8316 (DRV8316P).
pub struct Drv8316P<SPI> {
    pub spi_dev: Drv8316S<SPI>,
}

impl<SPI> Drv8316P<SPI> {
    /// Construct a new `Drv8316P` driver wrapping an SPI interface.
    pub const fn new(spi: SPI) -> Self {
        Self {
            spi_dev: Drv8316S::new(spi),
        }
    }
}

impl<SPI> Drv8316P<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    /// Initialize the DRV8316P device.
    pub fn init(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.spi_dev.init()
    }

    /// Read diagnostic status.
    pub fn get_status(&mut self) -> Result<Drv8316Status, DrvError<SPI::Error>> {
        self.spi_dev.get_status()
    }

    /// Set PWM mode.
    pub fn set_pwm_mode(&mut self, mode: Drv8316PwmMode) -> Result<(), DrvError<SPI::Error>> {
        self.spi_dev.set_pwm_mode(mode)
    }
}

impl<SPI> Drv83xxDriver for Drv8316P<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    type Error = DrvError<SPI::Error>;

    fn clear_fault(&mut self) -> Result<(), Self::Error> {
        self.spi_dev.clear_fault()
    }

    fn is_fault(&mut self) -> Result<bool, Self::Error> {
        self.spi_dev.is_fault()
    }
}

impl<SPI> CsaGainProvider for Drv8316P<SPI> {
    fn csa_gain_v_per_a(&self) -> f32 {
        self.spi_dev.csa_gain_v_per_a()
    }
}
