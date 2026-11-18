//! # drv83xx-rs — Pure-Rust, `no_std` Texas Instruments DRV83xx 3-phase gate driver library
//!
//! Provides support for the TI DRV83xx family (DRV8316, DRV8311, etc.) with:
//! - 16-bit and 24-bit TI SPI framing engines with hardware parity and tSPI addressing.
//! - Multi-variant support:
//!   - DRV8316: `Drv8316S` (SPI), `Drv8316H` (HW), `Drv8316P` (PWM).
//!   - DRV8311: `Drv8311S` (24-bit SPI), `Drv8311H` (HW), `Drv8311P` (tSPI / PWMG).
//! - Diagnostic status decoders for over-current (per FET), thermal, undervoltage, and SPI faults.
//! - Integrated BareFOC bundles (`Drv8316Driver6PWM`, `Drv8311DriverPwmGen`, etc.) implementing `BLDCDriver`
//!   and 1-line current sensing (`create_current_sense`).

#![no_std]

#[cfg(feature = "std")]
extern crate std;

pub mod spi;
pub mod drv8316;
pub mod drv8311;

pub use drv8316::{
    Drv8316CsaGain, Drv8316H, Drv8316P, Drv8316PwmMode, Drv8316S, Drv8316SlewRate, Drv8316Status,
};
#[cfg(feature = "barefoc")]
pub use drv8316::{Drv8316Driver3PWM, Drv8316Driver6PWM};

pub use drv8311::{
    Drv8311CounterMode, Drv8311CsaGain, Drv8311CurrentSenseFilter, Drv8311DeadTime, Drv8311H,
    Drv8311OcpMode, Drv8311OtsdMode, Drv8311P, Drv8311PhaseState, Drv8311PwmMode,
    Drv8311PwmSync, Drv8311S, Drv8311SlewRate, Drv8311Status, Drv8311UvpMode,
};
#[cfg(feature = "barefoc")]
pub use drv8311::{
    Drv8311Driver3PWM, Drv8311Driver6PWM, Drv8311DriverPwmGen, Drv8311HDriver3PWM,
};

/// Common driver errors for DRV83xx devices.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DrvError<E> {
    /// Underlying bus communication error (e.g. SPI or GPIO)
    Bus(E),
    /// SPI protocol / framing / parity error detected by device
    SpiFault,
    /// Device hardware fault asserted (e.g. overcurrent, overtemperature)
    HardwareFault,
    /// Invalid register configuration or initialization failure
    InvalidConfig,
    /// Registers are currently locked against writes
    RegistersLocked,
    /// Driver output stage disabled
    DriverDisabled,
}

impl<E> From<E> for DrvError<E> {
    fn from(e: E) -> Self {
        DrvError::Bus(e)
    }
}

/// Common trait implemented by all DRV83xx driver variants.
pub trait Drv83xxDriver {
    type Error;

    /// Clears any latched device faults.
    fn clear_fault(&mut self) -> Result<(), Self::Error>;

    /// Checks if a hardware fault condition is currently active.
    fn is_fault(&mut self) -> Result<bool, Self::Error>;
}

/// Trait providing the configured Current Sense Amplifier (CSA) gain.
pub trait CsaGainProvider {
    /// Returns the configured Current Sense Amplifier gain in V/A (e.g. 0.15, 0.1875, 0.25, 0.375).
    fn csa_gain_v_per_a(&self) -> f32;

    /// Returns the volts-to-amps conversion ratio in A/V ($1 / G_{CSA}$).
    fn csa_gain_a_per_v(&self) -> f32 {
        1.0 / self.csa_gain_v_per_a()
    }
}

/// Dummy pin implementing embedded-hal digital traits when no physical pin is connected.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NoPin;

impl embedded_hal::digital::ErrorType for NoPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::InputPin for NoPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }
}

impl embedded_hal::digital::OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
