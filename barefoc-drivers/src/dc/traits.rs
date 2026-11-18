//! # dc::traits — abstract DC motor driver interface
//!
//! Ports [`DCDriver.h`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver.h)
//! and [`DCDriver.cpp`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver.cpp)
//! from Arduino-FOC-drivers (origin/dc-motor branch).

#![allow(non_camel_case_types)]

use embedded_hal::digital::OutputPin;

// ---------------------------------------------------------------------------
// DCDriverParams — configuration & state block
// ---------------------------------------------------------------------------

/// Per-driver configuration and runtime state.
///
/// Mirrors the public fields of upstream `DCDriver`
/// ([`DCDriver.h`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver.h) lines 18–25).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DCDriverParams {
    /// PWM switching frequency in Hz.
    pub pwm_frequency: u32,
    /// Power supply voltage in volts (e.g. 12.0 for a 12 V bus).
    pub voltage_power_supply: f32,
    /// Maximum voltage limit applied to the motor phases.
    pub voltage_limit: f32,
    /// Flag indicating whether the driver hardware has been initialized.
    pub initialized: bool,
    /// Opaque hardware-specific parameter handle.
    pub params: usize,
    /// Polarity of the hardware enable pin (`true` for active-high, `false` for active-low).
    pub enable_active_high: bool,
}

impl DCDriverParams {
    /// Create driver parameters seeded with standard defaults:
    /// 25 kHz PWM, 12 V power supply, 12 V limit, active-high enable.
    pub const fn new() -> Self {
        Self {
            pwm_frequency: 25_000,
            voltage_power_supply: 12.0,
            voltage_limit: 12.0,
            initialized: false,
            params: 0,
            enable_active_high: true,
        }
    }
}

impl Default for DCDriverParams {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// DCDriver trait
// ---------------------------------------------------------------------------

/// Abstract single-phase / H-bridge DC motor driver interface.
///
/// Ports the upstream `DCDriver` class
/// ([`DCDriver.h`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver.h)).
pub trait DCDriver {
    /// Initialise the driver hardware (PWM timers, GPIOs).
    ///
    /// Must set [`params.initialized`](DCDriverParams::initialized) to `true` on success.
    fn init(&mut self) -> Result<(), ()>;

    /// Enable the driver hardware outputs.
    fn enable(&mut self);

    /// Disable the driver hardware outputs.
    fn disable(&mut self);

    /// Set phase voltage to the hardware.
    ///
    /// Positive voltages drive in the forward direction;
    /// negative voltages drive in the reverse direction.
    fn set_pwm(&mut self, voltage: f32);

    /// Borrow the driver configuration and status parameters.
    fn params(&self) -> &DCDriverParams;

    /// Borrow the driver configuration and status parameters mutably.
    fn params_mut(&mut self) -> &mut DCDriverParams;
}

// ---------------------------------------------------------------------------
// NoPin — dummy no-op output pin
// ---------------------------------------------------------------------------

/// Dummy / no-op output pin for drivers configured without an external enable pin.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NoPin;

impl embedded_hal::digital::ErrorType for NoPin {
    type Error = core::convert::Infallible;
}

impl OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
