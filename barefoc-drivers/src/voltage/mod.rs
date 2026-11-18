//! # voltage — Bus and Phase Voltage Sensing
//!
//! Ports [`VoltageSense`](source/Arduino-FOC-drivers/src/voltage/VoltageSense.h)
//! from Arduino-FOC-drivers.
//!
//! Provides voltage sensing with low-pass filtering, gain calibration, and zero offset
//! compensation for DC bus monitoring, phase voltage measurement, and undervoltage protection.

pub mod generic;

pub use generic::{FnAdcPin, GenericVoltageSense};

use barefoc::common::filter::LowPassFilter;
use barefoc::common::time::Clock;
use barefoc::math::NOT_SET;

/// Trait implemented by voltage sensing drivers.
pub trait VoltageSensor {
    /// Return the most recent filtered voltage measurement (Volts).
    fn get_voltage(&self) -> f32;
}

/// Voltage sense base struct maintaining a low-pass filter, gain, and offset.
///
/// Ports the upstream `VoltageSense` C++ class.
#[derive(Clone)]
pub struct VoltageSense<C: Clock> {
    /// Low-pass filter for smoothing raw voltage readings.
    pub filter: LowPassFilter<C>,
    /// Calibration gain factor.
    pub gain: f32,
    /// Voltage offset (Volts).
    pub offset: f32,
    /// Most recent filtered voltage reading (Volts).
    pub voltage: f32,
}

impl<C: Clock> core::fmt::Debug for VoltageSense<C> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("VoltageSense")
            .field("gain", &self.gain)
            .field("offset", &self.offset)
            .field("voltage", &self.voltage)
            .finish()
    }
}

impl<C: Clock> VoltageSense<C> {
    /// Construct a new `VoltageSense` with specified gain, offset, and filter time constant.
    ///
    /// # Arguments
    /// * `gain` - Voltage scaling gain (default 1.0).
    /// * `offset` - Voltage offset in Volts (default 0.0).
    /// * `tf` - Low-pass filter time constant in seconds (default 0.1).
    /// * `clock` - Clock reference to initialize the filter timing.
    pub fn new(gain: f32, offset: f32, tf: f32, clock: &C) -> Self {
        Self {
            filter: LowPassFilter::new(tf, NOT_SET, clock),
            gain,
            offset,
            voltage: 0.0,
        }
    }

    /// Construct a `VoltageSense` with default parameters (gain 1.0, offset 0.0, tf 0.1).
    pub fn default_with_clock(clock: &C) -> Self {
        Self::new(1.0, 0.0, 0.1, clock)
    }

    /// Return the current filtered voltage in Volts.
    #[inline]
    pub fn get_voltage(&self) -> f32 {
        self.voltage
    }

    /// Update the voltage calculation with a raw voltage input and filter it.
    ///
    /// Applies $(v_{\text{raw}} - \text{offset}) \cdot \text{gain}$ and passes
    /// the result through the low-pass filter.
    pub fn update(&mut self, raw_voltage: f32, clock: &C) -> f32 {
        let v = (raw_voltage - self.offset) * self.gain;
        self.voltage = self.filter.update(v, clock);
        self.voltage
    }
}

impl<C: Clock> VoltageSensor for VoltageSense<C> {
    #[inline]
    fn get_voltage(&self) -> f32 {
        self.voltage
    }
}
