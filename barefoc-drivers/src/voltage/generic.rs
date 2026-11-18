//! # voltage::generic — Generic ADC Voltage Sensor
//!
//! Ports [`GenericVoltageSense`](source/Arduino-FOC-drivers/src/voltage/GenericVoltageSense.h)
//! and [`GenericVoltageSense.cpp`](source/Arduino-FOC-drivers/src/voltage/GenericVoltageSense.cpp)
//! from Arduino-FOC-drivers.
//!
//! Supports reading voltage via hardware ADC pins implementing [`AdcPin`] or via custom closures.

use barefoc::common::time::Clock;
use barefoc::traits::adc::AdcPin;
use super::{VoltageSense, VoltageSensor};

/// Wrapper struct adapting a closure `FnMut() -> u16` into an [`AdcPin`].
pub struct FnAdcPin<F>(pub F);

impl<F: FnMut() -> u16> AdcPin for FnAdcPin<F> {
    #[inline]
    fn read(&mut self) -> u16 {
        (self.0)()
    }
}

/// Generic voltage sensor reading raw ADC counts.
///
/// Ports the upstream `GenericVoltageSense` class. Converts raw ADC counts
/// to voltage via:
///
/// $$V_{\text{raw}} = \frac{\text{ADC count}}{\text{max\_value}} \cdot V_{\text{FS}}$$
///
/// and then feeds the result into the inner [`VoltageSense`] instance.
#[derive(Clone)]
pub struct GenericVoltageSense<C: Clock, P> {
    /// Inner voltage sensor instance handling calibration and low-pass filtering.
    pub base: VoltageSense<C>,
    /// ADC pin or closure adapter used to read counts.
    pub pin: P,
    /// Full-scale analog reference voltage (Volts). Default: 3.3V.
    pub full_scale_voltage: f32,
    /// Maximum count corresponding to full-scale voltage ($2^N$). Default: 1024 (10-bit).
    pub max_value: u32,
}

impl<C: Clock, P: core::fmt::Debug> core::fmt::Debug for GenericVoltageSense<C, P> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("GenericVoltageSense")
            .field("base", &self.base)
            .field("pin", &self.pin)
            .field("full_scale_voltage", &self.full_scale_voltage)
            .field("max_value", &self.max_value)
            .finish()
    }
}

impl<C: Clock, P: AdcPin> GenericVoltageSense<C, P> {
    /// Construct a new `GenericVoltageSense` instance.
    ///
    /// # Arguments
    /// * `pin` - ADC pin implementation.
    /// * `gain` - Scaling gain factor.
    /// * `offset` - Zero voltage offset.
    /// * `tf` - Low-pass filter time constant (seconds).
    /// * `full_scale_voltage` - Full-scale analog reference voltage in Volts (e.g. 3.3 or 5.0).
    /// * `clock` - Clock reference to initialize filter timing.
    pub fn new(
        pin: P,
        gain: f32,
        offset: f32,
        tf: f32,
        full_scale_voltage: f32,
        clock: &C,
    ) -> Self {
        Self {
            base: VoltageSense::new(gain, offset, tf, clock),
            pin,
            full_scale_voltage,
            max_value: 1024,
        }
    }

    /// Construct with default gain (1.0), offset (0.0), filter (0.1s), and 3.3V full-scale.
    pub fn default_with_pin(pin: P, clock: &C) -> Self {
        Self::new(pin, 1.0, 0.0, 0.1, 3.3, clock)
    }

    /// Configure the ADC resolution in bits (e.g., 10 for 10-bit = 1024, 12 for 12-bit = 4096).
    pub fn init(&mut self, resolution_bits: u32) {
        if resolution_bits == 0 {
            self.max_value = 1;
        } else {
            self.max_value = 1u32.checked_shl(resolution_bits).unwrap_or(u32::MAX);
        }
    }

    /// Read raw ADC count and convert it to full-scale input voltage.
    pub fn read_raw_voltage(&mut self) -> f32 {
        let val = self.pin.read() as f32;
        (val / self.max_value as f32) * self.full_scale_voltage
    }

    /// Read the ADC, apply gain & offset, update the low-pass filter, and return the filtered voltage.
    pub fn update(&mut self, clock: &C) -> f32 {
        let raw = self.read_raw_voltage();
        self.base.update(raw, clock)
    }

    /// Return the filtered voltage in Volts.
    #[inline]
    pub fn get_voltage(&self) -> f32 {
        self.base.get_voltage()
    }
}

impl<C: Clock, F: FnMut() -> u16> GenericVoltageSense<C, FnAdcPin<F>> {
    /// Construct a `GenericVoltageSense` backed by a closure returning raw ADC counts.
    pub fn from_fn(
        read_fn: F,
        gain: f32,
        offset: f32,
        tf: f32,
        full_scale_voltage: f32,
        clock: &C,
    ) -> Self {
        Self::new(FnAdcPin(read_fn), gain, offset, tf, full_scale_voltage, clock)
    }
}

impl<C: Clock, P: AdcPin> VoltageSensor for GenericVoltageSense<C, P> {
    #[inline]
    fn get_voltage(&self) -> f32 {
        self.base.get_voltage()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestClock {
        time_us: core::cell::Cell<u32>,
    }

    impl TestClock {
        fn new() -> Self {
            Self {
                time_us: core::cell::Cell::new(0),
            }
        }

        fn advance_us(&self, us: u32) {
            self.time_us.set(self.time_us.get() + us);
        }
    }

    impl Clock for TestClock {
        fn now_us(&self) -> u32 {
            self.time_us.get()
        }
    }

    #[derive(Debug)]
    struct TestAdcPin {
        value: u16,
    }

    impl AdcPin for TestAdcPin {
        fn read(&mut self) -> u16 {
            self.value
        }
    }

    #[test]
    fn test_voltage_sense_basic() {
        let clock = TestClock::new();
        let mut vs = VoltageSense::new(2.0, 0.5, 0.01, &clock);

        // Advance clock by > 0.3s to pass through initial value
        clock.advance_us(500_000);
        // Raw voltage = 1.5V. (1.5 - 0.5) * 2.0 = 2.0V
        let v = vs.update(1.5, &clock);
        assert!((v - 2.0).abs() < 1e-4);
        assert!((vs.get_voltage() - 2.0).abs() < 1e-4);
    }

    #[test]
    fn test_generic_voltage_sense_scaling() {
        let clock = TestClock::new();
        let pin = TestAdcPin { value: 512 }; // Half scale of 1024
        let mut gvs = GenericVoltageSense::new(pin, 1.0, 0.0, 0.01, 3.3, &clock);

        let raw = gvs.read_raw_voltage();
        // 512 / 1024 * 3.3 = 1.65V
        assert!((raw - 1.65).abs() < 1e-4);

        clock.advance_us(500_000);
        let v = gvs.update(&clock);
        assert!((v - 1.65).abs() < 1e-4);
        assert!((gvs.get_voltage() - 1.65).abs() < 1e-4);
    }

    #[test]
    fn test_generic_voltage_sense_resolution_init() {
        let clock = TestClock::new();
        let pin = TestAdcPin { value: 2048 };
        let mut gvs = GenericVoltageSense::new(pin, 1.0, 0.0, 0.01, 3.3, &clock);
        gvs.init(12); // 12-bit ADC -> 4096
        assert_eq!(gvs.max_value, 4096);

        let raw = gvs.read_raw_voltage();
        // 2048 / 4096 * 3.3 = 1.65V
        assert!((raw - 1.65).abs() < 1e-4);
    }

    #[test]
    fn test_generic_voltage_sense_from_fn() {
        let clock = TestClock::new();
        let mut count = 1000u16;
        let mut gvs = GenericVoltageSense::from_fn(
            move || {
                count += 24;
                count
            },
            1.0,
            0.0,
            0.01,
            3.3,
            &clock,
        );
        gvs.init(10); // 1024
        let raw = gvs.read_raw_voltage();
        // count = 1024 -> 1024 / 1024 * 3.3 = 3.3V
        assert!((raw - 3.3).abs() < 1e-4);
    }

    #[test]
    fn test_generic_voltage_sense_gain_and_offset() {
        let clock = TestClock::new();
        // 12-bit ADC reading 4096 (3.3V raw)
        let pin = TestAdcPin { value: 4096 };
        // Voltage divider: gain = 10.0 (33V bus), offset = 0.3V
        let mut gvs = GenericVoltageSense::new(pin, 10.0, 0.3, 0.01, 3.3, &clock);
        gvs.init(12);

        clock.advance_us(500_000);
        // raw = 3.3V. (3.3 - 0.3) * 10.0 = 30.0V
        let v = gvs.update(&clock);
        assert!((v - 30.0).abs() < 1e-3);
    }

    #[test]
    fn test_generic_voltage_sense_filtering() {
        let clock = TestClock::new();
        let pin = TestAdcPin { value: 1024 };
        let mut gvs = GenericVoltageSense::new(pin, 1.0, 0.0, 0.01, 3.3, &clock);
        gvs.init(10);

        // Initial update with dt = 0 -> output matches
        clock.advance_us(1000);
        let v1 = gvs.update(&clock);
        assert!(v1 > 0.0 && v1 <= 3.3);
    }
}
