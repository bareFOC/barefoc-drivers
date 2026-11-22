//! # temperature — NTC thermistor temperature sensing
//!
//! Provides [`NtcThermistor`], a zero-heap ADC divider front end that fulfills
//! the BareFOC [`TemperatureSensor`] contract for inverter PCB copper-pour
//! thermistors and motor-winding embedded NTCs.
//!
//! Contract sources:
//! - `bareFOC/src/traits/temperature.rs` — `TemperatureSensor::read_temperature`
//!   returning degrees Celsius as `Result<f32, E>`.
//! - `bareFOC/src/traits/adc.rs` — `AdcPin` raw-sample source.
//! - `bareFOC/src/control_table.rs` — `REG_PRESENT_TEMP_MOSFET` /
//!   `REG_PRESENT_TEMP_MOTOR` telemetry consumers.
//!
//! Divider topology (NTC to ground):
//!
//! ```text
//! VREF ── R_series ──●── NTC ── GND
//!                    └── ADC input
//! ```
//!
//! With an ideal ADC of `max_count` counts at `VREF`, the thermistor resistance
//! is $R_{NTC} = R_{series}\,\frac{n}{n_{max}-n}$ where $n$ is the raw sample.
//! An open circuit pulls the node to `VREF` ($n \ge n_{max}$) and reports
//! `f32::NAN`; a short to ground ($n = 0$) reports negative saturation.

use barefoc::traits::adc::AdcPin;
use barefoc::traits::temperature::TemperatureSensor;

/// Nominal thermistor temperature $T_{25}$ in Kelvin (25 °C).
const T25_KELVIN: f32 = 298.15;
/// Kelvin-to-Celsius offset.
const KELVIN_TO_CELSIUS: f32 = 273.15;

/// Value reported for an open-circuit thermistor input.
pub const OPEN_CIRCUIT_TEMPERATURE_C: f32 = f32::NAN;
/// Value reported for a shorted thermistor input (absolute-zero saturation).
pub const SHORT_CIRCUIT_TEMPERATURE_C: f32 = -273.15;

/// Resistance-to-temperature conversion model of an NTC thermistor.
///
/// Both models use `libm` for transcendental math so the driver stays
/// `#![no_std]`. See `bareFOC/src/traits/temperature.rs` for the consumer
/// contract.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThermistorModel {
    /// Beta-parameter $B_{25/85}$ approximation, valid near the nominal point:
    /// $\frac{1}{T} = \frac{1}{T_{25}} + \frac{1}{B}\ln\!\left(\frac{R}{R_{25}}\right)$.
    Beta {
        /// $B_{25/85}$ coefficient in Kelvin (typically 3300–4300 K).
        beta: f32,
    },
    /// Steinhart–Hart three-coefficient model:
    /// $\frac{1}{T} = A + B\ln R + C(\ln R)^3$.
    SteinhartHart {
        /// First coefficient $A$ (K⁻¹).
        a: f32,
        /// Second coefficient $B$ (K⁻¹).
        b: f32,
        /// Third coefficient $C$ (K⁻¹).
        c: f32,
    },
}

/// ADC-based NTC thermistor sensor.
///
/// Generic over [`AdcPin`] so the same driver covers on-board inverter
/// thermistors and winding sensors on any MCU family. No heap, no `std`.
pub struct NtcThermistor<P: AdcPin> {
    pin: P,
    model: ThermistorModel,
    /// Nominal resistance at 25 °C in Ohms ($R_{25}$).
    r25: f32,
    /// Fixed divider leg from VREF to the ADC node in Ohms.
    series_r: f32,
    /// ADC full-scale count (e.g. 4095 for a 12-bit converter).
    max_count: u16,
}

impl<P: AdcPin> NtcThermistor<P> {
    /// Construct a thermistor with an explicit conversion model.
    ///
    /// `r25` and `series_r` are in Ohms; `max_count` is the ADC full-scale
    /// count (4095 for a 12-bit converter).
    pub const fn new(
        pin: P,
        model: ThermistorModel,
        r25: f32,
        series_r: f32,
        max_count: u16,
    ) -> Self {
        Self {
            pin,
            model,
            r25,
            series_r,
            max_count,
        }
    }

    /// Construct a thermistor using the Beta-parameter model.
    pub const fn with_beta(
        pin: P,
        beta: f32,
        r25: f32,
        series_r: f32,
        max_count: u16,
    ) -> Self {
        Self::new(pin, ThermistorModel::Beta { beta }, r25, series_r, max_count)
    }

    /// Construct a thermistor using the Steinhart–Hart model.
    pub const fn with_steinhart_hart(
        pin: P,
        a: f32,
        b: f32,
        c: f32,
        r25: f32,
        series_r: f32,
        max_count: u16,
    ) -> Self {
        Self::new(
            pin,
            ThermistorModel::SteinhartHart { a, b, c },
            r25,
            series_r,
            max_count,
        )
    }

    /// Borrow the underlying ADC pin (e.g. to change a mux channel).
    pub fn pin_mut(&mut self) -> &mut P {
        &mut self.pin
    }

    /// Active conversion model.
    pub const fn model(&self) -> ThermistorModel {
        self.model
    }

    /// Convert a raw ADC sample to thermistor resistance in Ohms.
    ///
    /// Returns `f32::NAN` for an open circuit or an invalid divider config,
    /// and `0.0` for a shorted input.
    pub fn resistance_from_raw(&self, raw: u16) -> f32 {
        if self.max_count == 0 || self.r25 <= 0.0 || self.series_r <= 0.0 {
            return f32::NAN;
        }
        let n = raw as f32;
        let n_max = self.max_count as f32;
        if n >= n_max {
            // Open circuit: node pulled to VREF through R_series.
            return f32::NAN;
        }
        if n <= 0.0 {
            // Short to ground: R_NTC approaches zero.
            return 0.0;
        }
        self.series_r * n / (n_max - n)
    }

    /// Convert a thermistor resistance in Ohms to degrees Celsius.
    pub fn temperature_from_resistance(&self, resistance: f32) -> f32 {
        if resistance <= 0.0 || resistance.is_nan() {
            return if resistance == 0.0 {
                SHORT_CIRCUIT_TEMPERATURE_C
            } else {
                OPEN_CIRCUIT_TEMPERATURE_C
            };
        }

        match self.model {
            ThermistorModel::Beta { beta } => {
                if beta <= 0.0 || beta.is_nan() {
                    return f32::NAN;
                }
                let ln_ratio = libm::logf(resistance / self.r25);
                let inverse_kelvin = 1.0 / T25_KELVIN + ln_ratio / beta;
                1.0 / inverse_kelvin - KELVIN_TO_CELSIUS
            }
            ThermistorModel::SteinhartHart { a, b, c } => {
                let ln_r = libm::logf(resistance);
                let inverse_kelvin = a + b * ln_r + c * libm::powf(ln_r, 3.0);
                1.0 / inverse_kelvin - KELVIN_TO_CELSIUS
            }
        }
    }
}

impl<P: AdcPin> TemperatureSensor for NtcThermistor<P> {
    type Error = ();

    /// Sample the ADC divider and convert to degrees Celsius.
    ///
    /// Open circuits report `f32::NAN`, shorts report
    /// [`SHORT_CIRCUIT_TEMPERATURE_C`], and invalid configurations never panic.
    fn read_temperature(&mut self) -> Result<f32, Self::Error> {
        let raw = self.pin.read();
        let resistance = self.resistance_from_raw(raw);
        Ok(self.temperature_from_resistance(resistance))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FixedPin(u16);

    impl AdcPin for FixedPin {
        fn read(&mut self) -> u16 {
            self.0
        }
    }

    #[test]
    fn test_divider_resistance() {
        let ntc = NtcThermistor::with_beta(FixedPin(2048), 3950.0, 10_000.0, 10_000.0, 4096);
        assert_eq!(ntc.resistance_from_raw(2048), 10_000.0);
        assert_eq!(ntc.resistance_from_raw(0), 0.0);
        assert!(ntc.resistance_from_raw(4096).is_nan());
        assert_eq!(ntc.resistance_from_raw(1024), 10_000.0 / 3.0);
    }

    #[test]
    fn test_beta_conversion_at_r25_is_25_celsius() {
        // 12-bit midpoint with equal R25 and series resistor gives exactly R25.
        let mut ntc = NtcThermistor::with_beta(FixedPin(2048), 3950.0, 10_000.0, 10_000.0, 4096);
        let temp = ntc.read_temperature().unwrap();
        assert!((temp - 25.0).abs() < 0.01, "got {} C", temp);
    }

    #[test]
    fn test_steinhart_hart_conversion_at_r25_is_25_celsius() {
        // Choose A so that ln(R25) maps exactly to 1/298.15 K.
        let ln_r25 = libm::logf(10_000.0);
        let b = 0.00025;
        let c = 5.0e-8;
        let a = 1.0 / T25_KELVIN - b * ln_r25 - c * libm::powf(ln_r25, 3.0);
        let mut ntc = NtcThermistor::with_steinhart_hart(
            FixedPin(2048),
            a,
            b,
            c,
            10_000.0,
            10_000.0,
            4096,
        );
        let temp = ntc.read_temperature().unwrap();
        assert!((temp - 25.0).abs() < 0.01, "got {} C", temp);
    }

    #[test]
    fn test_open_short_and_invalid_config_do_not_panic() {
        let mut open = NtcThermistor::with_beta(FixedPin(4096), 3950.0, 10_000.0, 10_000.0, 4096);
        assert!(open.read_temperature().unwrap().is_nan());

        let mut short = NtcThermistor::with_beta(FixedPin(0), 3950.0, 10_000.0, 10_000.0, 4096);
        assert_eq!(short.read_temperature().unwrap(), SHORT_CIRCUIT_TEMPERATURE_C);

        let mut invalid = NtcThermistor::new(
            FixedPin(100),
            ThermistorModel::Beta { beta: 0.0 },
            0.0,
            0.0,
            0,
        );
        assert!(invalid.read_temperature().unwrap().is_nan());

        // A valid configuration with an out-of-range sample must stay finite
        // or NAN, never panic.
        let mut saturated = NtcThermistor::with_beta(FixedPin(4095), 3950.0, 10_000.0, 10_000.0, 4096);
        let t = saturated.read_temperature().unwrap();
        assert!(t.is_finite() || t.is_nan());
    }
}
