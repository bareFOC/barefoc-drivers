//! # dc::current_sense — Current mirror sensing
//!
//! Provides [`CurrentMirrorSense`] for motor drivers with integrated current mirrors
//! (e.g. Texas Instruments DRV8411A, DRV8874/DRV8876, DRV8434S, and IPROPI/ISENS outputs).
//!
//! Unlike 2/3-phase inline or low-side shunts that produce bidirectional differential
//! signals centered around a reference voltage midpoint ($V_{REF}/2$), current mirrors
//! output a unipolar ground-referenced current ($I_{mirror} = I_{motor} \times A_{IPROPI}$)
//! converted to a voltage via an external sense resistor $R_{SENS}$.
//!
//! [`CurrentMirrorSense`] implements [`barefoc::traits::current_sense::CurrentSense`],
//! reconstructing bidirectional phase currents using the H-bridge excitation sign.

#![allow(non_camel_case_types)]

use core::marker::PhantomData;

use barefoc::common::time::Clock;
use barefoc::math::PhaseCurrent;
use barefoc::traits::adc::AdcPin;
use barefoc::traits::current_sense::{CurrentSense, CurrentSenseBase};

/// Current mirror sensing implementation.
///
/// Monomorphized generic over an ADC pin (`P: AdcPin`) and system clock (`C: Clock`).
pub struct CurrentMirrorSense<P, C>
where
    P: AdcPin,
    C: Clock,
{
    base: CurrentSenseBase,
    pin: P,
    gain_a_per_count: f32,
    offset_count: f32,
    sign: f32,
    _clock: PhantomData<C>,
}

impl<P, C> CurrentMirrorSense<P, C>
where
    P: AdcPin,
    C: Clock,
{
    /// Construct a new current mirror sense.
    ///
    /// # Arguments
    /// * `pin` - The ADC pin reading the mirror voltage across the sense resistor.
    /// * `a_ipropi_ua_per_a` - Mirror ratio in µA/A (e.g. 200.0 for DRV8411A).
    /// * `r_sens_ohms` - Sense resistor value in Ohms (e.g. 1500.0).
    /// * `vref` - ADC reference voltage in Volts (e.g. 3.3).
    /// * `adc_resolution_bits` - ADC resolution in bits (e.g. 12 for 12-bit ADC).
    pub fn new(
        pin: P,
        a_ipropi_ua_per_a: f32,
        r_sens_ohms: f32,
        vref: f32,
        adc_resolution_bits: u32,
    ) -> Self {
        let max_count = if adc_resolution_bits >= 32 {
            u32::MAX as f32
        } else {
            ((1u32 << adc_resolution_bits) - 1) as f32
        };
        let denom = max_count * (a_ipropi_ua_per_a * 1e-6) * r_sens_ohms;
        let gain_a_per_count = if denom > 0.0 { vref / denom } else { 0.0 };

        let mut base = CurrentSenseBase::new();
        base.gain_a = gain_a_per_count;
        base.gain_b = 0.0;
        base.gain_c = 0.0;
        base.offset_ia = 0.0;
        base.offset_ib = 0.0;
        base.offset_ic = 0.0;

        Self {
            base,
            pin,
            gain_a_per_count,
            offset_count: 0.0,
            sign: 1.0,
            _clock: PhantomData,
        }
    }

    /// Set the current direction sign.
    ///
    /// Positive values (or 0.0) set sign to `+1.0`, negative values set sign to `-1.0`.
    /// This can be linked directly to motor drive voltage setpoints or direction flags.
    pub fn set_sign(&mut self, sign: f32) {
        self.sign = if sign < 0.0 { -1.0 } else { 1.0 };
    }

    /// Current direction sign (`1.0` or `-1.0`).
    pub fn sign(&self) -> f32 {
        self.sign
    }

    /// Read raw current magnitude (Amperes, unsigned/non-negative).
    ///
    /// Clamps readings at or below baseline `offset_count` to 0.0.
    pub fn get_current_magnitude(&mut self) -> f32 {
        let raw = self.pin.read() as f32;
        let diff = raw - self.offset_count;
        if diff > 0.0 {
            diff * self.gain_a_per_count
        } else {
            0.0
        }
    }

    /// Calibrate quiescent zero-current offset over 1000 rounds with 1ms delay.
    pub fn calibrate_offsets(&mut self, clock: &C) {
        self.calibrate_offsets_rounds(clock, 1000);
    }

    /// Calibrate quiescent zero-current offset over a specified number of rounds.
    pub fn calibrate_offsets_rounds(&mut self, clock: &C, rounds: usize) {
        let count = if rounds == 0 { 1 } else { rounds };
        let mut sum = 0.0;
        for _ in 0..count {
            sum += self.pin.read() as f32;
            clock.delay_ms(1);
        }
        self.offset_count = sum / count as f32;
        self.base.offset_ia = self.offset_count;
    }

    /// Configured gain in Amperes per ADC count.
    pub fn gain_a_per_count(&self) -> f32 {
        self.gain_a_per_count
    }

    /// Set gain in Amperes per ADC count manually.
    pub fn set_gain_a_per_count(&mut self, gain: f32) {
        self.gain_a_per_count = gain;
        self.base.gain_a = gain;
    }

    /// Quiescent offset ADC count.
    pub fn offset_count(&self) -> f32 {
        self.offset_count
    }

    /// Set quiescent offset ADC count manually.
    pub fn set_offset_count(&mut self, offset: f32) {
        self.offset_count = offset;
        self.base.offset_ia = offset;
    }

    /// Immutable reference to the underlying ADC pin.
    pub fn pin(&self) -> &P {
        &self.pin
    }

    /// Mutable reference to the underlying ADC pin.
    pub fn pin_mut(&mut self) -> &mut P {
        &mut self.pin
    }
}

impl<P, C> CurrentSense for CurrentMirrorSense<P, C>
where
    P: AdcPin,
    C: Clock,
{
    type Error = core::convert::Infallible;

    fn base(&self) -> &CurrentSenseBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut CurrentSenseBase {
        &mut self.base
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        self.base.initialized = true;
        Ok(())
    }

    fn get_phase_currents(&mut self) -> PhaseCurrent {
        let mag = self.get_current_magnitude();
        PhaseCurrent {
            a: self.sign * mag,
            b: 0.0,
            c: 0.0,
        }
    }

    fn driver_align(&mut self, _voltage: f32, _modulation_centered: bool) -> i32 {
        if self.base.skip_align {
            return 1;
        }
        if !self.base.initialized {
            return 0;
        }
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use barefoc::motor::enums::{MotionControlType, TorqueControlType};
    use crate::dc::motor::DummySensor;
    use crate::dc::traits::{DCDriver, DCDriverParams};
    use crate::dc::DCMotor;

    struct MockAdcPin {
        pub count: u16,
    }

    impl AdcPin for MockAdcPin {
        fn read(&mut self) -> u16 {
            self.count
        }
    }

    #[derive(Clone, Copy, Default)]
    struct TestClock;

    impl Clock for TestClock {
        fn now_us(&self) -> u32 {
            0
        }
    }

    struct MockDCDriver {
        params: DCDriverParams,
        pub last_voltage: f32,
    }

    impl MockDCDriver {
        fn new() -> Self {
            let mut params = DCDriverParams::new();
            params.initialized = true;
            params.voltage_limit = 12.0;
            params.voltage_power_supply = 12.0;
            Self {
                params,
                last_voltage: 0.0,
            }
        }
    }

    impl DCDriver for MockDCDriver {
        fn init(&mut self) -> Result<(), ()> {
            self.params.initialized = true;
            Ok(())
        }

        fn enable(&mut self) {}

        fn disable(&mut self) {
            self.last_voltage = 0.0;
        }

        fn set_pwm(&mut self, voltage: f32) {
            self.last_voltage = voltage;
        }

        fn params(&self) -> &DCDriverParams {
            &self.params
        }

        fn params_mut(&mut self) -> &mut DCDriverParams {
            &mut self.params
        }
    }

    #[test]
    fn test_gain_calculation_and_current_reading() {
        // DRV8411A example: 200 µA/A mirror, 1500 Ω resistor, 3.3V reference, 12-bit ADC.
        // At 1.0 A motor current:
        // I_mirror = 1.0 A * 200e-6 = 0.2 mA
        // V_sens = 0.2 mA * 1500 Ω = 0.3 V
        // At 12-bit (max count 4095), 0.3 V corresponds to:
        // count = 0.3 * 4095 / 3.3 = 372.2727... counts
        let pin = MockAdcPin { count: 372 };
        let mut cs: CurrentMirrorSense<MockAdcPin, TestClock> =
            CurrentMirrorSense::new(pin, 200.0, 1500.0, 3.3, 12);

        // Verification of gain
        let expected_gain = 3.3 / (4095.0 * 200e-6 * 1500.0);
        assert!((cs.gain_a_per_count() - expected_gain).abs() < 1e-7);

        // An ADC reading of 372.27 counts produces 1.0 A
        let ideal_count = 0.3 * 4095.0 / 3.3;
        let ideal_current = ideal_count * cs.gain_a_per_count();
        assert!((ideal_current - 1.0).abs() < 1e-5);

        // Integer count 372 is ~0.999268 A
        let mag = cs.get_current_magnitude();
        assert!((mag - 0.999268).abs() < 1e-4);
    }

    #[test]
    fn test_offset_calibration_and_clamping() {
        let pin = MockAdcPin { count: 42 };
        let mut cs: CurrentMirrorSense<MockAdcPin, TestClock> =
            CurrentMirrorSense::new(pin, 200.0, 1500.0, 3.3, 12);

        let clock = TestClock;
        cs.calibrate_offsets(&clock);
        assert_eq!(cs.offset_count(), 42.0);
        assert_eq!(cs.base().offset_ia, 42.0);

        // Readings <= offset_count are clamped to 0.0
        assert_eq!(cs.get_current_magnitude(), 0.0);

        cs.pin_mut().count = 30; // Below baseline
        assert_eq!(cs.get_current_magnitude(), 0.0);

        cs.pin_mut().count = 42; // Exactly at baseline
        assert_eq!(cs.get_current_magnitude(), 0.0);

        cs.pin_mut().count = 414; // 372 counts above baseline
        let mag = cs.get_current_magnitude();
        assert!((mag - 0.999268).abs() < 1e-4);
    }

    #[test]
    fn test_sign_reversal() {
        let pin = MockAdcPin { count: 372 };
        let mut cs: CurrentMirrorSense<MockAdcPin, TestClock> =
            CurrentMirrorSense::new(pin, 200.0, 1500.0, 3.3, 12);

        assert_eq!(cs.sign(), 1.0);
        let phase = cs.get_phase_currents();
        assert!(phase.a > 0.0);
        assert_eq!(phase.b, 0.0);
        assert_eq!(phase.c, 0.0);

        // Negative sign
        cs.set_sign(-1.0);
        assert_eq!(cs.sign(), -1.0);
        let phase_neg = cs.get_phase_currents();
        assert_eq!(phase_neg.a, -phase.a);
        assert_eq!(phase_neg.b, 0.0);
        assert_eq!(phase_neg.c, 0.0);

        // Setting from negative voltage setpoint
        cs.set_sign(-4.5);
        assert_eq!(cs.sign(), -1.0);

        // Setting from positive voltage setpoint
        cs.set_sign(5.2);
        assert_eq!(cs.sign(), 1.0);
    }

    #[test]
    fn test_driver_align_and_initialization() {
        let pin = MockAdcPin { count: 100 };
        let mut cs: CurrentMirrorSense<MockAdcPin, TestClock> =
            CurrentMirrorSense::new(pin, 200.0, 1500.0, 3.3, 12);

        assert!(!cs.base().initialized);
        // Align fails if not initialized
        assert_eq!(cs.driver_align(1.0, false), 0);

        assert!(cs.init().is_ok());
        assert!(cs.base().initialized);
        assert_eq!(cs.driver_align(1.0, false), 1);
    }

    #[test]
    fn test_integration_with_dc_motor() {
        let clock = TestClock;
        let mut motor: DCMotor<'_, DummySensor<TestClock>, MockDCDriver, CurrentMirrorSense<MockAdcPin, TestClock>, TestClock> =
            DCMotor::new(&clock);
        motor.link_driver(MockDCDriver::new());

        let pin = MockAdcPin { count: 372 };
        let cs = CurrentMirrorSense::new(pin, 200.0, 1500.0, 3.3, 12);
        motor.link_current_sense(cs);

        assert_eq!(motor.init(), 1);
        assert!(motor.current_sense().unwrap().base().initialized);

        motor.controller = MotionControlType::torque;
        motor.torque_controller = TorqueControlType::dc_current;

        // Configure current PID
        motor.LPF_current_q.set_tf(0.0);
        motor.PID_current_q.set_output_ramp(0.0);
        motor.PID_current_q.set_p(2.0);
        motor.PID_current_q.set_i(0.0);

        // Forward target: 2.0 A setpoint, measured ~1.0 A
        // Error = 2.0 - ~0.9993 = ~1.0007 A -> Voltage = 2.0 * error ≈ 2.0014 V
        motor.move_motor(2.0);
        let v = motor.driver().unwrap().last_voltage;
        assert!((v - 2.0014).abs() < 1e-2);

        // Reverse target: set sensor sign to reverse (-1.0)
        motor.current_sense_mut().unwrap().set_sign(-1.0);
        // Target -2.0 A setpoint, measured -0.9993 A
        // Error = -2.0 - (-0.9993) = -1.0007 A -> Voltage = 2.0 * error ≈ -2.0014 V
        motor.move_motor(-2.0);
        let v_rev = motor.driver().unwrap().last_voltage;
        assert!((v_rev - (-2.0014)).abs() < 1e-2);
    }
}
