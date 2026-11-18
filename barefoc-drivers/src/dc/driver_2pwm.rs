//! # dc::driver_2pwm — 2-PWM H-bridge DC motor driver
//!
//! Ports [`DCDriver2PWM.h`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver2PWM.h)
//! and [`DCDriver2PWM.cpp`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver2PWM.cpp)
//! from Arduino-FOC-drivers (origin/dc-motor branch).
//!
//! Intended to drive DC motor drivers with two PWM control signals, i.e. independent
//! control of the half-bridges (such as L298N, DRV8833, TB6612FNG).
//!
//! Speed is determined by duty cycle. By convention the on-state half-bridge is driven
//! with PWM, and the off-state half-bridge is held low.
//! To protect against shoot-through and accidental cross-conduction, the inactive
//! half-bridge is always zeroed before updating the active half-bridge.

#![allow(non_camel_case_types)]

use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

use crate::dc::traits::{DCDriver, DCDriverParams, NoPin};

/// 2-PWM DC motor driver for single-phase H-bridge topologies.
///
/// Ports upstream `DCDriver2PWM`
/// ([`DCDriver2PWM.h`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver2PWM.h)).
///
/// The struct is generic over:
/// - `P1`, `P2`: PWM channel types implementing [`SetDutyCycle`].
/// - `EN`: Enable pin type implementing [`OutputPin`]. Defaults to [`NoPin`].
pub struct DCDriver2PWM<P1, P2, EN = NoPin>
where
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    EN: OutputPin,
{
    params: DCDriverParams,
    pin1: P1,
    pin2: P2,
    enable: Option<EN>,
}

impl<P1, P2, EN> DCDriver2PWM<P1, P2, EN>
where
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    EN: OutputPin,
{
    /// Construct a new 2-PWM driver with an optional enable pin.
    ///
    /// Mirrors upstream `DCDriver2PWM(int pinA, int pinB, int pinEN)`
    /// ([`DCDriver2PWM.cpp`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver2PWM.cpp) lines 7–11).
    pub fn new(pin1: P1, pin2: P2, enable: Option<EN>) -> Self {
        Self {
            params: DCDriverParams::new(),
            pin1,
            pin2,
            enable,
        }
    }

    /// Borrow the PWM channel for pin 1.
    pub fn pin1(&self) -> &P1 {
        &self.pin1
    }

    /// Borrow the PWM channel for pin 1 mutably.
    pub fn pin1_mut(&mut self) -> &mut P1 {
        &mut self.pin1
    }

    /// Borrow the PWM channel for pin 2.
    pub fn pin2(&self) -> &P2 {
        &self.pin2
    }

    /// Borrow the PWM channel for pin 2 mutably.
    pub fn pin2_mut(&mut self) -> &mut P2 {
        &mut self.pin2
    }

    /// Borrow the enable pin reference, if configured.
    pub fn enable_pin(&self) -> Option<&EN> {
        self.enable.as_ref()
    }

    /// Borrow the enable pin mutably, if configured.
    pub fn enable_pin_mut(&mut self) -> Option<&mut EN> {
        self.enable.as_mut()
    }
}

impl<P1, P2> DCDriver2PWM<P1, P2, NoPin>
where
    P1: SetDutyCycle,
    P2: SetDutyCycle,
{
    /// Construct a new 2-PWM driver without an external enable pin.
    pub fn without_enable(pin1: P1, pin2: P2) -> Self {
        Self::new(pin1, pin2, None)
    }
}

impl<P1, P2, EN> DCDriver for DCDriver2PWM<P1, P2, EN>
where
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    EN: OutputPin,
{
    /// Initialise driver parameters and set outputs to safe idle state.
    ///
    /// Mirrors upstream `DCDriver2PWM::init()`
    /// ([`DCDriver2PWM.cpp`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver2PWM.cpp) lines 15–24).
    fn init(&mut self) -> Result<(), ()> {
        if self.params.voltage_limit <= 0.0
            || self.params.voltage_limit > self.params.voltage_power_supply
        {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }
        self.params.initialized = true;

        // Ensure enable pin is in disabled state upon init
        let active = self.params.enable_active_high;
        if let Some(en) = self.enable.as_mut() {
            let _ = if active { en.set_low() } else { en.set_high() };
        }

        self.set_pwm(0.0);
        Ok(())
    }

    /// Enable driver hardware outputs.
    ///
    /// Mirrors upstream `DCDriver::enable()`
    /// ([`DCDriver.cpp`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver.cpp) lines 4–7).
    fn enable(&mut self) {
        let active = self.params.enable_active_high;
        if let Some(en) = self.enable.as_mut() {
            let _ = if active { en.set_high() } else { en.set_low() };
        }
        self.set_pwm(0.0);
    }

    /// Disable driver hardware outputs (zero PWM duty and deassert enable).
    ///
    /// Mirrors upstream `DCDriver::disable()`
    /// ([`DCDriver.cpp`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver.cpp) lines 10–14).
    fn disable(&mut self) {
        self.set_pwm(0.0);
        let active = self.params.enable_active_high;
        if let Some(en) = self.enable.as_mut() {
            let _ = if active { en.set_low() } else { en.set_high() };
        }
    }

    /// Set phase voltage to the hardware with half-bridge zero-before-drive sequencing.
    ///
    /// Mirrors upstream `DCDriver2PWM::setPwm(float U)`
    /// ([`DCDriver2PWM.cpp`](source/Arduino-FOC-drivers/src/drivers/dc/DCDriver2PWM.cpp) lines 28–40).
    fn set_pwm(&mut self, voltage: f32) {
        let v_limit = self.params.voltage_limit;
        let v_supply = if self.params.voltage_power_supply > 0.0 {
            self.params.voltage_power_supply
        } else {
            1.0
        };

        if voltage > 0.0 {
            let u = if voltage > v_limit { v_limit } else { voltage };
            let duty_fraction = (u / v_supply).clamp(0.0, 1.0);
            let max_duty = self.pin1.max_duty_cycle() as f32;
            let duty = (duty_fraction * max_duty) as u16;

            // Zero inactive half-bridge before driving active half-bridge
            let _ = self.pin2.set_duty_cycle(0);
            let _ = self.pin1.set_duty_cycle(duty);
        } else if voltage < 0.0 {
            let abs_v = -voltage;
            let u = if abs_v > v_limit { v_limit } else { abs_v };
            let duty_fraction = (u / v_supply).clamp(0.0, 1.0);
            let max_duty = self.pin2.max_duty_cycle() as f32;
            let duty = (duty_fraction * max_duty) as u16;

            // Zero inactive half-bridge before driving active half-bridge
            let _ = self.pin1.set_duty_cycle(0);
            let _ = self.pin2.set_duty_cycle(duty);
        } else {
            let _ = self.pin1.set_duty_cycle(0);
            let _ = self.pin2.set_duty_cycle(0);
        }
    }

    fn params(&self) -> &DCDriverParams {
        &self.params
    }

    fn params_mut(&mut self) -> &mut DCDriverParams {
        &mut self.params
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::pin::{
        Mock as PinMock, State as PinStateMock, Transaction as PinTransaction,
    };
    use embedded_hal_mock::eh1::pwm::{
        Mock as PwmMock, Transaction as PwmTransaction,
    };

    const MAX_DUTY: u16 = 1000;

    #[test]
    fn test_init_sets_initialized_and_clamps_voltage_limit() {
        let exp1 = [PwmTransaction::set_duty_cycle(0)];
        let exp2 = [PwmTransaction::set_duty_cycle(0)];
        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);

        let pin_exp = [PinTransaction::set(PinStateMock::Low)];
        let mut en = PinMock::new(&pin_exp);

        let mut driver = DCDriver2PWM::new(&mut p1, &mut p2, Some(&mut en));
        driver.params_mut().voltage_power_supply = 12.0;
        driver.params_mut().voltage_limit = 24.0; // exceed supply

        assert!(!driver.params().initialized);
        driver.init().unwrap();
        assert!(driver.params().initialized);
        assert_eq!(driver.params().voltage_limit, 12.0);

        p1.done();
        p2.done();
        en.done();
    }

    #[test]
    fn test_set_pwm_forward_positive_voltage() {
        // Positive voltage: pin2 zeroed first, then pin1 set to duty
        let exp1 = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500), // 6.0 / 12.0 * 1000 = 500
        ];
        let exp2 = [PwmTransaction::set_duty_cycle(0)];

        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);

        let mut driver = DCDriver2PWM::without_enable(&mut p1, &mut p2);
        driver.params_mut().voltage_power_supply = 12.0;
        driver.params_mut().voltage_limit = 12.0;

        driver.set_pwm(6.0);

        p1.done();
        p2.done();
    }

    #[test]
    fn test_set_pwm_reverse_negative_voltage() {
        // Negative voltage: pin1 zeroed first, then pin2 set to duty
        let exp1 = [PwmTransaction::set_duty_cycle(0)];
        let exp2 = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500), // 6.0 / 12.0 * 1000 = 500
        ];

        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);

        let mut driver = DCDriver2PWM::without_enable(&mut p1, &mut p2);
        driver.params_mut().voltage_power_supply = 12.0;
        driver.params_mut().voltage_limit = 12.0;

        driver.set_pwm(-6.0);

        p1.done();
        p2.done();
    }

    #[test]
    fn test_set_pwm_zero_voltage() {
        // Zero voltage: both pins set to 0
        let exp1 = [PwmTransaction::set_duty_cycle(0)];
        let exp2 = [PwmTransaction::set_duty_cycle(0)];

        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);

        let mut driver = DCDriver2PWM::without_enable(&mut p1, &mut p2);
        driver.set_pwm(0.0);

        p1.done();
        p2.done();
    }

    #[test]
    fn test_set_pwm_clamps_to_voltage_limit() {
        // 18V commanded with 6V limit -> clamped to 6V (50% duty on 12V supply)
        let exp1 = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp2 = [PwmTransaction::set_duty_cycle(0)];

        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);

        let mut driver = DCDriver2PWM::without_enable(&mut p1, &mut p2);
        driver.params_mut().voltage_power_supply = 12.0;
        driver.params_mut().voltage_limit = 6.0;

        driver.set_pwm(18.0);

        p1.done();
        p2.done();
    }

    #[test]
    fn test_enable_and_disable_lifecycle_active_high() {
        let exp1 = [
            PwmTransaction::set_duty_cycle(0), // on enable()
            PwmTransaction::set_duty_cycle(0), // on disable()
        ];
        let exp2 = [
            PwmTransaction::set_duty_cycle(0), // on enable()
            PwmTransaction::set_duty_cycle(0), // on disable()
        ];
        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);

        let pin_exp = [
            PinTransaction::set(PinStateMock::High), // enable()
            PinTransaction::set(PinStateMock::Low),  // disable()
        ];
        let mut en = PinMock::new(&pin_exp);

        let mut driver = DCDriver2PWM::new(&mut p1, &mut p2, Some(&mut en));
        driver.params_mut().enable_active_high = true;

        driver.enable();
        driver.disable();

        p1.done();
        p2.done();
        en.done();
    }

    #[test]
    fn test_enable_and_disable_lifecycle_active_low() {
        let exp1 = [
            PwmTransaction::set_duty_cycle(0), // on enable()
            PwmTransaction::set_duty_cycle(0), // on disable()
        ];
        let exp2 = [
            PwmTransaction::set_duty_cycle(0), // on enable()
            PwmTransaction::set_duty_cycle(0), // on disable()
        ];
        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);

        let pin_exp = [
            PinTransaction::set(PinStateMock::Low),  // enable()
            PinTransaction::set(PinStateMock::High), // disable()
        ];
        let mut en = PinMock::new(&pin_exp);

        let mut driver = DCDriver2PWM::new(&mut p1, &mut p2, Some(&mut en));
        driver.params_mut().enable_active_high = false;

        driver.enable();
        driver.disable();

        p1.done();
        p2.done();
        en.done();
    }
}
