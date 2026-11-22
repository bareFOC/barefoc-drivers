//! # dc::driver_1pwm — 1-PWM + 1-DIR (Phase/DIR) sign-magnitude DC motor driver
//!
//! Intended to drive single-channel DC motor drivers with a PWM signal controlling
//! speed (magnitude) and a digital GPIO signal controlling direction (phase/sign),
//! such as DRV8871/DRV8874/DRV8876 in PH/EN mode, A4950, or discrete H-bridges.
//!
//! When direction is forward (positive voltage), DIR is driven low (or active according to polarity)
//! and PWM duty cycle is proportional to the demanded voltage.
//! When direction is reverse (negative voltage), DIR is driven high (or inverted)
//! and PWM duty cycle is proportional to the demanded voltage magnitude.
//! When commanded voltage is zero, PWM duty cycle is set to 0.

#![allow(non_camel_case_types)]

use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

use crate::dc::traits::{DCDriver, DCDriverParams, NoPin};

/// 1-PWM DC motor driver for sign-magnitude (Phase/DIR) H-bridge topologies.
///
/// Generic over:
/// - `PWM`: PWM channel type implementing [`SetDutyCycle`].
/// - `DIR`: Direction pin type implementing [`OutputPin`].
/// - `EN`: Enable pin type implementing [`OutputPin`]. Defaults to [`NoPin`].
pub struct DCDriver1PWM<PWM, DIR, EN = NoPin>
where
    PWM: SetDutyCycle,
    DIR: OutputPin,
    EN: OutputPin,
{
    params: DCDriverParams,
    pwm: PWM,
    dir: DIR,
    enable: Option<EN>,
    dir_reversed: bool,
}

impl<PWM, DIR, EN> DCDriver1PWM<PWM, DIR, EN>
where
    PWM: SetDutyCycle,
    DIR: OutputPin,
    EN: OutputPin,
{
    /// Construct a new 1-PWM sign-magnitude driver with an optional enable pin.
    pub fn new(pwm: PWM, dir: DIR, enable: Option<EN>) -> Self {
        Self {
            params: DCDriverParams::new(),
            pwm,
            dir,
            enable,
            dir_reversed: false,
        }
    }

    /// Set whether the direction output pin logic should be inverted.
    pub fn set_dir_reversed(&mut self, reversed: bool) {
        self.dir_reversed = reversed;
    }

    /// Builder pattern helper to set inverted direction logic.
    pub fn with_dir_reversed(mut self, reversed: bool) -> Self {
        self.dir_reversed = reversed;
        self
    }

    /// Borrow the PWM channel.
    pub fn pwm(&self) -> &PWM {
        &self.pwm
    }

    /// Borrow the PWM channel mutably.
    pub fn pwm_mut(&mut self) -> &mut PWM {
        &mut self.pwm
    }

    /// Borrow the direction pin.
    pub fn dir_pin(&self) -> &DIR {
        &self.dir
    }

    /// Borrow the direction pin mutably.
    pub fn dir_pin_mut(&mut self) -> &mut DIR {
        &mut self.dir
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

impl<PWM, DIR, EN> DCDriver for DCDriver1PWM<PWM, DIR, EN>
where
    PWM: SetDutyCycle,
    DIR: OutputPin,
    EN: OutputPin,
{
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

    fn enable(&mut self) {
        let active = self.params.enable_active_high;
        if let Some(en) = self.enable.as_mut() {
            let _ = if active { en.set_high() } else { en.set_low() };
        }
        self.set_pwm(0.0);
    }

    fn disable(&mut self) {
        self.set_pwm(0.0);
        let active = self.params.enable_active_high;
        if let Some(en) = self.enable.as_mut() {
            let _ = if active { en.set_low() } else { en.set_high() };
        }
    }

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
            let max_duty = self.pwm.max_duty_cycle() as f32;
            let duty = (duty_fraction * max_duty) as u16;

            if !self.dir_reversed {
                let _ = self.dir.set_low();
            } else {
                let _ = self.dir.set_high();
            }
            let _ = self.pwm.set_duty_cycle(duty);
        } else if voltage < 0.0 {
            let abs_v = -voltage;
            let u = if abs_v > v_limit { v_limit } else { abs_v };
            let duty_fraction = (u / v_supply).clamp(0.0, 1.0);
            let max_duty = self.pwm.max_duty_cycle() as f32;
            let duty = (duty_fraction * max_duty) as u16;

            if !self.dir_reversed {
                let _ = self.dir.set_high();
            } else {
                let _ = self.dir.set_low();
            }
            let _ = self.pwm.set_duty_cycle(duty);
        } else {
            let _ = self.pwm.set_duty_cycle(0);
        }
    }

    fn params(&self) -> &DCDriverParams {
        &self.params
    }

    fn params_mut(&mut self) -> &mut DCDriverParams {
        &mut self.params
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockPwm {
        duty: u16,
        max_duty: u16,
    }

    impl MockPwm {
        fn new(max: u16) -> Self {
            Self {
                duty: 0,
                max_duty: max,
            }
        }
    }

    impl embedded_hal::pwm::ErrorType for MockPwm {
        type Error = core::convert::Infallible;
    }

    impl SetDutyCycle for MockPwm {
        fn max_duty_cycle(&self) -> u16 {
            self.max_duty
        }
        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            self.duty = duty;
            Ok(())
        }
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    enum PinState {
        High,
        Low,
    }

    struct MockPin {
        state: PinState,
    }

    impl MockPin {
        fn new() -> Self {
            Self {
                state: PinState::Low,
            }
        }
    }

    impl embedded_hal::digital::ErrorType for MockPin {
        type Error = core::convert::Infallible;
    }

    impl OutputPin for MockPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.state = PinState::Low;
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.state = PinState::High;
            Ok(())
        }
    }

    #[test]
    fn test_driver_1pwm_forward_reverse_zero() {
        let pwm = MockPwm::new(1000);
        let dir = MockPin::new();
        let en = MockPin::new();
        let mut driver = DCDriver1PWM::new(pwm, dir, Some(en));
        driver.params_mut().voltage_power_supply = 12.0;
        driver.params_mut().voltage_limit = 12.0;
        assert!(driver.init().is_ok());

        // Enable driver
        driver.enable();
        assert_eq!(driver.enable_pin().unwrap().state, PinState::High);

        // Forward: 6V out of 12V -> 50% duty = 500
        driver.set_pwm(6.0);
        assert_eq!(driver.pwm().duty, 500);
        assert_eq!(driver.dir_pin().state, PinState::Low);

        // Reverse: -6V out of 12V -> 50% duty = 500, DIR High
        driver.set_pwm(-6.0);
        assert_eq!(driver.pwm().duty, 500);
        assert_eq!(driver.dir_pin().state, PinState::High);

        // Zero: 0V -> 0 duty
        driver.set_pwm(0.0);
        assert_eq!(driver.pwm().duty, 0);

        // Disable
        driver.disable();
        assert_eq!(driver.enable_pin().unwrap().state, PinState::Low);
    }

    #[test]
    fn test_driver_1pwm_inverted_direction() {
        let pwm = MockPwm::new(1000);
        let dir = MockPin::new();
        let mut driver = DCDriver1PWM::new(pwm, dir, None::<NoPin>).with_dir_reversed(true);
        driver.params_mut().voltage_power_supply = 12.0;
        driver.params_mut().voltage_limit = 12.0;
        assert!(driver.init().is_ok());

        // Forward with reversed DIR -> DIR High
        driver.set_pwm(6.0);
        assert_eq!(driver.pwm().duty, 500);
        assert_eq!(driver.dir_pin().state, PinState::High);

        // Reverse with reversed DIR -> DIR Low
        driver.set_pwm(-6.0);
        assert_eq!(driver.pwm().duty, 500);
        assert_eq!(driver.dir_pin().state, PinState::Low);
    }
}
