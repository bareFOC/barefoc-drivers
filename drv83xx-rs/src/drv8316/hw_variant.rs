//! # drv8316::hw_variant — Hardware pin-strapped variant (DRV8316H)
//!
//! The DRV8316H variant uses external resistors/pin connections to configure
//! PWM mode, slew rate, and CSA gain. Software interacts via the active-low
//! `nFAULT` pin and optional active-high `DRVOFF` pin.

use crate::{CsaGainProvider, Drv83xxDriver, DrvError};
use super::config::{Drv8316CsaGain, Drv8316PwmMode};

/// Hardware pin-strapped variant of the DRV8316 (DRV8316H).
pub struct Drv8316H<FAULT, OFF> {
    pub fault_pin: FAULT,
    pub drv_off_pin: Option<OFF>,
    pub csa_gain: Drv8316CsaGain,
    pub pwm_mode: Drv8316PwmMode,
}

impl<FAULT, OFF> Drv8316H<FAULT, OFF>
where
    FAULT: embedded_hal::digital::InputPin,
    OFF: embedded_hal::digital::OutputPin,
{
    /// Construct a new `Drv8316H` driver.
    ///
    /// * `fault_pin`: `nFAULT` pin (active-low open-drain from DRV8316).
    /// * `drv_off_pin`: Optional `DRVOFF` pin (active-high input to DRV8316).
    /// * `csa_gain`: Hardware pin-strapped CSA gain setting.
    /// * `pwm_mode`: Hardware pin-strapped PWM mode (3-PWM or 6-PWM).
    pub fn new(
        fault_pin: FAULT,
        drv_off_pin: Option<OFF>,
        csa_gain: Drv8316CsaGain,
        pwm_mode: Drv8316PwmMode,
    ) -> Self {
        Self {
            fault_pin,
            drv_off_pin,
            csa_gain,
            pwm_mode,
        }
    }

    /// Checks if the driver has asserted a fault condition (`nFAULT` pin is low).
    pub fn is_fault(&mut self) -> Result<bool, DrvError<FAULT::Error>> {
        self.fault_pin.is_low().map_err(DrvError::Bus)
    }

    /// Sets the `DRVOFF` pin state. When `off` is `true`, power MOSFETs are disabled.
    pub fn set_driver_off(&mut self, off: bool) -> Result<(), DrvError<OFF::Error>> {
        if let Some(ref mut pin) = self.drv_off_pin {
            if off {
                pin.set_high().map_err(DrvError::Bus)?;
            } else {
                pin.set_low().map_err(DrvError::Bus)?;
            }
        }
        Ok(())
    }
}

impl<FAULT, OFF> Drv83xxDriver for Drv8316H<FAULT, OFF>
where
    FAULT: embedded_hal::digital::InputPin,
    OFF: embedded_hal::digital::OutputPin,
{
    type Error = DrvError<FAULT::Error>;

    fn clear_fault(&mut self) -> Result<(), Self::Error> {
        // On DRV8316H, clearing a latched fault is achieved by toggling DRVOFF or cycling power/nSLEEP
        if let Some(ref mut pin) = self.drv_off_pin {
            let _ = pin.set_high();
            let _ = pin.set_low();
        }
        Ok(())
    }

    fn is_fault(&mut self) -> Result<bool, Self::Error> {
        self.is_fault()
    }
}

impl<FAULT, OFF> CsaGainProvider for Drv8316H<FAULT, OFF> {
    fn csa_gain_v_per_a(&self) -> f32 {
        self.csa_gain.v_per_a()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::pin::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };

    #[test]
    fn test_drv8316h_fault_detection() {
        // nFAULT pin is active low
        let fault_exp = [
            PinTransaction::get(PinState::High),
            PinTransaction::get(PinState::Low),
        ];
        let fault_pin = PinMock::new(&fault_exp);
        let mut drv = Drv8316H::new(
            fault_pin,
            None::<PinMock>,
            Drv8316CsaGain::Gain0V25,
            Drv8316PwmMode::Pwm6,
        );

        // First read: High (no fault)
        assert!(!drv.is_fault().unwrap());
        // Second read: Low (fault asserted)
        assert!(drv.is_fault().unwrap());

        assert_eq!(drv.csa_gain_v_per_a(), 0.25);
        assert_eq!(drv.csa_gain_a_per_v(), 4.0);

        drv.fault_pin.done();
    }

    #[test]
    fn test_drv8316h_driver_off() {
        let drv_off_exp = [
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
        ];
        let off_pin = PinMock::new(&drv_off_exp);
        let fault_pin = PinMock::new(&[]);
        let mut drv = Drv8316H::new(
            fault_pin,
            Some(off_pin),
            Drv8316CsaGain::Gain0V15,
            Drv8316PwmMode::Pwm3,
        );

        drv.set_driver_off(true).unwrap();
        drv.set_driver_off(false).unwrap();

        drv.fault_pin.done();
        drv.drv_off_pin.unwrap().done();
    }
}
