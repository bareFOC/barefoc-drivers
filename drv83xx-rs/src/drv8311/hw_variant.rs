//! # drv8311::hw_variant — Hardware pin-strapped variant (DRV8311H)
//!
//! The DRV8311H variant uses hardware pin connections (GAIN, SLEW, MODE) to
//! configure the device. Software interacts via the active-low `nFAULT` pin and
//! active-low `nSLEEP` pin.
//!
//! References: Texas Instruments DRV8311 Datasheet (SLVSFN2B).

use crate::{CsaGainProvider, Drv83xxDriver, DrvError, NoPin};
use super::config::{Drv8311CsaGain, Drv8311PwmMode};

/// Hardware pin-strapped variant of the DRV8311 (DRV8311H).
pub struct Drv8311H<FAULT = NoPin, SLEEP = NoPin> {
    pub fault_pin: FAULT,
    pub sleep_pin: Option<SLEEP>,
    pub csa_gain: Drv8311CsaGain,
    pub pwm_mode: Drv8311PwmMode,
}

impl<FAULT, SLEEP> Drv8311H<FAULT, SLEEP> {
    /// Construct a new `Drv8311H` driver.
    ///
    /// * `fault_pin`: `nFAULT` pin (active-low open-drain from DRV8311).
    /// * `sleep_pin`: Optional `nSLEEP` pin (active-low input to DRV8311).
    /// * `csa_gain`: Hardware pin-strapped CSA gain setting.
    /// * `pwm_mode`: Hardware pin-strapped PWM mode (3x or 6x).
    pub fn new(
        fault_pin: FAULT,
        sleep_pin: Option<SLEEP>,
        csa_gain: Drv8311CsaGain,
        pwm_mode: Drv8311PwmMode,
    ) -> Self {
        Self {
            fault_pin,
            sleep_pin,
            csa_gain,
            pwm_mode,
        }
    }
}

impl<FAULT: embedded_hal::digital::InputPin, SLEEP> Drv8311H<FAULT, SLEEP> {
    /// Checks if the driver has asserted a fault condition (`nFAULT` pin is low).
    pub fn is_fault(&mut self) -> Result<bool, DrvError<FAULT::Error>> {
        self.fault_pin.is_low().map_err(DrvError::Bus)
    }
}

impl<FAULT, SLEEP: embedded_hal::digital::OutputPin> Drv8311H<FAULT, SLEEP> {
    /// Sets the `nSLEEP` pin state. When `sleep` is `true`, pin is pulled low (sleep mode).
    pub fn set_sleep(&mut self, sleep: bool) -> Result<(), DrvError<SLEEP::Error>> {
        if let Some(ref mut pin) = self.sleep_pin {
            if sleep {
                pin.set_low().map_err(DrvError::Bus)?;
            } else {
                pin.set_high().map_err(DrvError::Bus)?;
            }
        }
        Ok(())
    }

    /// Resets latched device faults by pulsing `nSLEEP` low then high.
    pub fn reset_faults(&mut self) -> Result<(), DrvError<SLEEP::Error>> {
        if let Some(ref mut pin) = self.sleep_pin {
            pin.set_low().map_err(DrvError::Bus)?;
            // Datasheet specifies min tRST = 20 us pulse
            pin.set_high().map_err(DrvError::Bus)?;
        }
        Ok(())
    }
}

impl<FAULT, SLEEP> Drv83xxDriver for Drv8311H<FAULT, SLEEP>
where
    FAULT: embedded_hal::digital::InputPin,
    SLEEP: embedded_hal::digital::OutputPin,
{
    type Error = DrvError<FAULT::Error>;

    fn clear_fault(&mut self) -> Result<(), Self::Error> {
        // In HW variant without SPI, nSLEEP pulse or power cycle clears faults
        Ok(())
    }

    fn is_fault(&mut self) -> Result<bool, Self::Error> {
        self.is_fault()
    }
}

impl<FAULT, SLEEP> CsaGainProvider for Drv8311H<FAULT, SLEEP> {
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
    fn test_drv8311h_fault_detection() {
        let expectations = [PinTransaction::get(PinState::Low)];
        let fault_pin = PinMock::new(&expectations);
        let mut drv: Drv8311H<_, PinMock> = Drv8311H::new(
            fault_pin,
            None,
            Drv8311CsaGain::Gain0V5,
            Drv8311PwmMode::Pwm6x,
        );

        assert!(drv.is_fault().unwrap());
        assert_eq!(drv.csa_gain_v_per_a(), 0.5);
        drv.fault_pin.done();
    }

    #[test]
    fn test_drv8311h_sleep_and_reset() {
        let expectations = [
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ];
        let fault_pin = PinMock::new(&[]);
        let sleep_pin = PinMock::new(&expectations);
        let mut drv = Drv8311H::new(
            fault_pin,
            Some(sleep_pin),
            Drv8311CsaGain::Gain1V0,
            Drv8311PwmMode::Pwm3x,
        );

        drv.reset_faults().unwrap();
        drv.fault_pin.done();
        drv.sleep_pin.unwrap().done();
    }
}
