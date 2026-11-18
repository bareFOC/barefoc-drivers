//! # drv8316::bundle — integrated BareFOC BLDCDriver and CurrentSense bundles
//!
//! Provides `Drv8316Driver6PWM` and `Drv8316Driver3PWM` which wrap SPI configuration,
//! PWM generation, and hardware fault monitoring into a unified struct implementing
//! `barefoc::traits::driver::BLDCDriver`.
//!
//! Also provides the 1-line `create_current_sense()` factory method that automatically
//! configures `LowsideCurrentSense` with the internal CSA gain ($1 / G_{CSA}$) and
//! zero-offset ($V_{REF} / 2$).

use barefoc::common::time::Clock;
use barefoc::current_sense::lowside::LowsideCurrentSense;
use barefoc::traits::adc::AdcPin;
use barefoc::traits::current_sense::CurrentSense;
use barefoc::traits::driver::{
    BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState,
};
use embedded_hal::digital::InputPin;
use embedded_hal::spi::SpiDevice;

use super::config::Drv8316PwmMode;
use super::spi_variant::Drv8316S;
use super::status::Drv8316Status;
use crate::{DrvError, NoPin};

/// Integrated 6-PWM driver bundle for DRV8316.
pub struct Drv8316Driver6PWM<SPI, PWM, FAULT = NoPin> {
    pub spi_device: Drv8316S<SPI>,
    pub pwm_driver: PWM,
    pub fault_pin: Option<FAULT>,
}

impl<SPI, PWM> Drv8316Driver6PWM<SPI, PWM, NoPin> {
    /// Create a new 6-PWM bundle without a physical fault pin.
    pub fn new(spi_device: Drv8316S<SPI>, pwm_driver: PWM) -> Self {
        Self {
            spi_device,
            pwm_driver,
            fault_pin: None,
        }
    }
}

impl<SPI, PWM, FAULT> Drv8316Driver6PWM<SPI, PWM, FAULT> {
    /// Attach a physical `nFAULT` pin to the bundle.
    pub fn with_fault_pin<F: InputPin>(self, fault_pin: F) -> Drv8316Driver6PWM<SPI, PWM, F> {
        Drv8316Driver6PWM {
            spi_device: self.spi_device,
            pwm_driver: self.pwm_driver,
            fault_pin: Some(fault_pin),
        }
    }

    /// Read full diagnostic status over SPI.
    pub fn get_status(&mut self) -> Result<Drv8316Status, DrvError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.spi_device.get_status()
    }

    /// Clear latched device faults.
    pub fn clear_fault(&mut self) -> Result<(), DrvError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.spi_device.clear_fault()
    }

    /// Check if a fault is currently active.
    ///
    /// If an `nFAULT` pin was attached, checks the pin state (active low).
    /// Otherwise, queries the DRV8316 status over SPI.
    pub fn is_fault(&mut self) -> Result<bool, DrvError<SPI::Error>>
    where
        SPI: SpiDevice,
        FAULT: InputPin,
    {
        if let Some(ref mut pin) = self.fault_pin {
            if let Ok(low) = pin.is_low() {
                return Ok(low);
            }
        }
        let status = self.spi_device.get_status()?;
        Ok(status.is_fault())
    }

    /// Create an automatically calibrated `LowsideCurrentSense` for BareFOC.
    ///
    /// Computes $A/V$ sensitivity ($1 / G_{CSA}$) and default zero-current offset ($V_{REF} / 2$)
    /// directly from the configured DRV8316 CSA gain and provided ADC reference voltage.
    pub fn create_current_sense<P1, P2, P3, C>(
        &self,
        vref: f32,
        pin_a: P1,
        pin_b: P2,
        pin_c: Option<P3>,
    ) -> LowsideCurrentSense<P1, P2, P3, C>
    where
        P1: AdcPin,
        P2: AdcPin,
        P3: AdcPin,
        C: Clock,
    {
        let csa_gain = self.spi_device.csa_gain.v_per_a();
        let mut cs = LowsideCurrentSense::new(1.0, csa_gain, pin_a, pin_b, pin_c);
        let zero_offset = vref / 2.0;
        let base = cs.base_mut();
        base.offset_ia = zero_offset;
        base.offset_ib = zero_offset;
        base.offset_ic = zero_offset;
        cs
    }
}

impl<SPI, PWM, FAULT> FOCDriver for Drv8316Driver6PWM<SPI, PWM, FAULT>
where
    SPI: SpiDevice,
    PWM: BLDCDriver,
    FAULT: InputPin,
{
    type Error = DrvError<SPI::Error>;

    fn init(&mut self) -> Result<(), Self::Error> {
        self.spi_device.pwm_mode = Drv8316PwmMode::Pwm6;
        self.spi_device.init()?;
        self.pwm_driver
            .init()
            .map_err(|_| DrvError::InvalidConfig)?;
        Ok(())
    }

    fn enable(&mut self) {
        self.pwm_driver.enable();
        let _ = self.spi_device.set_driver_off(false);
    }

    fn disable(&mut self) {
        let _ = self.spi_device.set_driver_off(true);
        self.pwm_driver.disable();
    }

    fn driver_type(&self) -> DriverType {
        DriverType::BLDC
    }

    fn params(&self) -> &FocDriverParams {
        self.pwm_driver.params()
    }

    fn params_mut(&mut self) -> &mut FocDriverParams {
        self.pwm_driver.params_mut()
    }
}

impl<SPI, PWM, FAULT> BLDCDriver for Drv8316Driver6PWM<SPI, PWM, FAULT>
where
    SPI: SpiDevice,
    PWM: BLDCDriver,
    FAULT: InputPin,
{
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        self.pwm_driver.set_pwm(ua, ub, uc);
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        self.pwm_driver.set_phase_state(sa, sb, sc);
    }
}

// ---------------------------------------------------------------------------
// 3-PWM bundle
// ---------------------------------------------------------------------------

/// Integrated 3-PWM driver bundle for DRV8316.
pub struct Drv8316Driver3PWM<SPI, PWM, FAULT = NoPin> {
    pub spi_device: Drv8316S<SPI>,
    pub pwm_driver: PWM,
    pub fault_pin: Option<FAULT>,
}

impl<SPI, PWM> Drv8316Driver3PWM<SPI, PWM, NoPin> {
    /// Create a new 3-PWM bundle without a physical fault pin.
    pub fn new(spi_device: Drv8316S<SPI>, pwm_driver: PWM) -> Self {
        Self {
            spi_device,
            pwm_driver,
            fault_pin: None,
        }
    }
}

impl<SPI, PWM, FAULT> Drv8316Driver3PWM<SPI, PWM, FAULT> {
    /// Attach a physical `nFAULT` pin to the bundle.
    pub fn with_fault_pin<F: InputPin>(self, fault_pin: F) -> Drv8316Driver3PWM<SPI, PWM, F> {
        Drv8316Driver3PWM {
            spi_device: self.spi_device,
            pwm_driver: self.pwm_driver,
            fault_pin: Some(fault_pin),
        }
    }

    /// Read full diagnostic status over SPI.
    pub fn get_status(&mut self) -> Result<Drv8316Status, DrvError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.spi_device.get_status()
    }

    /// Clear latched device faults.
    pub fn clear_fault(&mut self) -> Result<(), DrvError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.spi_device.clear_fault()
    }

    /// Check if a fault is currently active.
    pub fn is_fault(&mut self) -> Result<bool, DrvError<SPI::Error>>
    where
        SPI: SpiDevice,
        FAULT: InputPin,
    {
        if let Some(ref mut pin) = self.fault_pin {
            if let Ok(low) = pin.is_low() {
                return Ok(low);
            }
        }
        let status = self.spi_device.get_status()?;
        Ok(status.is_fault())
    }

    /// Create an automatically calibrated `LowsideCurrentSense` for BareFOC.
    pub fn create_current_sense<P1, P2, P3, C>(
        &self,
        vref: f32,
        pin_a: P1,
        pin_b: P2,
        pin_c: Option<P3>,
    ) -> LowsideCurrentSense<P1, P2, P3, C>
    where
        P1: AdcPin,
        P2: AdcPin,
        P3: AdcPin,
        C: Clock,
    {
        let csa_gain = self.spi_device.csa_gain.v_per_a();
        let mut cs = LowsideCurrentSense::new(1.0, csa_gain, pin_a, pin_b, pin_c);
        let zero_offset = vref / 2.0;
        let base = cs.base_mut();
        base.offset_ia = zero_offset;
        base.offset_ib = zero_offset;
        base.offset_ic = zero_offset;
        cs
    }
}

impl<SPI, PWM, FAULT> FOCDriver for Drv8316Driver3PWM<SPI, PWM, FAULT>
where
    SPI: SpiDevice,
    PWM: BLDCDriver,
    FAULT: InputPin,
{
    type Error = DrvError<SPI::Error>;

    fn init(&mut self) -> Result<(), Self::Error> {
        self.spi_device.pwm_mode = Drv8316PwmMode::Pwm3;
        self.spi_device.init()?;
        self.pwm_driver
            .init()
            .map_err(|_| DrvError::InvalidConfig)?;
        Ok(())
    }

    fn enable(&mut self) {
        self.pwm_driver.enable();
        let _ = self.spi_device.set_driver_off(false);
    }

    fn disable(&mut self) {
        let _ = self.spi_device.set_driver_off(true);
        self.pwm_driver.disable();
    }

    fn driver_type(&self) -> DriverType {
        DriverType::BLDC
    }

    fn params(&self) -> &FocDriverParams {
        self.pwm_driver.params()
    }

    fn params_mut(&mut self) -> &mut FocDriverParams {
        self.pwm_driver.params_mut()
    }
}

impl<SPI, PWM, FAULT> BLDCDriver for Drv8316Driver3PWM<SPI, PWM, FAULT>
where
    SPI: SpiDevice,
    PWM: BLDCDriver,
    FAULT: InputPin,
{
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        self.pwm_driver.set_pwm(ua, ub, uc);
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        self.pwm_driver.set_phase_state(sa, sb, sc);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drv8316::config::Drv8316CsaGain;
    use barefoc::traits::adc::AdcPin;
    use barefoc::traits::current_sense::CurrentSense;
    use barefoc::traits::driver::{BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState};
    use embedded_hal_mock::eh1::spi::Mock as SpiMock;

    struct DummyClock;
    impl Clock for DummyClock {
        fn now_us(&self) -> u32 {
            0
        }
    }

    struct MockAdcPin {
        value: u16,
    }

    impl AdcPin for MockAdcPin {
        fn read(&mut self) -> u16 {
            self.value
        }
    }

    struct MockPwm {
        params: FocDriverParams,
        pwm: [f32; 3],
        phase_states: [PhaseState; 3],
        enabled: bool,
        inited: bool,
    }

    impl MockPwm {
        fn new() -> Self {
            Self {
                params: FocDriverParams::new(),
                pwm: [0.0; 3],
                phase_states: [PhaseState::PHASE_ON; 3],
                enabled: false,
                inited: false,
            }
        }
    }

    impl FOCDriver for MockPwm {
        type Error = core::convert::Infallible;

        fn init(&mut self) -> Result<(), Self::Error> {
            self.inited = true;
            self.params.initialized = true;
            Ok(())
        }

        fn enable(&mut self) {
            self.enabled = true;
        }

        fn disable(&mut self) {
            self.enabled = false;
        }

        fn driver_type(&self) -> DriverType {
            DriverType::BLDC
        }

        fn params(&self) -> &FocDriverParams {
            &self.params
        }

        fn params_mut(&mut self) -> &mut FocDriverParams {
            &mut self.params
        }
    }

    impl BLDCDriver for MockPwm {
        fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
            self.pwm = [ua, ub, uc];
        }

        fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
            self.phase_states = [sa, sb, sc];
        }
    }

    #[test]
    fn test_create_current_sense_csa_calibration() {
        let spi = SpiMock::<u8>::new(&[]);
        let spi_dev = Drv8316S::new(spi).with_csa_gain(Drv8316CsaGain::Gain0V25);
        let pwm = MockPwm::new();
        let mut bundle = Drv8316Driver6PWM::new(spi_dev, pwm);

        // VREF = 3.3V, ADC reading centered at 1.65V (offset)
        let pin_a = MockAdcPin { value: 1650 }; // zero current
        let pin_b = MockAdcPin { value: 1900 }; // positive current
        let pin_c = MockAdcPin { value: 1400 }; // negative current

        let mut cs = bundle.create_current_sense::<_, _, _, DummyClock>(
            1650.0,
            pin_a,
            pin_b,
            Some(pin_c),
        );

        // Verify gain computation: 1 / 0.25 = 4.0 A/V (or units)
        assert!((cs.base().gain_a - 4.0).abs() < 1e-5);
        assert!((cs.base().gain_b - 4.0).abs() < 1e-5);
        assert!((cs.base().gain_c - 4.0).abs() < 1e-5);

        // Verify offset
        assert_eq!(cs.base().offset_ia, 1650.0 / 2.0); // 825.0

        // Reset offset to match the test raw baseline
        cs.base_mut().offset_ia = 1650.0;
        cs.base_mut().offset_ib = 1650.0;
        cs.base_mut().offset_ic = 1650.0;

        let currents = cs.get_phase_currents();
        assert_eq!(currents.a, 0.0);
        assert_eq!(currents.b, (1900.0 - 1650.0) * 4.0); // 1000.0
        assert_eq!(currents.c, (1400.0 - 1650.0) * 4.0); // -1000.0

        bundle.spi_device.spi.done();
    }

    #[test]
    fn test_bundle_pwm_delegation() {
        let spi = SpiMock::<u8>::new(&[]);
        let spi_dev = Drv8316S::new(spi);
        let pwm = MockPwm::new();
        let mut bundle = Drv8316Driver6PWM::new(spi_dev, pwm);

        bundle.set_pwm(0.2, 0.5, 0.8);
        assert_eq!(bundle.pwm_driver.pwm, [0.2, 0.5, 0.8]);

        bundle.set_phase_state(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_HI);
        assert_eq!(
            bundle.pwm_driver.phase_states,
            [PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_HI]
        );

        assert_eq!(bundle.driver_type(), DriverType::BLDC);
        assert_eq!(bundle.params().pwm_frequency, 25_000);

        bundle.spi_device.spi.done();
    }
}
