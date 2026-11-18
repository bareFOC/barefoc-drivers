//! # drv8311::bundle — integrated BareFOC BLDCDriver and CurrentSense bundles
//!
//! Provides `Drv8311Driver6PWM`, `Drv8311Driver3PWM`, and `Drv8311DriverPwmGen`
//! which wrap SPI configuration, PWM generation, and hardware fault monitoring
//! into unified structs implementing `barefoc::traits::driver::BLDCDriver`.
//!
//! Also provides the 1-line `create_current_sense()` factory method that automatically
//! configures `LowsideCurrentSense` with internal CSA gain ($1 / G_{CSA}$) and
//! zero-offset ($V_{REF} / 2$).

use barefoc::common::time::Clock;
use barefoc::current_sense::lowside::LowsideCurrentSense;
use barefoc::traits::adc::AdcPin;
use barefoc::traits::current_sense::CurrentSense;
use barefoc::traits::driver::{
    BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState,
};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

use super::config::{Drv8311CsaGain, Drv8311PwmMode, Drv8311PwmSync};
use super::hw_variant::Drv8311H;
use super::pwmg_variant::Drv8311P;
use super::spi_variant::Drv8311S;
use super::status::Drv8311Status;
use crate::{DrvError, NoPin};

// ===========================================================================
// 6-PWM Bundle (Drv8311Driver6PWM)
// ===========================================================================

/// Integrated 6-PWM driver bundle for DRV8311.
pub struct Drv8311Driver6PWM<SPI, PWM, FAULT = NoPin> {
    pub spi_device: Drv8311S<SPI>,
    pub pwm_driver: PWM,
    pub fault_pin: Option<FAULT>,
}

impl<SPI, PWM> Drv8311Driver6PWM<SPI, PWM, NoPin> {
    /// Create a new 6-PWM bundle without a physical fault pin.
    pub fn new(spi_device: Drv8311S<SPI>, pwm_driver: PWM) -> Self {
        Self {
            spi_device,
            pwm_driver,
            fault_pin: None,
        }
    }
}

impl<SPI, PWM, FAULT> Drv8311Driver6PWM<SPI, PWM, FAULT> {
    /// Attach a physical `nFAULT` pin to the bundle.
    pub fn with_fault_pin<F: InputPin>(self, fault_pin: F) -> Drv8311Driver6PWM<SPI, PWM, F> {
        Drv8311Driver6PWM {
            spi_device: self.spi_device,
            pwm_driver: self.pwm_driver,
            fault_pin: Some(fault_pin),
        }
    }

    /// Read full diagnostic status over SPI.
    pub fn get_status(&mut self) -> Result<Drv8311Status, DrvError<SPI::Error>>
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

impl<SPI, PWM, FAULT> FOCDriver for Drv8311Driver6PWM<SPI, PWM, FAULT>
where
    SPI: SpiDevice,
    PWM: BLDCDriver,
    FAULT: InputPin,
{
    type Error = DrvError<SPI::Error>;

    fn init(&mut self) -> Result<(), Self::Error> {
        self.spi_device.pwm_mode = Drv8311PwmMode::Pwm6x;
        self.spi_device.init()?;
        self.pwm_driver
            .init()
            .map_err(|_| DrvError::InvalidConfig)?;
        Ok(())
    }

    fn enable(&mut self) {
        self.pwm_driver.enable();
    }

    fn disable(&mut self) {
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

impl<SPI, PWM, FAULT> BLDCDriver for Drv8311Driver6PWM<SPI, PWM, FAULT>
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

// ===========================================================================
// 3-PWM Bundle (Drv8311Driver3PWM)
// ===========================================================================

/// Integrated 3-PWM driver bundle for DRV8311.
pub struct Drv8311Driver3PWM<SPI, PWM, FAULT = NoPin> {
    pub spi_device: Drv8311S<SPI>,
    pub pwm_driver: PWM,
    pub fault_pin: Option<FAULT>,
}

impl<SPI, PWM> Drv8311Driver3PWM<SPI, PWM, NoPin> {
    /// Create a new 3-PWM bundle without a physical fault pin.
    pub fn new(spi_device: Drv8311S<SPI>, pwm_driver: PWM) -> Self {
        Self {
            spi_device,
            pwm_driver,
            fault_pin: None,
        }
    }
}

impl<SPI, PWM, FAULT> Drv8311Driver3PWM<SPI, PWM, FAULT> {
    /// Attach a physical `nFAULT` pin to the bundle.
    pub fn with_fault_pin<F: InputPin>(self, fault_pin: F) -> Drv8311Driver3PWM<SPI, PWM, F> {
        Drv8311Driver3PWM {
            spi_device: self.spi_device,
            pwm_driver: self.pwm_driver,
            fault_pin: Some(fault_pin),
        }
    }

    /// Read full diagnostic status over SPI.
    pub fn get_status(&mut self) -> Result<Drv8311Status, DrvError<SPI::Error>>
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

impl<SPI, PWM, FAULT> FOCDriver for Drv8311Driver3PWM<SPI, PWM, FAULT>
where
    SPI: SpiDevice,
    PWM: BLDCDriver,
    FAULT: InputPin,
{
    type Error = DrvError<SPI::Error>;

    fn init(&mut self) -> Result<(), Self::Error> {
        self.spi_device.pwm_mode = Drv8311PwmMode::Pwm3x;
        self.spi_device.init()?;
        self.pwm_driver
            .init()
            .map_err(|_| DrvError::InvalidConfig)?;
        Ok(())
    }

    fn enable(&mut self) {
        self.pwm_driver.enable();
    }

    fn disable(&mut self) {
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

impl<SPI, PWM, FAULT> BLDCDriver for Drv8311Driver3PWM<SPI, PWM, FAULT>
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

// ===========================================================================
// Hardware-Configured 3-PWM Bundle (Drv8311HDriver3PWM)
// ===========================================================================

/// Integrated 3-PWM driver bundle for DRV8311H (hardware pin-strapped variant).
pub struct Drv8311HDriver3PWM<PWM, FAULT = NoPin, SLEEP = NoPin> {
    pub drv: Drv8311H<FAULT, SLEEP>,
    pub pwm_driver: PWM,
}

impl<PWM> Drv8311HDriver3PWM<PWM, NoPin, NoPin> {
    /// Create a new 3-PWM bundle without physical fault or sleep pins.
    pub fn new(pwm_driver: PWM) -> Self {
        Self {
            drv: Drv8311H::new(NoPin, None, Drv8311CsaGain::Gain0V25, Drv8311PwmMode::Pwm3x),
            pwm_driver,
        }
    }
}

impl<PWM, FAULT, SLEEP> Drv8311HDriver3PWM<PWM, FAULT, SLEEP> {
    /// Construct from an existing `Drv8311H` driver and PWM generator.
    pub fn from_parts(drv: Drv8311H<FAULT, SLEEP>, pwm_driver: PWM) -> Self {
        Self { drv, pwm_driver }
    }

    /// Attach a physical active-low `nFAULT` input pin.
    pub fn with_fault_pin<F: InputPin>(self, fault_pin: F) -> Drv8311HDriver3PWM<PWM, F, SLEEP> {
        Drv8311HDriver3PWM {
            drv: Drv8311H {
                fault_pin,
                sleep_pin: self.drv.sleep_pin,
                csa_gain: self.drv.csa_gain,
                pwm_mode: self.drv.pwm_mode,
            },
            pwm_driver: self.pwm_driver,
        }
    }

    /// Attach an active-high `nSLEEP` (wake) output pin.
    pub fn with_sleep_pin<S: OutputPin>(self, sleep_pin: S) -> Drv8311HDriver3PWM<PWM, FAULT, S> {
        Drv8311HDriver3PWM {
            drv: Drv8311H {
                fault_pin: self.drv.fault_pin,
                sleep_pin: Some(sleep_pin),
                csa_gain: self.drv.csa_gain,
                pwm_mode: self.drv.pwm_mode,
            },
            pwm_driver: self.pwm_driver,
        }
    }

    /// Set hardware CSA gain setting.
    pub fn with_csa_gain(mut self, gain: Drv8311CsaGain) -> Self {
        self.drv.csa_gain = gain;
        self
    }

    /// Check if driver asserted a fault condition (`nFAULT` pin is low).
    pub fn is_fault(&mut self) -> Result<bool, DrvError<FAULT::Error>>
    where
        FAULT: InputPin,
    {
        self.drv.is_fault()
    }

    /// Reset latched device faults by pulsing `nSLEEP`.
    pub fn reset_faults(&mut self) -> Result<(), DrvError<SLEEP::Error>>
    where
        SLEEP: OutputPin,
    {
        self.drv.reset_faults()
    }

    /// Wake up the driver (`nSLEEP` high).
    pub fn wake(&mut self) -> Result<(), DrvError<SLEEP::Error>>
    where
        SLEEP: OutputPin,
    {
        self.drv.set_sleep(false)
    }

    /// Put the driver into low-power sleep mode (`nSLEEP` low).
    pub fn sleep(&mut self) -> Result<(), DrvError<SLEEP::Error>>
    where
        SLEEP: OutputPin,
    {
        self.drv.set_sleep(true)
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
        let csa_gain = self.drv.csa_gain.v_per_a();
        let mut cs = LowsideCurrentSense::new(1.0, csa_gain, pin_a, pin_b, pin_c);
        let zero_offset = vref / 2.0;
        let base = cs.base_mut();
        base.offset_ia = zero_offset;
        base.offset_ib = zero_offset;
        base.offset_ic = zero_offset;
        cs
    }
}

impl<PWM, FAULT, SLEEP> FOCDriver for Drv8311HDriver3PWM<PWM, FAULT, SLEEP>
where
    PWM: BLDCDriver,
    SLEEP: OutputPin,
{
    type Error = DrvError<SLEEP::Error>;

    fn init(&mut self) -> Result<(), Self::Error> {
        let _ = self.drv.set_sleep(false);
        self.pwm_driver
            .init()
            .map_err(|_| DrvError::InvalidConfig)?;
        Ok(())
    }

    fn enable(&mut self) {
        let _ = self.drv.set_sleep(false);
        self.pwm_driver.enable();
    }

    fn disable(&mut self) {
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

impl<PWM, FAULT, SLEEP> BLDCDriver for Drv8311HDriver3PWM<PWM, FAULT, SLEEP>
where
    PWM: BLDCDriver,
    SLEEP: OutputPin,
{
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        self.pwm_driver.set_pwm(ua, ub, uc);
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        self.pwm_driver.set_phase_state(sa, sb, sc);
    }
}

// ===========================================================================
// PWM Generator Bundle (Drv8311DriverPwmGen — Zero-Timer-Pin FOC Driver)
// ===========================================================================

/// Integrated zero-timer-pin FOC driver bundle for DRV8311.
///
/// Drives motor phase voltages entirely over SPI/tSPI into internal `PWMG_*_DUTY`
/// registers without requiring microcontroller PWM timer peripherals.
pub struct Drv8311DriverPwmGen<SPI, FAULT = NoPin, SYNC = NoPin> {
    pub drv: Drv8311P<SPI>,
    pub fault_pin: Option<FAULT>,
    pub sync_pin: Option<SYNC>,
    pub params: FocDriverParams,
    pub period: u16,
    pub center_aligned: bool,
    pub enabled: bool,
}

impl<SPI> Drv8311DriverPwmGen<SPI, NoPin, NoPin> {
    /// Create a new `Drv8311DriverPwmGen` bundle with default 1000-count PWM period (~20 kHz).
    pub fn new(drv: Drv8311P<SPI>) -> Self {
        Self {
            drv,
            fault_pin: None,
            sync_pin: None,
            params: FocDriverParams::new(),
            period: 1000,
            center_aligned: true,
            enabled: false,
        }
    }
}

impl<SPI, FAULT, SYNC> Drv8311DriverPwmGen<SPI, FAULT, SYNC> {
    /// Attach an active-low `nFAULT` input pin.
    pub fn with_fault_pin<F: InputPin>(self, fault_pin: F) -> Drv8311DriverPwmGen<SPI, F, SYNC> {
        Drv8311DriverPwmGen {
            drv: self.drv,
            fault_pin: Some(fault_pin),
            sync_pin: self.sync_pin,
            params: self.params,
            period: self.period,
            center_aligned: self.center_aligned,
            enabled: self.enabled,
        }
    }

    /// Attach a `PWM_SYNC` output pin for hardware timer synchronization.
    pub fn with_sync_pin<S: OutputPin>(self, sync_pin: S) -> Drv8311DriverPwmGen<SPI, FAULT, S> {
        Drv8311DriverPwmGen {
            drv: self.drv,
            fault_pin: self.fault_pin,
            sync_pin: Some(sync_pin),
            params: self.params,
            period: self.period,
            center_aligned: self.center_aligned,
            enabled: self.enabled,
        }
    }

    /// Set PWM generator period in 12-bit counts (e.g. 1000 for 20 kHz carrier).
    pub fn with_period(mut self, period: u16) -> Self {
        self.period = period & 0x0FFF;
        self
    }

    /// Set counter mode: `true` for Center-aligned (Up-Down), `false` for Edge-aligned (Up).
    pub fn with_center_aligned(mut self, center_aligned: bool) -> Self {
        self.center_aligned = center_aligned;
        self
    }

    /// Read full diagnostic status over SPI.
    pub fn get_status(&mut self) -> Result<Drv8311Status, DrvError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.drv.get_status()
    }

    /// Clear latched device faults.
    pub fn clear_fault(&mut self) -> Result<(), DrvError<SPI::Error>>
    where
        SPI: SpiDevice,
    {
        self.drv.clear_fault()
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
        let status = self.drv.get_status()?;
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
        let csa_gain = self.drv.csa_gain.v_per_a();
        let mut cs = LowsideCurrentSense::new(1.0, csa_gain, pin_a, pin_b, pin_c);
        let zero_offset = vref / 2.0;
        let base = cs.base_mut();
        base.offset_ia = zero_offset;
        base.offset_ib = zero_offset;
        base.offset_ic = zero_offset;
        cs
    }
}

impl<SPI, FAULT, SYNC> FOCDriver for Drv8311DriverPwmGen<SPI, FAULT, SYNC>
where
    SPI: SpiDevice,
    FAULT: InputPin,
    SYNC: OutputPin,
{
    type Error = DrvError<SPI::Error>;

    fn init(&mut self) -> Result<(), Self::Error> {
        self.drv.init()?;
        self.drv.enable_pwm_gen(self.period, self.center_aligned)?;
        if self.sync_pin.is_some() {
            self.drv.set_pwm_sync(Drv8311PwmSync::SetPeriod)?;
        }
        self.enabled = true;
        Ok(())
    }

    fn enable(&mut self) {
        let _ = self.drv.set_phase_states(7, 7, 7);
        self.enabled = true;
    }

    fn disable(&mut self) {
        let _ = self.drv.set_phase_states(0, 0, 0);
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

impl<SPI, FAULT, SYNC> BLDCDriver for Drv8311DriverPwmGen<SPI, FAULT, SYNC>
where
    SPI: SpiDevice,
    FAULT: InputPin,
    SYNC: OutputPin,
{
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        if !self.enabled {
            return;
        }

        let v_dc = self.params.voltage_power_supply;
        if v_dc <= 0.0 {
            return;
        }

        let v_limit = if self.params.voltage_limit > 0.0 && self.params.voltage_limit < v_dc {
            self.params.voltage_limit
        } else {
            v_dc
        };

        let ua_clamped = if ua < 0.0 { 0.0 } else if ua > v_limit { v_limit } else { ua };
        let ub_clamped = if ub < 0.0 { 0.0 } else if ub > v_limit { v_limit } else { ub };
        let uc_clamped = if uc < 0.0 { 0.0 } else if uc > v_limit { v_limit } else { uc };

        let period_f = self.period as f32;
        let duty_a = ((ua_clamped / v_dc) * period_f) as u16;
        let duty_b = ((ub_clamped / v_dc) * period_f) as u16;
        let duty_c = ((uc_clamped / v_dc) * period_f) as u16;

        let _ = self.drv.set_duties(duty_a, duty_b, duty_c);
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        let conv = |s| match s {
            PhaseState::PHASE_ON => 7u8,
            PhaseState::PHASE_OFF => 0u8,
            PhaseState::PHASE_HI => 2u8,
            PhaseState::PHASE_LO => 1u8,
        };
        let _ = self.drv.set_phase_states(conv(sa), conv(sb), conv(sc));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::registers::*;
    use std::vec;
    use barefoc::traits::adc::AdcPin;
    use barefoc::traits::current_sense::CurrentSense;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

    struct MockAdcPin {
        value: u16,
    }

    impl AdcPin for MockAdcPin {
        fn read(&mut self) -> u16 {
            self.value
        }
    }

    #[derive(Clone, Copy)]
    struct DummyClock;
    impl Clock for DummyClock {
        fn now_us(&self) -> u32 {
            0
        }
    }

    struct MockPwm {
        pub inited: bool,
        pub enabled: bool,
        pub params: FocDriverParams,
        pub pwm: [f32; 3],
        pub phase_states: [PhaseState; 3],
    }

    impl MockPwm {
        pub fn new() -> Self {
            Self {
                inited: false,
                enabled: false,
                params: FocDriverParams::new(),
                pwm: [0.0; 3],
                phase_states: [PhaseState::PHASE_ON; 3],
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
    fn test_drv8311h_bundle_pwm_and_sleep() {
        use embedded_hal_mock::eh1::pin::{
            Mock as PinMock, State as PinState, Transaction as PinTransaction,
        };

        let sleep_expectations = [
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::High),
        ];
        let sleep_pin = PinMock::new(&sleep_expectations);
        let mock_pwm = MockPwm::new();
        let mut bundle = Drv8311HDriver3PWM::new(mock_pwm)
            .with_sleep_pin(sleep_pin)
            .with_csa_gain(super::super::config::Drv8311CsaGain::Gain0V25);

        bundle.init().unwrap();
        bundle.enable();
        assert!(bundle.pwm_driver.enabled);

        bundle.set_pwm(1.0, 0.5, 0.0);
        assert_eq!(bundle.pwm_driver.pwm, [1.0, 0.5, 0.0]);

        bundle.disable();
        assert!(!bundle.pwm_driver.enabled);

        bundle.drv.sleep_pin.unwrap().done();
    }

    #[test]
    fn test_create_current_sense_csa_calibration() {
        let spi = SpiMock::<u8>::new(&[]);
        let drv = Drv8311S::new(spi).with_csa_gain(super::super::config::Drv8311CsaGain::Gain0V25);
        let pwm = MockPwm::new();
        let mut bundle = Drv8311Driver6PWM::new(drv, pwm);

        let pin_a = MockAdcPin { value: 1650 };
        let pin_b = MockAdcPin { value: 1900 };
        let pin_c = MockAdcPin { value: 1400 };

        let cs = bundle.create_current_sense::<_, _, _, DummyClock>(
            1650.0,
            pin_a,
            pin_b,
            Some(pin_c),
        );

        let base = cs.base();
        assert_eq!(base.offset_ia, 825.0);
        assert_eq!(base.offset_ib, 825.0);
        assert_eq!(base.offset_ic, 825.0);
        bundle.spi_device.spi.done();
    }

    #[test]
    fn test_pwm_gen_bundle_duty_updates() {
        use crate::spi::encode_tspi_frame;
        let tx_a = encode_tspi_frame(0, PWMG_A_DUTY_ADDR, 500, false, true).to_vec();
        let tx_b = encode_tspi_frame(0, PWMG_B_DUTY_ADDR, 250, false, true).to_vec();
        let tx_c = encode_tspi_frame(0, PWMG_C_DUTY_ADDR, 750, false, true).to_vec();

        let expectations = [
            // duty A (addr 0x19)
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(
                tx_a,
                vec![0x00, 0x00, 0x00, 0x00],
            ),
            SpiTransaction::transaction_end(),
            // duty B (addr 0x1A)
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(
                tx_b,
                vec![0x00, 0x00, 0x00, 0x00],
            ),
            SpiTransaction::transaction_end(),
            // duty C (addr 0x1B)
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(
                tx_c,
                vec![0x00, 0x00, 0x00, 0x00],
            ),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let drv = Drv8311P::new(spi, 0);
        let mut bundle = Drv8311DriverPwmGen::new(drv).with_period(1000);
        bundle.params.voltage_power_supply = 12.0;
        bundle.params.voltage_limit = 12.0;
        bundle.enabled = true;

        // 6.0V = 50% = 500 counts
        // 3.0V = 25% = 250 counts
        // 9.0V = 75% = 750 counts
        bundle.set_pwm(6.0, 3.0, 9.0);
        bundle.drv.spi.done();
    }
}
