//! # drv8316::config — strongly-typed configuration enums for DRV8316
//!
//! References upstream:
//! - `source/Arduino-FOC-drivers/src/drivers/drv8316/drv8316.h`

/// DRV8316 PWM Mode
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316PwmMode {
    #[default]
    Pwm6 = 0b00,
    Pwm6CurrentLimit = 0b01,
    Pwm3 = 0b10,
    Pwm3CurrentLimit = 0b11,
}

impl Drv8316PwmMode {
    pub const fn from_bits(val: u8) -> Self {
        match val & 0b11 {
            0b00 => Self::Pwm6,
            0b01 => Self::Pwm6CurrentLimit,
            0b10 => Self::Pwm3,
            _ => Self::Pwm3CurrentLimit,
        }
    }
}

/// Slew rate for MOSFET switching
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316SlewRate {
    Slew25Vus = 0b00,
    Slew50Vus = 0b01,
    #[default]
    Slew150Vus = 0b10,
    Slew200Vus = 0b11,
}

impl Drv8316SlewRate {
    pub const fn from_bits(val: u8) -> Self {
        match val & 0b11 {
            0b00 => Self::Slew25Vus,
            0b01 => Self::Slew50Vus,
            0b10 => Self::Slew150Vus,
            _ => Self::Slew200Vus,
        }
    }
}

/// SDO pin mode
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316SdoMode {
    #[default]
    OpenDrain = 0,
    PushPull = 1,
}

/// Supply overvoltage protection threshold
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316Ovp {
    #[default]
    Ovp32V = 0,
    Ovp20V = 1,
}

/// 100% duty cycle frequency
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316Pwm100Duty {
    #[default]
    Freq20kHz = 0,
    Freq40kHz = 1,
}

/// Overcurrent protection mode
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316OcpMode {
    #[default]
    LatchedFault = 0b00,
    AutoRetryFault = 0b01,
    ReportOnly = 0b10,
    NoAction = 0b11,
}

impl Drv8316OcpMode {
    pub const fn from_bits(val: u8) -> Self {
        match val & 0b11 {
            0b00 => Self::LatchedFault,
            0b01 => Self::AutoRetryFault,
            0b10 => Self::ReportOnly,
            _ => Self::NoAction,
        }
    }
}

/// Overcurrent protection current trip level
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316OcpLevel {
    #[default]
    Amps16 = 0,
    Amps24 = 1,
}

/// Overcurrent auto-retry time
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316OcpRetry {
    #[default]
    Retry5ms = 0,
    Retry500ms = 1,
}

/// Overcurrent deglitch filter time
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316OcpDeglitch {
    Deglitch0us2 = 0b00,
    #[default]
    Deglitch0us6 = 0b01,
    Deglitch1us1 = 0b10,
    Deglitch1us6 = 0b11,
}

impl Drv8316OcpDeglitch {
    pub const fn from_bits(val: u8) -> Self {
        match val & 0b11 {
            0b00 => Self::Deglitch0us2,
            0b01 => Self::Deglitch0us6,
            0b10 => Self::Deglitch1us1,
            _ => Self::Deglitch1us6,
        }
    }
}

/// Current Sense Amplifier (CSA) gain
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316CsaGain {
    #[default]
    Gain0V15 = 0b00,
    Gain0V1875 = 0b01,
    Gain0V25 = 0b10,
    Gain0V375 = 0b11,
}

impl Drv8316CsaGain {
    pub const fn from_bits(val: u8) -> Self {
        match val & 0b11 {
            0b00 => Self::Gain0V15,
            0b01 => Self::Gain0V1875,
            0b10 => Self::Gain0V25,
            _ => Self::Gain0V375,
        }
    }

    /// Internal sensitivity in Volts per Ampere (V/A)
    pub const fn v_per_a(&self) -> f32 {
        match self {
            Self::Gain0V15 => 0.15,
            Self::Gain0V1875 => 0.1875,
            Self::Gain0V25 => 0.25,
            Self::Gain0V375 => 0.375,
        }
    }

    /// Volts-to-Amps conversion factor in Amperes per Volt (A/V)
    pub fn a_per_v(&self) -> f32 {
        1.0 / self.v_per_a()
    }
}

/// Current recirculation mode when current limit is reached
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316Recirculation {
    #[default]
    BrakeMode = 0,
    CoastMode = 1,
}

/// Buck regulator output voltage setting
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316BuckVoltage {
    #[default]
    V3_3 = 0b00,
    V5_0 = 0b01,
    V4_0 = 0b10,
    V5_7 = 0b11,
}

impl Drv8316BuckVoltage {
    pub const fn from_bits(val: u8) -> Self {
        match val & 0b11 {
            0b00 => Self::V3_3,
            0b01 => Self::V5_0,
            0b10 => Self::V4_0,
            _ => Self::V5_7,
        }
    }
}

/// Buck regulator current limit
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316BuckCurrentLimit {
    #[default]
    Limit600mA = 0,
    Limit150mA = 1,
}

/// Propagation delay compensation target
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Drv8316DelayTarget {
    #[default]
    Delay0us = 0x0,
    Delay0us4 = 0x1,
    Delay0us6 = 0x2,
    Delay0us8 = 0x3,
    Delay1us = 0x4,
    Delay1us2 = 0x5,
    Delay1us4 = 0x6,
    Delay1us6 = 0x7,
    Delay1us8 = 0x8,
    Delay2us = 0x9,
    Delay2us2 = 0xA,
    Delay2us4 = 0xB,
    Delay2us6 = 0xC,
    Delay2us8 = 0xD,
    Delay3us = 0xE,
    Delay3us2 = 0xF,
}

impl Drv8316DelayTarget {
    pub const fn from_bits(val: u8) -> Self {
        match val & 0x0F {
            0x0 => Self::Delay0us,
            0x1 => Self::Delay0us4,
            0x2 => Self::Delay0us6,
            0x3 => Self::Delay0us8,
            0x4 => Self::Delay1us,
            0x5 => Self::Delay1us2,
            0x6 => Self::Delay1us4,
            0x7 => Self::Delay1us6,
            0x8 => Self::Delay1us8,
            0x9 => Self::Delay2us,
            0xA => Self::Delay2us2,
            0xB => Self::Delay2us4,
            0xC => Self::Delay2us6,
            0xD => Self::Delay2us8,
            0xE => Self::Delay3us,
            _ => Self::Delay3us2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_csa_gain_values() {
        assert!((Drv8316CsaGain::Gain0V15.v_per_a() - 0.15).abs() < 1e-6);
        assert!((Drv8316CsaGain::Gain0V1875.v_per_a() - 0.1875).abs() < 1e-6);
        assert!((Drv8316CsaGain::Gain0V25.v_per_a() - 0.25).abs() < 1e-6);
        assert!((Drv8316CsaGain::Gain0V375.v_per_a() - 0.375).abs() < 1e-6);

        assert!((Drv8316CsaGain::Gain0V15.a_per_v() - (1.0 / 0.15)).abs() < 1e-5);
        assert!((Drv8316CsaGain::Gain0V25.a_per_v() - 4.0).abs() < 1e-5);
    }
}
