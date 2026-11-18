//! # drv8311::config — Configuration types and strongly-typed enums
//!
//! References: Texas Instruments DRV8311 Datasheet (SLVSFN2B).

/// Current Sense Amplifier (CSA) gain options.
///
/// Configured in register `CSA_CTRL` (0x23) bits 1..0 or via hardware GAIN pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311CsaGain {
    /// 0.25 V/A gain setting (Bits = `00b`). Full-scale range up to ±6.6 A (for 3.3V VREF).
    Gain0V25 = 0,
    /// 0.5 V/A gain setting (Bits = `01b`). Full-scale range up to ±3.3 A.
    Gain0V5 = 1,
    /// 1.0 V/A gain setting (Bits = `10b`). Full-scale range up to ±1.65 A.
    Gain1V0 = 2,
    /// 2.0 V/A gain setting (Bits = `11b`). Full-scale range up to ±0.825 A.
    Gain2V0 = 3,
}

impl Drv8311CsaGain {
    /// Returns the CSA gain in Volts per Ampere (V/A).
    #[inline]
    pub const fn v_per_a(self) -> f32 {
        match self {
            Self::Gain0V25 => 0.25,
            Self::Gain0V5 => 0.5,
            Self::Gain1V0 => 1.0,
            Self::Gain2V0 => 2.0,
        }
    }

    /// Returns the CSA conversion factor in Amperes per Volt (A/V) ($1 / G_{CSA}$).
    #[inline]
    pub const fn a_per_v(self) -> f32 {
        match self {
            Self::Gain0V25 => 4.0,
            Self::Gain0V5 => 2.0,
            Self::Gain1V0 => 1.0,
            Self::Gain2V0 => 0.5,
        }
    }

    /// Convert from 2-bit register value.
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::Gain0V25,
            1 => Self::Gain0V5,
            2 => Self::Gain1V0,
            _ => Self::Gain2V0,
        }
    }
}

/// Slew rate settings for output stage MOSFETs.
///
/// Configured in `DRV_CTRL` (0x22) bits 1..0.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311SlewRate {
    /// 35 V/µs (Bits = `00b`)
    Slew35Vus = 0,
    /// 75 V/µs (Bits = `01b`)
    Slew75Vus = 1,
    /// 180 V/µs (Bits = `10b`)
    Slew180Vus = 2,
    /// 230 V/µs (Bits = `11b`)
    Slew230Vus = 3,
}

impl Drv8311SlewRate {
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::Slew35Vus,
            1 => Self::Slew75Vus,
            2 => Self::Slew180Vus,
            _ => Self::Slew230Vus,
        }
    }
}

/// Dead time insertion control for half-bridges.
///
/// Configured in `DRV_CTRL` (0x22) bits 6..4.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311DeadTime {
    /// No dead time (Handshake only)
    DeadTime0ns = 0,
    /// 200 ns
    DeadTime200ns = 1,
    /// 400 ns
    DeadTime400ns = 2,
    /// 600 ns
    DeadTime600ns = 3,
    /// 800 ns
    DeadTime800ns = 4,
    /// 1000 ns (1.0 µs)
    DeadTime1000ns = 5,
    /// 1200 ns (1.2 µs)
    DeadTime1200ns = 6,
    /// 1400 ns (1.4 µs)
    DeadTime1400ns = 7,
}

impl Drv8311DeadTime {
    #[inline]
    pub const fn nanoseconds(self) -> u16 {
        match self {
            Self::DeadTime0ns => 0,
            Self::DeadTime200ns => 200,
            Self::DeadTime400ns => 400,
            Self::DeadTime600ns => 600,
            Self::DeadTime800ns => 800,
            Self::DeadTime1000ns => 1000,
            Self::DeadTime1200ns => 1200,
            Self::DeadTime1400ns => 1400,
        }
    }

    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x07 {
            0 => Self::DeadTime0ns,
            1 => Self::DeadTime200ns,
            2 => Self::DeadTime400ns,
            3 => Self::DeadTime600ns,
            4 => Self::DeadTime800ns,
            5 => Self::DeadTime1000ns,
            6 => Self::DeadTime1200ns,
            _ => Self::DeadTime1400ns,
        }
    }
}

/// PWM control mode.
///
/// Configured in `PWM_CTRL1` (0x20) bits 1..0.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311PwmMode {
    /// 6x PWM mode (Bits = `00b` or `01b`). Independent control of 3 half-bridges (6 inputs).
    Pwm6x = 0,
    /// 3x PWM mode (Bits = `10b`). Internal complementary PWM with dead time from 3 inputs.
    Pwm3x = 2,
    /// PWM Generation mode (Bits = `11b`). Internal hardware PWM counter driven via SPI registers.
    PwmGen = 3,
}

impl Drv8311PwmMode {
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 | 1 => Self::Pwm6x,
            2 => Self::Pwm3x,
            _ => Self::PwmGen,
        }
    }
}

/// Overcurrent protection (OCP) response mode.
///
/// Configured in `FLT_MODE` (0x10) bits 6..4.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311OcpMode {
    /// Automatic retry with slow retry time (Bits = `000b`)
    AutoRetrySlow = 0,
    /// Automatic retry with fast retry time (Bits = `001b`)
    AutoRetryFast = 1,
    /// Latched shutdown until cleared (Bits = `010b`)
    LatchedShutdown = 2,
    /// Report only on nFAULT / status (Bits = `011b`)
    ReportOnly = 3,
    /// OCP disabled (Bits = `111b`)
    Disabled = 7,
}

impl Drv8311OcpMode {
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x07 {
            0 => Self::AutoRetrySlow,
            1 => Self::AutoRetryFast,
            2 => Self::LatchedShutdown,
            3 => Self::ReportOnly,
            _ => Self::Disabled,
        }
    }
}

/// Undervoltage protection (UVP) response mode.
///
/// Configured in `FLT_MODE` (0x10) bits 3..2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311UvpMode {
    /// Automatic retry slow (Bits = `00b`)
    AutoRetrySlow = 0,
    /// Automatic retry fast (Bits = `01b`)
    AutoRetryFast = 1,
    /// Latched shutdown (Bits = `10b`)
    LatchedShutdown = 2,
    /// Report only (Bits = `11b`)
    ReportOnly = 3,
}

impl Drv8311UvpMode {
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::AutoRetrySlow,
            1 => Self::AutoRetryFast,
            2 => Self::LatchedShutdown,
            _ => Self::ReportOnly,
        }
    }
}

/// Thermal shutdown (OTSD) response mode.
///
/// Configured in `FLT_MODE` (0x10) bits 1..0.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311OtsdMode {
    /// Automatic retry slow (Bits = `00b`)
    AutoRetrySlow = 0,
    /// Automatic retry fast (Bits = `01b`)
    AutoRetryFast = 1,
    /// Latched shutdown (Bits = `10b`)
    LatchedShutdown = 2,
    /// Report only (Bits = `11b`)
    ReportOnly = 3,
}

impl Drv8311OtsdMode {
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::AutoRetrySlow,
            1 => Self::AutoRetryFast,
            2 => Self::LatchedShutdown,
            _ => Self::ReportOnly,
        }
    }
}

/// Individual Phase Output State.
///
/// Configured in `PWM_STATE` (0x1C) for each phase A, B, C.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311PhaseState {
    /// High-side OFF, Low-side OFF (Hi-Z output)
    Off = 0,
    /// High-side OFF, Low-side forced ON (Low-side braking / discharge)
    LowForced = 1,
    /// High-side forced ON, Low-side OFF
    HighForced = 2,
    /// High-side PWM, Low-side complementary (with dead time)
    Complementary = 5,
    /// High-side PWM, Low-side OFF
    HighPwmLowOff = 6,
    /// Normal PWM mode (controlled by PWM generator or input pins)
    Pwm = 7,
}

impl Drv8311PhaseState {
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x07 {
            0 => Self::Off,
            1 => Self::LowForced,
            2 => Self::HighForced,
            5 => Self::Complementary,
            6 => Self::HighPwmLowOff,
            _ => Self::Pwm,
        }
    }
}

/// PWM Generator Counter Mode.
///
/// Configured in `PWMG_CTRL` (0x1D) bits 9..8.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311CounterMode {
    /// Center-aligned PWM (Up and Down counter)
    CenterAligned = 0,
    /// Edge-aligned up PWM (Up counter)
    EdgeAlignedUp = 1,
    /// Edge-aligned down PWM (Down counter)
    EdgeAlignedDown = 2,
}

impl Drv8311CounterMode {
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::CenterAligned,
            1 => Self::EdgeAlignedUp,
            _ => Self::EdgeAlignedDown,
        }
    }
}

/// PWM Oscillator and PWM_SYNC Synchronization.
///
/// Configured in `PWMG_CTRL` (0x1D) bits 7..5.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drv8311PwmSync {
    /// Oscillator synchronization is disabled
    Disabled = 0,
    /// PWM_SYNC_PRD indicates period of PWM_SYNC signal and can calibrate PWM period
    MeasurePeriod = 1,
    /// PWM_SYNC pin used to directly set PWM period
    SetPeriod = 2,
    /// PWM_SYNC used for oscillator synchronization (20 kHz frequency)
    Sync20k = 5,
    /// PWM_SYNC used for oscillator synchronization and PWM_SYNC_PRD indicates period
    SyncAndMeasure = 6,
}

impl Drv8311PwmSync {
    #[inline]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x07 {
            0 => Self::Disabled,
            1 => Self::MeasurePeriod,
            2 => Self::SetPeriod,
            5 => Self::Sync20k,
            6 => Self::SyncAndMeasure,
            _ => Self::Disabled,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_csa_gain_values() {
        assert_eq!(Drv8311CsaGain::Gain0V25.v_per_a(), 0.25);
        assert_eq!(Drv8311CsaGain::Gain0V25.a_per_v(), 4.0);

        assert_eq!(Drv8311CsaGain::Gain0V5.v_per_a(), 0.5);
        assert_eq!(Drv8311CsaGain::Gain0V5.a_per_v(), 2.0);

        assert_eq!(Drv8311CsaGain::Gain1V0.v_per_a(), 1.0);
        assert_eq!(Drv8311CsaGain::Gain1V0.a_per_v(), 1.0);

        assert_eq!(Drv8311CsaGain::Gain2V0.v_per_a(), 2.0);
        assert_eq!(Drv8311CsaGain::Gain2V0.a_per_v(), 0.5);
    }

    #[test]
    fn test_dead_time_values() {
        assert_eq!(Drv8311DeadTime::DeadTime0ns.nanoseconds(), 0);
        assert_eq!(Drv8311DeadTime::DeadTime200ns.nanoseconds(), 200);
        assert_eq!(Drv8311DeadTime::DeadTime1000ns.nanoseconds(), 1000);
        assert_eq!(Drv8311DeadTime::DeadTime1400ns.nanoseconds(), 1400);
    }
}
