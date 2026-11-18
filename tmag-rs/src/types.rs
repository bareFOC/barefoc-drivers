//! # types — Domain Types and Enums for TMAG5273
//!
//! Strongly typed representations of operating modes, magnetic channels, averaging,
//! ranges, device variants, and diagnostic status per TI TMAG5273 specifications.

use crate::registers::{
    extract, DIAG_STATUS_MASK, DIAG_STATUS_SHIFT, INTB_RB_MASK, INTB_RB_SHIFT, INT_ER_MASK,
    INT_ER_SHIFT, OSC_ER_MASK, OSC_ER_SHIFT, OTP_CRC_ER_MASK, OTP_CRC_ER_SHIFT, POR_MASK,
    POR_SHIFT, RESULT_STATUS_MASK, RESULT_STATUS_SHIFT, SET_COUNT_MASK, SET_COUNT_SHIFT,
    VCC_UV_ER_MASK, VCC_UV_ER_SHIFT, VER_V1, VER_V2,
};

/// I2C Read Mode configuration (`I2C_RD`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum I2cReadMode {
    /// Standard 3-byte read transaction (`[ADDR_W, REG]`, `[ADDR_R, DATA...]`).
    #[default]
    Standard = 0x00,
    /// 1-byte fast read mode returning 16-bit sensor data + `CONV_STATUS`.
    OneByte16Bit = 0x01,
    /// 1-byte fast read mode returning 8-bit MSB sensor data + `CONV_STATUS`.
    OneByte8Bit = 0x02,
}

impl TryFrom<u8> for I2cReadMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(Self::Standard),
            0x01 => Ok(Self::OneByte16Bit),
            0x02 => Ok(Self::OneByte8Bit),
            _ => Err(()),
        }
    }
}

/// Device operating mode (`OPERATING_MODE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum OperatingMode {
    /// Stand-by mode: conversions triggered on demand via I2C or INT pin.
    #[default]
    Standby = 0x00,
    /// Sleep mode: ultra-low power state with I2C logic active.
    Sleep = 0x01,
    /// Continuous conversion mode up to 20 kSPS.
    ContinuousMeasure = 0x02,
    /// Wake-up and sleep mode: periodic conversions with threshold monitoring.
    WakeUpAndSleep = 0x03,
}

impl TryFrom<u8> for OperatingMode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(Self::Standby),
            0x01 => Ok(Self::Sleep),
            0x02 => Ok(Self::ContinuousMeasure),
            0x03 => Ok(Self::WakeUpAndSleep),
            _ => Err(()),
        }
    }
}

/// Magnetic channel selection (`MAG_CH_EN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MagneticChannel {
    /// All channels disabled.
    Off = 0x0,
    /// X axis only.
    X = 0x1,
    /// Y axis only.
    Y = 0x2,
    /// X and Y axes.
    XY = 0x3,
    /// Z axis only.
    Z = 0x4,
    /// Z and X axes.
    ZX = 0x5,
    /// Y and Z axes.
    YZ = 0x6,
    /// X, Y, and Z axes (3D linear Hall).
    #[default]
    XYZ = 0x7,
    /// Pseudo-simultaneous X-Y-X channel sampling for low phase lag.
    XYX = 0x8,
    /// Pseudo-simultaneous Y-X-Y channel sampling.
    YXY = 0x9,
    /// Pseudo-simultaneous Y-Z-Y channel sampling.
    YZY = 0xA,
    /// Pseudo-simultaneous X-Z-X channel sampling.
    XZX = 0xB,
}

impl TryFrom<u8> for MagneticChannel {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x0 => Ok(Self::Off),
            0x1 => Ok(Self::X),
            0x2 => Ok(Self::Y),
            0x3 => Ok(Self::XY),
            0x4 => Ok(Self::Z),
            0x5 => Ok(Self::ZX),
            0x6 => Ok(Self::YZ),
            0x7 => Ok(Self::XYZ),
            0x8 => Ok(Self::XYX),
            0x9 => Ok(Self::YXY),
            0xA => Ok(Self::YZY),
            0xB => Ok(Self::XZX),
            _ => Err(()),
        }
    }
}

/// Conversion averaging filter count (`CONV_AVG`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ConversionAverage {
    /// 1x averaging (fastest, up to 20 kSPS).
    #[default]
    Avg1x = 0x0,
    /// 2x averaging.
    Avg2x = 0x1,
    /// 4x averaging.
    Avg4x = 0x2,
    /// 8x averaging.
    Avg8x = 0x3,
    /// 16x averaging.
    Avg16x = 0x4,
    /// 32x averaging (lowest noise).
    Avg32x = 0x5,
}

impl TryFrom<u8> for ConversionAverage {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x0 => Ok(Self::Avg1x),
            0x1 => Ok(Self::Avg2x),
            0x2 => Ok(Self::Avg4x),
            0x3 => Ok(Self::Avg8x),
            0x4 => Ok(Self::Avg16x),
            0x5 => Ok(Self::Avg32x),
            _ => Err(()),
        }
    }
}

/// Magnetic measurement range (`X_Y_RANGE` and `Z_RANGE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Range {
    /// Low range (±40 mT on Version 1, ±133 mT on Version 2).
    Low = 0,
    /// High range (±80 mT on Version 1, ±266 mT on Version 2).
    #[default]
    High = 1,
}

impl TryFrom<u8> for Range {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Self::Low),
            1 => Ok(Self::High),
            _ => Err(()),
        }
    }
}

/// TMAG5273 device variant identifying factory I2C address and magnetic sensitivity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceVariant {
    /// TMAG5273A1: factory address `0x35`, version 1 (±40/80 mT).
    A1,
    /// TMAG5273A2: factory address `0x35`, version 2 (±133/266 mT).
    A2,
    /// TMAG5273B1: factory address `0x22`, version 1 (±40/80 mT).
    B1,
    /// TMAG5273B2: factory address `0x22`, version 2 (±133/266 mT).
    B2,
    /// TMAG5273C1: factory address `0x78`, version 1 (±40/80 mT).
    C1,
    /// TMAG5273C2: factory address `0x78`, version 2 (±133/266 mT).
    C2,
    /// TMAG5273D1: factory address `0x44`, version 1 (±40/80 mT).
    D1,
    /// TMAG5273D2: factory address `0x44`, version 2 (±133/266 mT).
    D2,
}

impl DeviceVariant {
    /// Factory default 7-bit I2C address.
    pub const fn default_address(self) -> u8 {
        match self {
            Self::A1 | Self::A2 => 0x35,
            Self::B1 | Self::B2 => 0x22,
            Self::C1 | Self::C2 => 0x78,
            Self::D1 | Self::D2 => 0x44,
        }
    }

    /// Expected silicon version in `DEVICE_ID` register (0x0D).
    pub const fn version(self) -> u8 {
        match self {
            Self::A1 | Self::B1 | Self::C1 | Self::D1 => VER_V1,
            Self::A2 | Self::B2 | Self::C2 | Self::D2 => VER_V2,
        }
    }

    /// Magnetic range magnitude in milliTesla (mT) for the given range setting.
    pub const fn range_value(self, range: Range) -> f32 {
        match (self.version(), range) {
            (VER_V1, Range::Low) => 40.0,
            (VER_V1, Range::High) => 80.0,
            (VER_V2, Range::Low) => 133.0,
            (VER_V2, Range::High) => 266.0,
            _ => 80.0,
        }
    }
}

/// Internal CORDIC angle engine plane selection (`ANGLE_EN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AngleEnable {
    /// CORDIC engine disabled.
    #[default]
    None = 0x0,
    /// Angle calculated from X and Y axes.
    XY = 0x1,
    /// Angle calculated from Y and Z axes.
    YZ = 0x2,
    /// Angle calculated from X and Z axes.
    XZ = 0x3,
}

impl TryFrom<u8> for AngleEnable {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x0 => Ok(Self::None),
            0x1 => Ok(Self::XY),
            0x2 => Ok(Self::YZ),
            0x3 => Ok(Self::XZ),
            _ => Err(()),
        }
    }
}

/// Power vs noise optimization mode (`LP_LN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PowerNoiseMode {
    /// Low power mode.
    #[default]
    LowPower = 0,
    /// Low noise mode (higher bandwidth and SNR).
    LowNoise = 1,
}

/// Wake-and-Sleep sleep duration (`SLEEPTIME`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SleepTime {
    /// 1 ms sleep.
    #[default]
    Ms1 = 0x0,
    /// 5 ms sleep.
    Ms5 = 0x1,
    /// 10 ms sleep.
    Ms10 = 0x2,
    /// 15 ms sleep.
    Ms15 = 0x3,
    /// 20 ms sleep.
    Ms20 = 0x4,
    /// 30 ms sleep.
    Ms30 = 0x5,
    /// 50 ms sleep.
    Ms50 = 0x6,
    /// 100 ms sleep.
    Ms100 = 0x7,
    /// 500 ms sleep.
    Ms500 = 0x8,
    /// 1000 ms sleep.
    Ms1000 = 0x9,
    /// 2000 ms sleep.
    Ms2000 = 0xA,
    /// 5000 ms sleep.
    Ms5000 = 0xB,
    /// 20000 ms sleep.
    Ms20000 = 0xC,
}

/// Conversion trigger source (`TRIGGER_MODE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TriggerMode {
    /// Conversion triggered via I2C command.
    #[default]
    I2cCommand = 0,
    /// Conversion triggered via INT pin.
    IntPin = 1,
}

/// Magnetic temperature compensation coefficient (`MAG_TEMPCO`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MagneticTempCoefficient {
    /// No temperature compensation.
    #[default]
    None = 0,
    /// NdFeB magnet compensation (0.12%/°C).
    NdBFe = 1,
    /// Ceramic magnet compensation (0.2%/°C).
    Ceramic = 2,
}

/// Decoded status flags from `CONV_STATUS` register (0x18).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ConversionStatus {
    /// New conversion result available (`RESULT_STATUS`).
    pub result_ready: bool,
    /// Diagnostic failure detected (`DIAG_STATUS`).
    pub diag_status: bool,
    /// Power-on-reset occurred (`POR`).
    pub por_occurred: bool,
    /// 3-bit rolling counter of conversion sets (`SET_COUNT`).
    pub set_count: u8,
}

impl ConversionStatus {
    /// Decode from raw register byte.
    pub fn from_raw(reg: u8) -> Self {
        Self {
            result_ready: extract(reg, RESULT_STATUS_MASK, RESULT_STATUS_SHIFT) != 0,
            diag_status: extract(reg, DIAG_STATUS_MASK, DIAG_STATUS_SHIFT) != 0,
            por_occurred: extract(reg, POR_MASK, POR_SHIFT) != 0,
            set_count: extract(reg, SET_COUNT_MASK, SET_COUNT_SHIFT),
        }
    }
}

/// Decoded device status flags from `DEVICE_STATUS` register (0x1C).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceStatus {
    /// VCC undervoltage detected (`VCC_UV_ER`).
    pub vcc_undervoltage: bool,
    /// OTP memory CRC error (`OTP_CRC_ER`).
    pub otp_crc_error: bool,
    /// INT pin error (`INT_ER`).
    pub int_pin_error: bool,
    /// Internal oscillator error (`OSC_ER`).
    pub osc_error: bool,
    /// INTB pin level readback (1 = inactive / high).
    pub int_pin_readback: bool,
}

impl DeviceStatus {
    /// Decode from raw register byte.
    pub fn from_raw(reg: u8) -> Self {
        Self {
            vcc_undervoltage: extract(reg, VCC_UV_ER_MASK, VCC_UV_ER_SHIFT) != 0,
            otp_crc_error: extract(reg, OTP_CRC_ER_MASK, OTP_CRC_ER_SHIFT) != 0,
            int_pin_error: extract(reg, INT_ER_MASK, INT_ER_SHIFT) != 0,
            osc_error: extract(reg, OSC_ER_MASK, OSC_ER_SHIFT) != 0,
            int_pin_readback: extract(reg, INTB_RB_MASK, INTB_RB_SHIFT) != 0,
        }
    }

    /// Returns `true` if any hardware fault flag is set.
    pub fn has_fault(&self) -> bool {
        self.vcc_undervoltage || self.otp_crc_error || self.int_pin_error || self.osc_error
    }
}

/// Validated configuration for TMAG5273 device setup.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Operating mode.
    pub operating_mode: OperatingMode,
    /// Magnetic channel selection.
    pub channels: MagneticChannel,
    /// Averaging rate.
    pub averaging: ConversionAverage,
    /// CORDIC angle calculation enable.
    pub angle_enabled: AngleEnable,
    /// X and Y magnetic range.
    pub xy_range: Range,
    /// Z magnetic range.
    pub z_range: Range,
    /// Power / noise profile.
    pub power_noise: PowerNoiseMode,
    /// I2C glitch filter enable.
    pub glitch_filter: bool,
    /// Temperature channel enable.
    pub temp_enabled: bool,
    /// I2C read mode.
    pub read_mode: I2cReadMode,
    /// CRC enable on I2C transactions.
    pub crc_enabled: bool,
    /// Device variant.
    pub variant: DeviceVariant,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            operating_mode: OperatingMode::ContinuousMeasure,
            channels: MagneticChannel::XYZ,
            averaging: ConversionAverage::Avg1x,
            angle_enabled: AngleEnable::None,
            xy_range: Range::High,
            z_range: Range::High,
            power_noise: PowerNoiseMode::LowNoise,
            glitch_filter: true,
            temp_enabled: false,
            read_mode: I2cReadMode::Standard,
            crc_enabled: false,
            variant: DeviceVariant::A1,
        }
    }
}

/// Builder for constructing [`Config`].
#[derive(Debug, Clone, Copy, Default)]
pub struct ConfigBuilder {
    config: Config,
}

impl ConfigBuilder {
    /// Create a new configuration builder with defaults.
    pub const fn new() -> Self {
        Self {
            config: Config {
                operating_mode: OperatingMode::ContinuousMeasure,
                channels: MagneticChannel::XYZ,
                averaging: ConversionAverage::Avg1x,
                angle_enabled: AngleEnable::None,
                xy_range: Range::High,
                z_range: Range::High,
                power_noise: PowerNoiseMode::LowNoise,
                glitch_filter: true,
                temp_enabled: false,
                read_mode: I2cReadMode::Standard,
                crc_enabled: false,
                variant: DeviceVariant::A1,
            },
        }
    }

    /// Set operating mode.
    pub const fn operating_mode(mut self, mode: OperatingMode) -> Self {
        self.config.operating_mode = mode;
        self
    }

    /// Set magnetic channels.
    pub const fn channels(mut self, channels: MagneticChannel) -> Self {
        self.config.channels = channels;
        self
    }

    /// Set conversion averaging.
    pub const fn averaging(mut self, avg: ConversionAverage) -> Self {
        self.config.averaging = avg;
        self
    }

    /// Set CORDIC angle calculation mode.
    pub const fn angle_enabled(mut self, angle: AngleEnable) -> Self {
        self.config.angle_enabled = angle;
        self
    }

    /// Set X and Y range.
    pub const fn xy_range(mut self, range: Range) -> Self {
        self.config.xy_range = range;
        self
    }

    /// Set Z range.
    pub const fn z_range(mut self, range: Range) -> Self {
        self.config.z_range = range;
        self
    }

    /// Set power vs noise mode.
    pub const fn power_noise(mut self, mode: PowerNoiseMode) -> Self {
        self.config.power_noise = mode;
        self
    }

    /// Set I2C glitch filter.
    pub const fn glitch_filter(mut self, enabled: bool) -> Self {
        self.config.glitch_filter = enabled;
        self
    }

    /// Set temperature channel enable.
    pub const fn temp_enabled(mut self, enabled: bool) -> Self {
        self.config.temp_enabled = enabled;
        self
    }

    /// Set I2C read mode.
    pub const fn read_mode(mut self, mode: I2cReadMode) -> Self {
        self.config.read_mode = mode;
        self
    }

    /// Set CRC enable.
    pub const fn crc_enabled(mut self, enabled: bool) -> Self {
        self.config.crc_enabled = enabled;
        self
    }

    /// Set device variant.
    pub const fn variant(mut self, variant: DeviceVariant) -> Self {
        self.config.variant = variant;
        self
    }

    /// Build the validated configuration.
    pub fn build(self) -> Config {
        self.config
    }
}
