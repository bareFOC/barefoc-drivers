//! # comms::registers — Universal SimpleFOC Register Map and Serialization
//!
//! Ports [`SimpleFOCRegisters.h`](source/Arduino-FOC-drivers/src/comms/SimpleFOCRegisters.h)
//! and [`SimpleFOCRegisters.cpp`](source/Arduino-FOC-drivers/src/comms/SimpleFOCRegisters.cpp)
//! from Arduino-FOC-drivers.
//!
//! Provides the canonical register enumeration (`RegisterId`), payload enumeration
//! (`RegisterValue`), and `#![no_std]`, zero-allocation binary serialization routines.

use barefoc::traits::driver::PhaseState;

/// Protocol version identifier matching upstream `SIMPLEFOC_REGISTERS_VERSION`.
pub const SIMPLEFOC_REGISTERS_VERSION: u8 = 0x01;

/// Register access permission modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessMode {
    /// Read-only register (cannot be written).
    ReadOnly,
    /// Write-only register (cannot be read).
    WriteOnly,
    /// Read/Write register.
    ReadWrite,
}

/// Errors encountered when working with SimpleFOC registers and binary payloads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegisterError {
    /// Attempted to access an unknown or unmapped register ID.
    UnknownRegister(u8),
    /// Attempted to write to a read-only register.
    ReadOnly(RegisterId),
    /// Attempted to read from a write-only register.
    WriteOnly(RegisterId),
    /// The provided byte buffer was smaller than expected for the register payload.
    BufferTooSmall {
        /// Expected minimum buffer size in bytes.
        expected: usize,
        /// Actual buffer size in bytes.
        actual: usize,
    },
    /// The byte payload was invalid or could not be mapped to the target type.
    InvalidPayload,
}

/// SimpleFOC universal register identifiers matching upstream 1:1 (`0x00`..`0x7F`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum RegisterId {
    /// Motor status (lifecycle state machine) - RO (1 byte).
    Status = 0x00,
    /// Target setpoint (angle, velocity, or torque) - R/W (float, 4 bytes).
    Target = 0x01,
    /// Global enable command for all motors - WO (1 byte).
    EnableAll = 0x03,
    /// Motor enabled state - R/W (1 byte).
    Enable = 0x04,
    /// Motion control mode - R/W (1 byte).
    ControlMode = 0x05,
    /// Torque control mode - R/W (1 byte).
    TorqueMode = 0x06,
    /// PWM modulation strategy - R/W (1 byte).
    ModulationMode = 0x07,
    /// Current shaft angle - RO (float, 4 bytes).
    Angle = 0x09,
    /// Shaft position (rotations + angle in radians) - RO (8 bytes).
    Position = 0x10,
    /// Shaft velocity in rad/s - RO (float, 4 bytes).
    Velocity = 0x11,
    /// Raw sensor angle in radians - RO (float, 4 bytes).
    SensorAngle = 0x12,
    /// Mechanical sensor angle in radians - RO (float, 4 bytes).
    SensorMechanicalAngle = 0x13,
    /// Sensor velocity in rad/s - RO (float, 4 bytes).
    SensorVelocity = 0x14,
    /// Sensor measurement timestamp (microseconds) - RO (u32, 4 bytes).
    SensorTimestamp = 0x15,
    /// Three-phase voltages (Ua, Ub, Uc) - R/W (12 bytes).
    PhaseVoltage = 0x16,
    /// Three-phase drive states - R/W (3 bytes).
    PhaseState = 0x17,
    /// Driver stage enable flag - WO (1 byte).
    DriverEnable = 0x18,
    /// Telemetry configuration register - R/W (dynamic length).
    TelemetryReg = 0x1A,
    /// Selected telemetry channel index - R/W (1 byte).
    TelemetryCtrl = 0x1B,
    /// Telemetry downsample rate - R/W (u32, 4 bytes).
    TelemetryDownsample = 0x1C,
    /// Control loop iterations per second - RO (u32, 4 bytes).
    IterationsSec = 0x1D,
    /// Q-axis voltage - RO (float, 4 bytes).
    VoltageQ = 0x20,
    /// D-axis voltage - RO (float, 4 bytes).
    VoltageD = 0x21,
    /// Q-axis current - RO (float, 4 bytes).
    CurrentQ = 0x22,
    /// D-axis current - RO (float, 4 bytes).
    CurrentD = 0x23,
    /// Phase A current - RO (float, 4 bytes).
    CurrentA = 0x24,
    /// Phase B current - RO (float, 4 bytes).
    CurrentB = 0x25,
    /// Phase C current - RO (float, 4 bytes).
    CurrentC = 0x26,
    /// Three-phase currents (Ia, Ib, Ic) - RO (12 bytes).
    CurrentABC = 0x27,
    /// DC bus current - RO (float, 4 bytes).
    CurrentDC = 0x28,
    /// Velocity PID proportional gain - R/W (float, 4 bytes).
    VelPidP = 0x30,
    /// Velocity PID integral gain - R/W (float, 4 bytes).
    VelPidI = 0x31,
    /// Velocity PID derivative gain - R/W (float, 4 bytes).
    VelPidD = 0x32,
    /// Velocity PID output limit - R/W (float, 4 bytes).
    VelPidLim = 0x33,
    /// Velocity PID output ramp limit - R/W (float, 4 bytes).
    VelPidRamp = 0x34,
    /// Velocity low-pass filter time constant Tf - R/W (float, 4 bytes).
    VelLpfT = 0x35,
    /// Angle PID proportional gain - R/W (float, 4 bytes).
    AngPidP = 0x36,
    /// Angle PID integral gain - R/W (float, 4 bytes).
    AngPidI = 0x37,
    /// Angle PID derivative gain - R/W (float, 4 bytes).
    AngPidD = 0x38,
    /// Angle PID output limit - R/W (float, 4 bytes).
    AngPidLim = 0x39,
    /// Angle PID output ramp limit - R/W (float, 4 bytes).
    AngPidRamp = 0x3A,
    /// Angle low-pass filter time constant Tf - R/W (float, 4 bytes).
    AngLpfT = 0x3B,
    /// Q-axis current PID proportional gain - R/W (float, 4 bytes).
    CurQPidP = 0x40,
    /// Q-axis current PID integral gain - R/W (float, 4 bytes).
    CurQPidI = 0x41,
    /// Q-axis current PID derivative gain - R/W (float, 4 bytes).
    CurQPidD = 0x42,
    /// Q-axis current PID output limit - R/W (float, 4 bytes).
    CurQPidLim = 0x43,
    /// Q-axis current PID output ramp limit - R/W (float, 4 bytes).
    CurQPidRamp = 0x44,
    /// Q-axis current low-pass filter time constant Tf - R/W (float, 4 bytes).
    CurQLpfT = 0x45,
    /// D-axis current PID proportional gain - R/W (float, 4 bytes).
    CurDPidP = 0x46,
    /// D-axis current PID integral gain - R/W (float, 4 bytes).
    CurDPidI = 0x47,
    /// D-axis current PID derivative gain - R/W (float, 4 bytes).
    CurDPidD = 0x48,
    /// D-axis current PID output limit - R/W (float, 4 bytes).
    CurDPidLim = 0x49,
    /// D-axis current PID output ramp limit - R/W (float, 4 bytes).
    CurDPidRamp = 0x4A,
    /// D-axis current low-pass filter time constant Tf - R/W (float, 4 bytes).
    CurDLpfT = 0x4B,
    /// Global voltage limit - R/W (float, 4 bytes).
    VoltageLimit = 0x50,
    /// Global current limit - R/W (float, 4 bytes).
    CurrentLimit = 0x51,
    /// Global velocity limit - R/W (float, 4 bytes).
    VelocityLimit = 0x52,
    /// Driver voltage limit - R/W (float, 4 bytes).
    DriverVoltageLimit = 0x53,
    /// Driver PWM switching frequency - R/W (u32, 4 bytes).
    PwmFrequency = 0x54,
    /// Driver power supply voltage - R/W (float, 4 bytes).
    DriverVoltagePsu = 0x55,
    /// Sensor alignment voltage - R/W (float, 4 bytes).
    VoltageSensorAlign = 0x56,
    /// Motion control loop decimation - R/W (1 byte).
    MotionDownsample = 0x5F,
    /// Absolute zero electric angle - R/W (float, 4 bytes).
    ZeroElectricAngle = 0x60,
    /// Sensor rotation direction - R/W (1 byte).
    SensorDirection = 0x61,
    /// Sensor zero offset in radians - R/W (float, 4 bytes).
    ZeroOffset = 0x62,
    /// Number of motor pole pairs - R/W (1 byte).
    PolePairs = 0x63,
    /// Motor phase resistance in ohms - R/W (float, 4 bytes).
    PhaseResistance = 0x64,
    /// Motor KV rating (rpm/V) - R/W (float, 4 bytes).
    KV = 0x65,
    /// Motor phase inductance in henries - R/W (float, 4 bytes).
    Inductance = 0x66,
    /// Phase A current sense gain - R/W (float, 4 bytes).
    CurAGain = 0x67,
    /// Phase B current sense gain - R/W (float, 4 bytes).
    CurBGain = 0x68,
    /// Phase C current sense gain - R/W (float, 4 bytes).
    CurCGain = 0x69,
    /// Phase A current sense offset - R/W (float, 4 bytes).
    CurAOffset = 0x6A,
    /// Phase B current sense offset - R/W (float, 4 bytes).
    CurBOffset = 0x6B,
    /// Phase C current sense offset - R/W (float, 4 bytes).
    CurCOffset = 0x6C,
    /// Number of motors attached - RO (1 byte).
    NumMotors = 0x70,
    /// System uptime / timestamp (milliseconds) - RO (u32, 4 bytes).
    SysTime = 0x71,
    /// Motor communication address - R/W (1 byte).
    MotorAddress = 0x7F,
    /// Kinematics: gear ratio - R/W (float, 4 bytes).
    GearRatio = 0x80,
    /// Kinematics: backlash delta - R/W (float, 4 bytes).
    BacklashDelta = 0x81,
    /// Kinematics: omega threshold - R/W (float, 4 bytes).
    OmegaThresh = 0x82,
    /// Kinematics: torsional stiffness - R/W (float, 4 bytes).
    TorsionalStiffness = 0x83,
    /// Kinematics: zero offset (joint) - R/W (float, 4 bytes).
    ZeroOffsetJoint = 0x84,
    /// Kinematics: outer PID P - R/W (float, 4 bytes).
    OuterPidP = 0x85,
    /// Kinematics: outer PID I - R/W (float, 4 bytes).
    OuterPidI = 0x86,
    /// Kinematics: outer PID D - R/W (float, 4 bytes).
    OuterPidD = 0x87,
    /// Kinematics: outer PID limit - R/W (float, 4 bytes).
    OuterPidLim = 0x88,
    /// Kinematics: outer PID ramp - R/W (float, 4 bytes).
    OuterPidRamp = 0x89,
    /// Kinematics: calibration state - R/O (1 byte).
    CalibState = 0x8A,
    /// Kinematics: calibration progress - R/O (1 byte).
    CalibProgress = 0x8B,
}

impl TryFrom<u8> for RegisterId {
    type Error = RegisterError;

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0x00 => Ok(RegisterId::Status),
            0x01 => Ok(RegisterId::Target),
            0x03 => Ok(RegisterId::EnableAll),
            0x04 => Ok(RegisterId::Enable),
            0x05 => Ok(RegisterId::ControlMode),
            0x06 => Ok(RegisterId::TorqueMode),
            0x07 => Ok(RegisterId::ModulationMode),
            0x09 => Ok(RegisterId::Angle),
            0x10 => Ok(RegisterId::Position),
            0x11 => Ok(RegisterId::Velocity),
            0x12 => Ok(RegisterId::SensorAngle),
            0x13 => Ok(RegisterId::SensorMechanicalAngle),
            0x14 => Ok(RegisterId::SensorVelocity),
            0x15 => Ok(RegisterId::SensorTimestamp),
            0x16 => Ok(RegisterId::PhaseVoltage),
            0x17 => Ok(RegisterId::PhaseState),
            0x18 => Ok(RegisterId::DriverEnable),
            0x1A => Ok(RegisterId::TelemetryReg),
            0x1B => Ok(RegisterId::TelemetryCtrl),
            0x1C => Ok(RegisterId::TelemetryDownsample),
            0x1D => Ok(RegisterId::IterationsSec),
            0x20 => Ok(RegisterId::VoltageQ),
            0x21 => Ok(RegisterId::VoltageD),
            0x22 => Ok(RegisterId::CurrentQ),
            0x23 => Ok(RegisterId::CurrentD),
            0x24 => Ok(RegisterId::CurrentA),
            0x25 => Ok(RegisterId::CurrentB),
            0x26 => Ok(RegisterId::CurrentC),
            0x27 => Ok(RegisterId::CurrentABC),
            0x28 => Ok(RegisterId::CurrentDC),
            0x30 => Ok(RegisterId::VelPidP),
            0x31 => Ok(RegisterId::VelPidI),
            0x32 => Ok(RegisterId::VelPidD),
            0x33 => Ok(RegisterId::VelPidLim),
            0x34 => Ok(RegisterId::VelPidRamp),
            0x35 => Ok(RegisterId::VelLpfT),
            0x36 => Ok(RegisterId::AngPidP),
            0x37 => Ok(RegisterId::AngPidI),
            0x38 => Ok(RegisterId::AngPidD),
            0x39 => Ok(RegisterId::AngPidLim),
            0x3A => Ok(RegisterId::AngPidRamp),
            0x3B => Ok(RegisterId::AngLpfT),
            0x40 => Ok(RegisterId::CurQPidP),
            0x41 => Ok(RegisterId::CurQPidI),
            0x42 => Ok(RegisterId::CurQPidD),
            0x43 => Ok(RegisterId::CurQPidLim),
            0x44 => Ok(RegisterId::CurQPidRamp),
            0x45 => Ok(RegisterId::CurQLpfT),
            0x46 => Ok(RegisterId::CurDPidP),
            0x47 => Ok(RegisterId::CurDPidI),
            0x48 => Ok(RegisterId::CurDPidD),
            0x49 => Ok(RegisterId::CurDPidLim),
            0x4A => Ok(RegisterId::CurDPidRamp),
            0x4B => Ok(RegisterId::CurDLpfT),
            0x50 => Ok(RegisterId::VoltageLimit),
            0x51 => Ok(RegisterId::CurrentLimit),
            0x52 => Ok(RegisterId::VelocityLimit),
            0x53 => Ok(RegisterId::DriverVoltageLimit),
            0x54 => Ok(RegisterId::PwmFrequency),
            0x55 => Ok(RegisterId::DriverVoltagePsu),
            0x56 => Ok(RegisterId::VoltageSensorAlign),
            0x5F => Ok(RegisterId::MotionDownsample),
            0x60 => Ok(RegisterId::ZeroElectricAngle),
            0x61 => Ok(RegisterId::SensorDirection),
            0x62 => Ok(RegisterId::ZeroOffset),
            0x63 => Ok(RegisterId::PolePairs),
            0x64 => Ok(RegisterId::PhaseResistance),
            0x65 => Ok(RegisterId::KV),
            0x66 => Ok(RegisterId::Inductance),
            0x67 => Ok(RegisterId::CurAGain),
            0x68 => Ok(RegisterId::CurBGain),
            0x69 => Ok(RegisterId::CurCGain),
            0x6A => Ok(RegisterId::CurAOffset),
            0x6B => Ok(RegisterId::CurBOffset),
            0x6C => Ok(RegisterId::CurCOffset),
            0x70 => Ok(RegisterId::NumMotors),
            0x71 => Ok(RegisterId::SysTime),
            0x7F => Ok(RegisterId::MotorAddress),
            0x80 => Ok(RegisterId::GearRatio),
            0x81 => Ok(RegisterId::BacklashDelta),
            0x82 => Ok(RegisterId::OmegaThresh),
            0x83 => Ok(RegisterId::TorsionalStiffness),
            0x84 => Ok(RegisterId::ZeroOffsetJoint),
            0x85 => Ok(RegisterId::OuterPidP),
            0x86 => Ok(RegisterId::OuterPidI),
            0x87 => Ok(RegisterId::OuterPidD),
            0x88 => Ok(RegisterId::OuterPidLim),
            0x89 => Ok(RegisterId::OuterPidRamp),
            0x8A => Ok(RegisterId::CalibState),
            0x8B => Ok(RegisterId::CalibProgress),
            other => Err(RegisterError::UnknownRegister(other)),
        }
    }
}

impl RegisterId {
    /// Return the access permission for this register.
    pub const fn access_mode(&self) -> AccessMode {
        match self {
            RegisterId::Status
            | RegisterId::Angle
            | RegisterId::Position
            | RegisterId::Velocity
            | RegisterId::SensorAngle
            | RegisterId::SensorMechanicalAngle
            | RegisterId::SensorVelocity
            | RegisterId::SensorTimestamp
            | RegisterId::IterationsSec
            | RegisterId::VoltageQ
            | RegisterId::VoltageD
            | RegisterId::CurrentQ
            | RegisterId::CurrentD
            | RegisterId::CurrentA
            | RegisterId::CurrentB
            | RegisterId::CurrentC
            | RegisterId::CurrentABC
            | RegisterId::CurrentDC
            | RegisterId::NumMotors
            | RegisterId::SysTime
            | RegisterId::CalibState
            | RegisterId::CalibProgress => AccessMode::ReadOnly,

            RegisterId::EnableAll | RegisterId::DriverEnable => AccessMode::WriteOnly,

            _ => AccessMode::ReadWrite,
        }
    }

    /// Returns `true` if this register is read-only.
    #[inline]
    pub const fn is_read_only(&self) -> bool {
        matches!(self.access_mode(), AccessMode::ReadOnly)
    }

    /// Returns `true` if this register is write-only.
    #[inline]
    pub const fn is_write_only(&self) -> bool {
        matches!(self.access_mode(), AccessMode::WriteOnly)
    }

    /// Return the expected binary payload size in bytes for this register.
    pub const fn expected_size(&self) -> usize {
        match self {
            RegisterId::Status
            | RegisterId::EnableAll
            | RegisterId::Enable
            | RegisterId::ControlMode
            | RegisterId::TorqueMode
            | RegisterId::ModulationMode
            | RegisterId::DriverEnable
            | RegisterId::TelemetryCtrl
            | RegisterId::TelemetryReg
            | RegisterId::MotionDownsample
            | RegisterId::SensorDirection
            | RegisterId::PolePairs
            | RegisterId::NumMotors
            | RegisterId::MotorAddress
            | RegisterId::CalibState
            | RegisterId::CalibProgress => 1,

            RegisterId::PhaseState => 3,

            RegisterId::Position => 8,

            RegisterId::PhaseVoltage | RegisterId::CurrentABC => 12,

            // All remaining scalar registers (floats and u32s) are 4 bytes
            _ => 4,
        }
    }

    pub const ALL: [RegisterId; 90] = [
        RegisterId::Status,
        RegisterId::Target,
        RegisterId::EnableAll,
        RegisterId::Enable,
        RegisterId::ControlMode,
        RegisterId::TorqueMode,
        RegisterId::ModulationMode,
        RegisterId::Angle,
        RegisterId::Position,
        RegisterId::Velocity,
        RegisterId::SensorAngle,
        RegisterId::SensorMechanicalAngle,
        RegisterId::SensorVelocity,
        RegisterId::SensorTimestamp,
        RegisterId::PhaseVoltage,
        RegisterId::PhaseState,
        RegisterId::DriverEnable,
        RegisterId::TelemetryReg,
        RegisterId::TelemetryCtrl,
        RegisterId::TelemetryDownsample,
        RegisterId::IterationsSec,
        RegisterId::VoltageQ,
        RegisterId::VoltageD,
        RegisterId::CurrentQ,
        RegisterId::CurrentD,
        RegisterId::CurrentA,
        RegisterId::CurrentB,
        RegisterId::CurrentC,
        RegisterId::CurrentABC,
        RegisterId::CurrentDC,
        RegisterId::VelPidP,
        RegisterId::VelPidI,
        RegisterId::VelPidD,
        RegisterId::VelPidLim,
        RegisterId::VelPidRamp,
        RegisterId::VelLpfT,
        RegisterId::AngPidP,
        RegisterId::AngPidI,
        RegisterId::AngPidD,
        RegisterId::AngPidLim,
        RegisterId::AngPidRamp,
        RegisterId::AngLpfT,
        RegisterId::CurQPidP,
        RegisterId::CurQPidI,
        RegisterId::CurQPidD,
        RegisterId::CurQPidLim,
        RegisterId::CurQPidRamp,
        RegisterId::CurQLpfT,
        RegisterId::CurDPidP,
        RegisterId::CurDPidI,
        RegisterId::CurDPidD,
        RegisterId::CurDPidLim,
        RegisterId::CurDPidRamp,
        RegisterId::CurDLpfT,
        RegisterId::VoltageLimit,
        RegisterId::CurrentLimit,
        RegisterId::VelocityLimit,
        RegisterId::DriverVoltageLimit,
        RegisterId::PwmFrequency,
        RegisterId::DriverVoltagePsu,
        RegisterId::VoltageSensorAlign,
        RegisterId::MotionDownsample,
        RegisterId::ZeroElectricAngle,
        RegisterId::SensorDirection,
        RegisterId::ZeroOffset,
        RegisterId::PolePairs,
        RegisterId::PhaseResistance,
        RegisterId::KV,
        RegisterId::Inductance,
        RegisterId::CurAGain,
        RegisterId::CurBGain,
        RegisterId::CurCGain,
        RegisterId::CurAOffset,
        RegisterId::CurBOffset,
        RegisterId::CurCOffset,
        RegisterId::NumMotors,
        RegisterId::SysTime,
        RegisterId::MotorAddress,
        RegisterId::GearRatio,
        RegisterId::BacklashDelta,
        RegisterId::OmegaThresh,
        RegisterId::TorsionalStiffness,
        RegisterId::ZeroOffsetJoint,
        RegisterId::OuterPidP,
        RegisterId::OuterPidI,
        RegisterId::OuterPidD,
        RegisterId::OuterPidLim,
        RegisterId::OuterPidRamp,
        RegisterId::CalibState,
        RegisterId::CalibProgress,
    ];
}

/// Typed representation of register payload data.
#[derive(Debug, Clone, PartialEq)]
pub enum RegisterValue {
    /// Single unsigned 8-bit byte.
    U8(u8),
    /// 32-bit unsigned integer (little-endian).
    U32(u32),
    /// 32-bit signed integer (little-endian).
    I32(i32),
    /// 32-bit IEEE-754 floating point number (little-endian).
    Float(f32),
    /// Shaft position comprising signed full rotations and angle in `[0, 2π)`.
    Position {
        /// Number of full revolutions completed.
        rotations: i32,
        /// Current shaft angle in radians within the revolution.
        angle: f32,
    },
    /// Three 32-bit floating point values (e.g. 3-phase voltages or currents).
    Float3([f32; 3]),
    /// Three per-phase drive states.
    PhaseState([PhaseState; 3]),
}

impl RegisterValue {
    /// Return the encoded byte length of this value.
    pub const fn size(&self) -> usize {
        match self {
            RegisterValue::U8(_) => 1,
            RegisterValue::PhaseState(_) => 3,
            RegisterValue::U32(_) | RegisterValue::I32(_) | RegisterValue::Float(_) => 4,
            RegisterValue::Position { .. } => 8,
            RegisterValue::Float3(_) => 12,
        }
    }

    /// Serialize this register value into the provided byte buffer in little-endian format.
    ///
    /// Returns the number of bytes written on success, or [`RegisterError::BufferTooSmall`]
    /// if `buf` is too short.
    pub fn to_bytes(&self, buf: &mut [u8]) -> Result<usize, RegisterError> {
        let size = self.size();
        if buf.len() < size {
            return Err(RegisterError::BufferTooSmall {
                expected: size,
                actual: buf.len(),
            });
        }

        match self {
            RegisterValue::U8(val) => {
                buf[0] = *val;
            }
            RegisterValue::U32(val) => {
                buf[..4].copy_from_slice(&val.to_le_bytes());
            }
            RegisterValue::I32(val) => {
                buf[..4].copy_from_slice(&val.to_le_bytes());
            }
            RegisterValue::Float(val) => {
                buf[..4].copy_from_slice(&val.to_le_bytes());
            }
            RegisterValue::Position { rotations, angle } => {
                buf[..4].copy_from_slice(&rotations.to_le_bytes());
                buf[4..8].copy_from_slice(&angle.to_le_bytes());
            }
            RegisterValue::Float3(vals) => {
                buf[0..4].copy_from_slice(&vals[0].to_le_bytes());
                buf[4..8].copy_from_slice(&vals[1].to_le_bytes());
                buf[8..12].copy_from_slice(&vals[2].to_le_bytes());
            }
            RegisterValue::PhaseState(states) => {
                buf[0] = states[0] as u8;
                buf[1] = states[1] as u8;
                buf[2] = states[2] as u8;
            }
        }

        Ok(size)
    }

    /// Deserialize a register value from the provided byte slice according to the register's
    /// canonical schema.
    pub fn from_bytes(id: RegisterId, buf: &[u8]) -> Result<Self, RegisterError> {
        let expected = id.expected_size();
        if buf.len() < expected {
            return Err(RegisterError::BufferTooSmall {
                expected,
                actual: buf.len(),
            });
        }

        match id {
            RegisterId::Status
            | RegisterId::EnableAll
            | RegisterId::Enable
            | RegisterId::ControlMode
            | RegisterId::TorqueMode
            | RegisterId::ModulationMode
            | RegisterId::DriverEnable
            | RegisterId::TelemetryCtrl
            | RegisterId::TelemetryReg
            | RegisterId::MotionDownsample
            | RegisterId::SensorDirection
            | RegisterId::PolePairs
            | RegisterId::NumMotors
            | RegisterId::MotorAddress => Ok(RegisterValue::U8(buf[0])),

            RegisterId::PhaseState => {
                let s0 = PhaseState::try_from(buf[0]).map_err(|_| RegisterError::InvalidPayload)?;
                let s1 = PhaseState::try_from(buf[1]).map_err(|_| RegisterError::InvalidPayload)?;
                let s2 = PhaseState::try_from(buf[2]).map_err(|_| RegisterError::InvalidPayload)?;
                Ok(RegisterValue::PhaseState([s0, s1, s2]))
            }

            RegisterId::Position => {
                let rotations = i32::from_le_bytes(buf[0..4].try_into().unwrap());
                let angle = f32::from_le_bytes(buf[4..8].try_into().unwrap());
                Ok(RegisterValue::Position { rotations, angle })
            }

            RegisterId::PhaseVoltage | RegisterId::CurrentABC => {
                let v0 = f32::from_le_bytes(buf[0..4].try_into().unwrap());
                let v1 = f32::from_le_bytes(buf[4..8].try_into().unwrap());
                let v2 = f32::from_le_bytes(buf[8..12].try_into().unwrap());
                Ok(RegisterValue::Float3([v0, v1, v2]))
            }

            RegisterId::SensorTimestamp
            | RegisterId::TelemetryDownsample
            | RegisterId::IterationsSec
            | RegisterId::PwmFrequency
            | RegisterId::SysTime => {
                let val = u32::from_le_bytes(buf[0..4].try_into().unwrap());
                Ok(RegisterValue::U32(val))
            }

            _ => {
                let val = f32::from_le_bytes(buf[0..4].try_into().unwrap());
                Ok(RegisterValue::Float(val))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_id_try_from() {
        assert_eq!(RegisterId::try_from(0x00), Ok(RegisterId::Status));
        assert_eq!(RegisterId::try_from(0x01), Ok(RegisterId::Target));
        assert_eq!(RegisterId::try_from(0x09), Ok(RegisterId::Angle));
        assert_eq!(RegisterId::try_from(0x10), Ok(RegisterId::Position));
        assert_eq!(RegisterId::try_from(0x30), Ok(RegisterId::VelPidP));
        assert_eq!(RegisterId::try_from(0x50), Ok(RegisterId::VoltageLimit));
        assert_eq!(RegisterId::try_from(0x70), Ok(RegisterId::NumMotors));
        assert_eq!(RegisterId::try_from(0x7F), Ok(RegisterId::MotorAddress));

        assert_eq!(RegisterId::try_from(0x02), Err(RegisterError::UnknownRegister(0x02)));
        assert_eq!(RegisterId::try_from(0xFE), Err(RegisterError::UnknownRegister(0xFE)));
    }

    #[test]
    fn test_register_metadata() {
        // Read only
        assert!(RegisterId::Status.is_read_only());
        assert!(RegisterId::Angle.is_read_only());
        assert!(RegisterId::Position.is_read_only());
        assert!(RegisterId::VoltageQ.is_read_only());
        assert_eq!(RegisterId::Status.access_mode(), AccessMode::ReadOnly);

        // Write only
        assert!(RegisterId::EnableAll.is_write_only());
        assert!(RegisterId::DriverEnable.is_write_only());
        assert_eq!(RegisterId::EnableAll.access_mode(), AccessMode::WriteOnly);

        // Read/Write
        assert!(!RegisterId::Target.is_read_only());
        assert!(!RegisterId::Target.is_write_only());
        assert_eq!(RegisterId::Target.access_mode(), AccessMode::ReadWrite);
        assert_eq!(RegisterId::VoltageLimit.access_mode(), AccessMode::ReadWrite);

        // Sizes
        assert_eq!(RegisterId::Status.expected_size(), 1);
        assert_eq!(RegisterId::Target.expected_size(), 4);
        assert_eq!(RegisterId::PhaseState.expected_size(), 3);
        assert_eq!(RegisterId::Position.expected_size(), 8);
        assert_eq!(RegisterId::PhaseVoltage.expected_size(), 12);
        assert_eq!(RegisterId::CurrentABC.expected_size(), 12);
    }

    #[test]
    fn test_binary_round_trip_scalars() {
        let mut buf = [0u8; 16];

        // U8
        let val_u8 = RegisterValue::U8(0x42);
        assert_eq!(val_u8.to_bytes(&mut buf).unwrap(), 1);
        assert_eq!(buf[0], 0x42);
        let decoded = RegisterValue::from_bytes(RegisterId::Enable, &buf[..1]).unwrap();
        assert_eq!(decoded, val_u8);

        // Float
        let val_float = RegisterValue::Float(12.5f32);
        assert_eq!(val_float.to_bytes(&mut buf).unwrap(), 4);
        assert_eq!(buf[..4], 12.5f32.to_le_bytes());
        let decoded = RegisterValue::from_bytes(RegisterId::Target, &buf[..4]).unwrap();
        assert_eq!(decoded, val_float);

        // U32
        let val_u32 = RegisterValue::U32(25000);
        assert_eq!(val_u32.to_bytes(&mut buf).unwrap(), 4);
        assert_eq!(buf[..4], 25000u32.to_le_bytes());
        let decoded = RegisterValue::from_bytes(RegisterId::PwmFrequency, &buf[..4]).unwrap();
        assert_eq!(decoded, val_u32);
    }

    #[test]
    fn test_binary_round_trip_composite() {
        let mut buf = [0u8; 16];

        // Position: rotations (i32) + angle (f32)
        let pos = RegisterValue::Position {
            rotations: -42,
            angle: barefoc::math::_PI,
        };
        assert_eq!(pos.to_bytes(&mut buf).unwrap(), 8);
        assert_eq!(buf[..4], (-42i32).to_le_bytes());
        assert_eq!(buf[4..8], barefoc::math::_PI.to_le_bytes());
        let decoded = RegisterValue::from_bytes(RegisterId::Position, &buf[..8]).unwrap();
        assert_eq!(decoded, pos);

        // Float3: [f32; 3]
        let f3 = RegisterValue::Float3([1.0, 2.5, -3.75]);
        assert_eq!(f3.to_bytes(&mut buf).unwrap(), 12);
        let decoded = RegisterValue::from_bytes(RegisterId::PhaseVoltage, &buf[..12]).unwrap();
        assert_eq!(decoded, f3);

        // PhaseState: [PhaseState; 3]
        let ps = RegisterValue::PhaseState([
            PhaseState::PHASE_ON,
            PhaseState::PHASE_OFF,
            PhaseState::PHASE_HI,
        ]);
        assert_eq!(ps.to_bytes(&mut buf).unwrap(), 3);
        assert_eq!(buf[0], 1);
        assert_eq!(buf[1], 0);
        assert_eq!(buf[2], 2);
        let decoded = RegisterValue::from_bytes(RegisterId::PhaseState, &buf[..3]).unwrap();
        assert_eq!(decoded, ps);
    }

    #[test]
    fn test_buffer_error_handling() {
        let mut small_buf = [0u8; 2];
        let val = RegisterValue::Float(10.0);
        assert_eq!(
            val.to_bytes(&mut small_buf),
            Err(RegisterError::BufferTooSmall {
                expected: 4,
                actual: 2
            })
        );

        let slice = [0u8; 3];
        assert_eq!(
            RegisterValue::from_bytes(RegisterId::Target, &slice),
            Err(RegisterError::BufferTooSmall {
                expected: 4,
                actual: 3
            })
        );

        // Invalid PhaseState discriminant
        let invalid_phase_bytes = [1, 99, 0];
        assert_eq!(
            RegisterValue::from_bytes(RegisterId::PhaseState, &invalid_phase_bytes),
            Err(RegisterError::InvalidPayload)
        );
    }
}
