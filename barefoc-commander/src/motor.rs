//! # Motor command parser implementing SimpleFOC `commands.h`
//!
//! Ports [`commands.h`](source/Arduino-FOC/src/communication/commands.h),
//! [`Commander.h`](source/Arduino-FOC/src/communication/Commander.h), and
//! [`Commander.cpp`](source/Arduino-FOC/src/communication/Commander.cpp)
//! motor control command interface.
//!
//! ## Command Hierarchy
//!
//! - `T` : Motor target setpoint (`T<float>` or bare `<float>`)
//! - `E` : Motor enable/disable (`E0`: disable, `E1`: enable, `E`: get)
//! - `C` : Motion control type (`C0`..`C6`, `CD<int>` downsample, `CT` move time)
//! - `L` : Limits (`LC<float>` current, `LU<float>` voltage, `LV<float>` velocity)
//! - `V` : Velocity PID (`VP`, `VI`, `VD`, `VR`, `VL`, `VF`)
//! - `A` : Angle PID (`AP`, `AI`, `AD`, `AR`, `AL`, `AF`)
//! - `Q` : Current Q PID (`QP`, `QI`, `QD`, `QR`, `QL`, `QF`)
//! - `D` : Current D PID (`DP`, `DI`, `DD`, `DR`, `DL`, `DF`)
//! - `R` : Motor phase resistance (`R<float>`, `R`)
//! - `I` : Motor phase inductance (`ID<float>`, `IQ<float>`, `I<float>`)
//! - `K` : Motor KV rating (`K<float>`, `K`)
//! - `W` : PWM modulation (`WT<int>`, `WC<int>`)
//! - `S` : Sensor offsets (`SM<float>` mechanical, `SE<float>` electrical)
//! - `M` : Monitoring (`MG<int>` get variable, `MD<int>` downsample, `MC` clear)

#![allow(non_snake_case)]
#![allow(clippy::too_many_arguments)]

use barefoc::common::filter::LowPassFilter;
use barefoc::common::pid::PIDController;
use barefoc::common::time::Clock;
use barefoc::math::isset;
use barefoc::motor::base::{FOCMotor, MotorConfig};
use barefoc::motor::enums::{FOCModulationType, MotionControlType, TorqueControlType};
use barefoc::traits::current_sense::CurrentSense;
use barefoc::traits::driver::FOCDriver;
use barefoc::traits::sensor::Sensor;

use crate::commander::VerboseMode;

// SimpleFOC commands.h command constants
pub const CMD_C_D_PID: char = 'D';
pub const CMD_C_Q_PID: char = 'Q';
pub const CMD_V_PID: char = 'V';
pub const CMD_A_PID: char = 'A';
pub const CMD_STATUS: char = 'E';
pub const CMD_LIMITS: char = 'L';
pub const CMD_MOTION_TYPE: char = 'C';
pub const CMD_TORQUE_TYPE: char = 'T';
pub const CMD_SENSOR: char = 'S';
pub const CMD_MONITOR: char = 'M';
pub const CMD_RESIST: char = 'R';
pub const CMD_INDUCTANCE: char = 'I';
pub const CMD_KV_RATING: char = 'K';
pub const CMD_PWMMOD: char = 'W';

// Subcommands
pub const SCMD_PID_P: char = 'P';
pub const SCMD_PID_I: char = 'I';
pub const SCMD_PID_D: char = 'D';
pub const SCMD_PID_RAMP: char = 'R';
pub const SCMD_PID_LIM: char = 'L';
pub const SCMD_LPF_TF: char = 'F';

pub const SCMD_LIM_CURR: char = 'C';
pub const SCMD_LIM_VOLT: char = 'U';
pub const SCMD_LIM_VEL: char = 'V';

pub const SCMD_SENS_MECH_OFFSET: char = 'M';
pub const SCMD_SENS_ELEC_OFFSET: char = 'E';

pub const SCMD_DOWNSAMPLE: char = 'D';
pub const SCMD_CLEAR: char = 'C';
pub const SCMD_GET: char = 'G';
pub const SCMD_SET: char = 'S';
pub const SCMD_TIME: char = 'T';

pub const SCMD_PWMMOD_TYPE: char = 'T';
pub const SCMD_PWMMOD_CENTER: char = 'C';

pub const SCMD_INDUCT_D: char = 'D';
pub const SCMD_INDUCT_Q: char = 'Q';

/// Errors returned when parsing or dispatching motor commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorCommandError {
    /// Command string was empty.
    EmptyCommand,
    /// Unknown primary command ID.
    UnknownCommand(char),
    /// Unknown subcommand ID.
    UnknownSubcommand(char),
    /// Numerical value parsing failed.
    InvalidValue,
    /// Invalid motion control mode identifier.
    InvalidMotionMode(i32),
    /// Invalid PWM modulation type identifier.
    InvalidModulationMode(i32),
}

/// SimpleFOC ASCII motor tuning and inspection commander.
pub struct MotorCommander;

impl MotorCommander {
    /// Parse and execute a command against `MotorConfig<C>`.
    ///
    /// Returns the updated or queried floating-point value on success, or a
    /// [`MotorCommandError`] on syntax/range failure.
    pub fn handle<C: Clock>(
        config: &mut MotorConfig<C>,
        user_cmd: &str,
    ) -> Result<f32, MotorCommandError> {
        let trimmed = user_cmd.trim();
        if trimmed.is_empty() {
            return Err(MotorCommandError::EmptyCommand);
        }

        let first_char = trimmed.chars().next().unwrap();

        // 1. Target setting: naked number (+1.2, -5.0, 10.5) or explicit 'T'
        if first_char.is_ascii_digit() || first_char == '-' || first_char == '+' {
            return Self::parse_target(config, trimmed);
        }

        if first_char == CMD_TORQUE_TYPE {
            let payload = trimmed[1..].trim_start();
            if payload.is_empty() {
                return Ok(config.target);
            }
            return Self::parse_target(config, payload);
        }

        let cmd = first_char;
        let rest = &trimmed[cmd.len_utf8()..];

        match cmd {
            CMD_STATUS => {
                // E0 / E1 / E
                let payload = rest.trim();
                if payload.is_empty() {
                    return Ok(config.enabled as f32);
                }
                match payload.parse::<i32>() {
                    Ok(val) => {
                        config.enabled = if val != 0 { 1 } else { 0 };
                        Ok(config.enabled as f32)
                    }
                    Err(_) => Err(MotorCommandError::InvalidValue),
                }
            }

            CMD_MOTION_TYPE => {
                // C<int> or CD<int> or CT or C
                let payload = rest.trim();
                if payload.is_empty() {
                    return Ok(config.controller as u8 as f32);
                }
                if let Some(sub) = payload.chars().next() {
                    if sub == SCMD_DOWNSAMPLE {
                        let sub_payload = payload[sub.len_utf8()..].trim();
                        if sub_payload.is_empty() {
                            return Ok(config.motion_downsample as f32);
                        }
                        let val = sub_payload.parse::<u32>().map_err(|_| MotorCommandError::InvalidValue)?;
                        config.motion_downsample = val;
                        return Ok(config.motion_downsample as f32);
                    } else if sub == SCMD_TIME {
                        return Ok(config.move_time_us as f32);
                    }
                }
                let mode_num = payload.parse::<i32>().map_err(|_| MotorCommandError::InvalidValue)?;
                let mode = match mode_num {
                    0 => MotionControlType::torque,
                    1 => MotionControlType::velocity,
                    2 => MotionControlType::angle,
                    3 => MotionControlType::velocity_openloop,
                    4 => MotionControlType::angle_openloop,
                    5 => MotionControlType::angle_nocascade,
                    6 => MotionControlType::custom,
                    other => return Err(MotorCommandError::InvalidMotionMode(other)),
                };
                config.update_motion_control_type(mode);
                Ok(config.controller as u8 as f32)
            }

            CMD_LIMITS => {
                // LC<float>, LU<float>, LV<float>
                let mut chars = rest.chars();
                let sub_cmd = chars.next().ok_or(MotorCommandError::UnknownSubcommand('\0'))?;
                let payload = chars.as_str().trim();
                match sub_cmd {
                    SCMD_LIM_CURR => {
                        if payload.is_empty() {
                            Ok(config.current_limit)
                        } else {
                            let val = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                            config.update_current_limit(val);
                            Ok(config.current_limit)
                        }
                    }
                    SCMD_LIM_VOLT => {
                        if payload.is_empty() {
                            Ok(config.voltage_limit)
                        } else {
                            let val = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                            config.update_voltage_limit(val);
                            Ok(config.voltage_limit)
                        }
                    }
                    SCMD_LIM_VEL => {
                        if payload.is_empty() {
                            Ok(config.velocity_limit)
                        } else {
                            let val = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                            config.update_velocity_limit(val);
                            Ok(config.velocity_limit)
                        }
                    }
                    other => Err(MotorCommandError::UnknownSubcommand(other)),
                }
            }

            CMD_V_PID => {
                // VP, VI, VD, VR, VL, VF
                let mut chars = rest.chars();
                let sub_cmd = chars.next().ok_or(MotorCommandError::UnknownSubcommand('\0'))?;
                let payload = chars.as_str().trim();
                if sub_cmd == SCMD_LPF_TF {
                    Self::tune_lpf(&mut config.LPF_velocity, sub_cmd, payload)
                } else {
                    Self::tune_pid(&mut config.PID_velocity, sub_cmd, payload)
                }
            }

            CMD_A_PID => {
                // AP, AI, AD, AR, AL, AF
                let mut chars = rest.chars();
                let sub_cmd = chars.next().ok_or(MotorCommandError::UnknownSubcommand('\0'))?;
                let payload = chars.as_str().trim();
                if sub_cmd == SCMD_LPF_TF {
                    Self::tune_lpf(&mut config.LPF_angle, sub_cmd, payload)
                } else {
                    Self::tune_pid(&mut config.P_angle, sub_cmd, payload)
                }
            }

            CMD_C_Q_PID => {
                // QP, QI, QD, QR, QL, QF
                let mut chars = rest.chars();
                let sub_cmd = chars.next().ok_or(MotorCommandError::UnknownSubcommand('\0'))?;
                let payload = chars.as_str().trim();
                if sub_cmd == SCMD_LPF_TF {
                    Self::tune_lpf(&mut config.LPF_current_q, sub_cmd, payload)
                } else {
                    Self::tune_pid(&mut config.PID_current_q, sub_cmd, payload)
                }
            }

            CMD_C_D_PID => {
                // DP, DI, DD, DR, DL, DF
                let mut chars = rest.chars();
                let sub_cmd = chars.next().ok_or(MotorCommandError::UnknownSubcommand('\0'))?;
                let payload = chars.as_str().trim();
                if sub_cmd == SCMD_LPF_TF {
                    Self::tune_lpf(&mut config.LPF_current_d, sub_cmd, payload)
                } else {
                    Self::tune_pid(&mut config.PID_current_d, sub_cmd, payload)
                }
            }

            CMD_RESIST => {
                // R<float> or R
                let payload = rest.trim();
                if payload.is_empty() {
                    Ok(if isset(config.phase_resistance) { config.phase_resistance } else { 0.0 })
                } else {
                    let val = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    config.phase_resistance = val;
                    Ok(config.phase_resistance)
                }
            }

            CMD_INDUCTANCE => {
                // ID<float>, IQ<float>, I<float>, or I
                let payload = rest.trim();
                if payload.is_empty() {
                    return Ok(if isset(config.phase_inductance) { config.phase_inductance } else { 0.0 });
                }
                let first = payload.chars().next().unwrap();
                if first == SCMD_INDUCT_D {
                    let sub_payload = payload[first.len_utf8()..].trim();
                    if sub_payload.is_empty() {
                        Ok(if isset(config.axis_inductance.d) { config.axis_inductance.d } else { 0.0 })
                    } else {
                        let val = sub_payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                        config.axis_inductance.d = val;
                        config.phase_inductance = val;
                        Ok(config.axis_inductance.d)
                    }
                } else if first == SCMD_INDUCT_Q {
                    let sub_payload = payload[first.len_utf8()..].trim();
                    if sub_payload.is_empty() {
                        Ok(if isset(config.axis_inductance.q) { config.axis_inductance.q } else { 0.0 })
                    } else {
                        let val = sub_payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                        config.axis_inductance.q = val;
                        Ok(config.axis_inductance.q)
                    }
                } else {
                    let val = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    config.phase_inductance = val;
                    config.axis_inductance.d = val;
                    config.axis_inductance.q = val;
                    Ok(config.phase_inductance)
                }
            }

            CMD_KV_RATING => {
                // K<float> or K
                let payload = rest.trim();
                if payload.is_empty() {
                    Ok(if isset(config.KV_rating) { config.KV_rating } else { 0.0 })
                } else {
                    let val = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    config.KV_rating = val;
                    Ok(config.KV_rating)
                }
            }

            CMD_PWMMOD => {
                // WT<int>, WC<int>
                let mut chars = rest.chars();
                let sub_cmd = chars.next().ok_or(MotorCommandError::UnknownSubcommand('\0'))?;
                let payload = chars.as_str().trim();
                match sub_cmd {
                    SCMD_PWMMOD_TYPE => {
                        if payload.is_empty() {
                            Ok(config.foc_modulation as u8 as f32)
                        } else {
                            let val = payload.parse::<i32>().map_err(|_| MotorCommandError::InvalidValue)?;
                            let mod_type = match val {
                                0 => FOCModulationType::SinePWM,
                                1 => FOCModulationType::SpaceVectorPWM,
                                2 => FOCModulationType::Trapezoid_120,
                                3 => FOCModulationType::Trapezoid_150,
                                other => return Err(MotorCommandError::InvalidModulationMode(other)),
                            };
                            config.foc_modulation = mod_type;
                            Ok(config.foc_modulation as u8 as f32)
                        }
                    }
                    SCMD_PWMMOD_CENTER => {
                        if payload.is_empty() {
                            Ok(config.modulation_centered as f32)
                        } else {
                            let val = payload.parse::<i8>().map_err(|_| MotorCommandError::InvalidValue)?;
                            config.modulation_centered = val;
                            Ok(config.modulation_centered as f32)
                        }
                    }
                    other => Err(MotorCommandError::UnknownSubcommand(other)),
                }
            }

            CMD_SENSOR => {
                // SM<float>, SE<float>
                let mut chars = rest.chars();
                let sub_cmd = chars.next().ok_or(MotorCommandError::UnknownSubcommand('\0'))?;
                let payload = chars.as_str().trim();
                match sub_cmd {
                    SCMD_SENS_MECH_OFFSET => {
                        if payload.is_empty() {
                            Ok(config.sensor_offset)
                        } else {
                            let val = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                            config.sensor_offset = val;
                            Ok(config.sensor_offset)
                        }
                    }
                    SCMD_SENS_ELEC_OFFSET => {
                        if payload.is_empty() {
                            Ok(if isset(config.zero_electric_angle) { config.zero_electric_angle } else { 0.0 })
                        } else {
                            let val = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                            config.zero_electric_angle = val;
                            Ok(config.zero_electric_angle)
                        }
                    }
                    other => Err(MotorCommandError::UnknownSubcommand(other)),
                }
            }

            CMD_MONITOR => {
                // MG<int>, MD<int>, MC
                let mut chars = rest.chars();
                let sub_cmd = chars.next().ok_or(MotorCommandError::UnknownSubcommand('\0'))?;
                let payload = chars.as_str().trim();
                match sub_cmd {
                    SCMD_GET => {
                        let idx = payload.parse::<u8>().map_err(|_| MotorCommandError::InvalidValue)?;
                        match idx {
                            0 => Ok(config.target),
                            1 => Ok(config.voltage.q),
                            2 => Ok(config.voltage.d),
                            3 => Ok(config.current.q),
                            4 => Ok(config.current.d),
                            5 => Ok(config.shaft_velocity),
                            6 => Ok(config.shaft_angle),
                            _ => Err(MotorCommandError::InvalidValue),
                        }
                    }
                    SCMD_DOWNSAMPLE => {
                        if payload.is_empty() {
                            Ok(config.motion_downsample as f32)
                        } else {
                            let val = payload.parse::<u32>().map_err(|_| MotorCommandError::InvalidValue)?;
                            config.motion_downsample = val;
                            Ok(config.motion_downsample as f32)
                        }
                    }
                    SCMD_CLEAR => Ok(0.0),
                    other => Err(MotorCommandError::UnknownSubcommand(other)),
                }
            }

            other => Err(MotorCommandError::UnknownCommand(other)),
        }
    }

    /// Parse and execute a command against an `M: FOCMotor` instance.
    ///
    /// Specifically intercepts `E1` / `E0` to invoke `motor.enable()` and `motor.disable()`
    /// so hardware drivers and phase outputs are updated properly.
    pub fn handle_motor<M, S, D, CS, C>(
        motor: &mut M,
        user_cmd: &str,
    ) -> Result<f32, MotorCommandError>
    where
        M: FOCMotor<S, D, CS, C>,
        S: Sensor<C>,
        D: FOCDriver,
        CS: CurrentSense,
        C: Clock,
    {
        let trimmed = user_cmd.trim();
        if trimmed.starts_with(CMD_STATUS) {
            let payload = trimmed[1..].trim();
            if payload == "1" {
                motor.enable();
                return Ok(1.0);
            } else if payload == "0" {
                motor.disable();
                return Ok(0.0);
            }
        }
        Self::handle(motor.cfg_mut(), user_cmd)
    }

    /// Parse and execute a command, writing formatted responses to `writer`.
    #[cfg(feature = "embedded-io")]
    pub fn handle_with_writer<C: Clock, W: embedded_io::Write>(
        config: &mut MotorConfig<C>,
        user_cmd: &str,
        writer: &mut W,
        verbose: VerboseMode,
        decimals: u8,
    ) -> Result<(), W::Error> {
        let trimmed = user_cmd.trim();
        let cmd = match trimmed.chars().next() {
            Some(c) => c,
            None => {
                return Self::write_error(writer, verbose);
            }
        };

        let result = Self::handle(config, trimmed);
        match result {
            Ok(val) => {
                if verbose == VerboseMode::nothing {
                    return Ok(());
                }

                if verbose == VerboseMode::machine_readable {
                    let mut b = [0u8; 4];
                    writer.write_all(cmd.encode_utf8(&mut b).as_bytes())?;
                    if trimmed.len() > 1 {
                        let sub = trimmed.chars().nth(1).unwrap();
                        if sub.is_ascii_alphabetic() {
                            writer.write_all(sub.encode_utf8(&mut b).as_bytes())?;
                        }
                    }
                } else if verbose == VerboseMode::user_friendly {
                    Self::write_user_prefix(cmd, trimmed, writer)?;
                }

                Self::write_float(writer, val, decimals)?;
                writer.write_all(b"\r\n")?;
            }
            Err(_) => {
                Self::write_error(writer, verbose)?;
            }
        }
        Ok(())
    }

    fn parse_target<C: Clock>(
        config: &mut MotorConfig<C>,
        payload: &str,
    ) -> Result<f32, MotorCommandError> {
        let mut tokens = payload.split_ascii_whitespace();
        let target_str = tokens.next().ok_or(MotorCommandError::EmptyCommand)?;
        let target = target_str.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
        config.target = target;

        // Parse optional extra limits (velocity limit, current/voltage limit)
        if let Some(vel_str) = tokens.next() {
            if let Ok(vel) = vel_str.parse::<f32>() {
                config.update_velocity_limit(vel);
            }
            if let Some(torque_str) = tokens.next() {
                if let Ok(torque) = torque_str.parse::<f32>() {
                    if config.torque_controller == TorqueControlType::voltage {
                        config.update_voltage_limit(torque);
                    } else {
                        config.update_current_limit(torque);
                    }
                }
            }
        }
        Ok(config.target)
    }

    fn tune_pid<C: Clock>(
        pid: &mut PIDController<C>,
        sub_cmd: char,
        payload: &str,
    ) -> Result<f32, MotorCommandError> {
        match sub_cmd {
            SCMD_PID_P => {
                if payload.is_empty() {
                    Ok(pid.p())
                } else {
                    let v = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    pid.set_p(v);
                    Ok(v)
                }
            }
            SCMD_PID_I => {
                if payload.is_empty() {
                    Ok(pid.i())
                } else {
                    let v = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    pid.set_i(v);
                    Ok(v)
                }
            }
            SCMD_PID_D => {
                if payload.is_empty() {
                    Ok(pid.d())
                } else {
                    let v = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    pid.set_d(v);
                    Ok(v)
                }
            }
            SCMD_PID_RAMP => {
                if payload.is_empty() {
                    Ok(pid.output_ramp())
                } else {
                    let v = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    pid.set_output_ramp(v);
                    Ok(v)
                }
            }
            SCMD_PID_LIM => {
                if payload.is_empty() {
                    Ok(pid.limit())
                } else {
                    let v = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    pid.set_limit(v);
                    Ok(v)
                }
            }
            other => Err(MotorCommandError::UnknownSubcommand(other)),
        }
    }

    fn tune_lpf<C: Clock>(
        lpf: &mut LowPassFilter<C>,
        sub_cmd: char,
        payload: &str,
    ) -> Result<f32, MotorCommandError> {
        match sub_cmd {
            SCMD_LPF_TF => {
                if payload.is_empty() {
                    Ok(lpf.tf())
                } else {
                    let v = payload.parse::<f32>().map_err(|_| MotorCommandError::InvalidValue)?;
                    lpf.set_tf(v);
                    Ok(v)
                }
            }
            other => Err(MotorCommandError::UnknownSubcommand(other)),
        }
    }

    #[cfg(feature = "embedded-io")]
    fn write_error<W: embedded_io::Write>(
        writer: &mut W,
        verbose: VerboseMode,
    ) -> Result<(), W::Error> {
        if verbose != VerboseMode::nothing {
            if verbose == VerboseMode::user_friendly {
                writer.write_all(b"err\r\n")?;
            } else {
                writer.write_all(b"err\r\n")?;
            }
        }
        Ok(())
    }

    #[cfg(feature = "embedded-io")]
    fn write_user_prefix<W: embedded_io::Write>(
        cmd: char,
        user_cmd: &str,
        writer: &mut W,
    ) -> Result<(), W::Error> {
        match cmd {
            CMD_V_PID => writer.write_all(b"PID vel| ")?,
            CMD_A_PID => writer.write_all(b"PID angle| ")?,
            CMD_C_Q_PID => writer.write_all(b"PID curr q| ")?,
            CMD_C_D_PID => writer.write_all(b"PID curr d| ")?,
            CMD_LIMITS => writer.write_all(b"Limits| ")?,
            CMD_STATUS => writer.write_all(b"Status: ")?,
            CMD_MOTION_TYPE => writer.write_all(b"Motion: ")?,
            CMD_TORQUE_TYPE => writer.write_all(b"Target: ")?,
            CMD_RESIST => writer.write_all(b"R phase: ")?,
            CMD_INDUCTANCE => writer.write_all(b"L phase: ")?,
            CMD_KV_RATING => writer.write_all(b"Motor KV: ")?,
            _ => {
                if user_cmd.chars().next().map_or(false, |c| c.is_ascii_digit() || c == '-' || c == '+') {
                    writer.write_all(b"Target: ")?;
                }
            }
        }
        Ok(())
    }

    #[cfg(feature = "embedded-io")]
    fn write_float<W: embedded_io::Write>(
        writer: &mut W,
        val: f32,
        decimals: u8,
    ) -> Result<(), W::Error> {
        struct IoAdapter<'a, W: embedded_io::Write> {
            inner: &'a mut W,
            err: Option<W::Error>,
        }
        impl<'a, W: embedded_io::Write> core::fmt::Write for IoAdapter<'a, W> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                match self.inner.write_all(s.as_bytes()) {
                    Ok(()) => Ok(()),
                    Err(e) => {
                        self.err = Some(e);
                        Err(core::fmt::Error)
                    }
                }
            }
        }

        let mut adapter = IoAdapter { inner: writer, err: None };
        let prec = (decimals as usize).min(8);
        let _ = core::fmt::write(&mut adapter, format_args!("{:.prec$}", val, prec = prec));
        if let Some(err) = adapter.err {
            return Err(err);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use barefoc::motor::base::NullClock;

    #[test]
    fn test_target_command_parsing() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        // 'T25.5'
        let res = MotorCommander::handle(&mut config, "T25.5").unwrap();
        assert_eq!(res, 25.5);
        assert_eq!(config.target, 25.5);

        // Bare float '12.25'
        let res = MotorCommander::handle(&mut config, "12.25").unwrap();
        assert_eq!(res, 12.25);
        assert_eq!(config.target, 12.25);

        // Negative bare float '-3.14'
        let res = MotorCommander::handle(&mut config, "-3.14").unwrap();
        assert_eq!(res, -3.14);
        assert_eq!(config.target, -3.14);

        // Target with velocity limit
        MotorCommander::handle(&mut config, "T50.0 10.0").unwrap();
        assert_eq!(config.target, 50.0);
        assert_eq!(config.velocity_limit, 10.0);
    }

    #[test]
    fn test_enable_disable_status() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        MotorCommander::handle(&mut config, "E1").unwrap();
        assert_eq!(config.enabled, 1);

        MotorCommander::handle(&mut config, "E0").unwrap();
        assert_eq!(config.enabled, 0);

        let query = MotorCommander::handle(&mut config, "E").unwrap();
        assert_eq!(query, 0.0);
    }

    #[test]
    fn test_motion_mode() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        MotorCommander::handle(&mut config, "C1").unwrap();
        assert_eq!(config.controller, MotionControlType::velocity);

        MotorCommander::handle(&mut config, "C2").unwrap();
        assert_eq!(config.controller, MotionControlType::angle);

        MotorCommander::handle(&mut config, "CD4").unwrap();
        assert_eq!(config.motion_downsample, 4);

        assert!(MotorCommander::handle(&mut config, "C99").is_err());
    }

    #[test]
    fn test_limits() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        MotorCommander::handle(&mut config, "LC2.5").unwrap();
        assert_eq!(config.current_limit, 2.5);

        MotorCommander::handle(&mut config, "LU12.0").unwrap();
        assert_eq!(config.voltage_limit, 12.0);

        MotorCommander::handle(&mut config, "LV100.0").unwrap();
        assert_eq!(config.velocity_limit, 100.0);

        // Query limits
        assert_eq!(MotorCommander::handle(&mut config, "LC").unwrap(), 2.5);
        assert_eq!(MotorCommander::handle(&mut config, "LU").unwrap(), 12.0);
        assert_eq!(MotorCommander::handle(&mut config, "LV").unwrap(), 100.0);
    }

    #[test]
    fn test_pid_tuning_all_branches() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        // Velocity PID
        MotorCommander::handle(&mut config, "VP0.2").unwrap();
        assert_eq!(config.PID_velocity.p(), 0.2);
        MotorCommander::handle(&mut config, "VI1.5").unwrap();
        assert_eq!(config.PID_velocity.i(), 1.5);
        MotorCommander::handle(&mut config, "VD0.01").unwrap();
        assert_eq!(config.PID_velocity.d(), 0.01);
        MotorCommander::handle(&mut config, "VR100.0").unwrap();
        assert_eq!(config.PID_velocity.output_ramp(), 100.0);
        MotorCommander::handle(&mut config, "VL12.0").unwrap();
        assert_eq!(config.PID_velocity.limit(), 12.0);
        MotorCommander::handle(&mut config, "VF0.05").unwrap();
        assert_eq!(config.LPF_velocity.tf(), 0.05);

        // Angle PID
        MotorCommander::handle(&mut config, "AP20.0").unwrap();
        assert_eq!(config.P_angle.p(), 20.0);
        MotorCommander::handle(&mut config, "AF0.02").unwrap();
        assert_eq!(config.LPF_angle.tf(), 0.02);

        // Current Q PID
        MotorCommander::handle(&mut config, "QP3.0").unwrap();
        assert_eq!(config.PID_current_q.p(), 3.0);
        MotorCommander::handle(&mut config, "QF0.005").unwrap();
        assert_eq!(config.LPF_current_q.tf(), 0.005);

        // Current D PID
        MotorCommander::handle(&mut config, "DP3.0").unwrap();
        assert_eq!(config.PID_current_d.p(), 3.0);
        MotorCommander::handle(&mut config, "DF0.005").unwrap();
        assert_eq!(config.LPF_current_d.tf(), 0.005);
    }

    #[test]
    fn test_motor_specs() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        MotorCommander::handle(&mut config, "R1.25").unwrap();
        assert_eq!(config.phase_resistance, 1.25);

        MotorCommander::handle(&mut config, "I0.002").unwrap();
        assert_eq!(config.phase_inductance, 0.002);
        assert_eq!(config.axis_inductance.d, 0.002);
        assert_eq!(config.axis_inductance.q, 0.002);

        MotorCommander::handle(&mut config, "K150.0").unwrap();
        assert_eq!(config.KV_rating, 150.0);
    }

    #[test]
    fn test_sensor_and_pwmmod() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        MotorCommander::handle(&mut config, "SM0.785").unwrap();
        assert_eq!(config.sensor_offset, 0.785);

        MotorCommander::handle(&mut config, "SE1.57").unwrap();
        assert_eq!(config.zero_electric_angle, 1.57);

        MotorCommander::handle(&mut config, "WT1").unwrap();
        assert_eq!(config.foc_modulation, FOCModulationType::SpaceVectorPWM);

        MotorCommander::handle(&mut config, "WC0").unwrap();
        assert_eq!(config.modulation_centered, 0);
    }
}
