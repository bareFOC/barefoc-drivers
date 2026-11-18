//! # comms::bridge — Motor State Register Bridge
//!
//! Maps between SimpleFOC registers ([`RegisterId`], [`RegisterValue`]) and
//! [`barefoc::motor::base::MotorConfig`].
//!
//! References upstream [`SimpleFOCRegisters.cpp`](source/Arduino-FOC-drivers/src/comms/SimpleFOCRegisters.cpp).

use barefoc::common::time::Clock;
use barefoc::math::{self, NOT_SET};
use barefoc::motor::base::MotorConfig;
use barefoc::motor::enums::{
    FOCModulationType, MotionControlType, TorqueControlType,
};
use barefoc::traits::sensor::Direction;

use super::registers::{RegisterError, RegisterId, RegisterValue};

/// Read the current value of a register from [`MotorConfig`].
///
/// Returns [`RegisterError::WriteOnly`] if `id` is a write-only register,
/// or [`RegisterError::InvalidPayload`] if the requested register cannot be read
/// from motor configuration.
pub fn read_register<C: Clock>(
    config: &MotorConfig<C>,
    id: RegisterId,
) -> Result<RegisterValue, RegisterError> {
    if id.is_write_only() {
        return Err(RegisterError::WriteOnly(id));
    }

    match id {
        RegisterId::Status => Ok(RegisterValue::U8(config.motor_status as u8)),
        RegisterId::Target => Ok(RegisterValue::Float(config.target)),
        RegisterId::Enable => Ok(RegisterValue::U8(config.enabled as u8)),
        RegisterId::ControlMode => Ok(RegisterValue::U8(config.controller as u8)),
        RegisterId::TorqueMode => Ok(RegisterValue::U8(config.torque_controller as u8)),
        RegisterId::ModulationMode => Ok(RegisterValue::U8(config.foc_modulation as u8)),

        RegisterId::Angle => Ok(RegisterValue::Float(config.shaft_angle)),
        RegisterId::Position => {
            let rotations = (config.shaft_angle / math::_2PI) as i32;
            let angle = libm::fmodf(config.shaft_angle, math::_2PI);
            Ok(RegisterValue::Position { rotations, angle })
        }
        RegisterId::Velocity => Ok(RegisterValue::Float(config.shaft_velocity)),
        RegisterId::SensorAngle => Ok(RegisterValue::Float(config.shaft_angle)),
        RegisterId::SensorMechanicalAngle => Ok(RegisterValue::Float(config.shaft_angle)),
        RegisterId::SensorVelocity => Ok(RegisterValue::Float(config.shaft_velocity)),
        RegisterId::SensorTimestamp => Ok(RegisterValue::U32(0)),

        RegisterId::VoltageQ => Ok(RegisterValue::Float(config.voltage.q)),
        RegisterId::VoltageD => Ok(RegisterValue::Float(config.voltage.d)),
        RegisterId::CurrentQ => Ok(RegisterValue::Float(config.current.q)),
        RegisterId::CurrentD => Ok(RegisterValue::Float(config.current.d)),
        RegisterId::CurrentA => Ok(RegisterValue::Float(0.0)),
        RegisterId::CurrentB => Ok(RegisterValue::Float(0.0)),
        RegisterId::CurrentC => Ok(RegisterValue::Float(0.0)),
        RegisterId::CurrentABC => Ok(RegisterValue::Float3([0.0, 0.0, 0.0])),
        RegisterId::CurrentDC => Ok(RegisterValue::Float(0.0)),

        RegisterId::VelPidP => Ok(RegisterValue::Float(config.PID_velocity.p())),
        RegisterId::VelPidI => Ok(RegisterValue::Float(config.PID_velocity.i())),
        RegisterId::VelPidD => Ok(RegisterValue::Float(config.PID_velocity.d())),
        RegisterId::VelPidLim => Ok(RegisterValue::Float(config.PID_velocity.limit())),
        RegisterId::VelPidRamp => Ok(RegisterValue::Float(config.PID_velocity.output_ramp())),
        RegisterId::VelLpfT => Ok(RegisterValue::Float(config.LPF_velocity.tf())),

        RegisterId::AngPidP => Ok(RegisterValue::Float(config.P_angle.p())),
        RegisterId::AngPidI => Ok(RegisterValue::Float(config.P_angle.i())),
        RegisterId::AngPidD => Ok(RegisterValue::Float(config.P_angle.d())),
        RegisterId::AngPidLim => Ok(RegisterValue::Float(config.P_angle.limit())),
        RegisterId::AngPidRamp => Ok(RegisterValue::Float(config.P_angle.output_ramp())),
        RegisterId::AngLpfT => Ok(RegisterValue::Float(config.LPF_angle.tf())),

        RegisterId::CurQPidP => Ok(RegisterValue::Float(config.PID_current_q.p())),
        RegisterId::CurQPidI => Ok(RegisterValue::Float(config.PID_current_q.i())),
        RegisterId::CurQPidD => Ok(RegisterValue::Float(config.PID_current_q.d())),
        RegisterId::CurQPidLim => Ok(RegisterValue::Float(config.PID_current_q.limit())),
        RegisterId::CurQPidRamp => Ok(RegisterValue::Float(config.PID_current_q.output_ramp())),
        RegisterId::CurQLpfT => Ok(RegisterValue::Float(config.LPF_current_q.tf())),

        RegisterId::CurDPidP => Ok(RegisterValue::Float(config.PID_current_d.p())),
        RegisterId::CurDPidI => Ok(RegisterValue::Float(config.PID_current_d.i())),
        RegisterId::CurDPidD => Ok(RegisterValue::Float(config.PID_current_d.d())),
        RegisterId::CurDPidLim => Ok(RegisterValue::Float(config.PID_current_d.limit())),
        RegisterId::CurDPidRamp => Ok(RegisterValue::Float(config.PID_current_d.output_ramp())),
        RegisterId::CurDLpfT => Ok(RegisterValue::Float(config.LPF_current_d.tf())),

        RegisterId::VoltageLimit => Ok(RegisterValue::Float(config.voltage_limit)),
        RegisterId::CurrentLimit => Ok(RegisterValue::Float(config.current_limit)),
        RegisterId::VelocityLimit => Ok(RegisterValue::Float(config.velocity_limit)),
        RegisterId::DriverVoltageLimit => Ok(RegisterValue::Float(0.0)),
        RegisterId::PwmFrequency => Ok(RegisterValue::U32(0)),
        RegisterId::DriverVoltagePsu => Ok(RegisterValue::Float(0.0)),
        RegisterId::VoltageSensorAlign => Ok(RegisterValue::Float(config.voltage_sensor_align)),
        RegisterId::MotionDownsample => Ok(RegisterValue::U8(config.motion_downsample as u8)),

        RegisterId::ZeroElectricAngle => Ok(RegisterValue::Float(config.zero_electric_angle)),
        RegisterId::SensorDirection => Ok(RegisterValue::U8(config.sensor_direction as i8 as u8)),
        RegisterId::ZeroOffset => Ok(RegisterValue::Float(config.sensor_offset)),
        RegisterId::PolePairs => Ok(RegisterValue::U8(config.pole_pairs as u8)),
        RegisterId::PhaseResistance => Ok(RegisterValue::Float(config.phase_resistance)),
        RegisterId::KV => Ok(RegisterValue::Float(config.KV_rating)),
        RegisterId::Inductance => Ok(RegisterValue::Float(config.phase_inductance)),

        RegisterId::CurAGain
        | RegisterId::CurBGain
        | RegisterId::CurCGain
        | RegisterId::CurAOffset
        | RegisterId::CurBOffset
        | RegisterId::CurCOffset => Ok(RegisterValue::Float(0.0)),

        RegisterId::NumMotors => Ok(RegisterValue::U8(1)),
        RegisterId::SysTime => Ok(RegisterValue::U32(0)),

        _ => Err(RegisterError::InvalidPayload),
    }
}

/// Write a register value into [`MotorConfig`].
///
/// Enforces read-only validation: returns [`RegisterError::ReadOnly`] if `id` is read-only,
/// leaving `config` unmodified.
pub fn write_register<C: Clock>(
    config: &mut MotorConfig<C>,
    id: RegisterId,
    value: RegisterValue,
) -> Result<(), RegisterError> {
    if id.is_read_only() {
        return Err(RegisterError::ReadOnly(id));
    }

    match (id, value) {
        (RegisterId::EnableAll, RegisterValue::U8(val)) => {
            config.enabled = if val > 0 { 1 } else { 0 };
            Ok(())
        }
        (RegisterId::Enable, RegisterValue::U8(val)) => {
            config.enabled = if val > 0 { 1 } else { 0 };
            Ok(())
        }
        (RegisterId::ModulationMode, RegisterValue::U8(val)) => {
            config.foc_modulation =
                FOCModulationType::try_from(val).map_err(|_| RegisterError::InvalidPayload)?;
            Ok(())
        }
        (RegisterId::TorqueMode, RegisterValue::U8(val)) => {
            config.torque_controller =
                TorqueControlType::try_from(val).map_err(|_| RegisterError::InvalidPayload)?;
            Ok(())
        }
        (RegisterId::ControlMode, RegisterValue::U8(val)) => {
            config.controller =
                MotionControlType::try_from(val).map_err(|_| RegisterError::InvalidPayload)?;
            Ok(())
        }
        (RegisterId::Target, RegisterValue::Float(val)) => {
            config.target = val;
            Ok(())
        }

        (RegisterId::VelPidP, RegisterValue::Float(val)) => {
            config.PID_velocity.set_p(val);
            Ok(())
        }
        (RegisterId::VelPidI, RegisterValue::Float(val)) => {
            config.PID_velocity.set_i(val);
            Ok(())
        }
        (RegisterId::VelPidD, RegisterValue::Float(val)) => {
            config.PID_velocity.set_d(val);
            Ok(())
        }
        (RegisterId::VelPidLim, RegisterValue::Float(val)) => {
            config.PID_velocity.set_limit(val);
            Ok(())
        }
        (RegisterId::VelPidRamp, RegisterValue::Float(val)) => {
            config.PID_velocity.set_output_ramp(val);
            Ok(())
        }
        (RegisterId::VelLpfT, RegisterValue::Float(val)) => {
            config.LPF_velocity.set_tf(val);
            Ok(())
        }

        (RegisterId::AngPidP, RegisterValue::Float(val)) => {
            config.P_angle.set_p(val);
            Ok(())
        }
        (RegisterId::AngPidI, RegisterValue::Float(val)) => {
            config.P_angle.set_i(val);
            Ok(())
        }
        (RegisterId::AngPidD, RegisterValue::Float(val)) => {
            config.P_angle.set_d(val);
            Ok(())
        }
        (RegisterId::AngPidLim, RegisterValue::Float(val)) => {
            config.P_angle.set_limit(val);
            Ok(())
        }
        (RegisterId::AngPidRamp, RegisterValue::Float(val)) => {
            config.P_angle.set_output_ramp(val);
            Ok(())
        }
        (RegisterId::AngLpfT, RegisterValue::Float(val)) => {
            config.LPF_angle.set_tf(val);
            Ok(())
        }

        (RegisterId::CurQPidP, RegisterValue::Float(val)) => {
            config.PID_current_q.set_p(val);
            Ok(())
        }
        (RegisterId::CurQPidI, RegisterValue::Float(val)) => {
            config.PID_current_q.set_i(val);
            Ok(())
        }
        (RegisterId::CurQPidD, RegisterValue::Float(val)) => {
            config.PID_current_q.set_d(val);
            Ok(())
        }
        (RegisterId::CurQPidLim, RegisterValue::Float(val)) => {
            config.PID_current_q.set_limit(val);
            Ok(())
        }
        (RegisterId::CurQPidRamp, RegisterValue::Float(val)) => {
            config.PID_current_q.set_output_ramp(val);
            Ok(())
        }
        (RegisterId::CurQLpfT, RegisterValue::Float(val)) => {
            config.LPF_current_q.set_tf(val);
            Ok(())
        }

        (RegisterId::CurDPidP, RegisterValue::Float(val)) => {
            config.PID_current_d.set_p(val);
            Ok(())
        }
        (RegisterId::CurDPidI, RegisterValue::Float(val)) => {
            config.PID_current_d.set_i(val);
            Ok(())
        }
        (RegisterId::CurDPidD, RegisterValue::Float(val)) => {
            config.PID_current_d.set_d(val);
            Ok(())
        }
        (RegisterId::CurDPidLim, RegisterValue::Float(val)) => {
            config.PID_current_d.set_limit(val);
            Ok(())
        }
        (RegisterId::CurDPidRamp, RegisterValue::Float(val)) => {
            config.PID_current_d.set_output_ramp(val);
            Ok(())
        }
        (RegisterId::CurDLpfT, RegisterValue::Float(val)) => {
            config.LPF_current_d.set_tf(val);
            Ok(())
        }

        (RegisterId::VoltageLimit, RegisterValue::Float(val)) => {
            config.voltage_limit = val;
            if config.phase_resistance == NOT_SET {
                config.PID_velocity.set_limit(val);
            }
            Ok(())
        }
        (RegisterId::CurrentLimit, RegisterValue::Float(val)) => {
            config.current_limit = val;
            if config.phase_resistance != NOT_SET {
                config.PID_velocity.set_limit(val);
            }
            Ok(())
        }
        (RegisterId::VelocityLimit, RegisterValue::Float(val)) => {
            config.velocity_limit = val;
            Ok(())
        }
        (RegisterId::MotionDownsample, RegisterValue::U8(val)) => {
            config.motion_downsample = val as u32;
            Ok(())
        }
        (RegisterId::MotionDownsample, RegisterValue::U32(val)) => {
            config.motion_downsample = val;
            Ok(())
        }
        (RegisterId::VoltageSensorAlign, RegisterValue::Float(val)) => {
            config.voltage_sensor_align = val;
            Ok(())
        }
        (RegisterId::ZeroElectricAngle, RegisterValue::Float(val)) => {
            config.zero_electric_angle = val;
            Ok(())
        }
        (RegisterId::SensorDirection, RegisterValue::U8(val)) => {
            config.sensor_direction =
                Direction::try_from(val).map_err(|_| RegisterError::InvalidPayload)?;
            Ok(())
        }
        (RegisterId::ZeroOffset, RegisterValue::Float(val)) => {
            config.sensor_offset = val;
            Ok(())
        }
        (RegisterId::PolePairs, RegisterValue::U8(val)) => {
            config.pole_pairs = val as i32;
            Ok(())
        }
        (RegisterId::PolePairs, RegisterValue::U32(val)) => {
            config.pole_pairs = val as i32;
            Ok(())
        }
        (RegisterId::PolePairs, RegisterValue::I32(val)) => {
            config.pole_pairs = val;
            Ok(())
        }
        (RegisterId::PhaseResistance, RegisterValue::Float(val)) => {
            config.phase_resistance = val;
            Ok(())
        }
        (RegisterId::KV, RegisterValue::Float(val)) => {
            config.KV_rating = val;
            Ok(())
        }
        (RegisterId::Inductance, RegisterValue::Float(val)) => {
            config.phase_inductance = val;
            Ok(())
        }

        _ => Err(RegisterError::InvalidPayload),
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use barefoc::motor::base::NullClock;
    use barefoc::motor::enums::FOCMotorStatus;

    #[test]
    fn test_read_motor_config() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);
        config.voltage_limit = 12.0;
        config.target = 25.5;
        config.motor_status = FOCMotorStatus::motor_ready;
        config.shaft_angle = 1.23;

        // VoltageLimit
        assert_eq!(
            read_register(&config, RegisterId::VoltageLimit),
            Ok(RegisterValue::Float(12.0))
        );

        // Target
        assert_eq!(
            read_register(&config, RegisterId::Target),
            Ok(RegisterValue::Float(25.5))
        );

        // Status
        assert_eq!(
            read_register(&config, RegisterId::Status),
            Ok(RegisterValue::U8(FOCMotorStatus::motor_ready as u8))
        );

        // Angle
        assert_eq!(
            read_register(&config, RegisterId::Angle),
            Ok(RegisterValue::Float(1.23))
        );

        // Write-only registers reject read
        assert_eq!(
            read_register(&config, RegisterId::EnableAll),
            Err(RegisterError::WriteOnly(RegisterId::EnableAll))
        );
    }

    #[test]
    fn test_write_motor_config() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        // Target
        assert_eq!(
            write_register(&mut config, RegisterId::Target, RegisterValue::Float(20.0)),
            Ok(())
        );
        assert_eq!(config.target, 20.0);

        // VoltageLimit
        assert_eq!(
            write_register(
                &mut config,
                RegisterId::VoltageLimit,
                RegisterValue::Float(15.0)
            ),
            Ok(())
        );
        assert_eq!(config.voltage_limit, 15.0);

        // Enable
        assert_eq!(
            write_register(&mut config, RegisterId::Enable, RegisterValue::U8(1)),
            Ok(())
        );
        assert_eq!(config.enabled, 1);

        // ControlMode
        assert_eq!(
            write_register(
                &mut config,
                RegisterId::ControlMode,
                RegisterValue::U8(MotionControlType::velocity as u8)
            ),
            Ok(())
        );
        assert_eq!(config.controller, MotionControlType::velocity);

        // VelPidP
        assert_eq!(
            write_register(&mut config, RegisterId::VelPidP, RegisterValue::Float(0.5)),
            Ok(())
        );
        assert_eq!(config.PID_velocity.p(), 0.5);

        // Pole pairs
        assert_eq!(
            write_register(&mut config, RegisterId::PolePairs, RegisterValue::U8(7)),
            Ok(())
        );
        assert_eq!(config.pole_pairs, 7);
    }

    #[test]
    fn test_write_read_only_rejection() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);
        config.shaft_angle = 1.0;

        // Writing to read-only Angle
        assert_eq!(
            write_register(&mut config, RegisterId::Angle, RegisterValue::Float(5.0)),
            Err(RegisterError::ReadOnly(RegisterId::Angle))
        );
        // Ensure state wasn't mutated
        assert_eq!(config.shaft_angle, 1.0);

        // Writing to read-only Status
        assert_eq!(
            write_register(&mut config, RegisterId::Status, RegisterValue::U8(4)),
            Err(RegisterError::ReadOnly(RegisterId::Status))
        );

        // Writing to read-only VoltageQ
        assert_eq!(
            write_register(
                &mut config,
                RegisterId::VoltageQ,
                RegisterValue::Float(12.0)
            ),
            Err(RegisterError::ReadOnly(RegisterId::VoltageQ))
        );
    }

    #[test]
    fn test_write_invalid_payload() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);

        // Target expects Float, passing U8
        assert_eq!(
            write_register(&mut config, RegisterId::Target, RegisterValue::U8(5)),
            Err(RegisterError::InvalidPayload)
        );

        // ModulationMode invalid enum index (e.g. 99)
        assert_eq!(
            write_register(&mut config, RegisterId::ModulationMode, RegisterValue::U8(99)),
            Err(RegisterError::InvalidPayload)
        );
    }
}

/// Trait exposing register-level read/write for any SimpleFOC object.
pub trait RegisterBridge {
    /// Read a register value.
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError>;
    /// Write a register value.
    fn write_reg(&mut self, id: RegisterId, val: RegisterValue) -> Result<(), RegisterError>;
}

impl<C: barefoc::common::time::Clock> RegisterBridge for MotorConfig<C> {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        read_register(self, id)
    }
    fn write_reg(&mut self, id: RegisterId, val: RegisterValue) -> Result<(), RegisterError> {
        write_register(self, id, val)
    }
}

// ---------------------------------------------------------------------------
// RegisterBridge impls for the concrete motor types (task 1.4)
// ---------------------------------------------------------------------------

use barefoc::motor::base::FOCMotor;
use barefoc::motor::bldc::BLDCMotor;
use barefoc::motor::stepper::StepperMotor;
use barefoc::motor::hybrid_stepper::HybridStepperMotor;
use barefoc::traits::driver::{BLDCDriver, StepperDriver};

macro_rules! motor_register_bridge {
    ($MotorType:ident, $DriverBound:ident) => {
        impl<'clk, S, D, CS, C> RegisterBridge for $MotorType<'clk, S, D, CS, C>
        where
            S: barefoc::traits::sensor::Sensor<C>,
            D: $DriverBound,
            CS: barefoc::traits::current_sense::CurrentSense,
            C: barefoc::common::time::Clock,
        {
            fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
                match id {
                    RegisterId::DriverVoltageLimit => Ok(RegisterValue::Float(
                        self.driver().map(|d| d.params().voltage_limit).unwrap_or(0.0),
                    )),
                    RegisterId::PwmFrequency => Ok(RegisterValue::U32(
                        self.driver().map(|d| d.params().pwm_frequency).unwrap_or(0),
                    )),
                    RegisterId::DriverVoltagePsu => Ok(RegisterValue::Float(
                        self.driver().map(|d| d.params().voltage_power_supply).unwrap_or(0.0),
                    )),
                    RegisterId::CurAGain => Ok(RegisterValue::Float(
                        self.current_sense().map(|cs| cs.base().gain_a).unwrap_or(0.0),
                    )),
                    RegisterId::CurBGain => Ok(RegisterValue::Float(
                        self.current_sense().map(|cs| cs.base().gain_b).unwrap_or(0.0),
                    )),
                    RegisterId::CurCGain => Ok(RegisterValue::Float(
                        self.current_sense().map(|cs| cs.base().gain_c).unwrap_or(0.0),
                    )),
                    RegisterId::CurAOffset => Ok(RegisterValue::Float(
                        self.current_sense().map(|cs| cs.base().offset_ia).unwrap_or(0.0),
                    )),
                    RegisterId::CurBOffset => Ok(RegisterValue::Float(
                        self.current_sense().map(|cs| cs.base().offset_ib).unwrap_or(0.0),
                    )),
                    RegisterId::CurCOffset => Ok(RegisterValue::Float(
                        self.current_sense().map(|cs| cs.base().offset_ic).unwrap_or(0.0),
                    )),
                    _ => read_register(self.cfg(), id),
                }
            }

            fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
                match id {
                    RegisterId::DriverVoltageLimit => {
                        if let RegisterValue::Float(v) = value {
                            if let Some(d) = self.driver_mut() {
                                d.params_mut().voltage_limit = v;
                            }
                        }
                        Ok(())
                    }
                    RegisterId::PwmFrequency => {
                        if let RegisterValue::U32(v) = value {
                            if let Some(d) = self.driver_mut() {
                                d.params_mut().pwm_frequency = v;
                            }
                        }
                        Ok(())
                    }
                    RegisterId::DriverVoltagePsu => {
                        if let RegisterValue::Float(v) = value {
                            if let Some(d) = self.driver_mut() {
                                d.params_mut().voltage_power_supply = v;
                            }
                        }
                        Ok(())
                    }
                    RegisterId::CurAGain => {
                        if let RegisterValue::Float(v) = value {
                            if let Some(cs) = self.current_sense_mut() {
                                cs.base_mut().gain_a = v;
                            }
                        }
                        Ok(())
                    }
                    RegisterId::CurBGain => {
                        if let RegisterValue::Float(v) = value {
                            if let Some(cs) = self.current_sense_mut() {
                                cs.base_mut().gain_b = v;
                            }
                        }
                        Ok(())
                    }
                    RegisterId::CurCGain => {
                        if let RegisterValue::Float(v) = value {
                            if let Some(cs) = self.current_sense_mut() {
                                cs.base_mut().gain_c = v;
                            }
                        }
                        Ok(())
                    }
                    RegisterId::CurAOffset => {
                        if let RegisterValue::Float(v) = value {
                            if let Some(cs) = self.current_sense_mut() {
                                cs.base_mut().offset_ia = v;
                            }
                        }
                        Ok(())
                    }
                    RegisterId::CurBOffset => {
                        if let RegisterValue::Float(v) = value {
                            if let Some(cs) = self.current_sense_mut() {
                                cs.base_mut().offset_ib = v;
                            }
                        }
                        Ok(())
                    }
                    RegisterId::CurCOffset => {
                        if let RegisterValue::Float(v) = value {
                            if let Some(cs) = self.current_sense_mut() {
                                cs.base_mut().offset_ic = v;
                            }
                        }
                        Ok(())
                    }
                    _ => write_register(self.cfg_mut(), id, value),
                }
            }
        }
    };
}

motor_register_bridge!(BLDCMotor, BLDCDriver);
motor_register_bridge!(StepperMotor, StepperDriver);
motor_register_bridge!(HybridStepperMotor, BLDCDriver);
