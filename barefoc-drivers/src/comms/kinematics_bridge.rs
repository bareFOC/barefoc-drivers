//! # Kinematics register bridge (feature `kinematics`)
//!
//! Implements [`RegisterBridge`] for
//! [`barefoc::kinematics::actuator::Actuator1DOF`] and
//! [`barefoc::kinematics::transmission::LinearGear`].

#[cfg(feature = "kinematics")]
use barefoc::common::time::Clock;
#[cfg(feature = "kinematics")]
use crate::comms::{RegisterBridge, RegisterError, RegisterId, RegisterValue};
#[cfg(feature = "kinematics")]
use barefoc::kinematics::actuator::Actuator1DOF;
#[cfg(feature = "kinematics")]
use barefoc::kinematics::transmission::LinearGear;

#[cfg(feature = "kinematics")]
impl<M, T, SOut, C> RegisterBridge for Actuator1DOF<M, T, SOut, C>
where
    M: RegisterBridge,
    T: RegisterBridge,
    C: Clock,
{
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::GearRatio
            | RegisterId::BacklashDelta
            | RegisterId::OmegaThresh
            | RegisterId::TorsionalStiffness
            | RegisterId::ZeroOffsetJoint => self.transmission.read_reg(id),
            RegisterId::OuterPidP => Ok(RegisterValue::Float(self.pid_position.p())),
            RegisterId::OuterPidI => Ok(RegisterValue::Float(self.pid_position.i())),
            RegisterId::OuterPidD => Ok(RegisterValue::Float(self.pid_position.d())),
            RegisterId::OuterPidLim => Ok(RegisterValue::Float(self.pid_position.limit())),
            RegisterId::OuterPidRamp => Ok(RegisterValue::Float(self.pid_position.output_ramp())),
            RegisterId::CalibState => Ok(RegisterValue::U8(self.calib_state)),
            RegisterId::CalibProgress => Ok(RegisterValue::U8(self.calib_progress)),
            _ => self.motor.read_reg(id),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match id {
            RegisterId::GearRatio => {
                if let RegisterValue::Float(v) = value {
                    self.transmission.write_reg(RegisterId::GearRatio, RegisterValue::Float(v))?;
                }
                Ok(())
            }
            RegisterId::BacklashDelta => {
                if let RegisterValue::Float(v) = value {
                    self.transmission.write_reg(RegisterId::BacklashDelta, RegisterValue::Float(v))?;
                }
                Ok(())
            }
            RegisterId::OmegaThresh => {
                if let RegisterValue::Float(v) = value {
                    self.transmission.write_reg(RegisterId::OmegaThresh, RegisterValue::Float(v))?;
                }
                Ok(())
            }
            RegisterId::TorsionalStiffness => {
                if let RegisterValue::Float(v) = value {
                    self.transmission.write_reg(RegisterId::TorsionalStiffness, RegisterValue::Float(v))?;
                }
                Ok(())
            }
            RegisterId::ZeroOffsetJoint => {
                if let RegisterValue::Float(v) = value {
                    self.transmission.write_reg(RegisterId::ZeroOffsetJoint, RegisterValue::Float(v))?;
                }
                Ok(())
            }
            RegisterId::OuterPidP => {
                if let RegisterValue::Float(v) = value { self.pid_position.set_p(v); }
                Ok(())
            }
            RegisterId::OuterPidI => {
                if let RegisterValue::Float(v) = value { self.pid_position.set_i(v); }
                Ok(())
            }
            RegisterId::OuterPidD => {
                if let RegisterValue::Float(v) = value { self.pid_position.set_d(v); }
                Ok(())
            }
            RegisterId::OuterPidLim => {
                if let RegisterValue::Float(v) = value { self.pid_position.set_limit(v); }
                Ok(())
            }
            RegisterId::OuterPidRamp => {
                if let RegisterValue::Float(v) = value { self.pid_position.set_output_ramp(v); }
                Ok(())
            }
            RegisterId::CalibState | RegisterId::CalibProgress => {
                Err(RegisterError::ReadOnly(id))
            }
            _ => self.motor.write_reg(id, value),
        }
    }
}

#[cfg(feature = "kinematics")]
impl RegisterBridge for LinearGear {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::GearRatio => Ok(RegisterValue::Float(self.gear_ratio)),
            RegisterId::BacklashDelta => Ok(RegisterValue::Float(self.backlash_delta)),
            RegisterId::OmegaThresh => Ok(RegisterValue::Float(self.omega_thresh)),
            RegisterId::TorsionalStiffness => Ok(RegisterValue::Float(self.torsional_stiffness)),
            RegisterId::ZeroOffsetJoint => Ok(RegisterValue::Float(self.zero_offset)),
            _ => Err(RegisterError::InvalidPayload),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match (id, value) {
            (RegisterId::GearRatio, RegisterValue::Float(v)) => { self.gear_ratio = v; Ok(()) }
            (RegisterId::BacklashDelta, RegisterValue::Float(v)) => { self.backlash_delta = v; Ok(()) }
            (RegisterId::OmegaThresh, RegisterValue::Float(v)) => { self.omega_thresh = v; Ok(()) }
            (RegisterId::TorsionalStiffness, RegisterValue::Float(v)) => { self.torsional_stiffness = v; Ok(()) }
            (RegisterId::ZeroOffsetJoint, RegisterValue::Float(v)) => { self.zero_offset = v; Ok(()) }
            _ => Err(RegisterError::InvalidPayload),
        }
    }
}