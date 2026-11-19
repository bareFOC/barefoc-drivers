//! # Kinematics register bridge (feature `kinematics`)
//!
//! Implements [`RegisterBridge`] for:
//! - [`barefoc::kinematics::actuator::Actuator1DOF`]
//! - [`barefoc::kinematics::actuator::Actuator2DOF`]
//! - [`barefoc::kinematics::transmission::LinearGear`]
//! - [`barefoc::kinematics::transmission::CoupledDifferential`]
//! - [`barefoc::kinematics::transmission::LutTransmission`]
//! - [`barefoc::kinematics::transmission::FourBarLinkage`]
//! - [`barefoc::kinematics::transmission::EccentricCam`]
//! - [`barefoc::kinematics::transmission::AnalyticalLinkage`]

#[cfg(feature = "kinematics")]
use barefoc::common::time::Clock;
#[cfg(feature = "kinematics")]
use crate::comms::{RegisterBridge, RegisterError, RegisterId, RegisterValue};
#[cfg(feature = "kinematics")]
use barefoc::kinematics::actuator::{Actuator1DOF, Actuator2DOF};
#[cfg(feature = "kinematics")]
use barefoc::kinematics::sensor::Sensor2D;
#[cfg(feature = "kinematics")]
use barefoc::kinematics::transmission::{
    AnalyticalLinkage, CoupledDifferential, EccentricCam, FourBarLinkage, LinearGear,
    LutTransmission,
};

#[cfg(feature = "kinematics")]
impl<M, T, SOut, C> RegisterBridge for Actuator1DOF<M, T, SOut, C>
where
    M: RegisterBridge,
    T: RegisterBridge,
    C: Clock,
{
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::JointTargetAlpha => Ok(RegisterValue::Float(self.target_joint_angle)),
            RegisterId::JointAngleAlpha => Ok(RegisterValue::Float(self.current_joint_angle)),
            RegisterId::OuterDownsample => Ok(RegisterValue::U8(self.outer_downsample as u8)),
            RegisterId::OuterPidAlpha_P => Ok(RegisterValue::Float(self.pid_position.p())),
            RegisterId::OuterPidAlpha_I => Ok(RegisterValue::Float(self.pid_position.i())),
            RegisterId::OuterPidAlpha_D => Ok(RegisterValue::Float(self.pid_position.d())),
            RegisterId::OuterPidAlpha_Lim => Ok(RegisterValue::Float(self.pid_position.limit())),
            RegisterId::OuterPidAlpha_Ramp => Ok(RegisterValue::Float(self.pid_position.output_ramp())),
            RegisterId::CalibState => Ok(RegisterValue::U8(self.calib_state)),
            RegisterId::CalibProgress => Ok(RegisterValue::U8(self.calib_progress)),

            // Transmission registers
            RegisterId::TransmissionType
            | RegisterId::GearRatio
            | RegisterId::BacklashDelta
            | RegisterId::OmegaThresh
            | RegisterId::TorsionalStiffness
            | RegisterId::JointOffsetAlpha
            | RegisterId::JointOffsetBeta
            | RegisterId::DiffMatrix_T11
            | RegisterId::DiffMatrix_T12
            | RegisterId::DiffMatrix_T21
            | RegisterId::DiffMatrix_T22
            | RegisterId::LinkageL1
            | RegisterId::LinkageL2_L4
            | RegisterId::LutMotMin
            | RegisterId::LutMotMax
            | RegisterId::LutPoints
            | RegisterId::LutPointIndex
            | RegisterId::LutPointValue
            | RegisterId::LutStatus => self.transmission.read_reg(id),

            // Inner motor registers
            _ => self.motor.read_reg(id),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match id {
            RegisterId::JointTargetAlpha => {
                if let RegisterValue::Float(v) = value {
                    self.target_joint_angle = v;
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::JointAngleAlpha => Err(RegisterError::ReadOnly(id)),
            RegisterId::OuterDownsample => {
                match value {
                    RegisterValue::U8(v) => {
                        self.outer_downsample = if v == 0 { 1 } else { v as u32 };
                        Ok(())
                    }
                    RegisterValue::U32(v) => {
                        self.outer_downsample = if v == 0 { 1 } else { v };
                        Ok(())
                    }
                    _ => Err(RegisterError::InvalidPayload),
                }
            }
            RegisterId::OuterPidAlpha_P => {
                if let RegisterValue::Float(v) = value {
                    self.pid_position.set_p(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidAlpha_I => {
                if let RegisterValue::Float(v) = value {
                    self.pid_position.set_i(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidAlpha_D => {
                if let RegisterValue::Float(v) = value {
                    self.pid_position.set_d(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidAlpha_Lim => {
                if let RegisterValue::Float(v) = value {
                    self.pid_position.set_limit(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidAlpha_Ramp => {
                if let RegisterValue::Float(v) = value {
                    self.pid_position.set_output_ramp(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::CalibState | RegisterId::CalibProgress => Err(RegisterError::ReadOnly(id)),

            // Transmission registers
            RegisterId::TransmissionType
            | RegisterId::GearRatio
            | RegisterId::BacklashDelta
            | RegisterId::OmegaThresh
            | RegisterId::TorsionalStiffness
            | RegisterId::JointOffsetAlpha
            | RegisterId::JointOffsetBeta
            | RegisterId::DiffMatrix_T11
            | RegisterId::DiffMatrix_T12
            | RegisterId::DiffMatrix_T21
            | RegisterId::DiffMatrix_T22
            | RegisterId::LinkageL1
            | RegisterId::LinkageL2_L4
            | RegisterId::LutMotMin
            | RegisterId::LutMotMax
            | RegisterId::LutPoints
            | RegisterId::LutPointIndex
            | RegisterId::LutPointValue
            | RegisterId::LutStatus => self.transmission.write_reg(id, value),

            // Inner motor registers
            _ => self.motor.write_reg(id, value),
        }
    }
}

#[cfg(feature = "kinematics")]
impl<M1, M2, T, S2D, C> RegisterBridge for Actuator2DOF<M1, M2, T, S2D, C>
where
    M1: RegisterBridge,
    M2: RegisterBridge,
    T: RegisterBridge,
    S2D: Sensor2D<C>,
    C: Clock,
{
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::MotorSelect => Ok(RegisterValue::U8(self.motor_selected)),
            RegisterId::NumMotors => Ok(RegisterValue::U8(2)),
            RegisterId::JointTargetAlpha => Ok(RegisterValue::Float(self.target_joint_angles[0])),
            RegisterId::JointTargetBeta => Ok(RegisterValue::Float(self.target_joint_angles[1])),
            RegisterId::JointAngleAlpha => Ok(RegisterValue::Float(self.current_joint_angles[0])),
            RegisterId::JointAngleBeta => Ok(RegisterValue::Float(self.current_joint_angles[1])),
            RegisterId::JointVelocityAlpha => {
                Ok(RegisterValue::Float(self.sensor_2d.get_velocities()[0]))
            }
            RegisterId::JointVelocityBeta => {
                Ok(RegisterValue::Float(self.sensor_2d.get_velocities()[1]))
            }
            RegisterId::OuterDownsample => Ok(RegisterValue::U8(self.outer_downsample as u8)),
            RegisterId::OuterPidAlpha_P => Ok(RegisterValue::Float(self.pid_alpha.p())),
            RegisterId::OuterPidAlpha_I => Ok(RegisterValue::Float(self.pid_alpha.i())),
            RegisterId::OuterPidAlpha_D => Ok(RegisterValue::Float(self.pid_alpha.d())),
            RegisterId::OuterPidAlpha_Lim => Ok(RegisterValue::Float(self.pid_alpha.limit())),
            RegisterId::OuterPidAlpha_Ramp => Ok(RegisterValue::Float(self.pid_alpha.output_ramp())),
            RegisterId::OuterPidBeta_P => Ok(RegisterValue::Float(self.pid_beta.p())),
            RegisterId::OuterPidBeta_I => Ok(RegisterValue::Float(self.pid_beta.i())),
            RegisterId::OuterPidBeta_D => Ok(RegisterValue::Float(self.pid_beta.d())),
            RegisterId::OuterPidBeta_Lim => Ok(RegisterValue::Float(self.pid_beta.limit())),
            RegisterId::OuterPidBeta_Ramp => Ok(RegisterValue::Float(self.pid_beta.output_ramp())),

            // Transmission registers
            RegisterId::TransmissionType
            | RegisterId::GearRatio
            | RegisterId::BacklashDelta
            | RegisterId::OmegaThresh
            | RegisterId::TorsionalStiffness
            | RegisterId::JointOffsetAlpha
            | RegisterId::JointOffsetBeta
            | RegisterId::DiffMatrix_T11
            | RegisterId::DiffMatrix_T12
            | RegisterId::DiffMatrix_T21
            | RegisterId::DiffMatrix_T22
            | RegisterId::LinkageL1
            | RegisterId::LinkageL2_L4
            | RegisterId::LutMotMin
            | RegisterId::LutMotMax
            | RegisterId::LutPoints
            | RegisterId::LutPointIndex
            | RegisterId::LutPointValue
            | RegisterId::LutStatus => self.transmission.read_reg(id),

            // Motor window registers: route based on motor_selected
            _ => {
                if self.motor_selected == 0 {
                    self.motor1.read_reg(id)
                } else {
                    self.motor2.read_reg(id)
                }
            }
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match id {
            RegisterId::MotorSelect => {
                match value {
                    RegisterValue::U8(sel) => {
                        self.motor_selected = sel;
                        Ok(())
                    }
                    _ => Err(RegisterError::InvalidPayload),
                }
            }
            RegisterId::NumMotors => Err(RegisterError::ReadOnly(id)),
            RegisterId::JointTargetAlpha => {
                if let RegisterValue::Float(v) = value {
                    self.target_joint_angles[0] = v;
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::JointTargetBeta => {
                if let RegisterValue::Float(v) = value {
                    self.target_joint_angles[1] = v;
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::JointAngleAlpha
            | RegisterId::JointAngleBeta
            | RegisterId::JointVelocityAlpha
            | RegisterId::JointVelocityBeta => Err(RegisterError::ReadOnly(id)),
            RegisterId::OuterDownsample => {
                match value {
                    RegisterValue::U8(v) => {
                        self.outer_downsample = if v == 0 { 1 } else { v as u32 };
                        Ok(())
                    }
                    RegisterValue::U32(v) => {
                        self.outer_downsample = if v == 0 { 1 } else { v };
                        Ok(())
                    }
                    _ => Err(RegisterError::InvalidPayload),
                }
            }
            RegisterId::OuterPidAlpha_P => {
                if let RegisterValue::Float(v) = value {
                    self.pid_alpha.set_p(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidAlpha_I => {
                if let RegisterValue::Float(v) = value {
                    self.pid_alpha.set_i(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidAlpha_D => {
                if let RegisterValue::Float(v) = value {
                    self.pid_alpha.set_d(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidAlpha_Lim => {
                if let RegisterValue::Float(v) = value {
                    self.pid_alpha.set_limit(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidAlpha_Ramp => {
                if let RegisterValue::Float(v) = value {
                    self.pid_alpha.set_output_ramp(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidBeta_P => {
                if let RegisterValue::Float(v) = value {
                    self.pid_beta.set_p(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidBeta_I => {
                if let RegisterValue::Float(v) = value {
                    self.pid_beta.set_i(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidBeta_D => {
                if let RegisterValue::Float(v) = value {
                    self.pid_beta.set_d(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidBeta_Lim => {
                if let RegisterValue::Float(v) = value {
                    self.pid_beta.set_limit(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::OuterPidBeta_Ramp => {
                if let RegisterValue::Float(v) = value {
                    self.pid_beta.set_output_ramp(v);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }

            // Transmission registers
            RegisterId::TransmissionType
            | RegisterId::GearRatio
            | RegisterId::BacklashDelta
            | RegisterId::OmegaThresh
            | RegisterId::TorsionalStiffness
            | RegisterId::JointOffsetAlpha
            | RegisterId::JointOffsetBeta
            | RegisterId::DiffMatrix_T11
            | RegisterId::DiffMatrix_T12
            | RegisterId::DiffMatrix_T21
            | RegisterId::DiffMatrix_T22
            | RegisterId::LinkageL1
            | RegisterId::LinkageL2_L4
            | RegisterId::LutMotMin
            | RegisterId::LutMotMax
            | RegisterId::LutPoints
            | RegisterId::LutPointIndex
            | RegisterId::LutPointValue
            | RegisterId::LutStatus => self.transmission.write_reg(id, value),

            // Motor window registers: route based on motor_selected
            _ => {
                if self.motor_selected == 0 {
                    self.motor1.write_reg(id, value)
                } else {
                    self.motor2.write_reg(id, value)
                }
            }
        }
    }
}

#[cfg(feature = "kinematics")]
impl RegisterBridge for LinearGear {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::TransmissionType => Ok(RegisterValue::U8(0)),
            RegisterId::GearRatio => Ok(RegisterValue::Float(self.gear_ratio)),
            RegisterId::BacklashDelta => Ok(RegisterValue::Float(self.backlash_delta)),
            RegisterId::OmegaThresh => Ok(RegisterValue::Float(self.omega_thresh)),
            RegisterId::TorsionalStiffness => Ok(RegisterValue::Float(self.torsional_stiffness)),
            RegisterId::JointOffsetAlpha => Ok(RegisterValue::Float(self.zero_offset)),
            _ => Err(RegisterError::InvalidPayload),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match (id, value) {
            (RegisterId::TransmissionType, RegisterValue::U8(_)) => Ok(()),
            (RegisterId::GearRatio, RegisterValue::Float(v)) => {
                self.gear_ratio = if v == 0.0 { 1.0 } else { v.abs() };
                Ok(())
            }
            (RegisterId::BacklashDelta, RegisterValue::Float(v)) => {
                self.backlash_delta = v.abs();
                Ok(())
            }
            (RegisterId::OmegaThresh, RegisterValue::Float(v)) => {
                self.omega_thresh = if v <= 0.0 { 0.05 } else { v };
                Ok(())
            }
            (RegisterId::TorsionalStiffness, RegisterValue::Float(v)) => {
                self.torsional_stiffness = if v < 0.0 { 0.0 } else { v };
                Ok(())
            }
            (RegisterId::JointOffsetAlpha, RegisterValue::Float(v)) => {
                self.zero_offset = v;
                Ok(())
            }
            _ => Err(RegisterError::InvalidPayload),
        }
    }
}

#[cfg(feature = "kinematics")]
impl RegisterBridge for CoupledDifferential {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::TransmissionType => Ok(RegisterValue::U8(1)),
            RegisterId::JointOffsetAlpha => Ok(RegisterValue::Float(self.joint_offsets[0])),
            RegisterId::JointOffsetBeta => Ok(RegisterValue::Float(self.joint_offsets[1])),
            RegisterId::DiffMatrix_T11 => Ok(RegisterValue::Float(self.matrix[0][0])),
            RegisterId::DiffMatrix_T12 => Ok(RegisterValue::Float(self.matrix[0][1])),
            RegisterId::DiffMatrix_T21 => Ok(RegisterValue::Float(self.matrix[1][0])),
            RegisterId::DiffMatrix_T22 => Ok(RegisterValue::Float(self.matrix[1][1])),
            _ => Err(RegisterError::InvalidPayload),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match (id, value) {
            (RegisterId::TransmissionType, RegisterValue::U8(_)) => Ok(()),
            (RegisterId::JointOffsetAlpha, RegisterValue::Float(v)) => {
                self.joint_offsets[0] = v;
                Ok(())
            }
            (RegisterId::JointOffsetBeta, RegisterValue::Float(v)) => {
                self.joint_offsets[1] = v;
                Ok(())
            }
            (RegisterId::DiffMatrix_T11, RegisterValue::Float(v)) => {
                self.matrix[0][0] = v;
                Ok(())
            }
            (RegisterId::DiffMatrix_T12, RegisterValue::Float(v)) => {
                self.matrix[0][1] = v;
                Ok(())
            }
            (RegisterId::DiffMatrix_T21, RegisterValue::Float(v)) => {
                self.matrix[1][0] = v;
                Ok(())
            }
            (RegisterId::DiffMatrix_T22, RegisterValue::Float(v)) => {
                self.matrix[1][1] = v;
                Ok(())
            }
            _ => Err(RegisterError::InvalidPayload),
        }
    }
}

#[cfg(feature = "kinematics")]
impl<const N: usize> RegisterBridge for LutTransmission<N> {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::TransmissionType => Ok(RegisterValue::U8(2)),
            RegisterId::LutMotMin => Ok(RegisterValue::Float(self.mot_min)),
            RegisterId::LutMotMax => Ok(RegisterValue::Float(self.mot_max)),
            RegisterId::LutPoints => Ok(RegisterValue::U16(N as u16)),
            RegisterId::LutPointIndex => Ok(RegisterValue::U16(self.point_index)),
            RegisterId::LutPointValue => {
                let idx = self.point_index as usize;
                if idx < N {
                    Ok(RegisterValue::Float(self.table[idx]))
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            RegisterId::LutStatus => {
                let status = if self.validate_monotonicity() { 1u8 } else { 0u8 };
                Ok(RegisterValue::U8(status))
            }
            _ => Err(RegisterError::InvalidPayload),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match (id, value) {
            (RegisterId::TransmissionType, RegisterValue::U8(_)) => Ok(()),
            (RegisterId::LutMotMin, RegisterValue::Float(v)) => {
                self.mot_min = v;
                Ok(())
            }
            (RegisterId::LutMotMax, RegisterValue::Float(v)) => {
                self.mot_max = v;
                Ok(())
            }
            (RegisterId::LutPoints, _) => Err(RegisterError::ReadOnly(id)),
            (RegisterId::LutPointIndex, RegisterValue::U16(idx)) => {
                self.point_index = idx;
                Ok(())
            }
            (RegisterId::LutPointIndex, RegisterValue::U8(idx)) => {
                self.point_index = idx as u16;
                Ok(())
            }
            (RegisterId::LutPointIndex, RegisterValue::U32(idx)) => {
                self.point_index = idx as u16;
                Ok(())
            }
            (RegisterId::LutPointValue, RegisterValue::Float(val)) => {
                let idx = self.point_index as usize;
                if idx < N {
                    self.table[idx] = val;
                    self.point_index = self.point_index.saturating_add(1);
                    Ok(())
                } else {
                    Err(RegisterError::InvalidPayload)
                }
            }
            (RegisterId::LutStatus, _) => Err(RegisterError::ReadOnly(id)),
            _ => Err(RegisterError::InvalidPayload),
        }
    }
}

#[cfg(feature = "kinematics")]
impl RegisterBridge for FourBarLinkage {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::TransmissionType => Ok(RegisterValue::U8(3)),
            RegisterId::LinkageL1 => Ok(RegisterValue::Float(self.l1)),
            RegisterId::LinkageL2_L4 => Ok(RegisterValue::Float3([self.l2, self.l3, self.l4])),
            _ => Err(RegisterError::InvalidPayload),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match (id, value) {
            (RegisterId::TransmissionType, RegisterValue::U8(_)) => Ok(()),
            (RegisterId::LinkageL1, RegisterValue::Float(v)) => {
                self.l1 = v.abs();
                Ok(())
            }
            (RegisterId::LinkageL2_L4, RegisterValue::Float3([l2, l3, l4])) => {
                self.l2 = l2.abs();
                self.l3 = l3.abs();
                self.l4 = l4.abs();
                Ok(())
            }
            _ => Err(RegisterError::InvalidPayload),
        }
    }
}

#[cfg(feature = "kinematics")]
impl RegisterBridge for EccentricCam {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::TransmissionType => Ok(RegisterValue::U8(4)),
            RegisterId::LinkageL1 => Ok(RegisterValue::Float(self.arm_length)),
            RegisterId::LinkageL2_L4 => Ok(RegisterValue::Float3([self.eccentricity, 0.0, 0.0])),
            _ => Err(RegisterError::InvalidPayload),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match (id, value) {
            (RegisterId::TransmissionType, RegisterValue::U8(_)) => Ok(()),
            (RegisterId::LinkageL1, RegisterValue::Float(v)) => {
                self.arm_length = if v <= 0.0 { 1.0 } else { v };
                self.eccentricity = self.eccentricity.min(self.arm_length * 0.99);
                Ok(())
            }
            (RegisterId::LinkageL2_L4, RegisterValue::Float3([e, ..])) => {
                self.eccentricity = e.abs().min(self.arm_length * 0.99);
                Ok(())
            }
            (RegisterId::LinkageL2_L4, RegisterValue::Float(e)) => {
                self.eccentricity = e.abs().min(self.arm_length * 0.99);
                Ok(())
            }
            _ => Err(RegisterError::InvalidPayload),
        }
    }
}

#[cfg(feature = "kinematics")]
impl RegisterBridge for AnalyticalLinkage {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match self {
            AnalyticalLinkage::FourBar(fb) => fb.read_reg(id),
            AnalyticalLinkage::Cam(cam) => cam.read_reg(id),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match self {
            AnalyticalLinkage::FourBar(fb) => fb.write_reg(id, value),
            AnalyticalLinkage::Cam(cam) => cam.write_reg(id, value),
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(all(test, feature = "kinematics"))]
mod tests {
    use super::*;
    use barefoc::common::pid::PIDController;
    use barefoc::kinematics::sensor::MockSensor2D;
    use barefoc::motor::base::{MotorConfig, NullClock};
    use barefoc::traits::sensor::{Sensor, SensorBase};

    struct MockSensor {
        base: SensorBase,
    }

    impl MockSensor {
        fn new() -> Self {
            Self {
                base: SensorBase::new(),
            }
        }
    }

    impl<C: Clock> Sensor<C> for MockSensor {
        fn base(&self) -> &SensorBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut SensorBase {
            &mut self.base
        }
        fn get_sensor_angle(&self) -> f32 {
            0.0
        }
    }

    #[test]
    fn test_actuator_1dof_register_bridge() {
        let clock = NullClock;
        let motor = MotorConfig::new(&clock);
        let transmission = LinearGear::new(2.0);
        let sensor = MockSensor::new();
        let pid = PIDController::new(10.0, 1.0, 0.1, 100.0, 20.0, 0.001, &clock);
        let mut act1 = Actuator1DOF::new(motor, transmission, sensor, pid);

        // 1. JointTargetAlpha sets target_joint_angle and isolates motor target (0x01)
        assert_eq!(act1.read_reg(RegisterId::JointTargetAlpha), Ok(RegisterValue::Float(0.0)));
        assert_eq!(
            act1.write_reg(RegisterId::JointTargetAlpha, RegisterValue::Float(1.57)),
            Ok(())
        );
        assert_eq!(act1.target_joint_angle, 1.57);
        assert_eq!(act1.read_reg(RegisterId::JointTargetAlpha), Ok(RegisterValue::Float(1.57)));
        // Inner motor target unchanged
        assert_eq!(act1.motor.target, 0.0);

        // 2. JointAngleAlpha returns current_joint_angle
        act1.current_joint_angle = 0.785;
        assert_eq!(act1.read_reg(RegisterId::JointAngleAlpha), Ok(RegisterValue::Float(0.785)));
        assert_eq!(
            act1.write_reg(RegisterId::JointAngleAlpha, RegisterValue::Float(0.0)),
            Err(RegisterError::ReadOnly(RegisterId::JointAngleAlpha))
        );

        // 3. OuterDownsample
        assert_eq!(act1.read_reg(RegisterId::OuterDownsample), Ok(RegisterValue::U8(10)));
        assert_eq!(
            act1.write_reg(RegisterId::OuterDownsample, RegisterValue::U8(5)),
            Ok(())
        );
        assert_eq!(act1.outer_downsample, 5);

        // 4. OuterPidAlpha_* gains
        assert_eq!(
            act1.write_reg(RegisterId::OuterPidAlpha_P, RegisterValue::Float(15.0)),
            Ok(())
        );
        assert_eq!(act1.pid_position.p(), 15.0);
        assert_eq!(act1.read_reg(RegisterId::OuterPidAlpha_P), Ok(RegisterValue::Float(15.0)));

        assert_eq!(
            act1.write_reg(RegisterId::OuterPidAlpha_I, RegisterValue::Float(2.5)),
            Ok(())
        );
        assert_eq!(act1.pid_position.i(), 2.5);

        assert_eq!(
            act1.write_reg(RegisterId::OuterPidAlpha_D, RegisterValue::Float(0.5)),
            Ok(())
        );
        assert_eq!(act1.pid_position.d(), 0.5);

        assert_eq!(
            act1.write_reg(RegisterId::OuterPidAlpha_Lim, RegisterValue::Float(25.0)),
            Ok(())
        );
        assert_eq!(act1.pid_position.limit(), 25.0);

        assert_eq!(
            act1.write_reg(RegisterId::OuterPidAlpha_Ramp, RegisterValue::Float(100.0)),
            Ok(())
        );
        assert_eq!(act1.pid_position.output_ramp(), 100.0);

        // 5. Calibration state & progress
        act1.calib_state = 2;
        act1.calib_progress = 100;
        assert_eq!(act1.read_reg(RegisterId::CalibState), Ok(RegisterValue::U8(2)));
        assert_eq!(act1.read_reg(RegisterId::CalibProgress), Ok(RegisterValue::U8(100)));
        assert_eq!(
            act1.write_reg(RegisterId::CalibState, RegisterValue::U8(0)),
            Err(RegisterError::ReadOnly(RegisterId::CalibState))
        );

        // 6. Transmission routing
        assert_eq!(act1.read_reg(RegisterId::GearRatio), Ok(RegisterValue::Float(2.0)));
        assert_eq!(
            act1.write_reg(RegisterId::GearRatio, RegisterValue::Float(3.5)),
            Ok(())
        );
        assert_eq!(act1.transmission.gear_ratio, 3.5);

        // 7. Inner motor delegation (e.g. VelocityLimit 0x52)
        assert_eq!(
            act1.write_reg(RegisterId::VelocityLimit, RegisterValue::Float(50.0)),
            Ok(())
        );
        assert_eq!(act1.motor.velocity_limit, 50.0);
    }

    #[test]
    fn test_actuator_2dof_register_bridge() {
        let clock = NullClock;
        let motor1 = MotorConfig::new(&clock);
        let motor2 = MotorConfig::new(&clock);
        let diff = CoupledDifferential::symmetric(2.0);
        let mut sensor2d = MockSensor2D::new([0.0, 0.0]);
        sensor2d.set_state([0.0, 0.0], [1.2, -3.4]);
        let pid_a = PIDController::new(5.0, 0.5, 0.05, 50.0, 10.0, 0.001, &clock);
        let pid_b = PIDController::new(6.0, 0.6, 0.06, 60.0, 12.0, 0.001, &clock);

        let mut act2 = Actuator2DOF::new(motor1, motor2, diff, sensor2d, pid_a, pid_b);

        // 1. Dual joint targets
        assert_eq!(
            act2.write_reg(RegisterId::JointTargetAlpha, RegisterValue::Float(0.5)),
            Ok(())
        );
        assert_eq!(
            act2.write_reg(RegisterId::JointTargetBeta, RegisterValue::Float(-0.5)),
            Ok(())
        );
        assert_eq!(act2.target_joint_angles, [0.5, -0.5]);
        assert_eq!(act2.read_reg(RegisterId::JointTargetAlpha), Ok(RegisterValue::Float(0.5)));
        assert_eq!(act2.read_reg(RegisterId::JointTargetBeta), Ok(RegisterValue::Float(-0.5)));

        // 2. Dual measured angles & velocities
        act2.current_joint_angles = [0.12, -0.34];
        assert_eq!(act2.read_reg(RegisterId::JointAngleAlpha), Ok(RegisterValue::Float(0.12)));
        assert_eq!(act2.read_reg(RegisterId::JointAngleBeta), Ok(RegisterValue::Float(-0.34)));
        assert_eq!(act2.read_reg(RegisterId::JointVelocityAlpha), Ok(RegisterValue::Float(1.2)));
        assert_eq!(act2.read_reg(RegisterId::JointVelocityBeta), Ok(RegisterValue::Float(-3.4)));

        // 3. Dual outer PIDs
        assert_eq!(
            act2.write_reg(RegisterId::OuterPidAlpha_P, RegisterValue::Float(10.0)),
            Ok(())
        );
        assert_eq!(
            act2.write_reg(RegisterId::OuterPidBeta_P, RegisterValue::Float(8.0)),
            Ok(())
        );
        assert_eq!(act2.pid_alpha.p(), 10.0);
        assert_eq!(act2.pid_beta.p(), 8.0);
        assert_eq!(act2.read_reg(RegisterId::OuterPidAlpha_P), Ok(RegisterValue::Float(10.0)));
        assert_eq!(act2.read_reg(RegisterId::OuterPidBeta_P), Ok(RegisterValue::Float(8.0)));

        // 4. NumMotors
        assert_eq!(act2.read_reg(RegisterId::NumMotors), Ok(RegisterValue::U8(2)));

        // 5. Motor-window selector gating
        // With MotorSelect == 0 (default), Target (0x01) routes to motor1
        assert_eq!(
            act2.write_reg(RegisterId::Target, RegisterValue::Float(100.0)),
            Ok(())
        );
        assert_eq!(act2.motor1.target, 100.0);
        assert_eq!(act2.motor2.target, 0.0);
        assert_eq!(act2.read_reg(RegisterId::Target), Ok(RegisterValue::Float(100.0)));

        // Switch MotorSelect to 1: Target routes to motor2
        assert_eq!(
            act2.write_reg(RegisterId::MotorSelect, RegisterValue::U8(1)),
            Ok(())
        );
        assert_eq!(act2.read_reg(RegisterId::MotorSelect), Ok(RegisterValue::U8(1)));
        assert_eq!(
            act2.write_reg(RegisterId::Target, RegisterValue::Float(200.0)),
            Ok(())
        );
        assert_eq!(act2.motor1.target, 100.0);
        assert_eq!(act2.motor2.target, 200.0);
        assert_eq!(act2.read_reg(RegisterId::Target), Ok(RegisterValue::Float(200.0)));
    }

    #[test]
    fn test_coupled_differential_register_bridge() {
        let mut diff = CoupledDifferential::symmetric(2.0);

        // Check initial matrix: scale = 0.5 / 2.0 = 0.25
        assert_eq!(diff.read_reg(RegisterId::TransmissionType), Ok(RegisterValue::U8(1)));
        assert_eq!(diff.read_reg(RegisterId::DiffMatrix_T11), Ok(RegisterValue::Float(0.25)));
        assert_eq!(diff.read_reg(RegisterId::DiffMatrix_T12), Ok(RegisterValue::Float(0.25)));
        assert_eq!(diff.read_reg(RegisterId::DiffMatrix_T21), Ok(RegisterValue::Float(0.25)));
        assert_eq!(diff.read_reg(RegisterId::DiffMatrix_T22), Ok(RegisterValue::Float(-0.25)));

        // Write custom matrix [0.5, 0.0, 0.0, 0.5]
        assert_eq!(diff.write_reg(RegisterId::DiffMatrix_T11, RegisterValue::Float(0.5)), Ok(()));
        assert_eq!(diff.write_reg(RegisterId::DiffMatrix_T12, RegisterValue::Float(0.0)), Ok(()));
        assert_eq!(diff.write_reg(RegisterId::DiffMatrix_T21, RegisterValue::Float(0.0)), Ok(()));
        assert_eq!(diff.write_reg(RegisterId::DiffMatrix_T22, RegisterValue::Float(0.5)), Ok(()));
        assert_eq!(diff.matrix, [[0.5, 0.0], [0.0, 0.5]]);

        // Offsets
        assert_eq!(diff.write_reg(RegisterId::JointOffsetAlpha, RegisterValue::Float(0.1)), Ok(()));
        assert_eq!(diff.write_reg(RegisterId::JointOffsetBeta, RegisterValue::Float(-0.2)), Ok(()));
        assert_eq!(diff.joint_offsets, [0.1, -0.2]);
        assert_eq!(diff.read_reg(RegisterId::JointOffsetAlpha), Ok(RegisterValue::Float(0.1)));
        assert_eq!(diff.read_reg(RegisterId::JointOffsetBeta), Ok(RegisterValue::Float(-0.2)));
    }

    #[test]
    fn test_lut_transmission_register_bridge() {
        let mut lut = LutTransmission::new(0.0, 3.0, [0.0; 4]);

        assert_eq!(lut.read_reg(RegisterId::TransmissionType), Ok(RegisterValue::U8(2)));
        assert_eq!(lut.read_reg(RegisterId::LutPoints), Ok(RegisterValue::U16(4)));
        assert_eq!(lut.read_reg(RegisterId::LutMotMin), Ok(RegisterValue::Float(0.0)));
        assert_eq!(lut.read_reg(RegisterId::LutMotMax), Ok(RegisterValue::Float(3.0)));

        // Burst upload sequential values: [0.0, 1.0, 2.0, 3.0]
        assert_eq!(lut.write_reg(RegisterId::LutPointIndex, RegisterValue::U16(0)), Ok(()));
        assert_eq!(lut.write_reg(RegisterId::LutPointValue, RegisterValue::Float(0.0)), Ok(()));
        assert_eq!(lut.read_reg(RegisterId::LutPointIndex), Ok(RegisterValue::U16(1)));

        assert_eq!(lut.write_reg(RegisterId::LutPointValue, RegisterValue::Float(1.0)), Ok(()));
        assert_eq!(lut.read_reg(RegisterId::LutPointIndex), Ok(RegisterValue::U16(2)));

        assert_eq!(lut.write_reg(RegisterId::LutPointValue, RegisterValue::Float(2.0)), Ok(()));
        assert_eq!(lut.read_reg(RegisterId::LutPointIndex), Ok(RegisterValue::U16(3)));

        assert_eq!(lut.write_reg(RegisterId::LutPointValue, RegisterValue::Float(3.0)), Ok(()));
        assert_eq!(lut.read_reg(RegisterId::LutPointIndex), Ok(RegisterValue::U16(4)));

        assert_eq!(lut.table, [0.0, 1.0, 2.0, 3.0]);

        // Validate monotonicity status
        assert_eq!(lut.read_reg(RegisterId::LutStatus), Ok(RegisterValue::U8(1)));

        // Make non-monotonic and check status
        assert_eq!(lut.write_reg(RegisterId::LutPointIndex, RegisterValue::U16(2)), Ok(()));
        assert_eq!(lut.write_reg(RegisterId::LutPointValue, RegisterValue::Float(0.5)), Ok(()));
        assert_eq!(lut.read_reg(RegisterId::LutStatus), Ok(RegisterValue::U8(0)));
    }

    #[test]
    fn test_linkage_and_cam_register_bridge() {
        let mut fb = FourBarLinkage::new(100.0, 30.0, 80.0, 50.0, barefoc::kinematics::transmission::FourBarBranch::Open);
        assert_eq!(fb.read_reg(RegisterId::TransmissionType), Ok(RegisterValue::U8(3)));
        assert_eq!(fb.read_reg(RegisterId::LinkageL1), Ok(RegisterValue::Float(100.0)));
        assert_eq!(fb.read_reg(RegisterId::LinkageL2_L4), Ok(RegisterValue::Float3([30.0, 80.0, 50.0])));

        assert_eq!(fb.write_reg(RegisterId::LinkageL1, RegisterValue::Float(120.0)), Ok(()));
        assert_eq!(fb.write_reg(RegisterId::LinkageL2_L4, RegisterValue::Float3([40.0, 90.0, 60.0])), Ok(()));
        assert_eq!(fb.l1, 120.0);
        assert_eq!(fb.l2, 40.0);
        assert_eq!(fb.l3, 90.0);
        assert_eq!(fb.l4, 60.0);

        let mut cam = EccentricCam::new(10.0, 50.0);
        assert_eq!(cam.read_reg(RegisterId::TransmissionType), Ok(RegisterValue::U8(4)));
        assert_eq!(cam.read_reg(RegisterId::LinkageL1), Ok(RegisterValue::Float(50.0)));
        assert_eq!(cam.read_reg(RegisterId::LinkageL2_L4), Ok(RegisterValue::Float3([10.0, 0.0, 0.0])));

        assert_eq!(cam.write_reg(RegisterId::LinkageL1, RegisterValue::Float(60.0)), Ok(()));
        assert_eq!(cam.write_reg(RegisterId::LinkageL2_L4, RegisterValue::Float3([15.0, 0.0, 0.0])), Ok(()));
        assert_eq!(cam.arm_length, 60.0);
        assert_eq!(cam.eccentricity, 15.0);
    }
}
