//! # utilities::trapezoidal — Trapezoidal Velocity Profile Planner
//!
//! Ports [`TrapezoidalPlanner`](source/Arduino-FOC-drivers/src/utilities/trapezoids/TrapezoidalPlanner.h)
//! and [`TrapezoidalPlanner.cpp`](source/Arduino-FOC-drivers/src/utilities/trapezoids/TrapezoidalPlanner.cpp)
//! from Arduino-FOC-drivers.
//!
//! Computes smooth target velocity setpoints adhering to acceleration, maximum velocity,
//! and deceleration limits for point-to-point moves.

use barefoc::common::time::Clock;
use barefoc::math::NOT_SET;
use barefoc::motor::base::FOCMotor;
use barefoc::traits::current_sense::CurrentSense;
use barefoc::traits::driver::FOCDriver;
use barefoc::traits::sensor::Sensor;

/// Velocity profile generator for trapezoidal motion planning.
///
/// Ports the upstream `TrapezoidalPlanner` C++ class.
#[derive(Clone, Debug)]
pub struct TrapezoidalPlanner {
    /// Maximum velocity ceiling in rad/s.
    pub max_velocity: f32,
    /// Minimum crawl velocity in rad/s (default 0.2 rad/s).
    pub min_velocity: f32,
    /// Acceleration limit in rad/s².
    pub accel: f32,
    /// Deceleration limit in rad/s².
    pub decel: f32,
    /// Distance threshold for arrival detection (rad, default 0.02 rad).
    pub arrived_distance: f32,
    /// Velocity threshold for arrival detection (rad/s, default 0.2 rad/s).
    pub arrived_velocity: f32,
    target: f32,
    last_target: f32,
    start_angle: f32,
}

impl TrapezoidalPlanner {
    /// Construct a new trapezoidal profile planner.
    ///
    /// # Arguments
    /// * `max_velocity` - Maximum allowable speed in rad/s.
    /// * `accel` - Acceleration limit in rad/s².
    /// * `decel` - Optional deceleration limit in rad/s²; defaults to `accel` if `None`.
    /// * `min_velocity` - Optional minimum crawl speed in rad/s; defaults to 0.2 rad/s if `None`.
    pub fn new(
        max_velocity: f32,
        accel: f32,
        decel: Option<f32>,
        min_velocity: Option<f32>,
    ) -> Self {
        Self {
            max_velocity,
            accel,
            decel: decel.unwrap_or(accel),
            min_velocity: min_velocity.unwrap_or(0.2),
            arrived_distance: 0.02,
            arrived_velocity: 0.2,
            target: NOT_SET,
            last_target: NOT_SET,
            start_angle: 0.0,
        }
    }

    /// Set a new target position (rad) with the current shaft angle as the acceleration origin.
    pub fn set_target(&mut self, target: f32, current_angle: f32) {
        self.target = target;
        self.start_angle = current_angle;
        self.last_target = target;
    }

    /// Convenience wrapper setting the target and sampling current angle from an [`FOCMotor`].
    pub fn set_target_motor<S, D, CS, C, M>(&mut self, target: f32, motor: &M)
    where
        S: Sensor<C>,
        D: FOCDriver,
        CS: CurrentSense,
        C: Clock,
        M: FOCMotor<S, D, CS, C>,
    {
        self.set_target(target, motor.shaft_angle());
    }

    /// Returns the active target angle (rad), or [`NOT_SET`] if no target has been set.
    #[inline]
    pub fn target(&self) -> f32 {
        self.target
    }

    /// Upstream SimpleFOC-compatible getter for the target angle.
    #[inline]
    pub fn get_target(&self) -> f32 {
        self.target
    }

    /// Returns the start angle recorded when the target was configured.
    #[inline]
    pub fn start_angle(&self) -> f32 {
        self.start_angle
    }

    /// Returns the last configured target.
    #[inline]
    pub fn last_target(&self) -> f32 {
        self.last_target
    }

    /// Step the motion planner using explicit angle and velocity state.
    ///
    /// Computes the desired velocity setpoint according to:
    ///
    /// $$v^* = \min\left(\sqrt{2 a_{\text{accel}} |\Delta\theta_{\text{start}}|}, \sqrt{2 a_{\text{decel}} |\Delta\theta_{\text{target}}|}, v_{\max}\right) \cdot \operatorname{sgn}(\Delta\theta)$$
    ///
    /// Clamps to `min_velocity` when in motion, and returns `0.0` when arrival conditions
    /// ($|\Delta\theta| < \text{arrived\_distance}$ and $|\omega| < \text{arrived\_velocity}$) are met.
    pub fn step(&mut self, current_angle: f32, current_velocity: f32) -> f32 {
        if self.target == NOT_SET {
            return 0.0;
        }

        let d = self.target - current_angle;
        let dd = current_angle - self.start_angle;

        // If target is reached within thresholds, return 0
        if libm::fabsf(d) < self.arrived_distance
            && libm::fabsf(current_velocity) < self.arrived_velocity
        {
            return 0.0;
        }

        // Maximum velocity allowed by deceleration to target
        let max_v_d = libm::sqrtf(2.0 * self.decel * libm::fabsf(d));
        // Maximum velocity allowed by acceleration from start
        let max_v_dd = libm::sqrtf(2.0 * self.accel * libm::fabsf(dd));
        let mut max_v = libm::fminf(max_v_d, max_v_dd);
        max_v = libm::fminf(max_v, self.max_velocity);
        max_v = libm::fmaxf(max_v, self.min_velocity);

        max_v * sign(d)
    }

    /// Step the planner by reading the shaft angle and velocity from an [`FOCMotor`].
    pub fn run<S, D, CS, C, M>(&mut self, motor: &M) -> f32
    where
        S: Sensor<C>,
        D: FOCDriver,
        CS: CurrentSense,
        C: Clock,
        M: FOCMotor<S, D, CS, C>,
    {
        self.step(motor.shaft_angle(), motor.cfg().shaft_velocity)
    }
}

#[inline]
fn sign(a: f32) -> f32 {
    if a < 0.0 {
        -1.0
    } else if a > 0.0 {
        1.0
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trapezoidal_planner_uninitialized() {
        let mut planner = TrapezoidalPlanner::new(10.0, 5.0, None, None);
        assert_eq!(planner.target(), NOT_SET);
        assert_eq!(planner.step(0.0, 0.0), 0.0);
    }

    #[test]
    fn test_trapezoidal_planner_acceleration_and_deceleration() {
        let mut planner = TrapezoidalPlanner::new(10.0, 4.0, Some(2.0), Some(0.1));
        // Target: 10.0 rad, starting from 0.0 rad
        planner.set_target(10.0, 0.0);
        assert_eq!(planner.target(), 10.0);
        assert_eq!(planner.start_angle(), 0.0);

        // Near start (angle = 0.5 rad):
        // dd = 0.5, d = 9.5
        // max_v_dd = sqrt(2 * 4 * 0.5) = 2.0 rad/s
        // max_v_d = sqrt(2 * 2 * 9.5) = sqrt(38) ≈ 6.16 rad/s
        // min(2.0, 6.16, 10.0) = 2.0 rad/s
        let v1 = planner.step(0.5, 1.5);
        assert!((v1 - 2.0).abs() < 1e-3);

        // Near middle (angle = 5.0 rad):
        // dd = 5.0 -> sqrt(2 * 4 * 5) = sqrt(40) ≈ 6.32 rad/s
        // d = 5.0 -> sqrt(2 * 2 * 5) = sqrt(20) ≈ 4.47 rad/s
        // min(6.32, 4.47) = 4.47 rad/s
        let v_mid = planner.step(5.0, 4.0);
        let expected_mid = libm::sqrtf(20.0);
        assert!((v_mid - expected_mid).abs() < 1e-3);

        // Near end (angle = 9.5 rad):
        // d = 0.5 -> max_v_d = sqrt(2 * 2 * 0.5) = sqrt(2) ≈ 1.414 rad/s
        let v_end = planner.step(9.5, 1.5);
        let expected_end = libm::sqrtf(2.0);
        assert!((v_end - expected_end).abs() < 1e-3);

        // Arrival threshold: distance < 0.02 and velocity < 0.2
        let v_arrived = planner.step(9.99, 0.05);
        assert_eq!(v_arrived, 0.0);
    }

    #[test]
    fn test_trapezoidal_planner_negative_direction() {
        let mut planner = TrapezoidalPlanner::new(10.0, 4.0, None, Some(0.1));
        // Move from 0.0 to -10.0 rad
        planner.set_target(-10.0, 0.0);

        // Near start (angle = -0.5 rad):
        // dd = -0.5, d = -9.5
        let v = planner.step(-0.5, -1.0);
        assert!(v < 0.0);
        assert!((v - (-2.0)).abs() < 1e-3);
    }
}
