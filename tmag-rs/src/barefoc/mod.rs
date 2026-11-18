//! # barefoc — BareFOC Trait Implementations
//!
//! Provides first-class integration with the BareFOC motor control and kinematics ecosystem:
//! - [`Tmag5273Sensor`]: Implements [`barefoc::traits::sensor::Sensor`] for 1-DOF joint control.
//! - [`Tmag5273Sensor2D`]: Implements [`barefoc_kinematics::sensor::Sensor2D`] for 2-DOF coupled differential joints.

pub mod sensor_1d;
pub mod sensor_2d;

pub use sensor_1d::Tmag5273Sensor;
pub use sensor_2d::Tmag5273Sensor2D;
