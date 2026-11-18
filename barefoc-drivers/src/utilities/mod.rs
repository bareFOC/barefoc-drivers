//! # utilities — High-Precision Tracking and Motion Trajectory Utilities
//!
//! Ports [`PreciseAngle`](source/Arduino-FOC-drivers/src/utilities/PreciseAngle.h)
//! and [`TrapezoidalPlanner`](source/Arduino-FOC-drivers/src/utilities/trapezoids/TrapezoidalPlanner.h)
//! from Arduino-FOC-drivers.

pub mod precise_angle;
pub mod trapezoidal;

pub use precise_angle::PreciseAngle;
pub use trapezoidal::TrapezoidalPlanner;
