//! # dc — brushed DC motor driver and controller support
//!
//! Ports the DC motor driver and controller from Arduino-FOC-drivers
//! (`origin/dc-motor` branch).
//!
//! Provides single-phase / H-bridge driver traits ([`DCDriver`]), 2-PWM
//! driver implementation ([`DCDriver2PWM`]), and the closed/open-loop
//! [`DCMotor`] controller.

pub mod current_sense;
pub mod driver_1pwm;
pub mod driver_2pwm;
pub mod motor;
pub mod traits;

pub use current_sense::CurrentMirrorSense;
pub use driver_1pwm::DCDriver1PWM;
pub use driver_2pwm::DCDriver2PWM;
pub use motor::{DCMotor, DummyCurrentSense, DummySensor};
pub use traits::{DCDriver, DCDriverParams, NoPin};
