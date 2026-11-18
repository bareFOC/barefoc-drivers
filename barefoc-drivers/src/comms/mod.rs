//! # comms — communication subsystems
//!
//! Ports communication modules from upstream Arduino-FOC-drivers:
//! - [`registers`]: Universal register map, typed values, and binary serialization.
//! - [`bridge`]: Translation between register values and [`barefoc::motor::base::MotorConfig`].

pub mod bridge;
pub mod registers;

#[cfg(feature = "kinematics")]
pub mod kinematics_bridge;

pub use bridge::{read_register, write_register, RegisterBridge};
pub use registers::{
    AccessMode, RegisterError, RegisterId, RegisterValue, SIMPLEFOC_REGISTERS_VERSION,
};
