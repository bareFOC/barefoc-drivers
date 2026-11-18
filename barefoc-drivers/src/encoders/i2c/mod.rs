//! I²C encoder drivers.

pub mod as5600;

pub use as5600::{As5600, As5600Status, AS5600_DEFAULT_I2C_ADDR};
