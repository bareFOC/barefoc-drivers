//! # drv8316 — Texas Instruments DRV8316 3-Phase Smart Gate Driver
//!
//! Upstream reference: `source/Arduino-FOC-drivers/src/drivers/drv8316/`

pub mod config;
pub mod hw_variant;
pub mod pwm_variant;
pub mod registers;
pub mod spi_variant;
pub mod status;

#[cfg(feature = "barefoc")]
pub mod bundle;

// Re-exports
pub use config::*;
pub use hw_variant::Drv8316H;
pub use pwm_variant::Drv8316P;
pub use registers::*;
pub use spi_variant::Drv8316S;
pub use status::Drv8316Status;

#[cfg(feature = "barefoc")]
pub use bundle::{Drv8316Driver3PWM, Drv8316Driver6PWM};
