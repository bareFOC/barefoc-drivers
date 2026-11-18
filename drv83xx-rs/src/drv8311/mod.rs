//! # drv8311 — Texas Instruments DRV8311 3-Phase Smart Gate Driver
//!
//! References: Texas Instruments DRV8311 Datasheet (SLVSFN2B).

pub mod config;
pub mod filter;
pub mod hw_variant;
pub mod pwmg_variant;
pub mod registers;
pub mod spi_variant;
pub mod status;

#[cfg(feature = "barefoc")]
pub mod bundle;

// Re-exports
pub use config::*;
pub use filter::Drv8311CurrentSenseFilter;
pub use hw_variant::Drv8311H;
pub use pwmg_variant::Drv8311P;
pub use registers::*;
pub use spi_variant::Drv8311S;
pub use status::Drv8311Status;

#[cfg(feature = "barefoc")]
pub use bundle::{
    Drv8311Driver3PWM, Drv8311Driver6PWM, Drv8311DriverPwmGen, Drv8311HDriver3PWM,
};
