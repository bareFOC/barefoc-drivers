//! # spi — TI SPI framing engines (16-bit, 24-bit, and 32-bit tSPI)
//!
//! Provides SPI protocol framing with hardware parity calculations for TI DRV83xx
//! gate drivers:
//! - 16-bit SPI framing with XNOR parity for DRV8316
//! - 24-bit SPI framing with dual even parity for DRV8311
//! - 32-bit `tSPI` framing with multi-device bus addressing for DRV8311

pub mod spi16;
pub mod spi24;

pub use spi16::*;
pub use spi24::*;
