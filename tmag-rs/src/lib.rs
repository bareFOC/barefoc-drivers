//! # tmag-rs
//!
//! Production-grade, `#![no_std]` Texas Instruments TMAG5273 3D Hall-effect sensor driver
//! with Embassy async DMA support, high-speed 1-byte streaming, online fault recovery,
//! native SI radians/degrees conversions, and first-class BareFOC 1D/2D joint integration.
//!
//! ## Features
//!
//! - **Full Register Coverage**: Complete typed mapping for all 29 hardware registers (`0x00`..=`0x1C`).
//! - **Dual Sync / Async APIs**:
//!   - Synchronous blocking I/O via `embedded-hal` 1.0 `I2c`.
//!   - Asynchronous non-blocking DMA I/O via `embedded-hal-async` 1.0 `I2c` (`feature = "async"`).
//! - **High-Throughput Fast Modes**:
//!   - 1-byte fast stream mode (`read_stream()`) for up to 20 kSPS loops.
//!   - 8-bit MSB fast mode (`read_magnetic_8bit()`) reducing I2C bus time by 50%.
//! - **Online Fault Recovery**:
//!   - `clear_faults()` and `clear_por()` APIs writing to W1CP flags in `DEVICE_STATUS` and `CONV_STATUS`.
//! - **First-Class SI Units**:
//!   - [`Angle`] struct providing zero-cost `.radians()` and `.degrees()` conversions with 0.0625° CORDIC resolution.
//!   - [`MagneticField3D`] with instantaneous $\mathcal{O}(1)$ `.plane_angles()` yielding $[\alpha, \beta]$ in ~60 ns.
//! - **BareFOC 1-Line Adapters (`feature = "barefoc"`)**:
//!   - [`Tmag5273Sensor`]: Implements [`barefoc::traits::sensor::Sensor`].
//!   - [`Tmag5273Sensor2D`]: Implements [`barefoc::kinematics::sensor::Sensor2D`].

#![cfg_attr(not(feature = "std"), no_std)]

pub mod angle;
pub mod crc;
pub mod discovery;
pub mod error;
pub mod magnetic;
pub mod registers;
pub mod tmag5273;
pub mod types;

#[cfg(feature = "barefoc")]
pub mod barefoc;

pub use angle::{Angle, _PI, _2PI};
pub use crc::crc8;
pub use discovery::{discover, discover_with_frequencies, probe_device, CANDIDATE_ADDRESSES, CANDIDATE_FREQUENCIES_HZ};
pub use error::Error;
pub use magnetic::MagneticField3D;
pub use tmag5273::{Tmag5273, Tmag5273Builder};
pub use types::*;

#[cfg(feature = "barefoc")]
pub use barefoc::{Tmag5273Sensor, Tmag5273Sensor2D};
