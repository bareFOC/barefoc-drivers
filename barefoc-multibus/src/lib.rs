//! # barefoc-multibus
//!
//! Portable, `#![no_std]`, zero-allocation **descriptor-driven register-map (control
//! table) server core** and controller-side **client** for address-based buses
//! (I2C / I3C / SPI register model) — the third protocol lane of the BareFOC workspace
//! (`multibus-server` capability, `openspec/changes/add-barefoc-multibus`).
//!
//! ## Upstream Attribution
//!
//! Register semantics are ported from the Miniata control-table firmware by the
//! Miniata / BareFOC team:
//! - Upstream Reference: `ref/Miniata/firmware/src/control_table.h`
//! - Upstream Reference: `ref/Miniata/firmware/src/control_table.cpp`
//!   (`writeRegister()` width/bounds validation, `checkAndClearCommand()` self-clear)
//! - Upstream Reference: `ref/Miniata/firmware/src/comm_i2c.cpp`
//!   (register-pointer idiom; the `0xFF` variable-width burst protocol of that file is
//!   intentionally not ported, see `design.md` D3)
//!
//! Register-family naming per the Dinnge/Dynamixel *control table* convention.
//!
//! ## Layers (design D1 — two-layer split)
//!
//! This protocol crate carries zero platform dependencies:
//! - [`table::RegMap`] — descriptor-driven server core (offset + width + access mode +
//!   write hook), bounded byte-bank storage, register-pointer latch / auto-increment
//!   wire semantics (`write_phase` / `read_phase`).
//! - [`transport::TargetTransport`] — the bus-agnostic event trait; the STM32/embassy
//!   glue in `barefoc-hal/barefoc-stm32` implements the vendor side of it, and
//!   [`transport::RegServer`] is the ready-made event-lowering adapter.
//! - [`client::RegClientI2c`] / [`client::RegClientSpi`] — controller side generic over
//!   `embedded-hal` 1.0 master traits; server and client are joined in `cargo test` on
//!   the host via `embedded-hal-mock`.
//!
//! ## Concurrency contract (design D4)
//!
//! Bus IRQs only buffer bytes and set an event flag; the single motor-loop task drains
//! the buffered events into the [`table::RegMap`] under `critical_section::with`. The
//! table is never touched from ISR context. Write hooks run synchronously inside that
//! drain path: they must be short and non-blocking, and they cannot re-enter the map
//! (the hook signature receives only the application handler context and a shared view
//! of the written value — no map or bank reference is aliased into hooks).

#![no_std]
#![doc(html_root_url = "https://github.com/simplefoc/Arduino-FOC")]

// Host-side `std` access for unit tests (workspace convention; the library itself
// never links std).
#[cfg(any(feature = "std", test))]
#[macro_use]
extern crate std;

#[cfg(feature = "simplefoc-profile")]
pub mod profile;
#[cfg(feature = "simplefoc-profile")]
pub use profile::{refresh_telemetry, simplefoc_profile};

pub mod client;
pub mod table;
pub mod transport;

pub use client::{ClientError, RegClientI2c, RegClientSpi};
pub use table::{AccessMode, Error, HookResult, RegEntry, RegMap, WriteHookFn};
pub use transport::{RegServer, TargetTransport};
