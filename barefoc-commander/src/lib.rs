//! # barefoc-commander
//!
//! Pure-Rust, `#![no_std]` implementation of the Arduino SimpleFOC
//! [`Commander`](https://docs.simplefoc.com/commander_interface) ASCII serial console
//! tuning and inspection protocol.
//!
//! ## Upstream Attribution
//!
//! Directly ported from and inspired by the SimpleFOC project created by
//! Antun Skuric and the SimpleFOC community.
//! - Upstream Reference: `source/Arduino-FOC/src/communication/Commander.h`
//! - Upstream Reference: `source/Arduino-FOC/src/communication/Commander.cpp`
//! - Upstream Reference: `source/Arduino-FOC/src/communication/commands.h`

#![no_std]

pub mod commander;
#[cfg(feature = "barefoc")]
pub mod motor;
pub mod io;

pub use commander::{
    CallbackEntry, CommandCallback, Commander, VerboseMode, CMD_DECIMAL, CMD_SCAN, CMD_VERBOSE,
};
#[cfg(feature = "barefoc")]
pub use motor::{MotorCommandError, MotorCommander};
pub use io::{LineError, LineProcessor, StreamError};
