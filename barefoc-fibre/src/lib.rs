//! # barefoc-fibre
//!
//! Pure-Rust, `#![no_std]` implementation of the Fibre RPC protocol compatible
//! with the official ODrive Web GUI (`gui.odriverobotics.com`) and Python CLI (`odrivetool`).
//!
//! ## Upstream Attribution
//!
//! Fibre was designed and implemented by the ODrive Robotics team. This crate is an
//! independent `#![no_std]` clean-room Rust implementation matching the canonical
//! legacy packet protocol specifications.
//!
//! - Reference: `ref/fibre-cpp/legacy_protocol.hpp`
//! - Reference: `ref/fibre-cpp/legacy_protocol.cpp`
//! - Reference: `ref/fibre-cpp/include/fibre/crc.hpp`

#![no_std]

pub mod crc;
pub mod framing;
pub mod schema;
pub mod server;
pub mod odrive;

pub use crc::{
    calc_crc16, calc_crc16_buffer, calc_crc8, calc_crc8_buffer, CANONICAL_CRC16_INIT,
    CANONICAL_CRC16_POLYNOMIAL, CANONICAL_CRC8_INIT, CANONICAL_CRC8_POLYNOMIAL, CANONICAL_PREFIX,
    PROTOCOL_VERSION,
};

pub use framing::{
    FramingError, PacketUnwrapper, PacketWrapper, UnwrapperState, HEADER_SIZE, MAX_PAYLOAD_SIZE,
    OVERHEAD_SIZE, TRAILER_SIZE,
};

pub use schema::{Endpoint0Error, Endpoint0Handler};

pub use server::{
    Endpoint, FibreServer, ProcessFramedError, PropertyHandler, RawReadFn, RawWriteFn, ServerError,
};

pub use odrive::{create_odrive_schema, BAREFOC_ODRIVE_JSON};

#[cfg(feature = "barefoc")]
pub use odrive::{make_axis_context_endpoints, make_motor_config_endpoints, BareFocAxisContext};
