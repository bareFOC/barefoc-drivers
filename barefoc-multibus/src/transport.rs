//! # Bus-agnostic target transport trait and ready-made server adapter
//!
//! Captures the address-bus event model shared by every address-based bus in a
//! trait that contains no platform, embassy, or chip-specific items
//! (spec `multibus-server`: *Transport-Agnostic Trait Surface*). A future bus
//! glue (e.g. STM32 embassy-glue I2C/SPI target adapters in `barefoc-stm32`, or an
//! I3C target) wires its bus event callbacks to a [`TargetTransport`] implementor;
//! [`RegServer`] is the ready-made adapter so glue code only forwards bytes.
//!
//! ## Concurrency contract (design D4)
//!
//! Bus IRQs only buffer bytes and set an event flag; a single task drains the
//! buffered events into the [`TargetTransport`] implementor (which forwards into
//! the [`RegMap`] core) under `critical_section::with`. The table is never touched
//! from ISR context.

use crate::table::{Error, RegMap};

/// Address-bus event model trait (server side).
///
/// A minimal trait the platform glue implements (or, in the common case, uses via
/// [`RegServer`]): it receives bus write-phase events and serves bus read-phase
/// requests. Nothing in this trait mentions a specific platform or bus.
///
/// Wire semantics are the design D3 register-pointer idiom
/// (`ref/Miniata/firmware/src/comm_i2c.cpp`):
///
/// - write phase: `[ptr]` latches the register pointer without modifying anything;
///   `[ptr, d0..dn]` latches, then writes `n` little-endian bytes at `ptr`.
/// - read phase: `len` bytes streamed from the latched pointer, little-endian,
///   auto-increment, crossing register boundaries as a plain memory-map read.
pub trait TargetTransport {
    /// Delivers one bus write-phase frame (pointer byte optionally followed by
    /// payload bytes). Errors are reported back to the bus adapter so it can map
    /// them to its native error condition (e.g. I2C NAK).
    fn on_write(&mut self, data: &[u8]) -> Result<(), Error>;

    /// Serves one bus read-phase request of exactly `len` bytes into `out`
    /// (`out.len() >= len`); returns `len` on success.
    fn on_read(&mut self, len: usize, out: &mut [u8]) -> Result<usize, Error>;
}

/// Blanket event lowering: a [`RegMap`] is itself a [`TargetTransport`] (its inherent
/// [`RegMap::write_phase`] / [`RegMap::read_phase`] are the canonical lowering).
impl<C, const CAP: usize, const MAX_ENTRIES: usize> TargetTransport
    for RegMap<C, CAP, MAX_ENTRIES>
{
    fn on_write(&mut self, data: &[u8]) -> Result<(), Error> {
        // Inherent-method dispatch takes precedence; forwarded explicitly.
        RegMap::write_phase(self, data)
    }

    fn on_read(&mut self, len: usize, out: &mut [u8]) -> Result<usize, Error> {
        RegMap::read_phase(self, len, out)
    }
}

/// Ready-made target server: owns a [`RegMap`] and implements the event lowering, so
/// bus glue code only forwards bytes received from the hardware and receives bytes
/// for the read response.
///
/// Upstream Reference: `ref/Miniata/firmware/src/comm_i2c.cpp` (blocking task that
/// drains a polled I2C event into the control table — modernized into the event
/// trait of this crate).
pub struct RegServer<C, const CAP: usize, const MAX_ENTRIES: usize> {
    map: RegMap<C, CAP, MAX_ENTRIES>,
}

impl<C, const CAP: usize, const MAX_ENTRIES: usize> RegServer<C, CAP, MAX_ENTRIES> {
    /// Creates a server around a fully-initialized register map.
    pub fn new(map: RegMap<C, CAP, MAX_ENTRIES>) -> Self {
        Self { map }
    }

    /// Shared view of the register map (application task: publishing telemetry
    /// through [`RegMap::set`], etc.).
    pub fn map(&self) -> &RegMap<C, CAP, MAX_ENTRIES> {
        &self.map
    }

    /// Mutable view of the register map (application task between bus events).
    pub fn map_mut(&mut self) -> &mut RegMap<C, CAP, MAX_ENTRIES> {
        &mut self.map
    }

    /// Consumes the server, returning the register map.
    pub fn into_map(self) -> RegMap<C, CAP, MAX_ENTRIES> {
        self.map
    }
}

impl<C, const CAP: usize, const MAX_ENTRIES: usize> TargetTransport
    for RegServer<C, CAP, MAX_ENTRIES>
{
    fn on_write(&mut self, data: &[u8]) -> Result<(), Error> {
        self.map.write_phase(data)
    }

    fn on_read(&mut self, len: usize, out: &mut [u8]) -> Result<usize, Error> {
        self.map.read_phase(len, out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::table::{AccessMode, Error, HookResult, RegEntry};

    // Static handler context: this test only exercises byte movement, so a `()`
    // context suffices and no hook is registered.
    type Server = RegServer<(), 64, 4>;

    fn server() -> Server {
        RegServer::new(
            RegMap::<(), 64, 4>::new(())
                .register(RegEntry::rw(0x00, 1))
                .unwrap()
                .register(RegEntry::rw(0x08, 4))
                .unwrap()
                .register(RegEntry::ro(0x20, 4))
                .unwrap()
                .register(RegEntry::rw(0x24, 1))
                .unwrap(),
        )
    }

    /// A glue layer that only knows the trait (no `RegMap` internals) drives the
    /// server purely through `TargetTransport` events.
    fn glue_session<G: TargetTransport>(
        transport: &mut G,
        write_frames: &[&[u8]],
        read_len: usize,
    ) -> Result<usize, Error> {
        for frame in write_frames {
            transport.on_write(frame)?;
        }
        let mut out = [0u8; 64];
        transport.on_read(read_len, &mut out)
    }

    #[test]
    fn regserver_lowens_bus_events_for_glue_layers() {
        let mut server = server();
        // Pointer latch to REG 0x08, then a 4-byte read phase.
        assert_eq!(glue_session(&mut server, &[&[0x08]], 4).unwrap(), 4);
        // Pointer latch + payload write phase (single frame).
        assert_eq!(
            glue_session(&mut server, &[&[0x08, 0x11, 0x22, 0x33, 0x44]], 0).unwrap(),
            0
        );
        assert_eq!(server.map().pointer(), 0x0C);
    }

    #[test]
    fn regserver_maps_error_back_to_the_bus_adapter() {
        let mut server = server();
        // Unmapped pointer: adapter receives the error and maps it (NAK upstream).
        assert_eq!(server.on_write(&[0xFE]), Err(Error::Unmapped));
        // Read beyond the extent: full-frame rejection.
        server.on_write(&[0x20]).unwrap();
        let mut out = [0u8; 64];
        assert_eq!(server.on_read(8, &mut out), Err(Error::Unmapped));
        // Adapter-independence: the RegMap itself also implements the trait, so glue
        // may embed the map directly without the server wrapper.
        let mut bare =
            RegMap::<(), 64, 4>::new(()).register(RegEntry::ro(0x20, 4)).unwrap();
        assert_eq!(bare.on_write(&[0x20]), Ok(()));
        assert_eq!(bare.on_read(4, &mut out).unwrap(), 4);
    }

    #[test]
    fn regserver_exposes_map_for_app_side_telemetry() {
        let mut server = server();
        // App task publishes Ro telemetry (upstream ControlTableData present_* path).
        server
            .map_mut()
            .set(0x20, &0.75f32.to_le_bytes())
            .unwrap();
        server.on_write(&[0x20]).unwrap();
        let mut out = [0u8; 4];
        server.on_read(4, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 0.75);
        // Ro enforcement is upstream-accurate over the bus.
        assert_eq!(server.on_write(&[0x20, 0, 0, 0, 0]), Err(Error::ReadOnly));
        let _ = AccessMode::Rw;
        let _ = core::mem::size_of::<HookResult>();
        let _ = server.into_map();
    }
}
