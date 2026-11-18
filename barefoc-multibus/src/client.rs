//! # Controller-side register client (`RegClient`) for host rigs and integrators
//!
//! Implements the controller side of the design D3 register-pointer protocol, generic
//! over `embedded-hal` 1.0 master traits, so [`crate::table::RegMap`] servers and
//! clients can be joined in `cargo test` on the host via `embedded-hal-mock`
//! (no hardware):
//!
//! - [`RegClientI2c`] — I2C (7-bit addressing) flavor over `embedded_hal::i2c::I2c`.
//! - [`RegClientSpi`] — SPI flavor over `embedded_hal::spi::SpiDevice` (one trait
//!   transaction == one CS frame; chip-select and framing owned by the `SpiDevice`
//!   implementation).
//!
//! ## Wire format, byte for byte (matching `RegMap::write_phase` / `read_phase`;
//! upstream `ref/Miniata/firmware/src/comm_i2c.cpp` register-pointer idiom, design D3)
//!
//! ```text
//! write_register:  I2C  [START][addr+W][reg][d0..dn][STOP]
//!                  SPI  [CS↓ reg][CS↑ payload]          (one transaction)
//! read_register:   I2C  [START][addr+W][reg][STOP]        (pointer latch)
//!                       [START][addr+R][b0..b(len-1)][STOP]; little-endian stream
//!                  SPI  [CS↓ reg][CS↑]                    (pointer latch frame)
//!                       [CS↓ clock-in len bytes][CS↑]     (reply-in-next-CS-frame,
//!                                                          TMC-style per design)
//! ```
//!
//! The pointer auto-increments server-side by every streamed byte, so spanning reads
//! and writes behave identically to the `RegMap` memory-map idiom.

use embedded_hal::i2c::I2c;
use embedded_hal::spi::{Operation, SpiDevice};

use crate::table::Error;

/// Errors surfaced by the controller-side client.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClientError<E> {
    /// Bus-level error surfaced by the underlying embedded-hal master (I2C NAK /
    /// arbitration loss, SPI CS or transfer fault). A rejected pointer latch on the
    /// server side arrives here as the mapped error condition of the adapter
    /// (spec: *Unknown-register error propagation*).
    Bus(E),
    /// Register pointer + payload exceed the client's internal staging frame
    /// ([`RegClientI2c`] `TX_BUF`).
    TxFrameTooLarge,
    /// The read `out` buffer is smaller than the requested length.
    RxBufferTooSmall,
    /// Register-map level error recoded for the controller side (e.g. reported by a
    /// peer that surfaces map errors directly).
    Server(Error),
}

/// I2C (7-bit) controller-side client of the register-pointer protocol.
///
/// Generic over `embedded_hal::i2c::I2c` (default `SevenBitAddress` addressing).
/// Because `I2c::write` requires one contiguous frame, the pointer byte and payload
/// are staged in an internal fixed-size frame buffer (`TX_BUF`, default `8`, which
/// comfortably holds the pointer byte plus a 4-byte LE `f32` register); payloads
/// larger than `TX_BUF - 1` are reported as [`ClientError::TxFrameTooLarge`].
///
/// Upstream Reference: `ref/Miniata/firmware/src/comm_i2c.cpp` (register-pointer
/// idiom; the `0xFF` burst protocol is intentionally not ported — design D3).
pub struct RegClientI2c<B, const TX_BUF: usize = 8> {
    bus: B,
}

impl<B: I2c, const TX_BUF: usize> RegClientI2c<B, TX_BUF> {
    /// Wraps a bus master.
    pub fn new(bus: B) -> Self {
        Self { bus }
    }

    /// Writes register `reg` (pointer latch) followed by the little-endian payload
    /// `bytes` in a single master write transaction, exactly matching the server-side
    /// `write_phase` frame `[reg][d0..dn]`.
    pub fn write_register(
        &mut self,
        addr: u8,
        reg: u8,
        bytes: &[u8],
    ) -> Result<(), ClientError<B::Error>> {
        if 1 + bytes.len() > TX_BUF {
            return Err(ClientError::TxFrameTooLarge);
        }
        let mut frame = [0u8; TX_BUF];
        frame[0] = reg;
        frame[1..1 + bytes.len()].copy_from_slice(bytes);
        self.bus
            .write(addr, &frame[..1 + bytes.len()])
            .map_err(ClientError::Bus)
    }

    /// Latches the register pointer with a `[reg]` write transaction, then reads
    /// exactly `len` little-endian bytes into `out` in a separate read transaction
    /// (`out.len() >= len`). Auto-increment on the server streams across register
    /// boundaries; little-endian float bytes are returned verbatim.
    pub fn read_register(
        &mut self,
        addr: u8,
        reg: u8,
        len: usize,
        out: &mut [u8],
    ) -> Result<(), ClientError<B::Error>> {
        if out.len() < len {
            return Err(ClientError::RxBufferTooSmall);
        }
        self.bus.write(addr, &[reg]).map_err(ClientError::Bus)?;
        self.bus
            .read(addr, &mut out[..len])
            .map_err(ClientError::Bus)
    }
}

/// SPI controller-side client of the register-pointer protocol.
///
/// Generic over `embedded_hal::spi::SpiDevice`: every call to
/// [`SpiDevice::transaction`] is one CS frame, so this client is CS-framed by the
/// `SpiDevice` implementation. Frame layout ("reply-in-next-CS-frame", TMC-style —
/// decision recorded in the change design):
///
/// - write: one frame carrying `[reg][payload...]` via sequential
///   [`Operation::Write`] ops (pointer byte op + payload op; no intermediate
///   staging buffer needed),
/// - read: frame 1 latches the pointer (`[reg]`); frame 2 clocks `len` bytes in
///   ([`Operation::Read`]), little-endian from the server's latched pointer.
pub struct RegClientSpi<B> {
    bus: B,
}

impl<B: SpiDevice> RegClientSpi<B> {
    /// Wraps the SPI device handle (the `SpiDevice` implementation owns CS/framing).
    pub fn new(bus: B) -> Self {
        Self { bus }
    }

    /// Writes register `reg` followed by the little-endian payload `bytes` within a
    /// single CS frame: the wire bytes are exactly the server `write_phase` frame
    /// `[reg][d0..dn]`.
    pub fn write_register(&mut self, reg: u8, bytes: &[u8]) -> Result<(), ClientError<B::Error>> {
        // One CS frame = one transaction. A single dedicated Write op avoids
        // fragmentation; a pointer byte + payload pair yields the exact server frame.
        if bytes.is_empty() {
            let reg_arr = [reg];
            return self
                .bus
                .transaction(&mut [Operation::Write(&reg_arr[..])])
                .map_err(ClientError::Bus);
        }
        let reg_arr = [reg];
        let mut ops = [Operation::Write(&reg_arr[..]), Operation::Write(bytes)];
        self.bus.transaction(&mut ops).map_err(ClientError::Bus)
    }

    /// Latches the register pointer in one CS frame, then clocks `len` little-endian
    /// bytes in during the next CS frame into `out` (`out.len() >= len`).
    pub fn read_register(
        &mut self,
        reg: u8,
        len: usize,
        out: &mut [u8],
    ) -> Result<(), ClientError<B::Error>> {
        if out.len() < len {
            return Err(ClientError::RxBufferTooSmall);
        }
        // CS frame 1: pointer latch.
        let reg_arr = [reg];
        self.bus
            .transaction(&mut [Operation::Write(&reg_arr[..])])
            .map_err(ClientError::Bus)?;
        // CS frame 2: reply-in-next-CS-frame read burst.
        let mut ops = [Operation::Read(&mut out[..len])];
        self.bus.transaction(&mut ops).map_err(ClientError::Bus)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::table::{Error as MapError, HookResult, RegEntry, RegMap};
    use crate::transport::{RegServer, TargetTransport};

    use embedded_hal::i2c as eh_i2c;
    use embedded_hal::spi::SpiBus;
    use embedded_hal::spi as eh_spi;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use embedded_hal_mock::eh1::spi::{Mock as SpiBusMock, Transaction as SpiTx};

    type SpiMock = SpiBusMock<u8>;

    const DEV: u8 = 0x30; // Miniata default REG_I2C_ADDR (control_table.h)

    // ------------------------------------------------------------------
    // RegServer + RegClient round-trip rigs (the "bus" adapters)
    // ------------------------------------------------------------------

    /// Test handler context (fixed-size log, no heap; library stays no_std).
    #[derive(Default)]
    struct Handler {
        log: [u8; 8],
        log_len: usize,
    }

    /// Self-clearing REG_SYSTEM_CMD-style hook (upstream control_table.cpp
    /// checkAndClearCommand semantics).
    fn sys_cmd_hook(ctx: &mut Handler, _offset: u8, value: &[u8]) -> HookResult {
        ctx.log[ctx.log_len] = value[0];
        ctx.log_len += 1;
        HookResult::SelfClear
    }

    type Server = RegServer<Handler, 96, 8>;

    fn server() -> Server {
        RegServer::new(
            RegMap::<Handler, 96, 8>::new(Handler::default())
                .register(RegEntry::rw(0x14, 4)) // f32 LE register
                .unwrap()
                .register(RegEntry::ro(0x20, 4)) // burst telemetry block start
                .unwrap()
                .register(RegEntry::ro(0x24, 4))
                .unwrap()
                .register(RegEntry::ro(0x28, 4))
                .unwrap()
                .register(RegEntry::ro(0x2C, 4))
                .unwrap(),
        )
    }

    /// I2C bus adapter: an `embedded_hal::i2c::I2c` master port lowered onto a
    /// [`RegServer`]. A master write transaction lowers to
    /// [`TargetTransport::on_write`]; a master read transaction lowers to
    /// [`TargetTransport::on_read`]. Server map errors are remapped to I2C
    /// [`eh_i2c::ErrorKind::NoAcknowledge`] — exactly the mapped error condition the
    /// spec requires.
    struct FakeI2cBus<'a> {
        server: &'a mut Server,
    }

    impl<'a> eh_i2c::ErrorType for FakeI2cBus<'a> {
        type Error = eh_i2c::ErrorKind;
    }

    impl<'a> I2c for FakeI2cBus<'a> {
        fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            self.server
                .on_write(bytes)
                .map_err(|_| eh_i2c::ErrorKind::NoAcknowledge(eh_i2c::NoAcknowledgeSource::Address))
        }

        fn read(&mut self, _address: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            match self.server.on_read(buf.len(), buf) {
                Ok(_) => Ok(()),
                Err(MapError::BufferTooSmall) => Err(eh_i2c::ErrorKind::Overrun),
                Err(_) => Err(eh_i2c::ErrorKind::NoAcknowledge(eh_i2c::NoAcknowledgeSource::Address)),
            }
        }

        fn write_read(
            &mut self,
            address: u8,
            initial: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.write(address, initial)?;
            self.read(address, buffer)
        }

        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [eh_i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            for op in operations.iter_mut() {
                match op {
                    eh_i2c::Operation::Write(b) => self.write(address, b)?,
                    eh_i2c::Operation::Read(b) => self.read(address, b)?,
                }
            }
            Ok(())
        }
    }

    /// SPI device adapter: `SpiDevice` transactions equal CS frames. Write operations
    /// within one frame concatenate into a single `on_write` call (the client splits
    /// `[reg][payload]` into sequential write ops with no staging buffer); Read
    /// operations lower to `on_read`. An in-flight partial write frame is flushed
    /// before any read op of the same transaction.
    struct FakeSpiDevice<'a> {
        server: &'a mut Server,
        frame: [u8; 32],
        frame_len: usize,
    }

    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    enum FakeSpiError {
        ServerError,
    }

    impl eh_spi::Error for FakeSpiError {
        fn kind(&self) -> eh_spi::ErrorKind {
            eh_spi::ErrorKind::Other
        }
    }

    impl<'a> eh_spi::ErrorType for FakeSpiDevice<'a> {
        type Error = FakeSpiError;
    }

    impl<'a> SpiDevice for FakeSpiDevice<'a> {
        fn transaction(
            &mut self,
            operations: &mut [eh_spi::Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations.iter_mut() {
                match op {
                    eh_spi::Operation::Write(b) => {
                        for &byte in b.iter() {
                            if self.frame_len < self.frame.len() {
                                self.frame[self.frame_len] = byte;
                                self.frame_len += 1;
                            }
                        }
                    }
                    flush_or_read => {
                        // Flush the accumulated write frame first so `[reg]` latch +
                        // read burst land in order even inside one transaction.
                        if self.frame_len > 0 {
                            let frame_len = self.frame_len;
                            self.frame_len = 0;
                            self.server
                                .on_write(&self.frame[..frame_len])
                                .map_err(|_| FakeSpiError::ServerError)?;
                        }
                        if let eh_spi::Operation::Read(buf) = flush_or_read {
                            let len = buf.len();
                            if self.server.on_read(len, buf).is_err() {
                                return Err(FakeSpiError::ServerError);
                            }
                        }
                    }
                }}
            if self.frame_len > 0 {
                let frame_len = self.frame_len;
                self.frame_len = 0;
                self.server
                    .on_write(&self.frame[..frame_len])
                    .map_err(|_| FakeSpiError::ServerError)?;
            }
            Ok(())
        }
    }

    // ------------------------------------------------------------------
    // I2C client: scripted wire-byte verification through the mock bus
    // ------------------------------------------------------------------

    #[test]
    fn i2c_wire_bytes_pointer_write_then_burst_read() {
        // Client write_register: single write transaction [reg][payload].
        // Client read_register: [reg] latch write, then 4 LE bytes (12.5) streamed.
        let expectations = [
            I2cTx::write(DEV, vec![0x14, 0, 0, 0x48, 0x41]), // WRITE: pointer + payload (12.5 LE)
            I2cTx::write(DEV, vec![0x14]),                   // READ: pointer latch
            I2cTx::read(DEV, vec![0, 0, 0x48, 0x41]),        // READ: 4 LE bytes = 12.5
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);
            let le = 12.5f32.to_le_bytes();
            client.write_register(DEV, 0x14, &le).unwrap();
            let mut out = [0u8; 4];
            client.read_register(DEV, 0x14, 4, &mut out).unwrap();
            assert_eq!(f32::from_le_bytes(out), 12.5);
        }
        bus.done();
    }

    #[test]
    fn i2c_client_maps_scripted_bus_error() {
        let expectations = [
            I2cTx::write(DEV, vec![0x50, 5]).with_error(eh_i2c::ErrorKind::NoAcknowledge(
                eh_i2c::NoAcknowledgeSource::Address,
            )),
        ];
        let mut bus = I2cMock::new(&expectations);
        let mut client = RegClientI2c::<_, 8>::new(&mut bus);
        assert_eq!(
            client.write_register(DEV, 0x50, &[5]),
            Err(ClientError::Bus(eh_i2c::ErrorKind::NoAcknowledge(
                eh_i2c::NoAcknowledgeSource::Address
            )))
        );
        bus.done();
    }

    #[test]
    fn i2c_client_staging_and_buffer_validation() {
        let expectations = [I2cTx::write(DEV, vec![0x14, 1, 2, 3, 4])];
        let mut bus = I2cMock::new(&expectations);
        let mut client = RegClientI2c::<_, 8>::new(&mut bus);
        // Payload does not fit the 8-byte staging frame: error with no bus traffic.
        assert_eq!(
            client.write_register(DEV, 0x14, &[0u8; 9]),
            Err(ClientError::TxFrameTooLarge)
        );
        // Read out-buffer smaller than the requested length: error, no bus traffic.
        let mut small = [0u8; 2];
        assert_eq!(
            client.read_register(DEV, 0x14, 4, &mut small),
            Err(ClientError::RxBufferTooSmall)
        );
        // The valid write proceeds and consumes the scripted transaction.
        client.write_register(DEV, 0x14, &[1, 2, 3, 4]).unwrap();
        bus.done();
    }

    // ------------------------------------------------------------------
    // I2C client ↔ RegServer FULL live round trips through the fake bus
    // ------------------------------------------------------------------

    #[test]
    fn i2c_full_round_trip_client_server_float() {
        let mut server = server();
        server.map_mut().set(0x14, &12.5f32.to_le_bytes()).unwrap();
        let out = {
            let mut bus = FakeI2cBus { server: &mut server };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);
            let mut out = [0u8; 4];
            client.read_register(DEV, 0x14, 4, &mut out).unwrap();
            out
        };
        assert_eq!(f32::from_le_bytes(out), 12.5);
        // Pointer auto-increment: after a 4-byte read at 0x14 the pointer is 0x18.
        assert_eq!(server.map().pointer(), 0x18);
    }

    #[test]
    fn i2c_full_round_trip_pointer_write_then_burst_read() {
        let mut server = server();
        // 16-byte telemetry block at 0x20 (auto-increment across register boundary).
        server.map_mut().set(0x20, &1.0f32.to_le_bytes()).unwrap();
        server.map_mut().set(0x24, &2.0f32.to_le_bytes()).unwrap();
        let out = {
            let mut bus = FakeI2cBus { server: &mut server };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);
            let mut out = [0u8; 16];
            client
                .read_register(DEV, 0x20, 16, &mut out)
                .unwrap();
            out
        };
        assert_eq!(f32::from_le_bytes([out[0], out[1], out[2], out[3]]), 1.0);
        assert_eq!(f32::from_le_bytes([out[4], out[5], out[6], out[7]]), 2.0);
        assert_eq!(server.map().pointer(), 0x30, "pointer left at 0x30 after 0x20+16");
    }

    #[test]
    fn i2c_unmapped_pointer_latch_surfaces_nak() {
        let mut server = server();
        let err = {
            let mut bus = FakeI2cBus { server: &mut server };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);
            // Server rejects the pointer latch (Unmapped) → adapter maps to NAK.
            client.write_register(DEV, 0xFE, &[]).err().unwrap()
        };
        assert_eq!(
            err,
            ClientError::Bus(eh_i2c::ErrorKind::NoAcknowledge(
                eh_i2c::NoAcknowledgeSource::Address
            ))
        );
        // The server pointer was not moved by the rejected latch.
        assert_eq!(server.map().pointer(), 0);
    }

    #[test]
    fn i2c_read_only_write_naked_by_adapter_mapping() {
        let mut server = server();
        let err = {
            let mut bus = FakeI2cBus { server: &mut server };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);
            // Ro register write rejected server-side (ReadOnly), mapped to NAK.
            client.write_register(DEV, 0x20, &0u32.to_le_bytes()).err().unwrap()
        };
        assert_eq!(
            err,
            ClientError::Bus(eh_i2c::ErrorKind::NoAcknowledge(
                eh_i2c::NoAcknowledgeSource::Address
            ))
        );
        server.map_mut().set(0x20, &7.0f32.to_le_bytes()).unwrap();
        let mut out = [0u8; 4];
        server.map().get(0x20, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 7.0, "rejected write leaves Ro untouched");
    }

    #[test]
    fn i2c_self_clearing_system_cmd_round_trip() {
        let mut server = server();
        server
            .map_mut()
            .add_register(RegEntry::rw_hooked(0x50, 1, sys_cmd_hook))
            .unwrap();
        {
            let mut bus = FakeI2cBus { server: &mut server };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);
            // CMD_REBOOT_BOOTLOADER = 5 (upstream control_table.h SystemCmd).
            client.write_register(DEV, 0x50, &[5]).unwrap();
        }
        // Hook fired exactly once; register self-cleared.
        {
            let map = server.map();
            let mut out = [0u8; 1];
            map.get(0x50, &mut out).unwrap();
            assert_eq!(out[0], 0);
        }
        assert_eq!(server.map_mut().handler().log_len, 1);
        assert_eq!(server.map_mut().handler().log[0], 5);
    }

    // ------------------------------------------------------------------
    // SPI client: scripted wire-byte verification through the mock bus
    // ------------------------------------------------------------------

    /// SpiDevice adapter over the embedded-hal-mock SpiBus: each transaction's
    /// operations are executed back-to-back on the bus (what a dedicated chip-select
    /// driver does), so the mock scripts exact per-operation byte frames.
    struct TestSpiDevice<'a> {
        bus: &'a mut SpiMock,
    }

    impl eh_spi::ErrorType for TestSpiDevice<'_> {
        type Error = eh_spi::ErrorKind;
    }

    impl SpiDevice for TestSpiDevice<'_> {
        fn transaction(
            &mut self,
            operations: &mut [eh_spi::Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations.iter_mut() {
                match op {
                    eh_spi::Operation::Write(b) => SpiBus::write(self.bus, b)?,
                    eh_spi::Operation::Read(b) => SpiBus::read(self.bus, b)?,
                    eh_spi::Operation::Transfer(r, w) => SpiBus::transfer(self.bus, r, w)?,
                    eh_spi::Operation::TransferInPlace(b) => SpiBus::transfer_in_place(self.bus, b)?,
                    _ => {}
                }
            }
            Ok(())
        }
    }

    #[test]
    fn spi_wire_bytes_write_and_reply_in_next_cs_frame_read() {
        // write_register(reg, [LE 12.5]) → one CS frame with [reg][payload] (two ops).
        // read_register(reg, 4) → CS frame 1 [reg] latch; CS frame 2: 4 LE bytes.
        let le = 12.5f32.to_le_bytes();
        let expectations = [
            SpiTx::write_vec(vec![0x14]), // CS frame, op 1: pointer byte
            SpiTx::write_vec(vec![le[0], le[1], le[2], le[3]]), // CS frame, op 2: payload
            SpiTx::write_vec(vec![0x14]), // latch frame of the read
            SpiTx::read_vec(vec![le[0], le[1], le[2], le[3]]),
        ];
        let mut bus = SpiMock::new(&expectations);
        {
            let mut device = TestSpiDevice { bus: &mut bus };
            let mut client = RegClientSpi::new(&mut device);
            client.write_register(0x14, &le).unwrap();
            let mut out = [0u8; 4];
            client.read_register(0x14, 4, &mut out).unwrap();
            assert_eq!(f32::from_le_bytes(out), 12.5);
        }
        bus.done();
    }

    #[test]
    fn spi_client_read_out_buffer_validation() {
        let mut bus = SpiMock::new(&[]);
        {
            let mut device = TestSpiDevice { bus: &mut bus };
            let mut client = RegClientSpi::new(&mut device);
            let mut small = [0u8; 2];
            assert_eq!(
                client.read_register(0x14, 4, &mut small),
                Err(ClientError::RxBufferTooSmall)
            );
        }
        bus.done();
    }

    // ------------------------------------------------------------------
    // SPI client ↔ RegServer FULL live round trips through the fake bus
    // ------------------------------------------------------------------

    #[test]
    fn spi_full_round_trip_client_server_float() {
        let mut server = server();
        server.map_mut().set(0x14, &12.5f32.to_le_bytes()).unwrap();
        {
            let mut device = FakeSpiDevice {
                server: &mut server,
                frame: [0u8; 32],
                frame_len: 0,
            };
            let mut client = RegClientSpi::new(&mut device);
            // Write 5.5 then read it back (reply-in-next-CS-frame).
            client.write_register(0x14, &5.5f32.to_le_bytes()).unwrap();
            let mut out = [0u8; 4];
            client.read_register(0x14, 4, &mut out).unwrap();
            assert_eq!(f32::from_le_bytes(out), 5.5);
        }
        assert_eq!(server.map().pointer(), 0x18);
    }

    #[test]
    fn spi_full_round_trip_self_clearing_system_cmd() {
        let mut server = server();
        server
            .map_mut()
            .add_register(RegEntry::rw_hooked(0x50, 1, sys_cmd_hook))
            .unwrap();
        {
            let mut device = FakeSpiDevice {
                server: &mut server,
                frame: [0u8; 32],
                frame_len: 0,
            };
            let mut client = RegClientSpi::new(&mut device);
            // CMD_SAVE_CONFIG = 1 (upstream SystemCmd), self-clearing hook.
            client.write_register(0x50, &[1]).unwrap();
        }
        let mut out = [0u8; 1];
        server.map().get(0x50, &mut out).unwrap();
        assert_eq!(out[0], 0, "self-cleared after hook dispatch");
        assert_eq!(server.map_mut().handler().log_len, 1);
        assert_eq!(server.map_mut().handler().log[0], 1);
    }

    #[test]
    fn spi_unmapped_pointer_latch_surfaces_error() {
        let mut server = server();
        let err = {
            let mut device = FakeSpiDevice {
                server: &mut server,
                frame: [0u8; 32],
                frame_len: 0,
            };
            let mut client = RegClientSpi::new(&mut device);
            client.write_register(0xFE, &[]).err().unwrap()
        };
        assert_eq!(err, ClientError::Bus(FakeSpiError::ServerError));
        assert_eq!(server.map().pointer(), 0);
    }
}
