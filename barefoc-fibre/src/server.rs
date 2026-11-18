//! Fibre endpoint server and property dispatcher.
//!
//! References upstream:
//! - `ref/fibre-cpp/legacy_protocol.hpp`
//! - `ref/fibre-cpp/legacy_protocol.cpp` (`LegacyProtocolPacketBased::on_read_finished`)
//! - `ref/fibre-cpp/endpoints_template.j2`

use crate::crc::PROTOCOL_VERSION;
use crate::framing::{FramingError, PacketUnwrapper, PacketWrapper};
use crate::schema::{Endpoint0Error, Endpoint0Handler};

/// Errors that can occur during packet processing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServerError {
    /// Packet size is less than 8 bytes (minimum header + trailer).
    PacketTooShort,
    /// Packet trailer did not match expected PROTOCOL_VERSION or json_crc.
    TrailerMismatch { expected: u16, actual: u16 },
    /// Requested endpoint ID is not registered.
    EndpointNotFound(u16),
    /// Attempted to write to a read-only endpoint.
    ReadOnlyEndpoint(u16),
    /// Writing to endpoint failed (e.g. invalid value format).
    WriteFailed(u16),
    /// Response buffer provided by caller is too small.
    ResponseBufferTooSmall,
    /// Endpoint 0 request error.
    Endpoint0(Endpoint0Error),
}

/// Errors when processing framed packets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessFramedError {
    /// Framing error on input or output.
    Framing(FramingError),
    /// Server error during packet dispatch.
    Server(ServerError),
}

impl From<FramingError> for ProcessFramedError {
    fn from(err: FramingError) -> Self {
        Self::Framing(err)
    }
}

impl From<ServerError> for ProcessFramedError {
    fn from(err: ServerError) -> Self {
        Self::Server(err)
    }
}

/// Raw reader function type writing typed bytes to output buffer.
pub type RawReadFn<Ctx> = fn(&Ctx, &mut [u8]) -> usize;

/// Raw writer function type parsing typed bytes from input buffer.
pub type RawWriteFn<Ctx> = fn(&mut Ctx, &[u8]) -> Result<(), ()>;

/// Internal handler storing typed property getter and optional setter functions.
pub enum PropertyHandler<Ctx> {
    /// Read-only `f32`.
    FloatRo(fn(&Ctx) -> f32),
    /// Read-write `f32`.
    FloatRw(fn(&Ctx) -> f32, fn(&mut Ctx, f32)),
    /// Read-only `u32`.
    UInt32Ro(fn(&Ctx) -> u32),
    /// Read-write `u32`.
    UInt32Rw(fn(&Ctx) -> u32, fn(&mut Ctx, u32)),
    /// Read-only `i32`.
    Int32Ro(fn(&Ctx) -> i32),
    /// Read-write `i32`.
    Int32Rw(fn(&Ctx) -> i32, fn(&mut Ctx, i32)),
    /// Read-only `bool`.
    BoolRo(fn(&Ctx) -> bool),
    /// Read-write `bool`.
    BoolRw(fn(&Ctx) -> bool, fn(&mut Ctx, bool)),
    /// Custom byte buffer read and optional write functions.
    Custom {
        read_fn: Option<RawReadFn<Ctx>>,
        write_fn: Option<RawWriteFn<Ctx>>,
    },
}

/// A registered Fibre property endpoint.
pub struct Endpoint<Ctx> {
    /// Endpoint identifier matching the JSON manifest.
    pub id: u16,
    /// Typed property handler.
    pub handler: PropertyHandler<Ctx>,
}

impl<Ctx> Endpoint<Ctx> {
    /// Creates a read-only 32-bit float (`f32`) endpoint.
    pub fn readonly_f32(id: u16, getter: fn(&Ctx) -> f32) -> Self {
        Self {
            id,
            handler: PropertyHandler::FloatRo(getter),
        }
    }

    /// Creates a read-write 32-bit float (`f32`) endpoint.
    pub fn readwrite_f32(
        id: u16,
        getter: fn(&Ctx) -> f32,
        setter: fn(&mut Ctx, f32),
    ) -> Self {
        Self {
            id,
            handler: PropertyHandler::FloatRw(getter, setter),
        }
    }

    /// Creates a read-only 32-bit unsigned integer (`u32`) endpoint.
    pub fn readonly_u32(id: u16, getter: fn(&Ctx) -> u32) -> Self {
        Self {
            id,
            handler: PropertyHandler::UInt32Ro(getter),
        }
    }

    /// Creates a read-write 32-bit unsigned integer (`u32`) endpoint.
    pub fn readwrite_u32(
        id: u16,
        getter: fn(&Ctx) -> u32,
        setter: fn(&mut Ctx, u32),
    ) -> Self {
        Self {
            id,
            handler: PropertyHandler::UInt32Rw(getter, setter),
        }
    }

    /// Creates a read-only 32-bit signed integer (`i32`) endpoint.
    pub fn readonly_i32(id: u16, getter: fn(&Ctx) -> i32) -> Self {
        Self {
            id,
            handler: PropertyHandler::Int32Ro(getter),
        }
    }

    /// Creates a read-write 32-bit signed integer (`i32`) endpoint.
    pub fn readwrite_i32(
        id: u16,
        getter: fn(&Ctx) -> i32,
        setter: fn(&mut Ctx, i32),
    ) -> Self {
        Self {
            id,
            handler: PropertyHandler::Int32Rw(getter, setter),
        }
    }

    /// Creates a read-only boolean (`bool`) endpoint.
    pub fn readonly_bool(id: u16, getter: fn(&Ctx) -> bool) -> Self {
        Self {
            id,
            handler: PropertyHandler::BoolRo(getter),
        }
    }

    /// Creates a read-write boolean (`bool`) endpoint.
    pub fn readwrite_bool(
        id: u16,
        getter: fn(&Ctx) -> bool,
        setter: fn(&mut Ctx, bool),
    ) -> Self {
        Self {
            id,
            handler: PropertyHandler::BoolRw(getter, setter),
        }
    }

    /// Creates a custom endpoint with raw buffer read/write handlers.
    pub fn custom(
        id: u16,
        read_fn: Option<RawReadFn<Ctx>>,
        write_fn: Option<RawWriteFn<Ctx>>,
    ) -> Self {
        Self {
            id,
            handler: PropertyHandler::Custom { read_fn, write_fn },
        }
    }

    /// Reads property value into output buffer.
    pub fn read(&self, ctx: &Ctx, out: &mut [u8]) -> usize {
        match &self.handler {
            PropertyHandler::FloatRo(get) | PropertyHandler::FloatRw(get, _) => {
                if out.len() >= 4 {
                    out[..4].copy_from_slice(&get(ctx).to_le_bytes());
                    4
                } else {
                    0
                }
            }
            PropertyHandler::UInt32Ro(get) | PropertyHandler::UInt32Rw(get, _) => {
                if out.len() >= 4 {
                    out[..4].copy_from_slice(&get(ctx).to_le_bytes());
                    4
                } else {
                    0
                }
            }
            PropertyHandler::Int32Ro(get) | PropertyHandler::Int32Rw(get, _) => {
                if out.len() >= 4 {
                    out[..4].copy_from_slice(&get(ctx).to_le_bytes());
                    4
                } else {
                    0
                }
            }
            PropertyHandler::BoolRo(get) | PropertyHandler::BoolRw(get, _) => {
                if !out.is_empty() {
                    out[0] = if get(ctx) { 1 } else { 0 };
                    1
                } else {
                    0
                }
            }
            PropertyHandler::Custom {
                read_fn: Some(read_fn),
                ..
            } => read_fn(ctx, out),
            _ => 0,
        }
    }

    /// Writes property value from input buffer.
    pub fn write(&self, ctx: &mut Ctx, input: &[u8]) -> Result<(), ()> {
        match &self.handler {
            PropertyHandler::FloatRw(_, set) => {
                if input.len() >= 4 {
                    let v = f32::from_le_bytes([input[0], input[1], input[2], input[3]]);
                    set(ctx, v);
                    Ok(())
                } else {
                    Err(())
                }
            }
            PropertyHandler::UInt32Rw(_, set) => {
                if input.len() >= 4 {
                    let v = u32::from_le_bytes([input[0], input[1], input[2], input[3]]);
                    set(ctx, v);
                    Ok(())
                } else {
                    Err(())
                }
            }
            PropertyHandler::Int32Rw(_, set) => {
                if input.len() >= 4 {
                    let v = i32::from_le_bytes([input[0], input[1], input[2], input[3]]);
                    set(ctx, v);
                    Ok(())
                } else {
                    Err(())
                }
            }
            PropertyHandler::BoolRw(_, set) => {
                if !input.is_empty() {
                    set(ctx, input[0] != 0);
                    Ok(())
                } else {
                    Err(())
                }
            }
            PropertyHandler::Custom {
                write_fn: Some(write_fn),
                ..
            } => write_fn(ctx, input),
            _ => Err(()),
        }
    }

    /// Whether this endpoint supports writing.
    pub fn is_writable(&self) -> bool {
        matches!(
            &self.handler,
            PropertyHandler::FloatRw(..)
                | PropertyHandler::UInt32Rw(..)
                | PropertyHandler::Int32Rw(..)
                | PropertyHandler::BoolRw(..)
                | PropertyHandler::Custom {
                    write_fn: Some(_),
                    ..
                }
        )
    }
}

/// Fibre RPC Server handling Endpoint 0 queries and property dispatching.
pub struct FibreServer<'a, Ctx> {
    schema: Endpoint0Handler,
    endpoints: &'a [Endpoint<Ctx>],
}

impl<'a, Ctx> FibreServer<'a, Ctx> {
    /// Creates a new `FibreServer` with given schema and endpoint registry.
    pub fn new(schema: Endpoint0Handler, endpoints: &'a [Endpoint<Ctx>]) -> Self {
        Self { schema, endpoints }
    }

    /// Reference to the underlying Endpoint 0 handler / schema.
    pub fn schema(&self) -> &Endpoint0Handler {
        &self.schema
    }

    /// Processes an unframed packet payload received from the client.
    ///
    /// If a response packet is required, writes the payload to `response_buf` and
    /// returns `Ok(Some(len))`. If no response is expected, returns `Ok(None)`.
    pub fn process_packet(
        &self,
        packet: &[u8],
        response_buf: &mut [u8],
        ctx: &mut Ctx,
    ) -> Result<Option<usize>, ServerError> {
        if packet.len() < 8 {
            return Err(ServerError::PacketTooShort);
        }

        let seq_no = u16::from_le_bytes([packet[0], packet[1]]);
        let endpoint_raw = u16::from_le_bytes([packet[2], packet[3]]);
        let expect_response = (endpoint_raw & 0x8000) != 0;
        let ep_id = endpoint_raw & 0x7FFF;
        let expected_response_len = u16::from_le_bytes([packet[4], packet[5]]) as usize;

        let input_data = &packet[6..packet.len() - 2];
        let actual_trailer =
            u16::from_le_bytes([packet[packet.len() - 2], packet[packet.len() - 1]]);

        let expected_trailer = if ep_id == 0 {
            PROTOCOL_VERSION
        } else {
            self.schema.json_crc()
        };

        if actual_trailer != expected_trailer {
            return Err(ServerError::TrailerMismatch {
                expected: expected_trailer,
                actual: actual_trailer,
            });
        }

        if ep_id == 0 {
            if expect_response && response_buf.len() < 2 {
                return Err(ServerError::ResponseBufferTooSmall);
            }

            let max_response = if expect_response {
                expected_response_len.min(response_buf.len() - 2)
            } else {
                0
            };

            let written = if expect_response {
                self.schema
                    .handle_request(input_data, &mut response_buf[2..2 + max_response])
                    .map_err(ServerError::Endpoint0)?
            } else {
                0
            };

            if expect_response {
                response_buf[0..2].copy_from_slice(&(seq_no | 0x8000).to_le_bytes());
                Ok(Some(2 + written))
            } else {
                Ok(None)
            }
        } else {
            // Find registered endpoint
            let ep = self
                .endpoints
                .iter()
                .find(|e| e.id == ep_id)
                .ok_or(ServerError::EndpointNotFound(ep_id))?;

            // If input data is present, execute write
            if !input_data.is_empty() {
                if !ep.is_writable() {
                    return Err(ServerError::ReadOnlyEndpoint(ep_id));
                }
                ep.write(ctx, input_data)
                    .map_err(|_| ServerError::WriteFailed(ep_id))?;
            }

            if expect_response {
                if response_buf.len() < 2 {
                    return Err(ServerError::ResponseBufferTooSmall);
                }

                let max_out = expected_response_len.min(response_buf.len() - 2);
                let written = ep.read(ctx, &mut response_buf[2..2 + max_out]).min(max_out);

                response_buf[0..2].copy_from_slice(&(seq_no | 0x8000).to_le_bytes());
                Ok(Some(2 + written))
            } else {
                Ok(None)
            }
        }
    }

    /// Convenience method to process a framed packet end-to-end:
    /// unwraps `framed_input`, dispatches via `process_packet`, and wraps the response into `framed_output`.
    pub fn process_framed_packet(
        &self,
        framed_input: &[u8],
        framed_output: &mut [u8],
        ctx: &mut Ctx,
    ) -> Result<Option<usize>, ProcessFramedError> {
        let mut rx_buf = [0u8; 128];
        let payload_len = PacketUnwrapper::unwrap(framed_input, &mut rx_buf)?;

        let mut tx_buf = [0u8; 128];
        match self.process_packet(&rx_buf[..payload_len], &mut tx_buf, ctx)? {
            Some(resp_len) => {
                let frame_len = PacketWrapper::wrap(&tx_buf[..resp_len], framed_output)?;
                Ok(Some(frame_len))
            }
            None => Ok(None),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestContext {
        vbus_voltage: f32,
        target_velocity: f32,
        counter: u32,
        mode: i32,
        enabled: bool,
    }

    const TEST_SCHEMA: &str = r#"[{"name":"vbus_voltage","id":1,"type":"float","access":"r"},{"name":"target_velocity","id":2,"type":"float","access":"rw"},{"name":"counter","id":3,"type":"uint32","access":"rw"},{"name":"mode","id":4,"type":"int32","access":"rw"},{"name":"enabled","id":5,"type":"bool","access":"rw"}]"#;

    #[test]
    fn test_server_endpoint0_query() {
        let schema = Endpoint0Handler::new(TEST_SCHEMA);
        let endpoints: [Endpoint<TestContext>; 0] = [];
        let server = FibreServer::new(schema, &endpoints);

        let mut ctx = TestContext {
            vbus_voltage: 24.0,
            target_velocity: 0.0,
            counter: 100,
            mode: -1,
            enabled: true,
        };

        // Client request to Endpoint 0: seq_no=1, ep=0 | 0x8000, rx_len=32, offset=0, trailer=1
        let mut req = [0u8; 12];
        req[0..2].copy_from_slice(&1u16.to_le_bytes()); // seq_no
        req[2..4].copy_from_slice(&0x8000u16.to_le_bytes()); // ep 0 with expect_response
        req[4..6].copy_from_slice(&32u16.to_le_bytes()); // expected rx length
        req[6..10].copy_from_slice(&0u32.to_le_bytes()); // offset 0
        req[10..12].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes()); // trailer

        let mut resp = [0u8; 64];
        let resp_len = server.process_packet(&req, &mut resp, &mut ctx).unwrap().unwrap();

        assert_eq!(u16::from_le_bytes([resp[0], resp[1]]), 1 | 0x8000); // ACK seq_no
        assert_eq!(resp_len, 2 + 32);
        assert_eq!(&resp[2..2 + 32], &TEST_SCHEMA.as_bytes()[..32]);
    }

    #[test]
    fn test_server_property_read_float() {
        let schema = Endpoint0Handler::new(TEST_SCHEMA);
        let endpoints = [
            Endpoint::readonly_f32(1, |ctx: &TestContext| ctx.vbus_voltage),
            Endpoint::readwrite_f32(
                2,
                |ctx: &TestContext| ctx.target_velocity,
                |ctx: &mut TestContext, val| ctx.target_velocity = val,
            ),
        ];
        let server = FibreServer::new(schema, &endpoints);

        let mut ctx = TestContext {
            vbus_voltage: 24.5,
            target_velocity: 15.0,
            counter: 100,
            mode: 0,
            enabled: true,
        };

        // Read ep 1 (vbus_voltage): seq=5, ep=1 | 0x8000, rx_len=4, empty input, trailer=json_crc
        let mut req = [0u8; 8];
        req[0..2].copy_from_slice(&5u16.to_le_bytes());
        req[2..4].copy_from_slice(&(1u16 | 0x8000).to_le_bytes());
        req[4..6].copy_from_slice(&4u16.to_le_bytes());
        req[6..8].copy_from_slice(&schema.json_crc().to_le_bytes());

        let mut resp = [0u8; 32];
        let len = server.process_packet(&req, &mut resp, &mut ctx).unwrap().unwrap();

        assert_eq!(u16::from_le_bytes([resp[0], resp[1]]), 5 | 0x8000);
        assert_eq!(len, 6);
        let val = f32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]);
        assert!((val - 24.5).abs() < 1e-6);
    }

    #[test]
    fn test_server_property_write_float() {
        let schema = Endpoint0Handler::new(TEST_SCHEMA);
        let endpoints = [
            Endpoint::readonly_f32(1, |ctx: &TestContext| ctx.vbus_voltage),
            Endpoint::readwrite_f32(
                2,
                |ctx: &TestContext| ctx.target_velocity,
                |ctx: &mut TestContext, val| ctx.target_velocity = val,
            ),
        ];
        let server = FibreServer::new(schema, &endpoints);

        let mut ctx = TestContext {
            vbus_voltage: 24.0,
            target_velocity: 0.0,
            counter: 100,
            mode: -1,
            enabled: true,
        };

        // Write 42.0 to ep 2: seq=7, ep=2 | 0x8000, rx_len=4, input=42.0f32, trailer=json_crc
        let mut req = [0u8; 12];
        req[0..2].copy_from_slice(&7u16.to_le_bytes());
        req[2..4].copy_from_slice(&(2u16 | 0x8000).to_le_bytes());
        req[4..6].copy_from_slice(&4u16.to_le_bytes());
        req[6..10].copy_from_slice(&42.0f32.to_le_bytes());
        req[10..12].copy_from_slice(&schema.json_crc().to_le_bytes());

        let mut resp = [0u8; 32];
        let len = server.process_packet(&req, &mut resp, &mut ctx).unwrap().unwrap();

        assert_eq!(ctx.target_velocity, 42.0);
        assert_eq!(u16::from_le_bytes([resp[0], resp[1]]), 7 | 0x8000);
        assert_eq!(len, 6);
        let returned_val = f32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]);
        assert_eq!(returned_val, 42.0);
    }

    #[test]
    fn test_server_trailer_mismatch_rejection() {
        let schema = Endpoint0Handler::new(TEST_SCHEMA);
        let endpoints = [Endpoint::readonly_f32(1, |ctx: &TestContext| ctx.vbus_voltage)];
        let server = FibreServer::new(schema, &endpoints);

        let mut ctx = TestContext {
            vbus_voltage: 24.0,
            target_velocity: 0.0,
            counter: 100,
            mode: -1,
            enabled: true,
        };

        let mut req = [0u8; 8];
        req[0..2].copy_from_slice(&1u16.to_le_bytes());
        req[2..4].copy_from_slice(&(1u16 | 0x8000).to_le_bytes());
        req[4..6].copy_from_slice(&4u16.to_le_bytes());
        req[6..8].copy_from_slice(&0xDEADu16.to_le_bytes()); // BAD trailer

        let mut resp = [0u8; 32];
        let res = server.process_packet(&req, &mut resp, &mut ctx);
        assert!(matches!(res, Err(ServerError::TrailerMismatch { .. })));
    }

    #[test]
    fn test_server_property_readwrite_int_and_bool() {
        let schema = Endpoint0Handler::new(TEST_SCHEMA);
        let endpoints = [
            Endpoint::readwrite_u32(
                3,
                |ctx: &TestContext| ctx.counter,
                |ctx: &mut TestContext, val| ctx.counter = val,
            ),
            Endpoint::readwrite_i32(
                4,
                |ctx: &TestContext| ctx.mode,
                |ctx: &mut TestContext, val| ctx.mode = val,
            ),
            Endpoint::readwrite_bool(
                5,
                |ctx: &TestContext| ctx.enabled,
                |ctx: &mut TestContext, val| ctx.enabled = val,
            ),
        ];
        let server = FibreServer::new(schema, &endpoints);

        let mut ctx = TestContext {
            vbus_voltage: 24.0,
            target_velocity: 0.0,
            counter: 12345,
            mode: -42,
            enabled: false,
        };

        // Write to counter (u32, ep 3)
        let mut req_u32 = [0u8; 12];
        req_u32[0..2].copy_from_slice(&10u16.to_le_bytes());
        req_u32[2..4].copy_from_slice(&(3u16 | 0x8000).to_le_bytes());
        req_u32[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_u32[6..10].copy_from_slice(&99999u32.to_le_bytes());
        req_u32[10..12].copy_from_slice(&schema.json_crc().to_le_bytes());

        let mut resp = [0u8; 32];
        let len = server.process_packet(&req_u32, &mut resp, &mut ctx).unwrap().unwrap();
        assert_eq!(len, 6);
        assert_eq!(ctx.counter, 99999);
        assert_eq!(u32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]), 99999);

        // Write to enabled (bool, ep 5)
        let mut req_bool = [0u8; 9];
        req_bool[0..2].copy_from_slice(&11u16.to_le_bytes());
        req_bool[2..4].copy_from_slice(&(5u16 | 0x8000).to_le_bytes());
        req_bool[4..6].copy_from_slice(&1u16.to_le_bytes());
        req_bool[6] = 1; // true
        req_bool[7..9].copy_from_slice(&schema.json_crc().to_le_bytes());

        let len_bool = server.process_packet(&req_bool, &mut resp, &mut ctx).unwrap().unwrap();
        assert_eq!(len_bool, 3);
        assert!(ctx.enabled);
        assert_eq!(resp[2], 1);
    }

    #[test]
    fn test_server_process_framed_packet_roundtrip() {
        let schema = Endpoint0Handler::new(TEST_SCHEMA);
        let endpoints = [
            Endpoint::readonly_f32(1, |ctx: &TestContext| ctx.vbus_voltage),
            Endpoint::readwrite_f32(
                2,
                |ctx: &TestContext| ctx.target_velocity,
                |ctx: &mut TestContext, val| ctx.target_velocity = val,
            ),
        ];
        let server = FibreServer::new(schema, &endpoints);

        let mut ctx = TestContext {
            vbus_voltage: 24.0,
            target_velocity: 0.0,
            counter: 100,
            mode: -1,
            enabled: true,
        };

        // Create write request packet payload
        let mut req_payload = [0u8; 12];
        req_payload[0..2].copy_from_slice(&1u16.to_le_bytes());
        req_payload[2..4].copy_from_slice(&(2u16 | 0x8000).to_le_bytes());
        req_payload[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_payload[6..10].copy_from_slice(&123.456f32.to_le_bytes());
        req_payload[10..12].copy_from_slice(&schema.json_crc().to_le_bytes());

        // Wrap request into framed buffer
        let mut framed_req = [0u8; 64];
        let framed_req_len = PacketWrapper::wrap(&req_payload, &mut framed_req).unwrap();

        // Process framed request
        let mut framed_resp = [0u8; 64];
        let resp_len = server
            .process_framed_packet(&framed_req[..framed_req_len], &mut framed_resp, &mut ctx)
            .unwrap()
            .unwrap();

        assert_eq!(ctx.target_velocity, 123.456);

        // Unwrap response frame
        let mut resp_payload = [0u8; 32];
        let resp_payload_len =
            PacketUnwrapper::unwrap(&framed_resp[..resp_len], &mut resp_payload).unwrap();
        assert_eq!(resp_payload_len, 6);
        assert_eq!(u16::from_le_bytes([resp_payload[0], resp_payload[1]]), 1 | 0x8000);
        let returned_val = f32::from_le_bytes([
            resp_payload[2],
            resp_payload[3],
            resp_payload[4],
            resp_payload[5],
        ]);
        assert_eq!(returned_val, 123.456);
    }

    #[test]
    fn test_server_write_to_readonly_rejected() {
        let schema = Endpoint0Handler::new(TEST_SCHEMA);
        let endpoints = [Endpoint::readonly_f32(1, |ctx: &TestContext| ctx.vbus_voltage)];
        let server = FibreServer::new(schema, &endpoints);

        let mut ctx = TestContext {
            vbus_voltage: 24.0,
            target_velocity: 0.0,
            counter: 100,
            mode: -1,
            enabled: true,
        };

        // Write to ep 1 (read-only)
        let mut req = [0u8; 12];
        req[0..2].copy_from_slice(&1u16.to_le_bytes());
        req[2..4].copy_from_slice(&(1u16 | 0x8000).to_le_bytes());
        req[4..6].copy_from_slice(&4u16.to_le_bytes());
        req[6..10].copy_from_slice(&48.0f32.to_le_bytes());
        req[10..12].copy_from_slice(&schema.json_crc().to_le_bytes());

        let mut resp = [0u8; 32];
        let res = server.process_packet(&req, &mut resp, &mut ctx);
        assert!(matches!(res, Err(ServerError::ReadOnlyEndpoint(1))));
    }
}
