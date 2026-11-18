//! Packet framing and stream serialization for Fibre.
//!
//! Encodes and decodes packets with the canonical `0xAA` sync byte,
//! CRC-8 header check, and CRC-16 trailer check.
//!
//! References upstream:
//! - `ref/fibre-cpp/legacy_protocol.hpp`
//! - `ref/fibre-cpp/legacy_protocol.cpp` (`PacketWrapper`, `PacketUnwrapper`)

use crate::crc::{
    calc_crc16_buffer, calc_crc8_buffer, CANONICAL_CRC16_INIT, CANONICAL_CRC8_INIT,
    CANONICAL_PREFIX,
};

/// Maximum payload size supported by Fibre single-byte length framing.
pub const MAX_PAYLOAD_SIZE: usize = 127;

/// Header size in bytes: sync (1) + length (1) + crc8 (1).
pub const HEADER_SIZE: usize = 3;

/// Trailer size in bytes: crc16 (2).
pub const TRAILER_SIZE: usize = 2;

/// Framing overhead in bytes (header + trailer = 5).
pub const OVERHEAD_SIZE: usize = HEADER_SIZE + TRAILER_SIZE;

/// Errors that can occur during packet framing / unwrapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FramingError {
    /// Payload exceeds maximum allowed length of 127 bytes.
    PayloadTooLarge,
    /// Provided buffer is too small to hold the frame or payload.
    BufferTooSmall,
    /// Header length byte had MSB set (`len >= 128`).
    InvalidLength(u8),
    /// Header CRC-8 check failed.
    CorruptHeaderCrc,
    /// Trailer CRC-16 check failed.
    CorruptPayloadCrc { expected: u16, actual: u16 },
    /// No sync byte `0xAA` found in stream.
    NoSync,
    /// Incomplete frame received.
    IncompleteFrame,
}

/// State of the streaming `PacketUnwrapper` state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnwrapperState {
    /// Searching for sync byte (`0xAA`).
    Sync,
    /// Expecting payload length byte.
    Length,
    /// Expecting header CRC-8 byte.
    HeaderCrc,
    /// Accumulating payload bytes.
    Payload,
    /// Expecting trailer CRC-16 high byte.
    TrailerHigh,
    /// Expecting trailer CRC-16 low byte.
    TrailerLow,
}

/// State machine for unwrapping framed packets from a byte stream.
#[derive(Debug, Clone)]
pub struct PacketUnwrapper {
    state: UnwrapperState,
    expected_len: usize,
    payload_pos: usize,
    trailer_high: u8,
}

impl Default for PacketUnwrapper {
    fn default() -> Self {
        Self::new()
    }
}

impl PacketUnwrapper {
    /// Creates a new `PacketUnwrapper` in the initial `Sync` search state.
    pub const fn new() -> Self {
        Self {
            state: UnwrapperState::Sync,
            expected_len: 0,
            payload_pos: 0,
            trailer_high: 0,
        }
    }

    /// Resets the unwrapper to search for the next sync byte.
    pub fn reset(&mut self) {
        self.state = UnwrapperState::Sync;
        self.expected_len = 0;
        self.payload_pos = 0;
        self.trailer_high = 0;
    }

    /// Current state of the parser.
    pub fn state(&self) -> UnwrapperState {
        self.state
    }

    /// Feed a single byte into the unwrapper.
    ///
    /// When a complete, CRC-verified packet has been received, returns
    /// `Ok(Some(len))` where `len` is the payload length stored in `payload_buf`.
    /// Otherwise returns `Ok(None)` to continue receiving bytes, or `Err(FramingError)`
    /// on corruption (the unwrapper automatically resets to recover sync on error).
    pub fn push_byte(
        &mut self,
        byte: u8,
        payload_buf: &mut [u8],
    ) -> Result<Option<usize>, FramingError> {
        match self.state {
            UnwrapperState::Sync => {
                if byte == CANONICAL_PREFIX {
                    self.state = UnwrapperState::Length;
                }
                Ok(None)
            }
            UnwrapperState::Length => {
                if (byte & 0x80) != 0 {
                    // Invalid length: reset. If this byte happens to be 0xAA, stay in Length.
                    if byte == CANONICAL_PREFIX {
                        self.state = UnwrapperState::Length;
                    } else {
                        self.state = UnwrapperState::Sync;
                    }
                    Err(FramingError::InvalidLength(byte))
                } else {
                    self.expected_len = byte as usize;
                    self.state = UnwrapperState::HeaderCrc;
                    Ok(None)
                }
            }
            UnwrapperState::HeaderCrc => {
                let header = [CANONICAL_PREFIX, self.expected_len as u8, byte];
                if calc_crc8_buffer(CANONICAL_CRC8_INIT, &header) != 0 {
                    // Corrupted header. Check if current byte is 0xAA to restart sync immediately.
                    if byte == CANONICAL_PREFIX {
                        self.state = UnwrapperState::Length;
                    } else {
                        self.state = UnwrapperState::Sync;
                    }
                    Err(FramingError::CorruptHeaderCrc)
                } else {
                    self.payload_pos = 0;
                    if self.expected_len == 0 {
                        self.state = UnwrapperState::TrailerHigh;
                    } else {
                        self.state = UnwrapperState::Payload;
                    }
                    Ok(None)
                }
            }
            UnwrapperState::Payload => {
                if self.payload_pos < payload_buf.len() {
                    payload_buf[self.payload_pos] = byte;
                }
                self.payload_pos += 1;
                if self.payload_pos == self.expected_len {
                    self.state = UnwrapperState::TrailerHigh;
                }
                Ok(None)
            }
            UnwrapperState::TrailerHigh => {
                self.trailer_high = byte;
                self.state = UnwrapperState::TrailerLow;
                Ok(None)
            }
            UnwrapperState::TrailerLow => {
                self.state = UnwrapperState::Sync;
                let actual_crc = ((self.trailer_high as u16) << 8) | (byte as u16);
                if self.payload_pos > payload_buf.len() {
                    return Err(FramingError::BufferTooSmall);
                }

                let payload = &payload_buf[..self.expected_len];
                let expected_crc = calc_crc16_buffer(CANONICAL_CRC16_INIT, payload);

                if actual_crc != expected_crc {
                    Err(FramingError::CorruptPayloadCrc {
                        expected: expected_crc,
                        actual: actual_crc,
                    })
                } else {
                    Ok(Some(self.expected_len))
                }
            }
        }
    }

    /// Unwraps a framed packet from a complete slice in memory.
    ///
    /// Scans for `0xAA`, validates header CRC and payload CRC-16, and copies
    /// the payload into `payload_buf`. Returns the number of payload bytes.
    pub fn unwrap(frame: &[u8], payload_buf: &mut [u8]) -> Result<usize, FramingError> {
        let mut i = 0;
        while i < frame.len() {
            if frame[i] == CANONICAL_PREFIX {
                if frame.len() - i < OVERHEAD_SIZE {
                    return Err(FramingError::IncompleteFrame);
                }
                let len = frame[i + 1];
                if (len & 0x80) != 0 {
                    i += 1;
                    continue;
                }
                let expected_total = (len as usize) + OVERHEAD_SIZE;
                if frame.len() - i < expected_total {
                    return Err(FramingError::IncompleteFrame);
                }

                // Verify header CRC
                let header = &frame[i..i + HEADER_SIZE];
                if calc_crc8_buffer(CANONICAL_CRC8_INIT, header) != 0 {
                    i += 1;
                    continue;
                }

                let payload_start = i + HEADER_SIZE;
                let payload_end = payload_start + (len as usize);
                let payload = &frame[payload_start..payload_end];

                // Verify trailer CRC
                let trailer_high = frame[payload_end];
                let trailer_low = frame[payload_end + 1];
                let actual_crc = ((trailer_high as u16) << 8) | (trailer_low as u16);
                let expected_crc = calc_crc16_buffer(CANONICAL_CRC16_INIT, payload);

                if actual_crc != expected_crc {
                    return Err(FramingError::CorruptPayloadCrc {
                        expected: expected_crc,
                        actual: actual_crc,
                    });
                }

                if payload_buf.len() < payload.len() {
                    return Err(FramingError::BufferTooSmall);
                }
                payload_buf[..payload.len()].copy_from_slice(payload);
                return Ok(payload.len());
            }
            i += 1;
        }

        Err(FramingError::NoSync)
    }

    /// Reads and unwraps a single frame from an `embedded_io::Read` stream.
    #[cfg(feature = "embedded-io")]
    pub fn read_frame<R: embedded_io::Read>(
        reader: &mut R,
        payload_buf: &mut [u8],
    ) -> Result<usize, ReadFrameError<R::Error>> {
        let mut unwrapper = Self::new();
        let mut byte_buf = [0u8; 1];

        loop {
            match reader.read_exact(&mut byte_buf) {
                Ok(()) => {}
                Err(embedded_io::ReadExactError::UnexpectedEof) => {
                    return Err(ReadFrameError::UnexpectedEof);
                }
                Err(embedded_io::ReadExactError::Other(err)) => {
                    return Err(ReadFrameError::Io(err));
                }
            }
            match unwrapper.push_byte(byte_buf[0], payload_buf) {
                Ok(Some(len)) => return Ok(len),
                Ok(None) => {}
                Err(err) => {
                    // On corrupted frame/header, keep going until valid frame or EOF
                    if matches!(err, FramingError::BufferTooSmall) {
                        return Err(ReadFrameError::Framing(err));
                    }
                }
            }
        }
    }
}

/// Packet wrapper for framing payloads into Fibre packets.
pub struct PacketWrapper;

impl PacketWrapper {
    /// Wraps a payload into a Fibre frame in `out_buf`.
    ///
    /// Returns the total frame size (`payload.len() + 5`).
    pub fn wrap(payload: &[u8], out_buf: &mut [u8]) -> Result<usize, FramingError> {
        if payload.len() > MAX_PAYLOAD_SIZE {
            return Err(FramingError::PayloadTooLarge);
        }
        let total_size = payload.len() + OVERHEAD_SIZE;
        if out_buf.len() < total_size {
            return Err(FramingError::BufferTooSmall);
        }

        out_buf[0] = CANONICAL_PREFIX;
        out_buf[1] = payload.len() as u8;
        out_buf[2] = calc_crc8_buffer(CANONICAL_CRC8_INIT, &out_buf[0..2]);

        out_buf[3..3 + payload.len()].copy_from_slice(payload);

        let crc16 = calc_crc16_buffer(CANONICAL_CRC16_INIT, payload);
        let trailer_idx = 3 + payload.len();
        out_buf[trailer_idx] = ((crc16 >> 8) & 0xFF) as u8;
        out_buf[trailer_idx + 1] = (crc16 & 0xFF) as u8;

        Ok(total_size)
    }

    /// Writes a framed payload to an `embedded_io::Write` stream.
    #[cfg(feature = "embedded-io")]
    pub fn write_frame<W: embedded_io::Write>(
        writer: &mut W,
        payload: &[u8],
    ) -> Result<(), WriteFrameError<W::Error>> {
        let mut buf = [0u8; MAX_PAYLOAD_SIZE + OVERHEAD_SIZE];
        let len = Self::wrap(payload, &mut buf).map_err(WriteFrameError::Framing)?;
        writer.write_all(&buf[..len]).map_err(WriteFrameError::Io)?;
        Ok(())
    }
}

/// Errors when reading a frame from an `embedded_io::Read` stream.
#[cfg(feature = "embedded-io")]
#[derive(Debug)]
pub enum ReadFrameError<E> {
    Io(E),
    UnexpectedEof,
    Framing(FramingError),
}

/// Errors when writing a frame to an `embedded_io::Write` stream.
#[cfg(feature = "embedded-io")]
#[derive(Debug)]
pub enum WriteFrameError<E> {
    Io(E),
    Framing(FramingError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wrap_and_unwrap_roundtrip() {
        let payload = b"Hello, Fibre!";
        let mut frame_buf = [0u8; 64];
        let frame_len = PacketWrapper::wrap(payload, &mut frame_buf).expect("wrap failed");
        assert_eq!(frame_len, payload.len() + 5);

        assert_eq!(frame_buf[0], CANONICAL_PREFIX);
        assert_eq!(frame_buf[1], payload.len() as u8);

        let mut rx_buf = [0u8; 64];
        let rx_len = PacketUnwrapper::unwrap(&frame_buf[..frame_len], &mut rx_buf)
            .expect("unwrap failed");
        assert_eq!(rx_len, payload.len());
        assert_eq!(&rx_buf[..rx_len], payload);
    }

    #[test]
    fn test_streaming_unwrapper_with_junk_prefix() {
        let payload = b"motor.current_q";
        let mut frame_buf = [0u8; 64];
        let frame_len = PacketWrapper::wrap(payload, &mut frame_buf).unwrap();

        // Prepend junk bytes before the valid 0xAA frame
        let junk = [0x55, 0x12, 0xAA, 0xFF, 0x00];
        let mut stream = [0u8; 128];
        stream[..junk.len()].copy_from_slice(&junk);
        stream[junk.len()..junk.len() + frame_len].copy_from_slice(&frame_buf[..frame_len]);

        let mut unwrapper = PacketUnwrapper::new();
        let mut rx_buf = [0u8; 64];
        let mut result = None;

        for &b in &stream[..junk.len() + frame_len] {
            match unwrapper.push_byte(b, &mut rx_buf) {
                Ok(Some(len)) => {
                    result = Some(len);
                    break;
                }
                Ok(None) => {}
                Err(_) => {} // Expected during junk recovery
            }
        }

        assert_eq!(result, Some(payload.len()));
        assert_eq!(&rx_buf[..payload.len()], payload);
    }

    #[test]
    fn test_corrupt_header_crc_rejection() {
        let payload = b"vbus_voltage";
        let mut frame = [0u8; 32];
        let len = PacketWrapper::wrap(payload, &mut frame).unwrap();

        // Corrupt header CRC byte (index 2)
        frame[2] ^= 0xFF;

        let mut rx = [0u8; 32];
        let res = PacketUnwrapper::unwrap(&frame[..len], &mut rx);
        assert!(res.is_err());
    }

    #[test]
    fn test_corrupt_payload_crc_rejection() {
        let payload = b"shaft_angle";
        let mut frame = [0u8; 32];
        let len = PacketWrapper::wrap(payload, &mut frame).unwrap();

        // Corrupt trailer CRC byte
        frame[len - 1] ^= 0x01;

        let mut rx = [0u8; 32];
        let res = PacketUnwrapper::unwrap(&frame[..len], &mut rx);
        assert!(matches!(res, Err(FramingError::CorruptPayloadCrc { .. })));
    }
}
