//! Endpoint 0 JSON interface descriptor manifest and CRC calculator.
//!
//! References upstream:
//! - `ref/fibre-cpp/legacy_protocol.cpp` (`endpoint0_handler`)
//! - `ref/fibre-cpp/endpoints_template.j2` (`json_crc_`, `json_version_id_`)

use crate::crc::{calc_crc16_buffer, PROTOCOL_VERSION};

/// Errors that can occur during Endpoint 0 processing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Endpoint0Error {
    /// Request buffer did not contain the required 4-byte offset.
    InputTooShort,
    /// Output buffer is too small to return requested data.
    OutputBufferTooSmall,
}

/// Handler for Fibre Endpoint 0 operations.
///
/// Serves chunks of the embedded JSON interface descriptor from flash and
/// computes canonical `json_crc` and `json_version_id`.
#[derive(Debug, Clone, Copy)]
pub struct Endpoint0Handler {
    json: &'static str,
    json_crc: u16,
    json_version_id: u32,
}

impl Endpoint0Handler {
    /// Creates a new `Endpoint0Handler` wrapping a static JSON descriptor.
    ///
    /// Computes `json_crc` with initial value `PROTOCOL_VERSION` (1)
    /// and `json_version_id` per canonical Fibre specification.
    pub fn new(json: &'static str) -> Self {
        let json_bytes = json.as_bytes();
        let json_crc = calc_crc16_buffer(PROTOCOL_VERSION, json_bytes);
        let version_crc2 = calc_crc16_buffer(json_crc, json_bytes);
        let json_version_id = ((json_crc as u32) << 16) | (version_crc2 as u32);

        Self {
            json,
            json_crc,
            json_version_id,
        }
    }

    /// Canonical CRC-16 of the JSON descriptor.
    ///
    /// Required trailer verification for all non-zero endpoint operations.
    #[inline]
    pub fn json_crc(&self) -> u16 {
        self.json_crc
    }

    /// JSON version identifier (32-bit composite CRC).
    #[inline]
    pub fn json_version_id(&self) -> u32 {
        self.json_version_id
    }

    /// Length of the embedded JSON descriptor in bytes.
    #[inline]
    pub fn json_len(&self) -> usize {
        self.json.len()
    }

    /// Raw JSON descriptor string slice.
    #[inline]
    pub fn json_str(&self) -> &'static str {
        self.json
    }

    /// Handles an incoming query to Endpoint 0.
    ///
    /// - If `offset == 0xFFFFFFFF`: returns `json_version_id` as little-endian `u32` (4 bytes).
    /// - If `offset >= json_len`: returns 0 bytes (EOF).
    /// - Else: copies a chunk of the JSON descriptor starting at `offset` into `output`.
    ///
    /// Returns the number of bytes written to `output`.
    pub fn handle_request(
        &self,
        input: &[u8],
        output: &mut [u8],
    ) -> Result<usize, Endpoint0Error> {
        if input.len() < 4 {
            return Err(Endpoint0Error::InputTooShort);
        }

        let offset = u32::from_le_bytes([input[0], input[1], input[2], input[3]]);

        if offset == 0xFFFF_FFFF {
            if output.len() < 4 {
                return Err(Endpoint0Error::OutputBufferTooSmall);
            }
            output[..4].copy_from_slice(&self.json_version_id.to_le_bytes());
            Ok(4)
        } else if (offset as usize) >= self.json.len() {
            // End of JSON stream
            Ok(0)
        } else {
            let start = offset as usize;
            let available = self.json.len() - start;
            let count = available.min(output.len());
            output[..count].copy_from_slice(&self.json.as_bytes()[start..start + count]);
            Ok(count)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_JSON: &str = r#"[{"name":"vbus_voltage","id":1,"type":"float","access":"r"}]"#;

    #[test]
    fn test_endpoint0_query_chunks() {
        let handler = Endpoint0Handler::new(SAMPLE_JSON);
        assert_ne!(handler.json_crc(), 0);
        assert_ne!(handler.json_version_id(), 0);

        // Query offset 0 with a 32-byte chunk
        let mut out = [0u8; 32];
        let offset0 = 0u32.to_le_bytes();
        let written = handler.handle_request(&offset0, &mut out).unwrap();
        assert_eq!(written, 32);
        assert_eq!(&out[..32], &SAMPLE_JSON.as_bytes()[..32]);

        // Query remaining bytes
        let mut out2 = [0u8; 64];
        let offset32 = 32u32.to_le_bytes();
        let written2 = handler.handle_request(&offset32, &mut out2).unwrap();
        let expected_remaining = SAMPLE_JSON.len() - 32;
        assert_eq!(written2, expected_remaining);
        assert_eq!(&out2[..written2], &SAMPLE_JSON.as_bytes()[32..]);

        // Query past end
        let offset_end = (SAMPLE_JSON.len() as u32).to_le_bytes();
        let written_end = handler.handle_request(&offset_end, &mut out2).unwrap();
        assert_eq!(written_end, 0);
    }

    #[test]
    fn test_endpoint0_version_id_query() {
        let handler = Endpoint0Handler::new(SAMPLE_JSON);
        let special_offset = 0xFFFF_FFFFu32.to_le_bytes();
        let mut out = [0u8; 4];
        let len = handler.handle_request(&special_offset, &mut out).unwrap();
        assert_eq!(len, 4);
        assert_eq!(u32::from_le_bytes(out), handler.json_version_id());
    }
}
