//! Canonical Fibre CRC implementations.
//!
//! References upstream:
//! - `ref/fibre-cpp/include/fibre/crc.hpp`
//! - `ref/fibre-cpp/legacy_protocol.hpp`

/// Default CRC-8 polynomial: x^8 + x^5 + x^4 + x^2 + x + 1 (0x37).
pub const CANONICAL_CRC8_POLYNOMIAL: u8 = 0x37;

/// Canonical CRC-8 initial value (0x42).
pub const CANONICAL_CRC8_INIT: u8 = 0x42;

/// Default CRC-16 polynomial: CRC-16-DNP (0x3d65).
pub const CANONICAL_CRC16_POLYNOMIAL: u16 = 0x3d65;

/// Canonical CRC-16 initial value (0x1337).
pub const CANONICAL_CRC16_INIT: u16 = 0x1337;

/// Canonical sync byte prefix for stream framing (0xAA).
pub const CANONICAL_PREFIX: u8 = 0xAA;

/// Fibre legacy protocol version (1).
pub const PROTOCOL_VERSION: u16 = 1;

/// Calculates CRC-8 for a single byte.
///
/// Adapted from Fibre C++ `calc_crc8`.
#[inline]
pub fn calc_crc8(mut remainder: u8, byte: u8) -> u8 {
    remainder ^= byte;
    for _ in 0..8 {
        if (remainder & 0x80) != 0 {
            remainder = (remainder << 1) ^ CANONICAL_CRC8_POLYNOMIAL;
        } else {
            remainder <<= 1;
        }
    }
    remainder
}

/// Calculates CRC-8 over a buffer.
#[inline]
pub fn calc_crc8_buffer(mut remainder: u8, buffer: &[u8]) -> u8 {
    for &b in buffer {
        remainder = calc_crc8(remainder, b);
    }
    remainder
}

/// Calculates CRC-16 for a single byte.
///
/// Adapted from Fibre C++ `calc_crc16`.
#[inline]
pub fn calc_crc16(mut remainder: u16, byte: u8) -> u16 {
    remainder ^= (byte as u16) << 8;
    for _ in 0..8 {
        if (remainder & 0x8000) != 0 {
            remainder = (remainder << 1) ^ CANONICAL_CRC16_POLYNOMIAL;
        } else {
            remainder <<= 1;
        }
    }
    remainder
}

/// Calculates CRC-16 over a buffer.
#[inline]
pub fn calc_crc16_buffer(mut remainder: u16, buffer: &[u8]) -> u16 {
    for &b in buffer {
        remainder = calc_crc16(remainder, b);
    }
    remainder
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc8_canonical_header() {
        // [0xAA, 0x04]
        let crc = calc_crc8_buffer(CANONICAL_CRC8_INIT, &[CANONICAL_PREFIX, 0x04]);
        assert_eq!(crc, 0x6E);

        // Feeding CRC into remainder yields 0 (canonical CRC property)
        let verify = calc_crc8_buffer(CANONICAL_CRC8_INIT, &[CANONICAL_PREFIX, 0x04, crc]);
        assert_eq!(verify, 0);
    }

    #[test]
    fn test_crc16_canonical_vector() {
        let payload = b"123456789";
        let crc = calc_crc16_buffer(CANONICAL_CRC16_INIT, payload);
        assert_eq!(crc, 0xAA01);

        // Big-endian trailer bytes fed into remainder yield 0
        let trailer = [(crc >> 8) as u8, (crc & 0xFF) as u8];
        let rem = calc_crc16_buffer(crc, &trailer);
        assert_eq!(rem, 0);

        let combined = calc_crc16_buffer(CANONICAL_CRC16_INIT, payload);
        let final_rem = calc_crc16_buffer(combined, &trailer);
        assert_eq!(final_rem, 0);
    }
}
