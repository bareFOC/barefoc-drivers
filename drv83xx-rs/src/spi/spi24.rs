//! # spi24 — TI 24-bit SPI and 32-bit tSPI framing engine with Dual Even Parity
//!
//! Texas Instruments DRV8311 gate drivers use a 24-bit SPI communication protocol
//! (or 32-bit `tSPI` with secondary device addressing) with dual even parity:
//! - Bit 16: Header Even Parity (over bits 23..17 in 24-bit SPI, or bits 31..17 in 32-bit tSPI)
//! - Bit 15: Data Even Parity (over data payload bits 14..0)
//!
//! ## 24-Bit Frame Structure (SDI Input, Standard SPI)
//! - Bit 23: R/W bit (`1` = Read, `0` = Write)
//! - Bits 22..17: 6-bit register address ($A_5..A_0$)
//! - Bit 16: Header Parity bit ($P_{hdr}$)
//! - Bit 15: Data Parity bit ($P_{data}$)
//! - Bits 14..0: 15-bit data payload ($D_{14}..D_0$)
//!
//! ## 32-Bit Frame Structure (SDI Input, tSPI)
//! - Bit 31: R/W bit (`1` = Read, `0` = Write)
//! - Bits 30..27: 4-bit Secondary Device ID (0..3, or 15 for broadcast)
//! - Bits 26..19: 8-bit register address ($A_7..A_0$)
//! - Bits 18..17: Reserved (`00`)
//! - Bit 16: Header Parity bit ($P_{hdr}$)
//! - Bit 15: Data Parity bit ($P_{data}$)
//! - Bits 14..0: 15-bit data payload ($D_{14}..D_0$)
//!
//! ## Response Frame Structure (SDO Output, 24 bits)
//! - Bits 23..16: 8-bit Device Status summary (`DEV_STS1` lower byte: S7..S0)
//! - Bits 15..0: 16-bit register data (Bit 15: Data Parity if enabled, Bits 14..0: Register content)

/// Broadcast / General Call device ID for tSPI bus.
pub const TSPI_BROADCAST_DEVICE_ID: u8 = 15;

/// Calculates even parity for an 8-bit value.
///
/// Returns `true` (1) if the number of 1s in `val` is odd,
/// thereby making the total number of 1s (including the parity bit) even.
#[inline]
pub const fn calculate_even_parity_u8(val: u8) -> bool {
    (val.count_ones() & 1) == 1
}

/// Calculates even parity for a 16-bit value.
///
/// Returns `true` (1) if the number of 1s in `val` is odd,
/// thereby making the total number of 1s (including the parity bit) even.
#[inline]
pub const fn calculate_even_parity_u16(val: u16) -> bool {
    (val.count_ones() & 1) == 1
}

/// Verifies whether a 16-bit word has valid even parity (even number of 1s).
#[inline]
pub const fn verify_even_parity_u16(val: u16) -> bool {
    (val.count_ones() & 1) == 0
}

/// Encodes a 24-bit command frame for DRV8311 standard SPI.
///
/// * `addr`: 6-bit register address (0x00 to 0x3F).
/// * `data`: 15-bit payload word (ignored / 0x0000 on read).
/// * `is_read`: `true` for read, `false` for write.
/// * `with_parity`: `true` to calculate and insert dual even parity bits.
///
/// Returns big-endian bytes `[header_byte, data_msb, data_lsb]`.
pub fn encode_spi24_frame(addr: u8, data: u16, is_read: bool, with_parity: bool) -> [u8; 3] {
    let header_7 = ((is_read as u8) << 6) | (addr & 0x3F);
    let p_hdr = if with_parity && calculate_even_parity_u8(header_7) {
        1u8
    } else {
        0u8
    };
    let byte_0 = (header_7 << 1) | p_hdr;

    let data_15 = if is_read { 0u16 } else { data & 0x7FFF };
    let p_data = if with_parity && calculate_even_parity_u16(data_15) {
        1u16 << 15
    } else {
        0u16
    };
    let data_word = p_data | data_15;

    [byte_0, (data_word >> 8) as u8, data_word as u8]
}

/// Decodes a 24-bit response `[status_byte, data_msb, data_lsb]`.
///
/// Returns `(status_byte, data_word)`. Note that `data_word` bit 15 is the parity bit.
#[inline]
pub const fn decode_spi24_response(bytes: [u8; 3]) -> (u8, u16) {
    let status = bytes[0];
    let data_word = ((bytes[1] as u16) << 8) | (bytes[2] as u16);
    (status, data_word)
}

/// Encodes a 32-bit command frame for DRV8311 tSPI multi-device communication.
///
/// * `device_id`: 4-bit secondary device ID (0..3, or 15 for broadcast).
/// * `addr`: 8-bit register address (0x00 to 0x3F).
/// * `data`: 15-bit payload word (ignored / 0x0000 on read).
/// * `is_read`: `true` for read, `false` for write.
/// * `with_parity`: `true` to calculate and insert dual even parity bits.
///
/// Returns big-endian bytes `[header_msb, header_lsb, data_msb, data_lsb]`.
pub fn encode_tspi_frame(
    device_id: u8,
    addr: u8,
    data: u16,
    is_read: bool,
    with_parity: bool,
) -> [u8; 4] {
    // 15 header bits: bit 14 is R/W, bits 13..10 is device_id, bits 9..2 is addr, bits 1..0 is 00
    let header_15 = ((is_read as u16) << 14)
        | (((device_id & 0x0F) as u16) << 10)
        | (((addr as u16) & 0xFF) << 2);

    let p_hdr = if with_parity && calculate_even_parity_u16(header_15) {
        1u16
    } else {
        0u16
    };
    let header_word = (header_15 << 1) | p_hdr;

    let data_15 = if is_read { 0u16 } else { data & 0x7FFF };
    let p_data = if with_parity && calculate_even_parity_u16(data_15) {
        1u16 << 15
    } else {
        0u16
    };
    let data_word = p_data | data_15;

    [
        (header_word >> 8) as u8,
        header_word as u8,
        (data_word >> 8) as u8,
        data_word as u8,
    ]
}

/// Decodes a 32-bit tSPI response `[status_byte, pad_byte, data_msb, data_lsb]`.
///
/// Returns `(status_byte, data_word)`.
#[inline]
pub const fn decode_tspi_response(bytes: [u8; 4]) -> (u8, u16) {
    let status = bytes[0];
    let data_word = ((bytes[2] as u16) << 8) | (bytes[3] as u16);
    (status, data_word)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spi24_parity_calculation() {
        assert!(!calculate_even_parity_u8(0x00));
        assert!(calculate_even_parity_u8(0x01));
        assert!(!calculate_even_parity_u8(0x03));
        assert!(calculate_even_parity_u8(0x07));

        assert!(!calculate_even_parity_u16(0x0000));
        assert!(calculate_even_parity_u16(0x0001));
        assert!(!calculate_even_parity_u16(0x0003));
        assert!(calculate_even_parity_u16(0x8000));
    }

    #[test]
    fn test_spi24_read_frame_parity() {
        // Read DEV_STS1 (addr 0x00) with parity
        // is_read = true (1), addr = 0x00 (0b000000). Total 1s in header_7 = 1 (odd) -> p_hdr = 1.
        // Byte 0 = (1 << 7) | 1 = 0x81.
        // Data = 0x0000. Total 1s in data_15 = 0 (even) -> p_data = 0.
        // Data word = 0x0000.
        let frame = encode_spi24_frame(0x00, 0x0000, true, true);
        assert_eq!(frame, [0x81, 0x00, 0x00]);
        // Verify even parity for header and data:
        assert_eq!(frame[0].count_ones() % 2, 0);
        let data_word = ((frame[1] as u16) << 8) | (frame[2] as u16);
        assert_eq!(data_word.count_ones() % 2, 0);

        // Read CSA_CTRL (addr 0x23 = 0b100011) with parity
        // is_read = 1, addr = 0x23 (3 ones). Total 1s in header_7 = 1 + 3 = 4 (even) -> p_hdr = 0.
        // Byte 0 = (1 << 7) | (0x23 << 1) = 0x80 | 0x46 = 0xC6.
        let frame_csa = encode_spi24_frame(0x23, 0x0000, true, true);
        assert_eq!(frame_csa[0], 0xC6);
        assert_eq!(frame_csa[0].count_ones() % 2, 0);
    }

    #[test]
    fn test_spi24_write_frame_parity() {
        // Write CSA_CTRL (addr 0x23 = 0b100011) with data 0x0008 (bit 3 set)
        // is_read = 0, addr = 0x23 (3 ones). Total 1s in header_7 = 3 (odd) -> p_hdr = 1.
        // Byte 0 = (0x23 << 1) | 1 = 0x47.
        // Data = 0x0008 (1 one). Total 1s in data_15 = 1 (odd) -> p_data = 1 (bit 15).
        // Data word = 0x8008.
        let frame = encode_spi24_frame(0x23, 0x0008, false, true);
        assert_eq!(frame, [0x47, 0x80, 0x08]);
        assert_eq!(frame[0].count_ones() % 2, 0);
        let data_word = ((frame[1] as u16) << 8) | (frame[2] as u16);
        assert_eq!(data_word.count_ones() % 2, 0);
        assert!(verify_even_parity_u16(data_word));
    }

    #[test]
    fn test_spi24_decode_response() {
        // 0x0043 has 3 ones -> bit 15 is 1 -> 0x8043 (4 ones, even)
        let rx = [0x55, 0x80, 0x43];
        let (status, data) = decode_spi24_response(rx);
        assert_eq!(status, 0x55);
        assert_eq!(data, 0x8043);
        assert!(verify_even_parity_u16(data));
        assert_eq!(data & 0x7FFF, 0x0043);
    }

    #[test]
    fn test_tspi_encode_decode() {
        // Device ID 1, addr 0x10, write data 0x0115
        let frame = encode_tspi_frame(1, 0x10, 0x0115, false, true);
        // Header word:
        // is_read = 0
        // device_id = 1 (0b0001)
        // addr = 0x10 (0b00010000)
        // header_15 = (1 << 10) | (0x10 << 2) = 0x0400 | 0x0040 = 0x0440 (two 1s, even)
        // p_hdr = 0
        // header_word = 0x0440 << 1 = 0x0880
        // header bytes: [0x08, 0x80]
        assert_eq!(frame[0], 0x08);
        assert_eq!(frame[1], 0x80);
        let hdr = ((frame[0] as u16) << 8) | (frame[1] as u16);
        assert_eq!(hdr.count_ones() % 2, 0);

        // Data: 0x0115 = 0b0000_0001_0001_0101 (4 ones, even) -> p_data = 0 -> 0x0115
        assert_eq!(frame[2], 0x01);
        assert_eq!(frame[3], 0x15);
        let data_w = ((frame[2] as u16) << 8) | (frame[3] as u16);
        assert_eq!(data_w.count_ones() % 2, 0);

        // Broadcast device ID 15 (0xF)
        let bcast_frame = encode_tspi_frame(TSPI_BROADCAST_DEVICE_ID, 0x17, 0x0001, false, true);
        let bcast_hdr = ((bcast_frame[0] as u16) << 8) | (bcast_frame[1] as u16);
        assert_eq!(bcast_hdr.count_ones() % 2, 0);

        // Decode tSPI response
        let rx = [0x80, 0x00, 0x80, 0x08];
        let (status, data) = decode_tspi_response(rx);
        assert_eq!(status, 0x80);
        assert_eq!(data, 0x8008);
        assert!(verify_even_parity_u16(data));
    }
}
