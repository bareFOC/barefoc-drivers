//! # spi — TI 16-bit SPI framing engine with XNOR parity
//!
//! Texas Instruments DRV83xx series (such as DRV8316) use a 16-bit SPI
//! communication protocol with even parity on SDI frames.
//!
//! ## Frame Structure (SDI Input, 16 bits)
//! - Bit 15: R/W bit (`1` = Read, `0` = Write)
//! - Bits 14..9: 6-bit register address ($A_5..A_0$)
//! - Bit 8: Parity bit ($P$)
//! - Bits 7..0: 8-bit data ($D_7..D_0$), `0x00` on read commands
//!
//! ## Parity Bit Calculation (XNOR / Even Parity)
//! Bit 8 is chosen such that the total number of `1`s in the complete 16-bit
//! word is **even** (XNOR of all other 15 bits).
//!
//! ## Response Frame Structure (SDO Output, 16 bits)
//! - Bits 15..8: `IC_STATUS` summary byte (Bit 15: FAULT, 14: OT, 13: OVP,
//!   12: NPOR, 11: OCP, 10: SPI_FLT, 9: BK_FLT, 8: reserved)
//! - Bits 7..0: Data of the addressed register

/// Calculate the parity bit (bit 8) for a 16-bit frame whose bit 8 is 0.
///
/// Returns `true` (1) if the count of `1` bits in the 15 bits is odd,
/// thereby making the total number of 1s in the complete word even.
#[inline]
pub const fn calculate_parity(word_without_parity: u16) -> bool {
    (word_without_parity.count_ones() & 1) == 1
}

/// Encodes a 16-bit command frame for the DRV83xx.
///
/// * `addr`: 6-bit register address (0x00 to 0x3F).
/// * `data`: 8-bit payload byte (ignored/0x00 for read).
/// * `is_read`: `true` for a read transaction, `false` for a write.
#[inline]
pub fn encode_frame(addr: u8, data: u8, is_read: bool) -> u16 {
    let cmd = if is_read { 1u16 << 15 } else { 0u16 };
    let addr_bits = ((addr & 0x3F) as u16) << 9;
    let data_bits = if is_read { 0u16 } else { data as u16 };
    let mut frame = cmd | addr_bits | data_bits;
    if calculate_parity(frame) {
        frame |= 1 << 8;
    }
    frame
}

/// Encodes a 16-bit command frame as big-endian bytes `[MSB, LSB]`.
#[inline]
pub fn encode_frame_bytes(addr: u8, data: u8, is_read: bool) -> [u8; 2] {
    let frame = encode_frame(addr, data, is_read);
    [(frame >> 8) as u8, frame as u8]
}

/// Decodes a 16-bit response word into `(status_byte, data_byte)`.
///
/// The high byte contains the `IC_STATUS` summary, while the low byte contains
/// the register payload.
#[inline]
pub const fn decode_response(response: u16) -> (u8, u8) {
    let status = (response >> 8) as u8;
    let data = (response & 0xFF) as u8;
    (status, data)
}

/// Decodes big-endian received bytes `[MSB, LSB]` into `(status_byte, data_byte)`.
#[inline]
pub const fn decode_response_bytes(bytes: [u8; 2]) -> (u8, u8) {
    (bytes[0], bytes[1])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parity_calculation() {
        // Frame with 0 ones: even -> parity is 0
        assert!(!calculate_parity(0x0000));

        // Frame with 1 one: odd -> parity is 1
        assert!(calculate_parity(0x0001));

        // Frame with 2 ones: even -> parity is 0
        assert!(!calculate_parity(0x0003));

        // Frame with 3 ones: odd -> parity is 1
        assert!(calculate_parity(0x0007));
    }

    #[test]
    fn test_encode_read_frame() {
        // Read IC_STATUS (addr 0x00)
        // bit 15 = 1 (read), addr = 0x00, data = 0x00
        // total 1s before parity = 1 (bit 15) -> odd -> parity bit 8 must be 1.
        // Expected: (1 << 15) | (1 << 8) = 0x8100
        let frame = encode_frame(0x00, 0x00, true);
        assert_eq!(frame, 0x8100);
        assert_eq!(frame.count_ones() % 2, 0);

        // Read Control_1 (addr 0x03 = 0b000011)
        // bit 15 = 1, addr << 9 = 0x03 << 9 = 0x0600.
        // ones in (bit 15 + addr) = 1 + 2 = 3 (odd) -> parity bit 8 must be 1.
        // Expected: 0x8000 | 0x0600 | 0x0100 = 0x8700
        let frame = encode_frame(0x03, 0x00, true);
        assert_eq!(frame, 0x8700);
        assert_eq!(frame.count_ones() % 2, 0);

        // Read Status_1 (addr 0x01 = 0b000001)
        // bit 15 = 1, addr << 9 = 0x0200.
        // ones in (bit 15 + addr) = 1 + 1 = 2 (even) -> parity bit 8 must be 0.
        // Expected: 0x8000 | 0x0200 = 0x8200
        let frame = encode_frame(0x01, 0x00, true);
        assert_eq!(frame, 0x8200);
        assert_eq!(frame.count_ones() % 2, 0);
    }

    #[test]
    fn test_encode_write_frame() {
        // Write Control_1 (addr 0x03) with UNLOCK (0b011 = 0x03)
        // bit 15 = 0 (write), addr << 9 = 0x0600 (two 1s), data = 0x03 (two 1s).
        // Total 1s = 2 + 2 = 4 (even) -> parity bit 8 must be 0.
        // Expected: 0x0603
        let frame = encode_frame(0x03, 0x03, false);
        assert_eq!(frame, 0x0603);
        assert_eq!(frame.count_ones() % 2, 0);

        // Write Control_2 (addr 0x04) with value 0x01
        // bit 15 = 0, addr << 9 = 0x04 << 9 = 0x0800 (one 1), data = 0x01 (one 1).
        // Total 1s = 2 (even) -> parity bit 8 must be 0.
        let frame = encode_frame(0x04, 0x01, false);
        assert_eq!(frame, 0x0801);
        assert_eq!(frame.count_ones() % 2, 0);

        // Write Control_2 (addr 0x04) with value 0x03 (two 1s)
        // Total 1s = 1 (addr) + 2 (data) = 3 (odd) -> parity bit 8 must be 1.
        // Expected: 0x0800 | 0x0100 | 0x0003 = 0x0903
        let frame = encode_frame(0x04, 0x03, false);
        assert_eq!(frame, 0x0903);
        assert_eq!(frame.count_ones() % 2, 0);
    }

    #[test]
    fn test_decode_response() {
        let resp = 0x4255;
        let (status, data) = decode_response(resp);
        assert_eq!(status, 0x42);
        assert_eq!(data, 0x55);

        let bytes = encode_frame_bytes(0x04, 0x03, false);
        assert_eq!(bytes, [0x09, 0x03]);
        let (st, dt) = decode_response_bytes(bytes);
        assert_eq!(st, 0x09);
        assert_eq!(dt, 0x03);
    }
}
