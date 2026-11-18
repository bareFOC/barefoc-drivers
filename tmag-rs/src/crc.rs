//! # crc — CRC-8 Implementation for TMAG5273 I2C Data Integrity
//!
//! Implements CRC-8 calculation per TI TMAG5273 Datasheet Section 6.5.1.3.6
//! using polynomial $x^8 + x^2 + x + 1$ (`0x07`) with initial value `0xFF`.

/// Calculate CRC-8 over a slice of bytes using polynomial `0x07` and initial value `0xFF`.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if (crc & 0x80) != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test vectors directly taken from TI TMAG5273 Datasheet Section 6.5.1.3.6:
    /// - 00h -> F3h
    /// - FFh -> 00h
    /// - 80h -> 7Ah
    /// - 4Ch -> 10h
    /// - E0h -> 5Dh
    /// - 00000000h -> D1h
    /// - FFFFFFFFh -> 0Fh
    #[test]
    fn test_datasheet_crc_vectors() {
        assert_eq!(crc8(&[0x00]), 0xF3);
        assert_eq!(crc8(&[0xFF]), 0x00);
        assert_eq!(crc8(&[0x80]), 0x7A);
        assert_eq!(crc8(&[0x4C]), 0x10);
        assert_eq!(crc8(&[0xE0]), 0x5D);
        assert_eq!(crc8(&[0x00, 0x00, 0x00, 0x00]), 0xD1);
        assert_eq!(crc8(&[0xFF, 0xFF, 0xFF, 0xFF]), 0x0F);
    }
}
