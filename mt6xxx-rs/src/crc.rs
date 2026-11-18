//! # crc — Hardware-verified CRC-6 engine for MT6701 SSI frames
//!
//! Upstream reference:
//! - [`MagneticSensorMT6701SSI.cpp`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.cpp)
//!
//! Implements CRC-6 calculation using generator polynomial $X^6 + X + 1$ (0x03 residual)
//! computed MSB-first across the 18 data bits (`D[13:0]` followed by `Mg[3:0]`).

/// Precomputed 64-entry lookup table for CRC-6 with polynomial $X^6 + X + 1$.
pub const TABLE_CRC6: [u8; 64] = [
    0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09, 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12,
    0x11, 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39, 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27,
    0x22, 0x21, 0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A, 0x3B, 0x38, 0x3D, 0x3E, 0x37,
    0x34, 0x31, 0x32, 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A, 0x0B, 0x08, 0x0D, 0x0E,
    0x07, 0x04, 0x01, 0x02,
];

/// Computes the 6-bit CRC over an 18-bit data payload using bitwise division.
///
/// Polynomial: $X^6 + X + 1$ (0x43 in 7-bit representation, 0x03 residual after implicit MSB).
/// Processed MSB-first across bits 17 down to 0.
#[inline]
pub fn compute_crc6_bitwise(data18: u32, init: u8) -> u8 {
    const POLY: u8 = 0x03;
    let mut crc = init & 0x3F;
    for i in (0..18).rev() {
        let in_bit = ((data18 >> i) & 1) as u8;
        let msb = (crc >> 5) & 1;
        crc = (crc << 1) & 0x3F;
        if (in_bit ^ msb) != 0 {
            crc ^= POLY;
        }
    }
    crc
}

/// Computes the 6-bit CRC using table lookup when `init == 0`, or fallback to bitwise for arbitrary `init`.
#[inline]
pub fn compute_crc6(data18: u32, init: u8) -> u8 {
    if init == 0 {
        let mut b_index = ((data18 >> 12) & 0x3F) as usize;
        let mut b_crc = ((data18 >> 6) & 0x3F) as u8;
        b_index = (b_crc ^ TABLE_CRC6[b_index]) as usize;
        b_crc = (data18 & 0x3F) as u8;
        b_index = (b_crc ^ TABLE_CRC6[b_index]) as usize;
        TABLE_CRC6[b_index]
    } else {
        compute_crc6_bitwise(data18, init)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc6_reference_vectors() {
        // Reference vectors from MagneticSensorMT6701SSI.cpp and spec.md:
        // 0x00000 -> 0x00
        // 0x3FFFF -> 0x0E
        // 0x2AAAA -> 0x35
        // 0x15555 -> 0x3B
        // 0x12345 -> 0x1D
        // 0x3FFC0 -> 0x0C
        assert_eq!(compute_crc6(0x00000, 0), 0x00);
        assert_eq!(compute_crc6(0x3FFFF, 0), 0x0E);
        assert_eq!(compute_crc6(0x2AAAA, 0), 0x35);
        assert_eq!(compute_crc6(0x15555, 0), 0x3B);
        assert_eq!(compute_crc6(0x12345, 0), 0x1D);
        assert_eq!(compute_crc6(0x3FFC0, 0), 0x0C);

        assert_eq!(compute_crc6_bitwise(0x00000, 0), 0x00);
        assert_eq!(compute_crc6_bitwise(0x3FFFF, 0), 0x0E);
        assert_eq!(compute_crc6_bitwise(0x2AAAA, 0), 0x35);
        assert_eq!(compute_crc6_bitwise(0x15555, 0), 0x3B);
        assert_eq!(compute_crc6_bitwise(0x12345, 0), 0x1D);
        assert_eq!(compute_crc6_bitwise(0x3FFC0, 0), 0x0C);
    }

    #[test]
    fn test_lut_matches_bitwise_exhaustive() {
        // Exhaustive check over all 2^18 combinations
        for data in 0..(1 << 18) {
            assert_eq!(compute_crc6(data, 0), compute_crc6_bitwise(data, 0));
        }
    }
}
