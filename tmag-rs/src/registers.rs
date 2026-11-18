//! # registers — TMAG5273 Register Map and Bitfield Definitions
//!
//! Covers all 29 hardware registers (`0x00` through `0x1C`) of the Texas Instruments
//! TMAG5273 3D linear Hall-effect sensor per TI Datasheet SLYS045C / SBASAI4 Rev C.

/// Bit-field extraction helper.
#[inline]
pub const fn extract(register: u8, mask: u8, shift: u8) -> u8 {
    (register & mask) >> shift
}

/// Bit-field insertion helper, preserving unmasked bits.
#[inline]
pub const fn insert(register: u8, mask: u8, shift: u8, value: u8) -> u8 {
    (register & !mask) | ((value << shift) & mask)
}

/// Bit 7 of register address — ORed into address on an I2C read to trigger a new
/// conversion simultaneously (Standby / Trigger mode). Datasheet §6.5.1.1 / §7.5.1.1.
pub const CONVERSION_START_BIT: u8 = 0x80;

// ---------------------------------------------------------------------------
// Register Addresses (0x00 ..= 0x1C)
// ---------------------------------------------------------------------------

/// Device Configuration 1 (0x00).
pub const DEVICE_CONFIG_1: u8 = 0x00;
/// Device Configuration 2 (0x01).
pub const DEVICE_CONFIG_2: u8 = 0x01;
/// Sensor Configuration 1 (0x02).
pub const SENSOR_CONFIG_1: u8 = 0x02;
/// Sensor Configuration 2 (0x03).
pub const SENSOR_CONFIG_2: u8 = 0x03;
/// X-axis threshold configuration (0x04).
pub const X_THR_CONFIG: u8 = 0x04;
/// Y-axis threshold configuration (0x05).
pub const Y_THR_CONFIG: u8 = 0x05;
/// Z-axis threshold configuration (0x06).
pub const Z_THR_CONFIG: u8 = 0x06;
/// Temperature sensor configuration (0x07).
pub const T_CONFIG: u8 = 0x07;
/// Interrupt configuration 1 (0x08).
pub const INT_CONFIG_1: u8 = 0x08;
/// Magnetic gain configuration (0x09).
pub const MAG_GAIN_CONFIG: u8 = 0x09;
/// Magnetic offset configuration 1 (0x0A).
pub const MAG_OFFSET_CONFIG_1: u8 = 0x0A;
/// Magnetic offset configuration 2 (0x0B).
pub const MAG_OFFSET_CONFIG_2: u8 = 0x0B;
/// I2C Address configuration (0x0C).
pub const I2C_ADDRESS: u8 = 0x0C;
/// Device identification register (0x0D).
pub const DEVICE_ID: u8 = 0x0D;
/// Manufacturer identification register LSB (0x0E) — expected `0x49` ('I').
pub const MANUFACTURER_ID_LSB: u8 = 0x0E;
/// Manufacturer identification register MSB (0x0F) — expected `0x54` ('T').
pub const MANUFACTURER_ID_MSB: u8 = 0x0F;
/// Temperature result MSB (0x10).
pub const T_MSB_RESULT: u8 = 0x10;
/// Temperature result LSB (0x11).
pub const T_LSB_RESULT: u8 = 0x11;
/// X-axis magnetic field result MSB (0x12).
pub const X_MSB_RESULT: u8 = 0x12;
/// X-axis magnetic field result LSB (0x13).
pub const X_LSB_RESULT: u8 = 0x13;
/// Y-axis magnetic field result MSB (0x14).
pub const Y_MSB_RESULT: u8 = 0x14;
/// Y-axis magnetic field result LSB (0x15).
pub const Y_LSB_RESULT: u8 = 0x15;
/// Z-axis magnetic field result MSB (0x16).
pub const Z_MSB_RESULT: u8 = 0x16;
/// Z-axis magnetic field result LSB (0x17).
pub const Z_LSB_RESULT: u8 = 0x17;
/// Conversion status register (0x18).
pub const CONV_STATUS: u8 = 0x18;
/// CORDIC angle result MSB (0x19).
pub const ANGLE_RESULT_MSB: u8 = 0x19;
/// CORDIC angle result LSB (0x1A).
pub const ANGLE_RESULT_LSB: u8 = 0x1A;
/// CORDIC magnitude result (0x1B).
pub const MAGNITUDE_RESULT: u8 = 0x1B;
/// Device status register (0x1C).
pub const DEVICE_STATUS: u8 = 0x1C;

// ---------------------------------------------------------------------------
// 0x00 — DEVICE_CONFIG_1 Bitfields
// ---------------------------------------------------------------------------
/// CRC enable bit (bit 7).
pub const CRC_EN_MASK: u8 = 0b1000_0000;
pub const CRC_EN_SHIFT: u8 = 7;

/// Magnetic temperature compensation coefficient (bits [6:5]).
pub const MAG_TEMPCO_MASK: u8 = 0b0110_0000;
pub const MAG_TEMPCO_SHIFT: u8 = 5;

/// Conversion averaging (bits [4:2]).
pub const CONV_AVG_MASK: u8 = 0b0001_1100;
pub const CONV_AVG_SHIFT: u8 = 2;

/// I2C read mode (bits [1:0]).
pub const I2C_RD_MASK: u8 = 0b0000_0011;
pub const I2C_RD_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x01 — DEVICE_CONFIG_2 Bitfields
// ---------------------------------------------------------------------------
/// Threshold hysteresis (bit 7): 0 = 1 LSB, 1 = 7 LSB.
pub const THR_HYST_MASK: u8 = 0b1000_0000;
pub const THR_HYST_SHIFT: u8 = 7;

/// Low-power vs Low-noise mode (bit 4): 0 = Low Power, 1 = Low Noise.
pub const LP_LN_MASK: u8 = 0b0001_0000;
pub const LP_LN_SHIFT: u8 = 4;

/// I2C glitch filter enable (bit 3).
pub const I2C_GLITCH_FILTER_MASK: u8 = 0b0000_1000;
pub const I2C_GLITCH_FILTER_SHIFT: u8 = 3;

/// Conversion trigger mode (bit 2): 0 = I2C command, 1 = INT pin.
pub const TRIGGER_MODE_MASK: u8 = 0b0000_0100;
pub const TRIGGER_MODE_SHIFT: u8 = 2;

/// Operating mode (bits [1:0]).
pub const OPERATING_MODE_MASK: u8 = 0b0000_0011;
pub const OPERATING_MODE_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x02 — SENSOR_CONFIG_1 Bitfields
// ---------------------------------------------------------------------------
/// Magnetic channel enable (bits [7:4]).
pub const MAG_CH_EN_MASK: u8 = 0b1111_0000;
pub const MAG_CH_EN_SHIFT: u8 = 4;

/// Sleep duration for Wake-and-Sleep mode (bits [3:0]).
pub const SLEEPTIME_MASK: u8 = 0b0000_1111;
pub const SLEEPTIME_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x03 — SENSOR_CONFIG_2 Bitfields
// ---------------------------------------------------------------------------
/// Threshold crossing count (bit 7): 0 = 1 count, 1 = 4 counts.
pub const THRX_COUNT_MASK: u8 = 0b1000_0000;
pub const THRX_COUNT_SHIFT: u8 = 7;

/// Threshold direction (bit 6).
pub const MAG_THR_DIR_MASK: u8 = 0b0100_0000;
pub const MAG_THR_DIR_SHIFT: u8 = 6;

/// Magnetic gain channel selection (bits [5:4]).
pub const MAG_GAIN_CH_MASK: u8 = 0b0011_0000;
pub const MAG_GAIN_CH_SHIFT: u8 = 4;

/// CORDIC angle calculation enable (bits [3:2]): 00 = None, 01 = XY, 10 = YZ, 11 = XZ.
pub const ANGLE_EN_MASK: u8 = 0b0000_1100;
pub const ANGLE_EN_SHIFT: u8 = 2;

/// X and Y magnetic range (bit 1): 0 = Low, 1 = High.
pub const X_Y_RANGE_MASK: u8 = 0b0000_0010;
pub const X_Y_RANGE_SHIFT: u8 = 1;

/// Z magnetic range (bit 0): 0 = Low, 1 = High.
pub const Z_RANGE_MASK: u8 = 0b0000_0001;
pub const Z_RANGE_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x07 — T_CONFIG Bitfields
// ---------------------------------------------------------------------------
/// Temperature threshold config (bits [7:1]).
pub const T_THR_CONFIG_MASK: u8 = 0b1111_1110;
pub const T_THR_CONFIG_SHIFT: u8 = 1;

/// Temperature channel enable (bit 0).
pub const T_CH_EN_MASK: u8 = 0b0000_0001;
pub const T_CH_EN_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x08 — INT_CONFIG_1 Bitfields
// ---------------------------------------------------------------------------
/// Conversion result interrupt assert (bit 7).
pub const RSLT_INT_MASK: u8 = 0b1000_0000;
pub const RSLT_INT_SHIFT: u8 = 7;

/// Threshold interrupt assert (bit 6).
pub const THRSLD_INT_MASK: u8 = 0b0100_0000;
pub const THRSLD_INT_SHIFT: u8 = 6;

/// Interrupt pin output behavior (bit 5): 0 = Latch, 1 = Pulse.
pub const INT_STATE_MASK: u8 = 0b0010_0000;
pub const INT_STATE_SHIFT: u8 = 5;

/// Interrupt mode configuration (bits [4:2]).
pub const INT_MODE_MASK: u8 = 0b0001_1100;
pub const INT_MODE_SHIFT: u8 = 2;

/// Mask INTB pin output (bit 0): 0 = Normal, 1 = Masked.
pub const MASK_INTB_MASK: u8 = 0b0000_0001;
pub const MASK_INTB_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x0D — DEVICE_ID Bitfields
// ---------------------------------------------------------------------------
/// Device silicon version (bits [1:0]): 0x01 = Version 1, 0x02 = Version 2.
pub const VER_MASK: u8 = 0b0000_0011;
pub const VER_SHIFT: u8 = 0;
pub const VER_V1: u8 = 0x01;
pub const VER_V2: u8 = 0x02;

// ---------------------------------------------------------------------------
// 0x18 — CONV_STATUS Bitfields
// ---------------------------------------------------------------------------
/// Rolling 3-bit conversion rolling counter (bits [7:5]).
pub const SET_COUNT_MASK: u8 = 0b1110_0000;
pub const SET_COUNT_SHIFT: u8 = 5;

/// Power-on reset occurred flag (bit 4, R/W1CP).
pub const POR_MASK: u8 = 0b0001_0000;
pub const POR_SHIFT: u8 = 4;

/// Diagnostic self-test failure flag (bit 1).
pub const DIAG_STATUS_MASK: u8 = 0b0000_0010;
pub const DIAG_STATUS_SHIFT: u8 = 1;

/// Conversion result ready flag (bit 0).
pub const RESULT_STATUS_MASK: u8 = 0b0000_0001;
pub const RESULT_STATUS_SHIFT: u8 = 0;

// ---------------------------------------------------------------------------
// 0x1C — DEVICE_STATUS Bitfields
// ---------------------------------------------------------------------------
/// INTB pin readback (bit 4): 1 = Inactive / high.
pub const INTB_RB_MASK: u8 = 0b0001_0000;
pub const INTB_RB_SHIFT: u8 = 4;

/// Oscillator error flag (bit 3, R/W1CP).
pub const OSC_ER_MASK: u8 = 0b0000_1000;
pub const OSC_ER_SHIFT: u8 = 3;

/// INT pin error flag (bit 2, R/W1CP).
pub const INT_ER_MASK: u8 = 0b0000_0100;
pub const INT_ER_SHIFT: u8 = 2;

/// OTP memory CRC error flag (bit 1, R/W1CP).
pub const OTP_CRC_ER_MASK: u8 = 0b0000_0010;
pub const OTP_CRC_ER_SHIFT: u8 = 1;

/// VCC undervoltage error flag (bit 0, R/W1CP).
pub const VCC_UV_ER_MASK: u8 = 0b0000_0001;
pub const VCC_UV_ER_SHIFT: u8 = 0;

/// Mask for all clearable error flags in DEVICE_STATUS (bits [3:0]).
pub const FAULT_CLEAR_MASK: u8 = 0x0F;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_sequential_addresses() {
        let registers = [
            DEVICE_CONFIG_1,
            DEVICE_CONFIG_2,
            SENSOR_CONFIG_1,
            SENSOR_CONFIG_2,
            X_THR_CONFIG,
            Y_THR_CONFIG,
            Z_THR_CONFIG,
            T_CONFIG,
            INT_CONFIG_1,
            MAG_GAIN_CONFIG,
            MAG_OFFSET_CONFIG_1,
            MAG_OFFSET_CONFIG_2,
            I2C_ADDRESS,
            DEVICE_ID,
            MANUFACTURER_ID_LSB,
            MANUFACTURER_ID_MSB,
            T_MSB_RESULT,
            T_LSB_RESULT,
            X_MSB_RESULT,
            X_LSB_RESULT,
            Y_MSB_RESULT,
            Y_LSB_RESULT,
            Z_MSB_RESULT,
            Z_LSB_RESULT,
            CONV_STATUS,
            ANGLE_RESULT_MSB,
            ANGLE_RESULT_LSB,
            MAGNITUDE_RESULT,
            DEVICE_STATUS,
        ];

        for (i, &addr) in registers.iter().enumerate() {
            assert_eq!(addr, i as u8, "Register at index {} mismatch", i);
        }
        assert_eq!(registers.len(), 29);
    }

    #[test]
    fn test_extract_insert() {
        let reg = insert(0, CRC_EN_MASK, CRC_EN_SHIFT, 1);
        assert_eq!(reg, 0x80);
        assert_eq!(extract(reg, CRC_EN_MASK, CRC_EN_SHIFT), 1);

        let reg2 = insert(reg, CONV_AVG_MASK, CONV_AVG_SHIFT, 0b101);
        assert_eq!(extract(reg2, CONV_AVG_MASK, CONV_AVG_SHIFT), 0b101);
        assert_eq!(extract(reg2, CRC_EN_MASK, CRC_EN_SHIFT), 1);
    }
}
