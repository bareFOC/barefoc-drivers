//! # mt6701::status — Status, diagnostics, and telemetry for MT6701
//!
//! Upstream reference:
//! - [`MagneticSensorMT6701SSI.h`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.h)

/// Decoded status flags from the 4-bit `Mg[3:0]` field in the MT6701 SSI frame.
///
/// In the 24-bit frame, bits [9:6] encode:
/// - Bit 9 (`raw & 0x08`): `MGH` — Magnetic field is too strong (gap too small or magnet too powerful).
/// - Bit 8 (`raw & 0x04`): `MGL` — Magnetic field is too weak (gap too large or magnet too weak).
/// - Bit 7 (`raw & 0x02`): `PUSH` — Push-button press detected (axial displacement).
/// - Bit 6 (`raw & 0x01`): `LOS` — Loss of signal / tracking lost.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Mt6701Status {
    /// Magnetic field too strong (MGH, bit 9).
    pub mgh: bool,
    /// Magnetic field too weak (MGL, bit 8).
    pub mgl: bool,
    /// Push-button active (PUSH, bit 7).
    pub push: bool,
    /// Loss of signal / tracking lost (LOS, bit 6).
    pub los: bool,
    /// Raw 4-bit status nibble (`Mg[3:0]`).
    pub raw: u8,
}

impl Mt6701Status {
    /// Decodes an MT6701 4-bit status nibble (`Mg[3:0]`).
    #[inline]
    pub const fn from_raw(raw: u8) -> Self {
        let nibble = raw & 0x0F;
        Self {
            mgh: (nibble & 0x08) != 0,
            mgl: (nibble & 0x04) != 0,
            push: (nibble & 0x02) != 0,
            los: (nibble & 0x01) != 0,
            raw: nibble,
        }
    }

    /// Returns `true` if magnetic field and tracking are normal (`!mgh && !mgl && !los`).
    #[inline]
    pub const fn is_normal(&self) -> bool {
        !self.mgh && !self.mgl && !self.los
    }

    /// Returns `true` if tracking is valid (`!los`).
    #[inline]
    pub const fn is_tracking_valid(&self) -> bool {
        !self.los
    }

    /// Returns `true` if the magnetic field strength is within acceptable limits (`!mgh && !mgl`).
    #[inline]
    pub const fn is_field_normal(&self) -> bool {
        !self.mgh && !self.mgl
    }
}

/// Cumulative communication and integrity telemetry counters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Mt6701Telemetry {
    /// Frames accepted with verified CRC checksum.
    pub crc_ok: u32,
    /// Individual read attempts that failed CRC checksum (including successfully retried attempts).
    pub crc_failed: u32,
    /// Logical read operations where all retry attempts failed CRC checksum.
    pub crc_exhausted: u32,
    /// Sticky fault flag: raised when a logical read operation exhausts all retry attempts.
    /// Cleared only via [`Mt6701Telemetry::clear_fault`].
    pub fault_flag: bool,
}

impl Mt6701Telemetry {
    /// Clears the sticky fault flag.
    #[inline]
    pub fn clear_fault(&mut self) {
        self.fault_flag = false;
    }

    /// Resets all cumulative counters to zero and clears the sticky fault flag.
    #[inline]
    pub fn reset(&mut self) {
        self.crc_ok = 0;
        self.crc_failed = 0;
        self.crc_exhausted = 0;
        self.fault_flag = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_status_decoding() {
        let st = Mt6701Status::from_raw(0b1001);
        assert!(st.mgh);
        assert!(!st.mgl);
        assert!(!st.push);
        assert!(st.los);
        assert_eq!(st.raw, 0b1001);
        assert!(!st.is_normal());
        assert!(!st.is_tracking_valid());
        assert!(!st.is_field_normal());

        let st_clean = Mt6701Status::from_raw(0b0000);
        assert!(st_clean.is_normal());
        assert!(st_clean.is_tracking_valid());
        assert!(st_clean.is_field_normal());

        let st_push = Mt6701Status::from_raw(0b0010);
        assert!(st_push.push);
        assert!(st_push.is_normal());
    }

    #[test]
    fn test_telemetry_fault_and_reset() {
        let mut telem = Mt6701Telemetry {
            crc_ok: 10,
            crc_failed: 2,
            crc_exhausted: 1,
            fault_flag: true,
        };

        telem.clear_fault();
        assert!(!telem.fault_flag);
        assert_eq!(telem.crc_exhausted, 1);

        telem.reset();
        assert_eq!(telem.crc_ok, 0);
        assert_eq!(telem.crc_failed, 0);
        assert_eq!(telem.crc_exhausted, 0);
        assert!(!telem.fault_flag);
    }
}
