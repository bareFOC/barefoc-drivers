//! # angle — CORDIC Angle Representation and SI Unit Conversion
//!
//! Provides the [`Angle`] struct representing rotational position extracted from the
//! TMAG5273 internal hardware CORDIC engine with 0.0625° ($1/16^\circ$) resolution.

/// Value of $\pi$ in single-precision floating point.
pub const _PI: f32 = core::f32::consts::PI;
/// Value of $2\pi$ in single-precision floating point.
pub const _2PI: f32 = 2.0 * core::f32::consts::PI;

/// Angle measurement produced by the hardware CORDIC engine or plane projection.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Angle {
    radians: f32,
    degrees: f32,
}

impl Angle {
    /// 9-bit integer mask for bits [12:4] in CORDIC angle register (0..=360).
    const INT_MASK: u16 = 0x1FF;
    /// Bit shift for integer field.
    const INT_SHIFT: u16 = 4;
    /// 4-bit fractional mask for bits [3:0] (resolution 1/16 = 0.0625°).
    const FRAC_MASK: u16 = 0x0F;
    /// Fractional scale factor: 1 / 16.
    const FRAC_SCALE: f32 = 0.0625;

    /// Construct [`Angle`] from the raw 16-bit 9.4 fixed-point register value
    /// (`ANGLE_RESULT_MSB` and `ANGLE_RESULT_LSB`).
    ///
    /// Per TI Datasheet Equation 14:
    /// `Angle = bits[12:4] + bits[3:0] / 16`
    ///
    /// Returns `None` if the decoded angle is outside $[0^\circ, 360^\circ]$.
    #[inline]
    pub fn from_raw(raw: u16) -> Option<Self> {
        let integer = ((raw >> Self::INT_SHIFT) & Self::INT_MASK) as f32;
        let fraction = (raw & Self::FRAC_MASK) as f32 * Self::FRAC_SCALE;
        let deg = integer + fraction;
        if (0.0..=360.0).contains(&deg) {
            let rad = deg * (_PI / 180.0);
            Some(Self {
                radians: rad,
                degrees: deg,
            })
        } else {
            None
        }
    }

    /// Construct [`Angle`] directly from degrees ($[0^\circ, 360^\circ]$).
    #[inline]
    pub fn from_degrees(degrees: f32) -> Self {
        let rad = degrees * (_PI / 180.0);
        Self {
            radians: rad,
            degrees,
        }
    }

    /// Construct [`Angle`] directly from radians ($[0, 2\pi]$).
    #[inline]
    pub fn from_radians(radians: f32) -> Self {
        let deg = radians * (180.0 / _PI);
        Self {
            radians,
            degrees: deg,
        }
    }

    /// Returns the angle in radians ($[0, 2\pi]$).
    #[inline]
    pub fn radians(&self) -> f32 {
        self.radians
    }

    /// Returns the angle in degrees ($[0^\circ, 360^\circ]$).
    #[inline]
    pub fn degrees(&self) -> f32 {
        self.degrees
    }

    /// Convert back to 16-bit 9.4 fixed-point raw register encoding.
    #[inline]
    pub fn to_raw(&self) -> Option<u16> {
        if self.degrees < 0.0 || self.degrees > 360.0 {
            return None;
        }
        let raw = libm::roundf(self.degrees * 16.0);
        if raw >= 0.0 && raw <= (360.0 * 16.0) {
            Some(raw as u16)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_raw_decoding() {
        // 0 degrees -> 0x0000
        let a0 = Angle::from_raw(0x0000).unwrap();
        assert_eq!(a0.degrees(), 0.0);
        assert_eq!(a0.radians(), 0.0);

        // 180.0 degrees -> 180 * 16 = 2880 = 0x0B40
        let a180 = Angle::from_raw(0x0B40).unwrap();
        assert!((a180.degrees() - 180.0).abs() < 1e-4);
        assert!((a180.radians() - _PI).abs() < 1e-4);

        // 360.0 degrees -> 360 * 16 = 5760 = 0x1680
        let a360 = Angle::from_raw(0x1680).unwrap();
        assert!((a360.degrees() - 360.0).abs() < 1e-4);
        assert!((a360.radians() - _2PI).abs() < 1e-4);

        // 0.0625 degrees (1 LSB of fraction) -> 0x0001
        let a_min = Angle::from_raw(0x0001).unwrap();
        assert!((a_min.degrees() - 0.0625).abs() < 1e-5);
    }

    #[test]
    fn test_angle_invalid_raw() {
        // > 360 degrees: 361 * 16 = 5776 = 0x1690
        assert!(Angle::from_raw(0x1690).is_none());
        assert!(Angle::from_raw(0xFFFF).is_none());
    }

    #[test]
    fn test_angle_from_radians_and_degrees() {
        let a = Angle::from_degrees(90.0);
        assert!((a.radians() - (_PI / 2.0)).abs() < 1e-5);
        assert!((a.degrees() - 90.0).abs() < 1e-5);

        let a2 = Angle::from_radians(_PI);
        assert!((a2.degrees() - 180.0).abs() < 1e-5);
    }
}
