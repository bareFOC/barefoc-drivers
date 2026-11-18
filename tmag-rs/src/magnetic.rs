//! # magnetic — 3D Magnetic Flux Density and 2-DOF Plane Projections
//!
//! Provides the [`MagneticField3D`] struct representing magnetic field measurements
//! in milliTesla (mT) across $X$, $Y$, and $Z$ axes, along with instantaneous 2-DOF
//! plane angle calculations for coupled spherical differential joints.

/// 3D magnetic flux density vector in milliTesla (mT).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagneticField3D {
    /// X-axis flux density in milliTesla (mT).
    pub x: f32,
    /// Y-axis flux density in milliTesla (mT).
    pub y: f32,
    /// Z-axis flux density in milliTesla (mT).
    pub z: f32,
}

impl MagneticField3D {
    /// Full-scale count for 16-bit ADC: $2^{15} = 32768$.
    const ADC_16BIT_FULL_SCALE: f32 = 32768.0;
    /// Full-scale count for 8-bit MSB ADC: $2^7 = 128$.
    const ADC_8BIT_FULL_SCALE: f32 = 128.0;

    /// Create a new 3D magnetic field vector in milliTesla.
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Convert from 16-bit raw signed 2's complement ADC codes.
    ///
    /// Follows TI TMAG5273 Datasheet Equation 10:
    /// $B = \frac{\text{raw}}{32768} \times B_R$
    #[inline]
    pub fn from_raw_16bit(
        x_raw: i16,
        y_raw: i16,
        z_raw: i16,
        range_xy_mt: f32,
        range_z_mt: f32,
    ) -> Self {
        Self {
            x: (x_raw as f32) * range_xy_mt / Self::ADC_16BIT_FULL_SCALE,
            y: (y_raw as f32) * range_xy_mt / Self::ADC_16BIT_FULL_SCALE,
            z: (z_raw as f32) * range_z_mt / Self::ADC_16BIT_FULL_SCALE,
        }
    }

    /// Convert from 8-bit raw signed MSB codes (Fast 8-bit stream mode).
    ///
    /// Follows TI TMAG5273 Datasheet Section 6.5.2.1:
    /// $B = \frac{\text{raw}}{128} \times B_R$
    #[inline]
    pub fn from_raw_8bit(
        x_raw: i8,
        y_raw: i8,
        z_raw: i8,
        range_xy_mt: f32,
        range_z_mt: f32,
    ) -> Self {
        Self {
            x: (x_raw as f32) * range_xy_mt / Self::ADC_8BIT_FULL_SCALE,
            y: (y_raw as f32) * range_xy_mt / Self::ADC_8BIT_FULL_SCALE,
            z: (z_raw as f32) * range_z_mt / Self::ADC_8BIT_FULL_SCALE,
        }
    }

    /// Compute instantaneous 2-DOF coupled joint plane angles $[\alpha, \beta]$ in radians.
    ///
    /// - $\alpha = \text{atan2}(B_x, B_z)$ (Pitch tilt angle)
    /// - $\beta = \text{atan2}(B_y, B_z)$ (Roll tilt angle)
    ///
    /// Executes in $\sim 60\text{ ns}$ using [`libm::atan2f`] without dynamic memory allocation.
    #[inline]
    pub fn plane_angles(&self) -> [f32; 2] {
        [libm::atan2f(self.x, self.z), libm::atan2f(self.y, self.z)]
    }

    /// Projected angle on the XY plane: $\theta_{xy} = \text{atan2}(B_y, B_x)$.
    #[inline]
    pub fn angle_xy(&self) -> f32 {
        libm::atan2f(self.y, self.x)
    }

    /// Projected angle on the YZ plane: $\theta_{yz} = \text{atan2}(B_z, B_y)$.
    #[inline]
    pub fn angle_yz(&self) -> f32 {
        libm::atan2f(self.z, self.y)
    }

    /// Projected angle on the XZ plane: $\theta_{xz} = \text{atan2}(B_z, B_x)$.
    #[inline]
    pub fn angle_xz(&self) -> f32 {
        libm::atan2f(self.z, self.x)
    }

    /// Vector Euclidean magnitude in milliTesla: $\|\mathbf{B}\| = \sqrt{B_x^2 + B_y^2 + B_z^2}$.
    #[inline]
    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_raw_16bit() {
        // Range 80 mT: 32768 / 2 = 16384 -> 40.0 mT
        let field = MagneticField3D::from_raw_16bit(16384, -16384, 32767, 80.0, 80.0);
        assert!((field.x - 40.0).abs() < 1e-3);
        assert!((field.y - (-40.0)).abs() < 1e-3);
        assert!((field.z - 80.0).abs() < 0.1);
    }

    #[test]
    fn test_from_raw_8bit() {
        // Range 80 mT: 64 / 128 * 80 = 40.0 mT
        let field = MagneticField3D::from_raw_8bit(64, -64, 127, 80.0, 80.0);
        assert!((field.x - 40.0).abs() < 1e-3);
        assert!((field.y - (-40.0)).abs() < 1e-3);
        assert!((field.z - 79.375).abs() < 1e-3);
    }

    #[test]
    fn test_plane_angles() {
        // Bx = 10.0, By = 0.0, Bz = 10.0 -> alpha = atan2(10, 10) = pi / 4, beta = atan2(0, 10) = 0
        let field = MagneticField3D::new(10.0, 0.0, 10.0);
        let [alpha, beta] = field.plane_angles();
        assert!((alpha - core::f32::consts::FRAC_PI_4).abs() < 1e-5);
        assert!((beta - 0.0).abs() < 1e-5);

        // Bx = 0.0, By = 10.0, Bz = 10.0 -> alpha = 0, beta = pi / 4
        let field2 = MagneticField3D::new(0.0, 10.0, 10.0);
        let [alpha2, beta2] = field2.plane_angles();
        assert!((alpha2 - 0.0).abs() < 1e-5);
        assert!((beta2 - core::f32::consts::FRAC_PI_4).abs() < 1e-5);
    }
}
