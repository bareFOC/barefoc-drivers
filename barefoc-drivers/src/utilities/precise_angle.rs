//! # utilities::precise_angle — Multi-Turn Fixed-Point Angle Tracking
//!
//! Ports [`PreciseAngle`](source/Arduino-FOC-drivers/src/utilities/PreciseAngle.h)
//! and [`PreciseAngle.cpp`](source/Arduino-FOC-drivers/src/utilities/PreciseAngle.cpp)
//! from Arduino-FOC-drivers.
//!
//! Represents angular position as an integer rotation count (`i32`) and a fine shaft angle
//! tick count (`u16`, default 14-bit CPR = 16384). Prevents floating-point precision loss
//! in multi-turn positioning systems.

use core::ops::{Add, AddAssign, Sub, SubAssign};

/// Multi-turn fixed-point angle tracking struct.
///
/// Ports the upstream `PreciseAngle` class.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct PreciseAngle {
    /// Fine angle in ticks within one revolution (`0..CPR`).
    pub angle: u16,
    /// Number of full mechanical revolutions (positive or negative).
    pub rotations: i32,
}

impl PreciseAngle {
    /// Default counts per revolution (14-bit CPR: 16384).
    pub const CPR: u16 = 16384;
    /// Threshold for detecting overflow / underflow between updates (0.8 * CPR: 13107).
    pub const CPR_OVERFLOW_CHECK: u16 = 13107;

    /// Construct a new `PreciseAngle` from tick count and rotation count.
    #[inline]
    pub const fn new(angle: u16, rotations: i32) -> Self {
        Self { angle, rotations }
    }

    /// Construct a `PreciseAngle` from an angle in radians (`f32`).
    pub fn from_radians(radians: f32) -> Self {
        let rot_f = libm::floorf(radians / barefoc::math::_2PI);
        let mut rotations = rot_f as i32;
        let rem = radians - (rot_f * barefoc::math::_2PI);
        let ticks_f = libm::roundf(rem * Self::CPR as f32 / barefoc::math::_2PI);
        let mut angle = ticks_f as u16;
        if angle >= Self::CPR {
            angle = 0;
            rotations += 1;
        }
        Self { angle, rotations }
    }

    /// Construct a `PreciseAngle` from an angle in radians with 64-bit precision (`f64`).
    pub fn from_radians_f64(radians: f64) -> Self {
        const _2PI_64: f64 = core::f64::consts::TAU;
        let rot_f = libm::floor(radians / _2PI_64);
        let mut rotations = rot_f as i32;
        let rem = radians - (rot_f * _2PI_64);
        let ticks_f = libm::round(rem * Self::CPR as f64 / _2PI_64);
        let mut angle = ticks_f as u16;
        if angle >= Self::CPR {
            angle = 0;
            rotations += 1;
        }
        Self { angle, rotations }
    }

    /// Returns the shaft angle modulo $2\pi$ in radians (`f32`).
    #[inline]
    pub fn get_shaft_angle(&self) -> f32 {
        self.angle as f32 * barefoc::math::_2PI / Self::CPR as f32
    }

    /// Returns the number of full motor revolutions.
    #[inline]
    pub fn get_rotations(&self) -> i32 {
        self.rotations
    }

    /// Returns the raw shaft ticks within the current revolution (`0..CPR`).
    #[inline]
    pub fn get_shaft_ticks(&self) -> u16 {
        self.angle
    }

    /// Returns the total angle including all rotations in radians (`f32`).
    #[inline]
    pub fn as_float(&self) -> f32 {
        (self.angle as f32 * barefoc::math::_2PI / Self::CPR as f32)
            + (self.rotations as f32 * barefoc::math::_2PI)
    }

    /// Returns the total angle including all rotations in radians with 64-bit precision (`f64`).
    #[inline]
    pub fn as_double(&self) -> f64 {
        const _2PI_64: f64 = core::f64::consts::TAU;
        (self.angle as f64 * _2PI_64 / Self::CPR as f64)
            + (self.rotations as f64 * _2PI_64)
    }

    /// Returns the total angle including all rotations in raw ticks (`i64`).
    #[inline]
    pub fn as_ticks(&self) -> i64 {
        self.angle as i64 + (self.rotations as i64 * Self::CPR as i64)
    }

    /// Update the angle with a new raw tick count (`0..CPR`).
    ///
    /// Detects rollover (e.g. $16300 \to 100$) and rollunder (e.g. $100 \to 16300$)
    /// across successive updates and adjusts the rotation counter accordingly.
    pub fn update(&mut self, current_angle: u16) {
        let diff = current_angle as i32 - self.angle as i32;
        if diff.abs() > Self::CPR_OVERFLOW_CHECK as i32 {
            if diff > 0 {
                self.rotations -= 1;
            } else {
                self.rotations += 1;
            }
        }
        self.angle = current_angle;
    }

    /// Compute the angular velocity in rad/s between this angle and a previous angle
    /// over an elapsed interval in microseconds.
    pub fn velocity(&self, previous: &Self, microseconds: u32) -> f32 {
        if microseconds == 0 {
            return 0.0;
        }
        let diff = *self - *previous;
        diff.as_float() * (1e6_f32 / microseconds as f32)
    }
}

impl Add for PreciseAngle {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        let mut rotations = self.rotations + other.rotations;
        let temp = self.angle as u32 + other.angle as u32;
        let angle = if temp >= Self::CPR as u32 {
            rotations += 1;
            (temp - Self::CPR as u32) as u16
        } else {
            temp as u16
        };
        Self { angle, rotations }
    }
}

impl AddAssign for PreciseAngle {
    fn add_assign(&mut self, other: Self) {
        *self = *self + other;
    }
}

impl Sub for PreciseAngle {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        let mut rotations = self.rotations - other.rotations;
        let temp = self.angle as i32 - other.angle as i32;
        let angle = if temp < 0 {
            rotations -= 1;
            (temp + Self::CPR as i32) as u16
        } else {
            temp as u16
        };
        Self { angle, rotations }
    }
}

impl SubAssign for PreciseAngle {
    fn sub_assign(&mut self, other: Self) {
        *self = *self - other;
    }
}

impl From<f32> for PreciseAngle {
    #[inline]
    fn from(radians: f32) -> Self {
        Self::from_radians(radians)
    }
}

impl From<f64> for PreciseAngle {
    #[inline]
    fn from(radians: f64) -> Self {
        Self::from_radians_f64(radians)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use barefoc::math::_2PI;

    #[test]
    fn test_precise_angle_conversions() {
        let p = PreciseAngle::new(8192, 2);
        assert_eq!(p.get_rotations(), 2);
        assert_eq!(p.get_shaft_ticks(), 8192);
        // Shaft angle = 8192 / 16384 * 2PI = PI
        assert!((p.get_shaft_angle() - core::f32::consts::PI).abs() < 1e-4);
        // Total angle = 2 * 2PI + PI = 5 * PI
        assert!((p.as_float() - 5.0 * core::f32::consts::PI).abs() < 1e-4);
        assert_eq!(p.as_ticks(), 8192 + 2 * 16384);
    }

    #[test]
    fn test_from_radians_positive_and_negative() {
        let p_pos = PreciseAngle::from_radians(1.5 * _2PI);
        assert_eq!(p_pos.rotations, 1);
        assert_eq!(p_pos.angle, 8192);
        assert!((p_pos.as_float() - 1.5 * _2PI).abs() < 1e-3);

        let p_neg = PreciseAngle::from_radians(-0.5 * _2PI);
        assert_eq!(p_neg.rotations, -1);
        assert_eq!(p_neg.angle, 8192);
        assert!((p_neg.as_float() - (-0.5 * _2PI)).abs() < 1e-3);
    }

    #[test]
    fn test_precise_angle_arithmetic() {
        let a = PreciseAngle::new(10000, 1);
        let b = PreciseAngle::new(10000, 2);

        // a + b: angle = 20000 - 16384 = 3616, rotations = 1 + 2 + 1 = 4
        let sum = a + b;
        assert_eq!(sum.angle, 3616);
        assert_eq!(sum.rotations, 4);

        // sum - b: 3616 - 10000 = -6384 -> angle = 10000, rotations = 4 - 2 - 1 = 1
        let diff = sum - b;
        assert_eq!(diff.angle, 10000);
        assert_eq!(diff.rotations, 1);
        assert_eq!(diff, a);
    }

    #[test]
    fn test_rollover_and_rollunder_detection() {
        let mut p = PreciseAngle::new(16000, 0);

        // Forward rollover: 16000 -> 200
        p.update(200);
        assert_eq!(p.rotations, 1);
        assert_eq!(p.angle, 200);

        // Backward rollunder: 200 -> 16000
        p.update(16000);
        assert_eq!(p.rotations, 0);
        assert_eq!(p.angle, 16000);
    }

    #[test]
    fn test_velocity_calculation() {
        let prev = PreciseAngle::new(0, 0);
        // Move quarter turn (4096 ticks = 0.5 * PI rad) in 100_000 microseconds (0.1s)
        let curr = PreciseAngle::new(4096, 0);
        let vel = curr.velocity(&prev, 100_000);
        // Expected velocity = (0.5 * PI) / 0.1 = 5.0 * PI rad/s = 15.707963 rad/s
        let expected = (0.5 * core::f32::consts::PI) / 0.1;
        assert!((vel - expected).abs() < 1e-3);
    }
}
