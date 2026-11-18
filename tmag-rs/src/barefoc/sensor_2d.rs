//! # barefoc::sensor_2d — 2-DOF Joint Sensor Adapter for BareFOC Kinematics
//!
//! Wraps [`Tmag5273`] to implement [`barefoc_kinematics::sensor::Sensor2D`] using synchronous
//! 6-byte 3D magnetic burst reads and instantaneous $\mathcal{O}(1)$ plane angle derivation.

use barefoc::common::time::Clock;
use barefoc_kinematics::sensor::Sensor2D;
use embedded_hal::i2c::I2c;

use crate::tmag5273::Tmag5273;

/// 2-DOF coupled differential joint sensor adapter implementing BareFOC [`Sensor2D`].
pub struct Tmag5273Sensor2D<I2C> {
    /// Internal TMAG5273 driver.
    pub driver: Tmag5273<I2C>,
    angles: [f32; 2],
    velocities: [f32; 2],
    prev_angles: [f32; 2],
    prev_ts_us: u32,
    /// Minimum elapsed time between velocity updates in seconds (default 100 µs).
    pub min_elapsed_time: f32,
}

impl<I2C> Tmag5273Sensor2D<I2C> {
    /// Create a new 2-DOF sensor adapter wrapping a configured [`Tmag5273`] driver.
    pub fn new(driver: Tmag5273<I2C>) -> Self {
        Self {
            driver,
            angles: [0.0, 0.0],
            velocities: [0.0, 0.0],
            prev_angles: [0.0, 0.0],
            prev_ts_us: 0,
            min_elapsed_time: 0.000_100,
        }
    }

    /// Return latest 2D angles $[\alpha, \beta]$ in radians without trait turbofish.
    #[inline]
    pub fn angles(&self) -> [f32; 2] {
        self.angles
    }

    /// Return latest 2D velocities $[\dot{\alpha}, \dot{\beta}]$ in rad/s without trait turbofish.
    #[inline]
    pub fn velocities(&self) -> [f32; 2] {
        self.velocities
    }
}

impl<I2C: I2c, C: Clock> Sensor2D<C> for Tmag5273Sensor2D<I2C> {
    fn update_2d(&mut self, clock: &C) {
        let now = clock.now_us();
        if let Ok(mag) = self.driver.read_magnetic() {
            let [alpha, beta] = mag.plane_angles();
            self.angles = [alpha, beta];

            if self.prev_ts_us == 0 {
                self.prev_angles = [alpha, beta];
                self.prev_ts_us = now;
                return;
            }

            let dt = (now.wrapping_sub(self.prev_ts_us) as i32) as f32 * 1e-6;
            if dt >= self.min_elapsed_time && dt > 0.0 {
                let mut d_alpha = alpha - self.prev_angles[0];
                let mut d_beta = beta - self.prev_angles[1];

                // Shortest angular wrap around [-pi, pi]
                if d_alpha > core::f32::consts::PI {
                    d_alpha -= 2.0 * core::f32::consts::PI;
                } else if d_alpha < -core::f32::consts::PI {
                    d_alpha += 2.0 * core::f32::consts::PI;
                }

                if d_beta > core::f32::consts::PI {
                    d_beta -= 2.0 * core::f32::consts::PI;
                } else if d_beta < -core::f32::consts::PI {
                    d_beta += 2.0 * core::f32::consts::PI;
                }

                self.velocities = [d_alpha / dt, d_beta / dt];
                self.prev_angles = [alpha, beta];
                self.prev_ts_us = now;
            }
        }
    }

    fn get_angles(&self) -> [f32; 2] {
        self.angles
    }

    fn get_velocities(&self) -> [f32; 2] {
        self.velocities
    }
}
