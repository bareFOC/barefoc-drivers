//! # dc::motor — brushed DC motor controller
//!
//! Ports [`DCMotor.h`](source/Arduino-FOC-drivers/src/motors/dc/DCMotor.h)
//! and [`DCMotor.cpp`](source/Arduino-FOC-drivers/src/motors/dc/DCMotor.cpp)
//! from Arduino-FOC-drivers (origin/dc-motor branch).
//!
//! Implements open-loop and closed-loop motion control (angle, velocity, torque)
//! for brushed DC motors using SimpleFOC PID controllers and sensor abstractions.

#![allow(non_camel_case_types)]

use core::ops::{Deref, DerefMut};

use barefoc::common::time::Clock;
use barefoc::motor::base::MotorConfig;
use barefoc::motor::enums::{FOCMotorStatus, MotionControlType, TorqueControlType};
use barefoc::traits::current_sense::CurrentSense;
use barefoc::traits::sensor::Sensor;

use crate::dc::traits::DCDriver;

// ---------------------------------------------------------------------------
// DCMotor struct
// ---------------------------------------------------------------------------

/// Brushed DC motor controller with support for open-loop and closed-loop control.
///
/// Ports upstream `DCMotor`
/// ([`DCMotor.h`](source/Arduino-FOC-drivers/src/motors/dc/DCMotor.h)).
///
/// Generics:
/// - `'clk`: Lifetime of the referenced system clock.
/// - `S`: Shaft position/velocity sensor implementing [`Sensor`].
/// - `D`: Motor driver implementing [`DCDriver`].
/// - `CS`: Current sensor implementing [`CurrentSense`].
/// - `C`: System clock implementing [`Clock`].
pub struct DCMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: DCDriver,
    CS: CurrentSense,
    C: Clock,
{
    /// Motor configuration and state parameters.
    pub cfg: MotorConfig<C>,
    /// Linked hardware driver.
    pub driver: Option<D>,
    /// Linked shaft sensor.
    pub sensor: Option<S>,
    /// Linked current sensor.
    pub current_sense: Option<CS>,
    /// Current voltage set-point (V).
    pub voltage_sp: f32,
    /// Current current set-point (A).
    pub current_sp: f32,
    /// Reference to the system clock.
    pub clock: &'clk C,
}

impl<'clk, S, D, CS, C> Deref for DCMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: DCDriver,
    CS: CurrentSense,
    C: Clock,
{
    type Target = MotorConfig<C>;

    fn deref(&self) -> &Self::Target {
        &self.cfg
    }
}

impl<'clk, S, D, CS, C> DerefMut for DCMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: DCDriver,
    CS: CurrentSense,
    C: Clock,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.cfg
    }
}

impl<'clk, S, D, CS, C> DCMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: DCDriver,
    CS: CurrentSense,
    C: Clock,
{
    /// Construct a new `DCMotor` with default configurations.
    ///
    /// Pole pairs defaults to 1 per upstream DC motor convention.
    /// Torque control defaults to [`TorqueControlType::voltage`].
    pub fn new(clock: &'clk C) -> Self {
        let mut cfg = MotorConfig::new(clock);
        cfg.pole_pairs = 1;
        cfg.torque_controller = TorqueControlType::voltage;

        Self {
            cfg,
            driver: None,
            sensor: None,
            current_sense: None,
            voltage_sp: 0.0,
            current_sp: 0.0,
            clock,
        }
    }

    /// Construct a new `DCMotor` with physical electrical parameters.
    pub fn with_params(r: f32, kv: f32, inductance: f32, clock: &'clk C) -> Self {
        let mut m = Self::new(clock);
        m.cfg.phase_resistance = r;
        m.cfg.KV_rating = kv;
        m.cfg.phase_inductance = inductance;
        m.cfg.axis_inductance.q = inductance;
        m.cfg.axis_inductance.d = inductance;
        m
    }

    /// Link the DC driver hardware.
    pub fn link_driver(&mut self, driver: D) {
        self.driver = Some(driver);
    }

    /// Link the shaft position/velocity sensor.
    pub fn link_sensor(&mut self, sensor: S) {
        self.sensor = Some(sensor);
    }

    /// Link the current sense interface.
    pub fn link_current_sense(&mut self, current_sense: CS) {
        self.current_sense = Some(current_sense);
    }

    /// Read-only access to the linked sensor.
    pub fn sensor(&self) -> Option<&S> {
        self.sensor.as_ref()
    }

    /// Mutable access to the linked sensor.
    pub fn sensor_mut(&mut self) -> Option<&mut S> {
        self.sensor.as_mut()
    }

    /// Read-only access to the linked driver.
    pub fn driver(&self) -> Option<&D> {
        self.driver.as_ref()
    }

    /// Mutable access to the linked driver.
    pub fn driver_mut(&mut self) -> Option<&mut D> {
        self.driver.as_mut()
    }

    /// Read-only access to the linked current sense.
    pub fn current_sense(&self) -> Option<&CS> {
        self.current_sense.as_ref()
    }

    /// Mutable access to the linked current sense.
    pub fn current_sense_mut(&mut self) -> Option<&mut CS> {
        self.current_sense.as_mut()
    }

    /// Initialise motor controller and configure PID limits.
    ///
    /// Returns 1 on success, 0 on failure.
    ///
    /// Mirrors upstream `DCMotor::init()`
    /// ([`DCMotor.cpp`](source/Arduino-FOC-drivers/src/motors/dc/DCMotor.cpp) lines 18–57).
    pub fn init(&mut self) -> i32 {
        if self.driver.is_none() || !self.driver.as_ref().unwrap().params().initialized {
            self.cfg.motor_status = FOCMotorStatus::motor_init_failed;
            return 0;
        }

        self.cfg.motor_status = FOCMotorStatus::motor_initializing;

        // Sanity check for voltage limit: must not exceed driver limit
        let driver_v_limit = self.driver.as_ref().unwrap().params().voltage_limit;
        if self.cfg.voltage_limit <= 0.0 || self.cfg.voltage_limit > driver_v_limit {
            self.cfg.voltage_limit = driver_v_limit;
        }
        if self.cfg.voltage_sensor_align > self.cfg.voltage_limit {
            self.cfg.voltage_sensor_align = self.cfg.voltage_limit;
        }

        // Update controller limits
        if let Some(cs) = self.current_sense.as_mut() {
            let _ = cs.init();
            self.cfg.PID_current_q.set_limit(self.cfg.voltage_limit);
            self.cfg.PID_current_d.set_limit(self.cfg.voltage_limit);
        }
        if self.cfg.phase_resistance > 0.0
            || self.cfg.torque_controller != TorqueControlType::voltage
        {
            self.cfg.PID_velocity.set_limit(self.cfg.current_limit);
        } else {
            self.cfg.PID_velocity.set_limit(self.cfg.voltage_limit);
        }
        self.cfg.P_angle.set_limit(self.cfg.velocity_limit);

        self.enable();
        self.cfg.motor_status = FOCMotorStatus::motor_ready;
        1
    }

    /// Enable the motor and driver.
    ///
    /// Mirrors upstream `DCMotor::enable()`
    /// ([`DCMotor.cpp`](source/Arduino-FOC-drivers/src/motors/dc/DCMotor.cpp) lines 72–78).
    pub fn enable(&mut self) {
        if let Some(driver) = self.driver.as_mut() {
            driver.enable();
            driver.set_pwm(0.0);
        }
        if let Some(cs) = self.current_sense.as_mut() {
            cs.enable();
        }
        self.cfg.PID_velocity.reset();
        self.cfg.P_angle.reset();
        self.cfg.PID_current_q.reset();
        self.cfg.PID_current_d.reset();
        self.cfg.enabled = 1;
    }

    /// Disable the motor and driver.
    ///
    /// Mirrors upstream `DCMotor::disable()`
    /// ([`DCMotor.cpp`](source/Arduino-FOC-drivers/src/motors/dc/DCMotor.cpp) lines 61–68).
    pub fn disable(&mut self) {
        if let Some(driver) = self.driver.as_mut() {
            driver.set_pwm(0.0);
            driver.disable();
        }
        if let Some(cs) = self.current_sense.as_mut() {
            cs.disable();
        }
        self.cfg.enabled = 0;
    }

    /// Set phase voltage to the motor driver if motor is enabled.
    ///
    /// Mirrors upstream `DCMotor::setPhaseVoltage(float U)`
    /// ([`DCMotor.cpp`](source/Arduino-FOC-drivers/src/motors/dc/DCMotor.cpp) lines 125–128).
    pub fn set_phase_voltage(&mut self, voltage: f32) {
        if self.cfg.enabled != 0 {
            if let Some(driver) = self.driver.as_mut() {
                driver.set_pwm(voltage);
            }
        }
    }

    /// Current shaft angle in radians.
    pub fn shaft_angle(&self) -> f32 {
        match self.sensor.as_ref() {
            None => self.cfg.shaft_angle,
            Some(sensor) => {
                let sign = self.cfg.sensor_direction.sign() as f32;
                sign * sensor.get_angle() - self.cfg.sensor_offset
            }
        }
    }

    /// Current filtered shaft velocity in rad/s.
    pub fn shaft_velocity(&mut self) -> f32 {
        let sign = self.cfg.sensor_direction.sign() as f32;
        match self.sensor.as_mut() {
            None => self.cfg.shaft_velocity,
            Some(sensor) => {
                let raw = sensor.get_velocity();
                let filtered = self.cfg.LPF_velocity.update(raw, self.clock);
                sign * filtered
            }
        }
    }

    /// Execute motion control loops for the given setpoint.
    ///
    /// Supported modes:
    /// - [`MotionControlType::angle`]: Cascade control (Angle P -> Velocity PI -> Voltage/Current).
    /// - [`MotionControlType::velocity`]: Velocity control (Velocity PI -> Voltage/Current).
    /// - [`MotionControlType::torque`]: Torque control via voltage or measured DC current.
    /// - [`MotionControlType::velocity_openloop`]: Open-loop velocity ramping.
    /// - [`MotionControlType::angle_openloop`]: Open-loop position ramping.
    ///
    /// Mirrors upstream `DCMotor::move(float target)`
    /// ([`DCMotor.cpp`](source/Arduino-FOC-drivers/src/motors/dc/DCMotor.cpp) lines 82–122).
    pub fn move_motor(&mut self, target: f32) {
        let target = if barefoc::math::isset(target) {
            self.cfg.target = target;
            target
        } else {
            self.cfg.target
        };

        // Decimate motion loop if downsampling configured
        self.cfg.motion_cnt += 1;
        if self.cfg.motion_cnt <= self.cfg.motion_downsample {
            return;
        }
        self.cfg.motion_cnt = 0;

        // Sensor update
        if let Some(sensor) = self.sensor.as_mut() {
            sensor.update(self.clock);
        }

        let controller = self.cfg.controller;
        if controller != MotionControlType::angle_openloop
            && controller != MotionControlType::velocity_openloop
        {
            self.cfg.shaft_angle = self.shaft_angle();
            self.cfg.shaft_velocity = self.shaft_velocity();
        }

        if self.cfg.enabled == 0 {
            return;
        }

        match controller {
            MotionControlType::torque => {
                match self.cfg.torque_controller {
                    TorqueControlType::voltage => {
                        self.voltage_sp = target;
                    }
                    TorqueControlType::dc_current | TorqueControlType::foc_current => {
                        self.current_sp = target;
                        self.voltage_sp = self.current_loop(self.current_sp);
                    }
                    _ => {
                        self.voltage_sp = 0.0;
                    }
                }
            }
            MotionControlType::angle => {
                self.cfg.shaft_angle_sp = target;
                let angle_err = self.cfg.shaft_angle_sp - self.cfg.shaft_angle;
                self.cfg.shaft_velocity_sp = self.cfg.P_angle.update(angle_err, self.clock);
                let vel_err = self.cfg.shaft_velocity_sp - self.cfg.shaft_velocity;
                match self.cfg.torque_controller {
                    TorqueControlType::voltage => {
                        self.voltage_sp = self.cfg.PID_velocity.update(vel_err, self.clock);
                    }
                    TorqueControlType::dc_current | TorqueControlType::foc_current => {
                        self.current_sp = self.cfg.PID_velocity.update(vel_err, self.clock);
                        self.voltage_sp = self.current_loop(self.current_sp);
                    }
                    _ => {
                        self.voltage_sp = 0.0;
                    }
                }
            }
            MotionControlType::velocity => {
                self.cfg.shaft_velocity_sp = target;
                let vel_err = self.cfg.shaft_velocity_sp - self.cfg.shaft_velocity;
                match self.cfg.torque_controller {
                    TorqueControlType::voltage => {
                        self.voltage_sp = self.cfg.PID_velocity.update(vel_err, self.clock);
                    }
                    TorqueControlType::dc_current | TorqueControlType::foc_current => {
                        self.current_sp = self.cfg.PID_velocity.update(vel_err, self.clock);
                        self.voltage_sp = self.current_loop(self.current_sp);
                    }
                    _ => {
                        self.voltage_sp = 0.0;
                    }
                }
            }
            MotionControlType::velocity_openloop => {
                self.cfg.shaft_velocity_sp = target;
                let u = self.cfg.velocity_openloop(target, self.clock);
                let v_lim = if self.cfg.velocity_limit > 0.0 {
                    self.cfg.velocity_limit
                } else {
                    1.0
                };
                self.voltage_sp = (target / v_lim) * u;
            }
            MotionControlType::angle_openloop => {
                self.cfg.shaft_angle_sp = target;
                let u = self.cfg.angle_openloop(target, self.clock);
                let diff = target - self.cfg.shaft_angle;
                let sign = if diff > 0.0 {
                    1.0
                } else if diff < 0.0 {
                    -1.0
                } else {
                    0.0
                };
                self.voltage_sp = sign * u;
            }
            _ => {
                self.voltage_sp = 0.0;
            }
        }

        let v_lim = self.cfg.voltage_limit;
        self.voltage_sp = self.voltage_sp.clamp(-v_lim, v_lim);
        self.cfg.voltage.q = self.voltage_sp;
        self.set_phase_voltage(self.voltage_sp);
    }

    /// Internal current control loop. Regulates current error through `PID_current_q`.
    fn current_loop(&mut self, current_sp: f32) -> f32 {
        let i_lim = self.cfg.current_limit;
        let current_sp_clamped = current_sp.clamp(-i_lim, i_lim);
        self.cfg.current_sp = current_sp_clamped;

        if let Some(cs) = self.current_sense.as_mut() {
            let phase = cs.get_phase_currents();
            let current_raw = if phase.a != 0.0 {
                phase.a
            } else if phase.b != 0.0 {
                -phase.b
            } else {
                0.0
            };
            self.cfg.current.q = current_raw;
            let current_filtered = self.cfg.LPF_current_q.update(current_raw, self.clock);
            let error = current_sp_clamped - current_filtered;
            self.cfg.PID_current_q.update(error, self.clock)
        } else if self.cfg.phase_resistance > 0.0 {
            // No current sense, estimate voltage via Ohm's law (U = I * R)
            current_sp_clamped * self.cfg.phase_resistance
        } else {
            0.0
        }
    }
}

// ---------------------------------------------------------------------------
// Dummy peripherals for optional / sensorless usage
// ---------------------------------------------------------------------------

/// Dummy sensor for open-loop or sensorless DC motor configurations.
pub struct DummySensor<C: Clock> {
    base: barefoc::traits::sensor::SensorBase,
    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock> Default for DummySensor<C> {
    fn default() -> Self {
        Self::new()
    }
}

impl<C: Clock> DummySensor<C> {
    /// Create a new dummy sensor.
    pub const fn new() -> Self {
        Self {
            base: barefoc::traits::sensor::SensorBase::new(),
            _clock: core::marker::PhantomData,
        }
    }
}

impl<C: Clock> Sensor<C> for DummySensor<C> {
    fn base(&self) -> &barefoc::traits::sensor::SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut barefoc::traits::sensor::SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        0.0
    }
}

/// Dummy current sense for voltage-mode or sensorless DC motor configurations.
#[derive(Clone, Copy, Debug, Default)]
pub struct DummyCurrentSense {
    base: barefoc::traits::current_sense::CurrentSenseBase,
}

impl DummyCurrentSense {
    /// Create a new dummy current sense.
    pub const fn new() -> Self {
        Self {
            base: barefoc::traits::current_sense::CurrentSenseBase::new(),
        }
    }
}

impl CurrentSense for DummyCurrentSense {
    type Error = core::convert::Infallible;

    fn base(&self) -> &barefoc::traits::current_sense::CurrentSenseBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut barefoc::traits::current_sense::CurrentSenseBase {
        &mut self.base
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        self.base.initialized = true;
        Ok(())
    }

    fn get_phase_currents(&mut self) -> barefoc::math::PhaseCurrent {
        barefoc::math::PhaseCurrent {
            a: 0.0,
            b: 0.0,
            c: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;
    use crate::dc::traits::DCDriverParams;

    struct TestClock {
        us: Cell<u32>,
    }

    impl TestClock {
        fn new() -> Self {
            Self {
                us: Cell::new(0),
            }
        }
        fn set_us(&self, us: u32) {
            self.us.set(us);
        }
    }

    impl Clock for TestClock {
        fn now_us(&self) -> u32 {
            self.us.get()
        }
    }

    /// Mock DC Driver for testing.
    struct MockDCDriver {
        params: DCDriverParams,
        last_voltage: f32,
        enabled: bool,
    }

    impl MockDCDriver {
        fn new() -> Self {
            let mut params = DCDriverParams::new();
            params.initialized = true;
            params.voltage_power_supply = 12.0;
            params.voltage_limit = 12.0;
            Self {
                params,
                last_voltage: 0.0,
                enabled: false,
            }
        }
    }

    impl DCDriver for MockDCDriver {
        fn init(&mut self) -> Result<(), ()> {
            self.params.initialized = true;
            Ok(())
        }

        fn enable(&mut self) {
            self.enabled = true;
        }

        fn disable(&mut self) {
            self.enabled = false;
            self.last_voltage = 0.0;
        }

        fn set_pwm(&mut self, voltage: f32) {
            self.last_voltage = voltage;
        }

        fn params(&self) -> &DCDriverParams {
            &self.params
        }

        fn params_mut(&mut self) -> &mut DCDriverParams {
            &mut self.params
        }
    }

    /// Mock Sensor with configurable angle and velocity.
    struct MockSensor {
        base: barefoc::traits::sensor::SensorBase,
        angle: f32,
        velocity: f32,
    }

    impl MockSensor {
        fn new(angle: f32, velocity: f32) -> Self {
            Self {
                base: barefoc::traits::sensor::SensorBase::new(),
                angle,
                velocity,
            }
        }
    }

    impl Sensor<TestClock> for MockSensor {
        fn base(&self) -> &barefoc::traits::sensor::SensorBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut barefoc::traits::sensor::SensorBase {
            &mut self.base
        }

        fn get_sensor_angle(&self) -> f32 {
            self.angle
        }

        fn get_angle(&self) -> f32 {
            self.angle
        }

        fn get_velocity(&mut self) -> f32 {
            self.velocity
        }
    }

    /// Mock CurrentSense with configurable phase currents.
    struct MockCurrentSense {
        base: barefoc::traits::current_sense::CurrentSenseBase,
        current_a: f32,
    }

    impl MockCurrentSense {
        fn new(current_a: f32) -> Self {
            let mut base = barefoc::traits::current_sense::CurrentSenseBase::new();
            base.initialized = true;
            Self { base, current_a }
        }
    }

    impl CurrentSense for MockCurrentSense {
        type Error = core::convert::Infallible;

        fn base(&self) -> &barefoc::traits::current_sense::CurrentSenseBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut barefoc::traits::current_sense::CurrentSenseBase {
            &mut self.base
        }

        fn init(&mut self) -> Result<(), Self::Error> {
            self.base.initialized = true;
            Ok(())
        }

        fn get_phase_currents(&mut self) -> barefoc::math::PhaseCurrent {
            barefoc::math::PhaseCurrent {
                a: self.current_a,
                b: 0.0,
                c: 0.0,
            }
        }
    }

    type TestMotorDummy<'a> =
        DCMotor<'a, DummySensor<TestClock>, MockDCDriver, DummyCurrentSense, TestClock>;

    #[test]
    fn test_init_fails_without_driver_or_uninitialized() {
        let clock = TestClock::new();
        let mut motor: TestMotorDummy<'_> = DCMotor::new(&clock);

        assert_eq!(motor.init(), 0);
        assert_eq!(motor.cfg.motor_status, FOCMotorStatus::motor_init_failed);

        let mut driver = MockDCDriver::new();
        driver.params_mut().initialized = false;
        motor.link_driver(driver);

        assert_eq!(motor.init(), 0);
        assert_eq!(motor.cfg.motor_status, FOCMotorStatus::motor_init_failed);
    }

    #[test]
    fn test_init_success_and_enables_motor() {
        let clock = TestClock::new();
        let mut motor: TestMotorDummy<'_> = DCMotor::new(&clock);
        let driver = MockDCDriver::new();
        motor.link_driver(driver);

        assert_eq!(motor.init(), 1);
        assert_eq!(motor.cfg.motor_status, FOCMotorStatus::motor_ready);
        assert_eq!(motor.cfg.enabled, 1);
        assert!(motor.driver().unwrap().enabled);
    }

    #[test]
    fn test_closed_loop_torque_voltage_mode() {
        let clock = TestClock::new();
        let mut motor: TestMotorDummy<'_> = DCMotor::new(&clock);
        motor.link_driver(MockDCDriver::new());
        motor.init();

        motor.controller = MotionControlType::torque;
        motor.torque_controller = TorqueControlType::voltage;

        motor.move_motor(5.0);
        assert_eq!(motor.voltage_sp, 5.0);
        assert_eq!(motor.driver().unwrap().last_voltage, 5.0);

        // Clamped to voltage limit (12V)
        motor.move_motor(15.0);
        assert_eq!(motor.voltage_sp, 12.0);
        assert_eq!(motor.driver().unwrap().last_voltage, 12.0);
    }

    #[test]
    fn test_closed_loop_torque_current_mode() {
        let clock = TestClock::new();
        let mut motor: DCMotor<'_, DummySensor<TestClock>, MockDCDriver, MockCurrentSense, TestClock> =
            DCMotor::new(&clock);
        motor.link_driver(MockDCDriver::new());
        // Measured current = 1.0 A
        motor.link_current_sense(MockCurrentSense::new(1.0));
        motor.init();

        motor.controller = MotionControlType::torque;
        motor.torque_controller = TorqueControlType::dc_current;

        // Configure current PID: P = 2.0, I = 0.0, no ramp and no LPF lag for direct step test
        motor.LPF_current_q.set_tf(0.0);
        motor.PID_current_q.set_output_ramp(0.0);
        motor.PID_current_q.set_p(2.0);
        motor.PID_current_q.set_i(0.0);
        motor.PID_current_q.set_d(0.0);

        // Target current = 2.0 A -> Error = 2.0 - 1.0 = 1.0 A -> Voltage = 1.0 * 2.0 = 2.0 V
        motor.move_motor(2.0);
        assert!((motor.voltage_sp - 2.0).abs() < 1e-3);
        assert!((motor.driver().unwrap().last_voltage - 2.0).abs() < 1e-3);
    }

    #[test]
    fn test_closed_loop_velocity_mode() {
        let clock = TestClock::new();
        let mut motor: DCMotor<'_, MockSensor, MockDCDriver, DummyCurrentSense, TestClock> =
            DCMotor::new(&clock);
        motor.link_driver(MockDCDriver::new());
        // Sensor reporting velocity 5.0 rad/s
        motor.link_sensor(MockSensor::new(0.0, 5.0));
        motor.init();

        motor.controller = MotionControlType::velocity;
        motor.torque_controller = TorqueControlType::voltage;

        // Configure velocity PID: P = 0.5, I = 0.0, no ramp and no LPF lag for direct step test
        motor.LPF_velocity.set_tf(0.0);
        motor.PID_velocity.set_output_ramp(0.0);
        motor.PID_velocity.set_p(0.5);
        motor.PID_velocity.set_i(0.0);
        motor.PID_velocity.set_d(0.0);

        // Target velocity = 15.0 rad/s -> Error = 15.0 - 5.0 = 10.0 rad/s -> Voltage = 10.0 * 0.5 = 5.0 V
        motor.move_motor(15.0);
        assert!((motor.voltage_sp - 5.0).abs() < 1e-3);
        assert!((motor.driver().unwrap().last_voltage - 5.0).abs() < 1e-3);
    }

    #[test]
    fn test_closed_loop_angle_mode() {
        let clock = TestClock::new();
        let mut motor: DCMotor<'_, MockSensor, MockDCDriver, DummyCurrentSense, TestClock> =
            DCMotor::new(&clock);
        motor.link_driver(MockDCDriver::new());
        // Sensor reporting angle 1.0 rad, velocity 0.0 rad/s
        motor.link_sensor(MockSensor::new(1.0, 0.0));
        motor.init();

        motor.controller = MotionControlType::angle;
        motor.torque_controller = TorqueControlType::voltage;

        // Angle P = 10.0, Velocity P = 0.2
        motor.P_angle.set_p(10.0);
        motor.PID_velocity.set_p(0.2);
        motor.PID_velocity.set_i(0.0);
        motor.PID_velocity.set_d(0.0);

        // Target angle = 2.0 rad -> Angle Error = 1.0 rad -> Velocity Target = 10.0 rad/s
        // Velocity Error = 10.0 - 0.0 = 10.0 rad/s -> Voltage = 10.0 * 0.2 = 2.0 V
        motor.move_motor(2.0);
        assert!((motor.voltage_sp - 2.0).abs() < 1e-3);
        assert!((motor.driver().unwrap().last_voltage - 2.0).abs() < 1e-3);
    }

    #[test]
    fn test_open_loop_velocity_mode() {
        let clock = TestClock::new();
        clock.set_us(1000);
        let mut motor: TestMotorDummy<'_> = DCMotor::new(&clock);
        motor.link_driver(MockDCDriver::new());
        motor.init();

        motor.controller = MotionControlType::velocity_openloop;
        motor.velocity_limit = 20.0;
        motor.voltage_limit = 12.0;

        // Target = 10 rad/s (50% of velocity limit) -> Voltage = 50% of 12V = 6.0 V
        motor.move_motor(10.0);
        assert!((motor.voltage_sp - 6.0).abs() < 1e-3);
        assert!((motor.driver().unwrap().last_voltage - 6.0).abs() < 1e-3);
    }

    #[test]
    fn test_open_loop_angle_mode() {
        let clock = TestClock::new();
        clock.set_us(1000);
        let mut motor: TestMotorDummy<'_> = DCMotor::new(&clock);
        motor.link_driver(MockDCDriver::new());
        motor.init();

        motor.controller = MotionControlType::angle_openloop;
        motor.voltage_limit = 12.0;
        motor.velocity_limit = 20.0;

        // Step toward positive angle: voltage should be positive voltage_limit
        motor.move_motor(5.0);
        assert_eq!(motor.voltage_sp, 12.0);
        assert_eq!(motor.driver().unwrap().last_voltage, 12.0);
    }

    #[test]
    fn test_disabled_motor_does_not_actuate() {
        let clock = TestClock::new();
        let mut motor: TestMotorDummy<'_> = DCMotor::new(&clock);
        motor.link_driver(MockDCDriver::new());
        motor.init();
        motor.disable();

        motor.controller = MotionControlType::torque;
        motor.torque_controller = TorqueControlType::voltage;

        motor.move_motor(5.0);
        assert_eq!(motor.driver().unwrap().last_voltage, 0.0);
    }
}
