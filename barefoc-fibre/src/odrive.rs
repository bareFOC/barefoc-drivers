//! ODrive GUI and odrivetool compatibility schema and endpoint mappings.
//!
//! Provides preconfigured schemas and endpoint definitions mapping
//! `barefoc::motor::MotorConfig` to the canonical ODrive interface:
//! - `vbus_voltage`: DC bus voltage
//! - `axis0.target_velocity`: target velocity setpoint
//! - `axis0.current_q`: measured quadrature current
//! - `axis0.shaft_angle`: current mechanical shaft angle
//! - `axis0.shaft_velocity`: current shaft velocity
//!
//! References upstream:
//! - `ref/fibre-cpp/legacy_protocol.hpp`
//! - `ref/fibre-cpp/legacy_object_client.cpp`

use crate::schema::Endpoint0Handler;
#[cfg(feature = "barefoc")]
use crate::server::Endpoint;

#[cfg(feature = "barefoc")]
use barefoc::common::time::Clock;
#[cfg(feature = "barefoc")]
use barefoc::motor::base::MotorConfig;

/// Canonical minified JSON interface manifest for ODrive GUI / odrivetool.
pub const BAREFOC_ODRIVE_JSON: &str = r#"[{"name":"vbus_voltage","id":1,"type":"float","access":"r"},{"name":"axis0","type":"object","members":[{"name":"target_velocity","id":2,"type":"float","access":"rw"},{"name":"current_q","id":3,"type":"float","access":"r"},{"name":"shaft_angle","id":4,"type":"float","access":"r"},{"name":"shaft_velocity","id":5,"type":"float","access":"r"}]}]"#;

/// Returns an `Endpoint0Handler` initialized with `BAREFOC_ODRIVE_JSON`.
pub fn create_odrive_schema() -> Endpoint0Handler {
    Endpoint0Handler::new(BAREFOC_ODRIVE_JSON)
}

#[cfg(feature = "barefoc")]
/// Combined context holding motor configuration and measured bus voltage.
pub struct BareFocAxisContext<'a, C: Clock> {
    /// Measured DC bus voltage.
    pub vbus_voltage: f32,
    /// Reference to the BareFOC motor configuration state.
    pub motor: &'a mut MotorConfig<C>,
}

#[cfg(feature = "barefoc")]
/// Creates an endpoint registry mapping `BAREFOC_ODRIVE_JSON` to `MotorConfig<C>`.
///
/// In this mapping, `vbus_voltage` is mapped to `motor.voltage_limit`.
pub fn make_motor_config_endpoints<C: Clock>() -> [Endpoint<MotorConfig<C>>; 5] {
    [
        Endpoint::readonly_f32(1, |m| m.voltage_limit),
        Endpoint::readwrite_f32(2, |m| m.target, |m, v| m.target = v),
        Endpoint::readonly_f32(3, |m| m.current.q),
        Endpoint::readonly_f32(4, |m| m.shaft_angle),
        Endpoint::readonly_f32(5, |m| m.shaft_velocity),
    ]
}

#[cfg(feature = "barefoc")]
/// Creates an endpoint registry mapping `BAREFOC_ODRIVE_JSON` to `BareFocAxisContext`.
pub fn make_axis_context_endpoints<'a, C: Clock>() -> [Endpoint<BareFocAxisContext<'a, C>>; 5] {
    [
        Endpoint::readonly_f32(1, |ctx| ctx.vbus_voltage),
        Endpoint::readwrite_f32(2, |ctx| ctx.motor.target, |ctx, v| ctx.motor.target = v),
        Endpoint::readonly_f32(3, |ctx| ctx.motor.current.q),
        Endpoint::readonly_f32(4, |ctx| ctx.motor.shaft_angle),
        Endpoint::readonly_f32(5, |ctx| ctx.motor.shaft_velocity),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::server::FibreServer;
    use barefoc::motor::base::{MotorConfig, NullClock};

    #[test]
    fn test_odrive_schema_initialization() {
        let schema = create_odrive_schema();
        assert_ne!(schema.json_crc(), 0);
        assert_eq!(schema.json_str(), BAREFOC_ODRIVE_JSON);
    }

    #[test]
    fn test_motor_config_endpoints_integration() {
        let schema = create_odrive_schema();
        let endpoints = make_motor_config_endpoints::<NullClock>();
        let server = FibreServer::new(schema, &endpoints);

        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);
        config.voltage_limit = 24.0;
        config.target = 10.0;
        config.current.q = 1.25;
        config.shaft_angle = core::f32::consts::PI;
        config.shaft_velocity = 50.0;

        // Read vbus_voltage (ep 1)
        let mut req = [0u8; 8];
        req[0..2].copy_from_slice(&1u16.to_le_bytes());
        req[2..4].copy_from_slice(&(1u16 | 0x8000).to_le_bytes());
        req[4..6].copy_from_slice(&4u16.to_le_bytes());
        req[6..8].copy_from_slice(&schema.json_crc().to_le_bytes());

        let mut resp = [0u8; 32];
        let len = server.process_packet(&req, &mut resp, &mut config).unwrap().unwrap();
        assert_eq!(len, 6);
        let vbus = f32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]);
        assert_eq!(vbus, 24.0);

        // Write target_velocity (ep 2)
        let mut write_req = [0u8; 12];
        write_req[0..2].copy_from_slice(&2u16.to_le_bytes());
        write_req[2..4].copy_from_slice(&(2u16 | 0x8000).to_le_bytes());
        write_req[4..6].copy_from_slice(&4u16.to_le_bytes());
        write_req[6..10].copy_from_slice(&75.5f32.to_le_bytes());
        write_req[10..12].copy_from_slice(&schema.json_crc().to_le_bytes());

        let write_len = server
            .process_packet(&write_req, &mut resp, &mut config)
            .unwrap()
            .unwrap();
        assert_eq!(write_len, 6);
        assert_eq!(config.target, 75.5);
    }
}
