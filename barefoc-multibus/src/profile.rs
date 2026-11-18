//! # Generic SimpleFOC register profile
//!
//! Auto-generates a [`RegMap`] descriptor set from any type implementing
//! [`barefoc_drivers::comms::RegisterBridge`], exposing the full SimpleFOC
//! runtime-configurable register surface (motor params, PID gains, limits,
//! driver params, current-sense gains/offsets, sensor params, and kinematics
//! extensions) over any address-based bus without hand-written descriptors.

use barefoc_drivers::comms::registers::{RegisterId, RegisterValue};
use barefoc_drivers::comms::RegisterBridge;

use crate::table::{HookResult, RegEntry, RegMap};

/// Generic write hook for a SimpleFOC register. Deserializes the bank bytes into a
/// [`RegisterValue`], calls [`RegisterBridge::write_reg`], and returns
/// [`HookResult::Applied`]. On any deserialization or bridge error the write is
/// silently skipped (the bank still holds the bytes so read-back returns what the
/// host sent).
fn sf_hook<B: RegisterBridge>(ctx: &mut B, offset: u8, bytes: &[u8]) -> HookResult {
    let id = match RegisterId::try_from(offset) {
        Ok(id) => id,
        Err(_) => return HookResult::Applied,
    };
    let value = match RegisterValue::from_bytes(id, bytes) {
        Ok(v) => v,
        Err(_) => return HookResult::Applied,
    };
    let _ = ctx.write_reg(id, value);
    HookResult::Applied
}

/// Auto-generate a [`RegMap`] from a [`RegisterBridge`] implementation.
pub fn simplefoc_profile<B: RegisterBridge>(
    bridge: B,
) -> RegMap<B, 256, 128> {
    let mut map = RegMap::new(bridge);
    for id in RegisterId::ALL {
        let offset = id as u8;
        let width = id.expected_size() as u8;
        if id.is_read_only() {
            let _ = map.add_register(RegEntry::ro(offset, width));
        } else {
            let hook: crate::table::WriteHookFn<B> = sf_hook::<B>;
            let _ = map.add_register(RegEntry::rw_hooked(offset, width, hook));
        }
    }
    map
}

/// Refresh all read-only register bytes in the bank from the bridge.
pub fn refresh_telemetry<B: RegisterBridge, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<B, CAP, MAX>,
) {
    for id in RegisterId::ALL {
        if id.is_read_only() {
            let offset = id as u8;
            let width = id.expected_size();
            if let Ok(val) = map.handler().read_reg(id) {
                let mut buf = [0u8; 12];
                if let Ok(n) = val.to_bytes(&mut buf) {
                    let _ = map.set(offset, &buf[..width.min(n)]);
                }
            }
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use barefoc::motor::base::NullClock;
    use barefoc::motor::base::MotorConfig;
    use barefoc_drivers::comms::RegisterId;

    #[test]
    fn simplefoc_profile_entry_count() {
        let clock = NullClock;
        let config = MotorConfig::new(&clock);
        let map = simplefoc_profile(config);
        assert!(map.extent() >= 0x6C);
    }

    #[test]
    fn simplefoc_profile_ro_rejection() {
        let clock = NullClock;
        let config = MotorConfig::new(&clock);
        let mut map = simplefoc_profile(config);
        let res = map.write_phase(&[0x00, 0x01]);
        assert!(res.is_err());
    }

    #[test]
    fn simplefoc_profile_rw_roundtrip() {
        let clock = NullClock;
        let config = MotorConfig::new(&clock);
        let mut map = simplefoc_profile(config);

        let val: f32 = 12.5;
        let bytes = val.to_le_bytes();
        let mut phase = [0u8; 5];
        phase[0] = 0x50; // VoltageLimit
        phase[1..].copy_from_slice(&bytes);
        map.write_phase(&phase).unwrap();

        map.write_phase(&[0x50]).unwrap();
        let mut out = [0u8; 4];
        let n = map.read_phase(4, &mut out).unwrap();
        assert_eq!(n, 4);
        assert_eq!(f32::from_le_bytes(out), 12.5);
    }
}