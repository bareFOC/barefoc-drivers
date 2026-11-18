//! # barefoc-drivers
//!
//! Pure-Rust, `no_std` port of the [Arduino SimpleFOC-drivers](https://github.com/simplefoc/Arduino-FOC-drivers)
//! library (extended encoders, gate drivers, comms, voltage sense, utilities)
//! that sit on top of the core [`barefoc`](https://crates.io/crates/barefoc) engine.
//!
//! ## Upstream Attribution
//!
//! This crate is directly ported from and inspired by the SimpleFOC-drivers project
//! created by Antun Skuric and the SimpleFOC community:
//!
//! - Website: <https://simplefoc.com>
//! - Upstream Repository: <https://github.com/simplefoc/Arduino-FOC-drivers>
//!
//! See [`PORTING_GUIDELINES.md`] for the full set of porting rules and
//! [`PORTING_STATUS.md`] for the per-module status and progress log.
//!
//! [`PORTING_GUIDELINES.md`]: https://github.com/simplefoc/Rust-FOC/blob/main/PORTING_GUIDELINES.md
//! [`PORTING_STATUS.md`]: https://github.com/simplefoc/Rust-FOC/blob/main/PORTING_STATUS.md

#![cfg_attr(docsrs, feature(doc_auto_cfg))]
#![no_std]

#[cfg(feature = "std")]
extern crate std;

// ---------------------------------------------------------------------------
// Module placeholders. The crate layout is fixed by §2 of PORTING_GUIDELINES.md;
// each driver is ported in its own commit and is intentionally empty here.
// ---------------------------------------------------------------------------

#[cfg(feature = "encoders")]
pub mod encoders;

pub mod drivers {
    // Per gate-driver modules (STSPIN, TMC, DRV, L6234, …) —
    // pending port. See `PORTING_STATUS.md` Extended Drivers table.
    #[cfg(feature = "drv83xx")]
    pub use drv83xx_rs as drv83xx;
}

pub mod comms;

#[cfg(feature = "commander")]
pub use barefoc_commander as commander;

pub mod voltage;
pub mod utilities;
pub mod dc;

pub use dc::CurrentMirrorSense;

#[cfg(test)]
mod tests {
    #[cfg(feature = "commander")]
    #[test]
    fn test_commander_reexport() {
        use crate::commander::{Commander, VerboseMode};
        let cmd = Commander::<5>::new();
        assert_eq!(cmd.verbose, VerboseMode::user_friendly);
    }

    #[cfg(feature = "encoders")]
    #[test]
    fn test_encoders_reexport() {
        use crate::encoders::EncoderError;
        let err: EncoderError<()> = EncoderError::ParityError;
        assert_eq!(err, EncoderError::ParityError);
    }

    #[cfg(feature = "mt6xxx")]
    #[test]
    fn test_mt6xxx_reexport() {
        use crate::encoders::mt6xxx::{compute_crc6, Mt6701Status};
        assert_eq!(compute_crc6(0, 0), 0);
        let s = Mt6701Status::from_raw(0);
        assert!(s.is_normal());
    }

    #[test]
    fn test_voltage_reexport() {
        use crate::voltage::{GenericVoltageSense, VoltageSense, VoltageSensor};
        struct DummyClock;
        impl barefoc::common::time::Clock for DummyClock {
            fn now_us(&self) -> u32 {
                0
            }
        }
        let vs = VoltageSense::default_with_clock(&DummyClock);
        assert_eq!(vs.get_voltage(), 0.0);
        let gvs = GenericVoltageSense::from_fn(|| 512, 1.0, 0.0, 0.1, 3.3, &DummyClock);
        assert_eq!(VoltageSensor::get_voltage(&gvs), 0.0);
    }

    #[test]
    fn test_utilities_reexport() {
        use crate::utilities::{PreciseAngle, TrapezoidalPlanner};
        let p = PreciseAngle::default();
        assert_eq!(p.get_rotations(), 0);
        let planner = TrapezoidalPlanner::new(10.0, 5.0, None, None);
        assert_eq!(planner.get_target(), barefoc::math::NOT_SET);
    }

    #[test]
    fn test_dc_reexport() {
        use crate::dc::{CurrentMirrorSense, DCDriver, DCDriverParams, DCMotor, DummyCurrentSense, DummySensor};
        use barefoc::traits::AdcPin;

        struct DummyPin;
        impl AdcPin for DummyPin {
            fn read(&mut self) -> u16 {
                100
            }
        }

        struct DummyClock;
        impl barefoc::common::time::Clock for DummyClock {
            fn now_us(&self) -> u32 {
                0
            }
        }
        let params = DCDriverParams::new();
        assert_eq!(params.voltage_power_supply, 12.0);
        let m: DCMotor<'_, DummySensor<DummyClock>, DummyDriver, DummyCurrentSense, DummyClock> =
            DCMotor::new(&DummyClock);
        assert_eq!(m.cfg.pole_pairs, 1);

        let cs: CurrentMirrorSense<DummyPin, DummyClock> =
            CurrentMirrorSense::new(DummyPin, 200.0, 1500.0, 3.3, 12);
        assert_eq!(cs.sign(), 1.0);

        struct DummyDriver(DCDriverParams);
        impl DCDriver for DummyDriver {
            fn init(&mut self) -> Result<(), ()> { Ok(()) }
            fn enable(&mut self) {}
            fn disable(&mut self) {}
            fn set_pwm(&mut self, _voltage: f32) {}
            fn params(&self) -> &DCDriverParams { &self.0 }
            fn params_mut(&mut self) -> &mut DCDriverParams { &mut self.0 }
        }
    }
}
