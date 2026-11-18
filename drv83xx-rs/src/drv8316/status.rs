//! # drv8316::status — DRV8316 status and fault diagnostics
//!
//! References upstream:
//! - `source/Arduino-FOC-drivers/src/drivers/drv8316/drv8316.h`

use super::registers::{IcStatus, Status1, Status2};

/// Comprehensive status and diagnostic information from the DRV8316.
///
/// Combines the summary `IC_Status`, detailed overcurrent/thermal `Status_1`,
/// and SPI/voltage `Status_2` registers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Drv8316Status {
    pub ic_status: IcStatus,
    pub status_1: Status1,
    pub status_2: Status2,
}

impl Drv8316Status {
    pub const fn new(ic_status: IcStatus, status_1: Status1, status_2: Status2) -> Self {
        Self {
            ic_status,
            status_1,
            status_2,
        }
    }

    /// Device fault summary: `true` if any major fault is asserted
    #[inline]
    pub const fn is_fault(&self) -> bool {
        self.ic_status.fault
    }

    /// Overtemperature warning or shutdown
    #[inline]
    pub const fn is_overtemperature(&self) -> bool {
        self.ic_status.ot
    }

    /// Overcurrent protection trip summary
    #[inline]
    pub const fn is_overcurrent(&self) -> bool {
        self.ic_status.ocp
    }

    /// Supply overvoltage condition on VM
    #[inline]
    pub const fn is_overvoltage(&self) -> bool {
        self.ic_status.ovp
    }

    /// Power-on reset condition
    #[inline]
    pub const fn is_power_on_reset(&self) -> bool {
        self.ic_status.npor
    }

    /// SPI fault summary
    #[inline]
    pub const fn is_spi_error(&self) -> bool {
        self.ic_status.spi_flt
    }

    /// Buck regulator fault summary
    #[inline]
    pub const fn is_buck_error(&self) -> bool {
        self.ic_status.bk_flt
    }

    // Status 1 detailed breakdown

    /// Phase A High-side MOSFET overcurrent
    #[inline]
    pub const fn is_overcurrent_phase_a_high(&self) -> bool {
        self.status_1.ocp_ha
    }

    /// Phase A Low-side MOSFET overcurrent
    #[inline]
    pub const fn is_overcurrent_phase_a_low(&self) -> bool {
        self.status_1.ocp_la
    }

    /// Phase B High-side MOSFET overcurrent
    #[inline]
    pub const fn is_overcurrent_phase_b_high(&self) -> bool {
        self.status_1.ocp_hb
    }

    /// Phase B Low-side MOSFET overcurrent
    #[inline]
    pub const fn is_overcurrent_phase_b_low(&self) -> bool {
        self.status_1.ocp_lb
    }

    /// Phase C High-side MOSFET overcurrent
    #[inline]
    pub const fn is_overcurrent_phase_c_high(&self) -> bool {
        self.status_1.ocp_hc
    }

    /// Phase C Low-side MOSFET overcurrent
    #[inline]
    pub const fn is_overcurrent_phase_c_low(&self) -> bool {
        self.status_1.ocp_lc
    }

    /// Over-temperature thermal shutdown (OTSD)
    #[inline]
    pub const fn is_overtemperature_shutdown(&self) -> bool {
        self.status_1.ots
    }

    /// Over-temperature warning (OTW)
    #[inline]
    pub const fn is_overtemperature_warning(&self) -> bool {
        self.status_1.otw
    }

    // Status 2 detailed breakdown

    /// SPI address fault (accessed non-existent address)
    #[inline]
    pub const fn is_spi_address_error(&self) -> bool {
        self.status_2.spi_addr_flt
    }

    /// SPI clock framing error (non-16 clock cycles while CS active)
    #[inline]
    pub const fn is_spi_clock_error(&self) -> bool {
        self.status_2.spi_sclk_flt
    }

    /// SPI parity check error
    #[inline]
    pub const fn is_spi_parity_error(&self) -> bool {
        self.status_2.spi_parity
    }

    /// Charge pump undervoltage
    #[inline]
    pub const fn is_charge_pump_undervoltage(&self) -> bool {
        self.status_2.vcp_uv
    }

    /// Buck regulator undervoltage
    #[inline]
    pub const fn is_buck_undervoltage(&self) -> bool {
        self.status_2.buck_uv
    }

    /// Buck regulator overcurrent
    #[inline]
    pub const fn is_buck_overcurrent(&self) -> bool {
        self.status_2.buck_ocp
    }

    /// One-time programmable (OTP) memory fault
    #[inline]
    pub const fn is_otp_error(&self) -> bool {
        self.status_2.otp_err
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_status_diagnostics() {
        let ic = IcStatus {
            fault: true,
            ocp: true,
            ot: false,
            ovp: false,
            npor: false,
            spi_flt: false,
            bk_flt: false,
        };
        let s1 = Status1 {
            ocp_ha: true,
            ocp_la: false,
            ocp_hb: false,
            ocp_lb: false,
            ocp_hc: false,
            ocp_lc: false,
            ots: false,
            otw: false,
        };
        let s2 = Status2::default();

        let status = Drv8316Status::new(ic, s1, s2);
        assert!(status.is_fault());
        assert!(status.is_overcurrent());
        assert!(status.is_overcurrent_phase_a_high());
        assert!(!status.is_overcurrent_phase_b_high());
        assert!(!status.is_overtemperature());
        assert!(!status.is_spi_error());
    }
}
