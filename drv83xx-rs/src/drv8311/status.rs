//! # drv8311::status — DRV8311 status and fault diagnostics
//!
//! References: Texas Instruments DRV8311 Datasheet (SLVSFN2B).

use super::registers::{DevSts1, DrvSts, OtSts, SupSts, SysSts};

/// Comprehensive status and diagnostic information from the DRV8311.
///
/// Combines the summary `DEV_STS1`, over-temperature `OT_STS`, supply `SUP_STS`,
/// per-FET driver overcurrent `DRV_STS`, and system/SPI `SYS_STS` registers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Drv8311Status {
    pub dev_sts1: DevSts1,
    pub ot_sts: OtSts,
    pub sup_sts: SupSts,
    pub drv_sts: DrvSts,
    pub sys_sts: SysSts,
}

impl Drv8311Status {
    pub const fn new(
        dev_sts1: DevSts1,
        ot_sts: OtSts,
        sup_sts: SupSts,
        drv_sts: DrvSts,
        sys_sts: SysSts,
    ) -> Self {
        Self {
            dev_sts1,
            ot_sts,
            sup_sts,
            drv_sts,
            sys_sts,
        }
    }

    /// Master fault summary: `true` if any hardware fault condition is active.
    #[inline]
    pub const fn is_fault(&self) -> bool {
        self.dev_sts1.fault
    }

    /// Overtemperature condition detected (warning or shutdown).
    #[inline]
    pub const fn is_overtemperature(&self) -> bool {
        self.dev_sts1.ot || self.ot_sts.otw || self.ot_sts.otsd || self.ot_sts.ots_avdd
    }

    /// Overtemperature warning threshold exceeded.
    #[inline]
    pub const fn is_overtemperature_warning(&self) -> bool {
        self.ot_sts.otw
    }

    /// Overtemperature shutdown triggered (power stage disabled).
    #[inline]
    pub const fn is_overtemperature_shutdown(&self) -> bool {
        self.ot_sts.otsd
    }

    /// AVDD LDO overtemperature shutdown triggered.
    #[inline]
    pub const fn is_overtemperature_avdd(&self) -> bool {
        self.ot_sts.ots_avdd
    }

    /// Driver overcurrent protection trip summary.
    #[inline]
    pub const fn is_overcurrent(&self) -> bool {
        self.dev_sts1.ocp
    }

    /// Supply undervoltage summary.
    #[inline]
    pub const fn is_undervoltage(&self) -> bool {
        self.dev_sts1.uvp
    }

    /// CSA reference voltage (CSAREF) undervoltage fault.
    #[inline]
    pub const fn is_csaref_undervoltage(&self) -> bool {
        self.sup_sts.csaref_uv
    }

    /// Charge pump undervoltage fault.
    #[inline]
    pub const fn is_charge_pump_undervoltage(&self) -> bool {
        self.sup_sts.cp_uv
    }

    /// AVDD internal LDO output undervoltage fault.
    #[inline]
    pub const fn is_avdd_undervoltage(&self) -> bool {
        self.sup_sts.avdd_uv
    }

    /// VIN_AVDD supply input undervoltage fault.
    #[inline]
    pub const fn is_vinavdd_undervoltage(&self) -> bool {
        self.sup_sts.vinavdd_uv
    }

    /// SPI communication fault summary.
    #[inline]
    pub const fn is_spi_error(&self) -> bool {
        self.dev_sts1.spi_flt || self.sys_sts.spi_parity || self.sys_sts.frm_err
    }

    /// SPI parity check error detected by device.
    #[inline]
    pub const fn is_spi_parity_error(&self) -> bool {
        self.sys_sts.spi_parity
    }

    /// SPI framing error detected (SCLK cycles not a multiple of 16).
    #[inline]
    pub const fn is_spi_frame_error(&self) -> bool {
        self.sys_sts.frm_err
    }

    /// Power-on reset condition active.
    #[inline]
    pub const fn is_power_on_reset(&self) -> bool {
        self.dev_sts1.reset
    }

    // -----------------------------------------------------------------------
    // Per-FET Overcurrent Breakdown
    // -----------------------------------------------------------------------

    /// Phase A High-side MOSFET overcurrent.
    #[inline]
    pub const fn is_overcurrent_phase_a_high(&self) -> bool {
        self.drv_sts.ocpa_hs
    }

    /// Phase A Low-side MOSFET overcurrent.
    #[inline]
    pub const fn is_overcurrent_phase_a_low(&self) -> bool {
        self.drv_sts.ocpa_ls
    }

    /// Phase B High-side MOSFET overcurrent.
    #[inline]
    pub const fn is_overcurrent_phase_b_high(&self) -> bool {
        self.drv_sts.ocpb_hs
    }

    /// Phase B Low-side MOSFET overcurrent.
    #[inline]
    pub const fn is_overcurrent_phase_b_low(&self) -> bool {
        self.drv_sts.ocpb_ls
    }

    /// Phase C High-side MOSFET overcurrent.
    #[inline]
    pub const fn is_overcurrent_phase_c_high(&self) -> bool {
        self.drv_sts.ocpc_hs
    }

    /// Phase C Low-side MOSFET overcurrent.
    #[inline]
    pub const fn is_overcurrent_phase_c_low(&self) -> bool {
        self.drv_sts.ocpc_ls
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_status_diagnostics() {
        let mut status = Drv8311Status::default();
        assert!(!status.is_fault());
        assert!(!status.is_overtemperature());
        assert!(!status.is_overcurrent());

        status.dev_sts1.fault = true;
        status.dev_sts1.ocp = true;
        status.drv_sts.ocpa_hs = true;
        assert!(status.is_fault());
        assert!(status.is_overcurrent());
        assert!(status.is_overcurrent_phase_a_high());
        assert!(!status.is_overcurrent_phase_b_high());

        status.sup_sts.csaref_uv = true;
        assert!(status.is_csaref_undervoltage());

        status.ot_sts.otw = true;
        assert!(status.is_overtemperature());
        assert!(status.is_overtemperature_warning());
        assert!(!status.is_overtemperature_shutdown());

        status.sys_sts.spi_parity = true;
        assert!(status.is_spi_error());
        assert!(status.is_spi_parity_error());
    }
}
