//! # drv8316::registers — DRV8316 register map and bitfield representations
//!
//! References upstream:
//! - `source/Arduino-FOC-drivers/src/drivers/drv8316/drv8316_registers.h`

// Register Addresses
pub const IC_STATUS_ADDR: u8 = 0x00;
pub const STATUS_1_ADDR: u8 = 0x01;
pub const STATUS_2_ADDR: u8 = 0x02;
pub const CONTROL_1_ADDR: u8 = 0x03;
pub const CONTROL_2_ADDR: u8 = 0x04;
pub const CONTROL_3_ADDR: u8 = 0x05;
pub const CONTROL_4_ADDR: u8 = 0x06;
pub const CONTROL_5_ADDR: u8 = 0x07;
pub const CONTROL_6_ADDR: u8 = 0x08;
pub const CONTROL_10_ADDR: u8 = 0x0C;

// Constants
pub const REG_LOCK_LOCK: u8 = 0b110;
pub const REG_LOCK_UNLOCK: u8 = 0b011;
pub const CLR_FAULT_CLR: u8 = 0b1;

/// IC_Status register (Address 0x00)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IcStatus {
    pub fault: bool,
    pub ot: bool,
    pub ovp: bool,
    pub npor: bool,
    pub ocp: bool,
    pub spi_flt: bool,
    pub bk_flt: bool,
}

impl IcStatus {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            fault: (val & (1 << 0)) != 0,
            ot: (val & (1 << 1)) != 0,
            ovp: (val & (1 << 2)) != 0,
            npor: (val & (1 << 3)) != 0,
            ocp: (val & (1 << 4)) != 0,
            spi_flt: (val & (1 << 5)) != 0,
            bk_flt: (val & (1 << 6)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.fault as u8)
            | ((self.ot as u8) << 1)
            | ((self.ovp as u8) << 2)
            | ((self.npor as u8) << 3)
            | ((self.ocp as u8) << 4)
            | ((self.spi_flt as u8) << 5)
            | ((self.bk_flt as u8) << 6)
    }
}

/// Status_1 register (Address 0x01)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status1 {
    pub ocp_la: bool,
    pub ocp_ha: bool,
    pub ocp_lb: bool,
    pub ocp_hb: bool,
    pub ocp_lc: bool,
    pub ocp_hc: bool,
    pub ots: bool,
    pub otw: bool,
}

impl Status1 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            ocp_la: (val & (1 << 0)) != 0,
            ocp_ha: (val & (1 << 1)) != 0,
            ocp_lb: (val & (1 << 2)) != 0,
            ocp_hb: (val & (1 << 3)) != 0,
            ocp_lc: (val & (1 << 4)) != 0,
            ocp_hc: (val & (1 << 5)) != 0,
            ots: (val & (1 << 6)) != 0,
            otw: (val & (1 << 7)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.ocp_la as u8)
            | ((self.ocp_ha as u8) << 1)
            | ((self.ocp_lb as u8) << 2)
            | ((self.ocp_hb as u8) << 3)
            | ((self.ocp_lc as u8) << 4)
            | ((self.ocp_hc as u8) << 5)
            | ((self.ots as u8) << 6)
            | ((self.otw as u8) << 7)
    }
}

/// Status_2 register (Address 0x02)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status2 {
    pub spi_addr_flt: bool,
    pub spi_sclk_flt: bool,
    pub spi_parity: bool,
    pub vcp_uv: bool,
    pub buck_uv: bool,
    pub buck_ocp: bool,
    pub otp_err: bool,
}

impl Status2 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            spi_addr_flt: (val & (1 << 0)) != 0,
            spi_sclk_flt: (val & (1 << 1)) != 0,
            spi_parity: (val & (1 << 2)) != 0,
            vcp_uv: (val & (1 << 3)) != 0,
            buck_uv: (val & (1 << 4)) != 0,
            buck_ocp: (val & (1 << 5)) != 0,
            otp_err: (val & (1 << 6)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.spi_addr_flt as u8)
            | ((self.spi_sclk_flt as u8) << 1)
            | ((self.spi_parity as u8) << 2)
            | ((self.vcp_uv as u8) << 3)
            | ((self.buck_uv as u8) << 4)
            | ((self.buck_ocp as u8) << 5)
            | ((self.otp_err as u8) << 6)
    }
}

/// Control_1 register (Address 0x03)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Control1 {
    pub reg_lock: u8,
}

impl Control1 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            reg_lock: val & 0b111,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        self.reg_lock & 0b111
    }
}

/// Control_2 register (Address 0x04)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Control2 {
    pub clr_flt: bool,
    pub pwm_mode: u8,
    pub slew: u8,
    pub sdo_mode: bool,
}

impl Control2 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            clr_flt: (val & (1 << 0)) != 0,
            pwm_mode: (val >> 1) & 0b11,
            slew: (val >> 3) & 0b11,
            sdo_mode: (val & (1 << 5)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.clr_flt as u8)
            | ((self.pwm_mode & 0b11) << 1)
            | ((self.slew & 0b11) << 3)
            | ((self.sdo_mode as u8) << 5)
    }
}

/// Control_3 register (Address 0x05)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Control3 {
    pub otw_rep: bool,
    pub spi_flt_rep: bool,
    pub ovp_en: bool,
    pub ovp_sel: bool,
    pub pwm_100_duty_sel: bool,
}

impl Control3 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            otw_rep: (val & (1 << 0)) != 0,
            spi_flt_rep: (val & (1 << 1)) != 0,
            ovp_en: (val & (1 << 2)) != 0,
            ovp_sel: (val & (1 << 3)) != 0,
            pwm_100_duty_sel: (val & (1 << 4)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.otw_rep as u8)
            | ((self.spi_flt_rep as u8) << 1)
            | ((self.ovp_en as u8) << 2)
            | ((self.ovp_sel as u8) << 3)
            | ((self.pwm_100_duty_sel as u8) << 4)
    }
}

/// Control_4 register (Address 0x06)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Control4 {
    pub ocp_mode: u8,
    pub ocp_lvl: bool,
    pub ocp_retry: bool,
    pub ocp_deg: u8,
    pub ocp_cbc: bool,
    pub drv_off: bool,
}

impl Control4 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            ocp_mode: val & 0b11,
            ocp_lvl: (val & (1 << 2)) != 0,
            ocp_retry: (val & (1 << 3)) != 0,
            ocp_deg: (val >> 4) & 0b11,
            ocp_cbc: (val & (1 << 6)) != 0,
            drv_off: (val & (1 << 7)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.ocp_mode & 0b11)
            | ((self.ocp_lvl as u8) << 2)
            | ((self.ocp_retry as u8) << 3)
            | ((self.ocp_deg & 0b11) << 4)
            | ((self.ocp_cbc as u8) << 6)
            | ((self.drv_off as u8) << 7)
    }
}

/// Control_5 register (Address 0x07)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Control5 {
    pub csa_gain: u8,
    pub en_asr: bool,
    pub en_aar: bool,
    pub ilim_recir: bool,
}

impl Control5 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            csa_gain: val & 0b11,
            en_asr: (val & (1 << 2)) != 0,
            en_aar: (val & (1 << 3)) != 0,
            ilim_recir: (val & (1 << 6)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.csa_gain & 0b11)
            | ((self.en_asr as u8) << 2)
            | ((self.en_aar as u8) << 3)
            | ((self.ilim_recir as u8) << 6)
    }
}

/// Control_6 register (Address 0x08)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Control6 {
    pub buck_dis: bool,
    pub buck_sel: u8,
    pub buck_cl: bool,
    pub buck_ps_dis: bool,
}

impl Control6 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            buck_dis: (val & (1 << 0)) != 0,
            buck_sel: (val >> 1) & 0b11,
            buck_cl: (val & (1 << 3)) != 0,
            buck_ps_dis: (val & (1 << 4)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.buck_dis as u8)
            | ((self.buck_sel & 0b11) << 1)
            | ((self.buck_cl as u8) << 3)
            | ((self.buck_ps_dis as u8) << 4)
    }
}

/// Control_10 register (Address 0x0C)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Control10 {
    pub dly_target: u8,
    pub dlycmp_en: bool,
}

impl Control10 {
    pub const fn from_bits(val: u8) -> Self {
        Self {
            dly_target: val & 0b1111,
            dlycmp_en: (val & (1 << 4)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u8 {
        (self.dly_target & 0b1111) | ((self.dlycmp_en as u8) << 4)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ic_status_roundtrip() {
        let raw = 0b0110_1011;
        let s = IcStatus::from_bits(raw);
        assert!(s.fault);
        assert!(s.ot);
        assert!(!s.ovp);
        assert!(s.npor);
        assert!(!s.ocp);
        assert!(s.spi_flt);
        assert!(s.bk_flt);
        assert_eq!(s.to_bits(), raw);
    }

    #[test]
    fn test_status1_roundtrip() {
        let raw = 0b1010_0101;
        let s = Status1::from_bits(raw);
        assert!(s.ocp_la);
        assert!(!s.ocp_ha);
        assert!(s.ocp_lb);
        assert!(!s.ocp_hb);
        assert!(!s.ocp_lc);
        assert!(s.ocp_hc);
        assert!(!s.ots);
        assert!(s.otw);
        assert_eq!(s.to_bits(), raw);
    }

    #[test]
    fn test_status2_roundtrip() {
        let raw = 0b0101_1010;
        let s = Status2::from_bits(raw);
        assert!(!s.spi_addr_flt);
        assert!(s.spi_sclk_flt);
        assert!(!s.spi_parity);
        assert!(s.vcp_uv);
        assert!(s.buck_uv);
        assert!(!s.buck_ocp);
        assert!(s.otp_err);
        assert_eq!(s.to_bits(), raw);
    }

    #[test]
    fn test_control_registers_roundtrip() {
        let c1 = Control1 { reg_lock: REG_LOCK_UNLOCK };
        assert_eq!(c1.to_bits(), 0b011);
        assert_eq!(Control1::from_bits(0b011).reg_lock, REG_LOCK_UNLOCK);

        let c2 = Control2 {
            clr_flt: true,
            pwm_mode: 0b10,
            slew: 0b01,
            sdo_mode: true,
        };
        let c2_bits = c2.to_bits();
        assert_eq!(Control2::from_bits(c2_bits), c2);

        let c4 = Control4 {
            ocp_mode: 0b01,
            ocp_lvl: true,
            ocp_retry: false,
            ocp_deg: 0b11,
            ocp_cbc: true,
            drv_off: true,
        };
        let c4_bits = c4.to_bits();
        assert_eq!(Control4::from_bits(c4_bits), c4);

        let c5 = Control5 {
            csa_gain: 0b10,
            en_asr: true,
            en_aar: false,
            ilim_recir: true,
        };
        let c5_bits = c5.to_bits();
        assert_eq!(Control5::from_bits(c5_bits), c5);
    }
}
