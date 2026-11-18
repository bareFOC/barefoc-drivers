//! # drv8311::registers — DRV8311 register map and bitfield representations
//!
//! References: Texas Instruments DRV8311 Datasheet (SLVSFN2B).

// ---------------------------------------------------------------------------
// Register Addresses (0x00 - 0x3F)
// ---------------------------------------------------------------------------

pub const DEV_STS1_ADDR: u8 = 0x00;
pub const OT_STS_ADDR: u8 = 0x04;
pub const SUP_STS_ADDR: u8 = 0x05;
pub const DRV_STS_ADDR: u8 = 0x06;
pub const SYS_STS_ADDR: u8 = 0x07;
pub const PWM_SYNC_PRD_ADDR: u8 = 0x0C;
pub const FLT_MODE_ADDR: u8 = 0x10;
pub const SYSF_CTRL_ADDR: u8 = 0x12;
pub const DRVF_CTRL_ADDR: u8 = 0x13;
pub const FLT_TCTRL_ADDR: u8 = 0x16;
pub const FLT_CLR_ADDR: u8 = 0x17;
pub const PWMG_PERIOD_ADDR: u8 = 0x18;
pub const PWMG_A_DUTY_ADDR: u8 = 0x19;
pub const PWMG_B_DUTY_ADDR: u8 = 0x1A;
pub const PWMG_C_DUTY_ADDR: u8 = 0x1B;
pub const PWM_STATE_ADDR: u8 = 0x1C;
pub const PWMG_CTRL_ADDR: u8 = 0x1D;
pub const PWM_CTRL1_ADDR: u8 = 0x20;
pub const DRV_CTRL_ADDR: u8 = 0x22;
pub const CSA_CTRL_ADDR: u8 = 0x23;
pub const SYS_CTRL_ADDR: u8 = 0x3F;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Required write key (0x5 = 0b101) in bits 14..12 for SYS_CTRL writes.
pub const SYS_CTRL_WRITE_KEY: u8 = 0x5;

/// Register lock state: 1 = locked, 0 = unlocked.
pub const REG_LOCK_LOCKED: bool = true;
pub const REG_LOCK_UNLOCKED: bool = false;

/// Command word to clear latched faults (FLT_CLR bit 0 = 1).
pub const FLT_CLR_CMD: u16 = 0x0001;

// ---------------------------------------------------------------------------
// Bitfield Structs
// ---------------------------------------------------------------------------

/// DEV_STS1: Device Status 1 Register (0x00, Reset = 0x0080)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DevSts1 {
    pub parity_bit: bool,
    pub otp_flt: bool,
    pub reset: bool,
    pub spi_flt: bool,
    pub ocp: bool,
    pub uvp: bool,
    pub ot: bool,
    pub fault: bool,
}

impl DevSts1 {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            otp_flt: (val & (1 << 8)) != 0,
            reset: (val & (1 << 7)) != 0,
            spi_flt: (val & (1 << 6)) != 0,
            ocp: (val & (1 << 5)) != 0,
            uvp: (val & (1 << 2)) != 0,
            ot: (val & (1 << 1)) != 0,
            fault: (val & (1 << 0)) != 0,
        }
    }

    pub const fn from_status_byte(val: u8) -> Self {
        Self {
            parity_bit: false,
            otp_flt: false,
            reset: (val & (1 << 7)) != 0,
            spi_flt: (val & (1 << 6)) != 0,
            ocp: (val & (1 << 5)) != 0,
            uvp: (val & (1 << 2)) != 0,
            ot: (val & (1 << 1)) != 0,
            fault: (val & (1 << 0)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.otp_flt as u16) << 8)
            | ((self.reset as u16) << 7)
            | ((self.spi_flt as u16) << 6)
            | ((self.ocp as u16) << 5)
            | ((self.uvp as u16) << 2)
            | ((self.ot as u16) << 1)
            | (self.fault as u16)
    }
}

/// OT_STS: Over Temperature Status Register (0x04, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OtSts {
    pub parity_bit: bool,
    pub ots_avdd: bool,
    pub otw: bool,
    pub otsd: bool,
}

impl OtSts {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            ots_avdd: (val & (1 << 2)) != 0,
            otw: (val & (1 << 1)) != 0,
            otsd: (val & (1 << 0)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.ots_avdd as u16) << 2)
            | ((self.otw as u16) << 1)
            | (self.otsd as u16)
    }
}

/// SUP_STS: Supply Status Register (0x05, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SupSts {
    pub parity_bit: bool,
    pub csaref_uv: bool,
    pub cp_uv: bool,
    pub avdd_uv: bool,
    pub vinavdd_uv: bool,
}

impl SupSts {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            csaref_uv: (val & (1 << 5)) != 0,
            cp_uv: (val & (1 << 4)) != 0,
            avdd_uv: (val & (1 << 2)) != 0,
            vinavdd_uv: (val & (1 << 0)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.csaref_uv as u16) << 5)
            | ((self.cp_uv as u16) << 4)
            | ((self.avdd_uv as u16) << 2)
            | (self.vinavdd_uv as u16)
    }
}

/// DRV_STS: Driver Status Register (0x06, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DrvSts {
    pub parity_bit: bool,
    pub ocpc_hs: bool,
    pub ocpb_hs: bool,
    pub ocpa_hs: bool,
    pub ocpc_ls: bool,
    pub ocpb_ls: bool,
    pub ocpa_ls: bool,
}

impl DrvSts {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            ocpc_hs: (val & (1 << 6)) != 0,
            ocpb_hs: (val & (1 << 5)) != 0,
            ocpa_hs: (val & (1 << 4)) != 0,
            ocpc_ls: (val & (1 << 2)) != 0,
            ocpb_ls: (val & (1 << 1)) != 0,
            ocpa_ls: (val & (1 << 0)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.ocpc_hs as u16) << 6)
            | ((self.ocpb_hs as u16) << 5)
            | ((self.ocpa_hs as u16) << 4)
            | ((self.ocpc_ls as u16) << 2)
            | ((self.ocpb_ls as u16) << 1)
            | (self.ocpa_ls as u16)
    }
}

/// SYS_STS: System Status Register (0x07, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SysSts {
    pub parity_bit: bool,
    pub otpld_err: bool,
    pub spi_parity: bool,
    pub bus_cnt: bool,
    pub frm_err: bool,
}

impl SysSts {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            otpld_err: (val & (1 << 4)) != 0,
            spi_parity: (val & (1 << 2)) != 0,
            bus_cnt: (val & (1 << 1)) != 0,
            frm_err: (val & (1 << 0)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.otpld_err as u16) << 4)
            | ((self.spi_parity as u16) << 2)
            | ((self.bus_cnt as u16) << 1)
            | (self.frm_err as u16)
    }
}

/// PWM_SYNC_PRD: PWM Sync Period Register (0x0C, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmSyncPrd {
    pub parity_bit: bool,
    pub pwm_sync_prd: u16,
}

impl PwmSyncPrd {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            pwm_sync_prd: val & 0x0FFF,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15) | (self.pwm_sync_prd & 0x0FFF)
    }
}

/// FLT_MODE: Fault Mode Register (0x10, Reset = 0x0115)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FltMode {
    pub parity_bit: bool,
    pub otpflt_mode: bool,
    pub spiflt_mode: bool,
    pub ocp_mode: u8,
    pub uvp_mode: u8,
    pub otsd_mode: u8,
}

impl FltMode {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            otpflt_mode: (val & (1 << 8)) != 0,
            spiflt_mode: (val & (1 << 7)) != 0,
            ocp_mode: ((val >> 4) & 0x07) as u8,
            uvp_mode: ((val >> 2) & 0x03) as u8,
            otsd_mode: (val & 0x03) as u8,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.otpflt_mode as u16) << 8)
            | ((self.spiflt_mode as u16) << 7)
            | (((self.ocp_mode & 0x07) as u16) << 4)
            | (((self.uvp_mode & 0x03) as u16) << 2)
            | ((self.otsd_mode & 0x03) as u16)
    }
}

/// SYSF_CTRL: System Fault Control Register (0x12, Reset = 0x0515)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SysfCtrl {
    pub parity_bit: bool,
    pub otavdd_en: bool,
    pub otw_en: bool,
    pub csarefuv_en: bool,
}

impl SysfCtrl {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            otavdd_en: (val & (1 << 10)) != 0,
            otw_en: (val & (1 << 9)) != 0,
            csarefuv_en: (val & (1 << 5)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        // Bits 8..6 default 4, bit 4 default 1, bit 2 default 1, bit 0 default 1 (0x0115 in lower 9 bits)
        0x0115
            | ((self.parity_bit as u16) << 15)
            | ((self.otavdd_en as u16) << 10)
            | ((self.otw_en as u16) << 9)
            | ((self.csarefuv_en as u16) << 5)
    }
}

/// DRVF_CTRL: Driver Fault Control Register (0x13, Reset = 0x0030)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DrvfCtrl {
    pub parity_bit: bool,
    pub ocp_deg: u8,
    pub ocp_tblank: u8,
    pub ocp_lvl: bool,
}

impl DrvfCtrl {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            ocp_deg: ((val >> 4) & 0x03) as u8,
            ocp_tblank: ((val >> 2) & 0x03) as u8,
            ocp_lvl: (val & 0x01) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | (((self.ocp_deg & 0x03) as u16) << 4)
            | (((self.ocp_tblank & 0x03) as u16) << 2)
            | (self.ocp_lvl as u16)
    }
}

/// FLT_TCTRL: Fault Timing Control Register (0x16, Reset = 0x0003)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FltTctrl {
    pub parity_bit: bool,
    pub slow_tretry: u8,
    pub fast_tretry: u8,
}

impl FltTctrl {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            slow_tretry: ((val >> 2) & 0x03) as u8,
            fast_tretry: (val & 0x03) as u8,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | (((self.slow_tretry & 0x03) as u16) << 2)
            | ((self.fast_tretry & 0x03) as u16)
    }
}

/// FLT_CLR: Fault Clear Register (0x17, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FltClr {
    pub parity_bit: bool,
    pub flt_clr: bool,
}

impl FltClr {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            flt_clr: (val & 0x01) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15) | (self.flt_clr as u16)
    }
}

/// PWMG_PERIOD: PWM_GEN Period Register (0x18, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmgPeriod {
    pub parity_bit: bool,
    pub pwm_prd_out: u16,
}

impl PwmgPeriod {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            pwm_prd_out: val & 0x0FFF,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15) | (self.pwm_prd_out & 0x0FFF)
    }
}

/// PWMG_A_DUTY: PWM_GEN A Duty Register (0x19, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmgDutyA {
    pub parity_bit: bool,
    pub pwm_duty_outa: u16,
}

impl PwmgDutyA {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            pwm_duty_outa: val & 0x0FFF,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15) | (self.pwm_duty_outa & 0x0FFF)
    }
}

/// PWMG_B_DUTY: PWM_GEN B Duty Register (0x1A, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmgDutyB {
    pub parity_bit: bool,
    pub pwm_duty_outb: u16,
}

impl PwmgDutyB {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            pwm_duty_outb: val & 0x0FFF,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15) | (self.pwm_duty_outb & 0x0FFF)
    }
}

/// PWMG_C_DUTY: PWM_GEN C Duty Register (0x1B, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmgDutyC {
    pub parity_bit: bool,
    pub pwm_duty_outc: u16,
}

impl PwmgDutyC {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            pwm_duty_outc: val & 0x0FFF,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15) | (self.pwm_duty_outc & 0x0FFF)
    }
}

/// PWM_STATE: PWM State Register (0x1C, Reset = 0x0777)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmState {
    pub parity_bit: bool,
    pub pwmc_state: u8,
    pub pwmb_state: u8,
    pub pwma_state: u8,
}

impl PwmState {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            pwmc_state: ((val >> 8) & 0x07) as u8,
            pwmb_state: ((val >> 4) & 0x07) as u8,
            pwma_state: (val & 0x07) as u8,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | (((self.pwmc_state & 0x07) as u16) << 8)
            | (((self.pwmb_state & 0x07) as u16) << 4)
            | ((self.pwma_state & 0x07) as u16)
    }
}

/// PWMG_CTRL: PWM_GEN Control Register (0x1D, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmgCtrl {
    pub parity_bit: bool,
    pub pwm_en: bool,
    pub pwm_cntr_mode: u8,
    pub pwm_osc_sync: u8,
    pub spiclk_freq_sync: u8,
    pub spisync_acrcy: u8,
}

impl PwmgCtrl {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            pwm_en: (val & (1 << 10)) != 0,
            pwm_cntr_mode: ((val >> 8) & 0x03) as u8,
            pwm_osc_sync: ((val >> 5) & 0x07) as u8,
            spiclk_freq_sync: ((val >> 2) & 0x07) as u8,
            spisync_acrcy: (val & 0x03) as u8,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.pwm_en as u16) << 10)
            | (((self.pwm_cntr_mode & 0x03) as u16) << 8)
            | (((self.pwm_osc_sync & 0x07) as u16) << 5)
            | (((self.spiclk_freq_sync & 0x07) as u16) << 2)
            | ((self.spisync_acrcy & 0x03) as u16)
    }
}

/// PWM_CTRL1: PWM Control Register 1 (0x20, Reset = 0x0007)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmCtrl1 {
    pub parity_bit: bool,
    pub ssc_dis: bool,
    pub pwm_mode: u8,
}

impl PwmCtrl1 {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            ssc_dis: (val & (1 << 2)) != 0,
            pwm_mode: (val & 0x03) as u8,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.ssc_dis as u16) << 2)
            | ((self.pwm_mode & 0x03) as u16)
    }
}

/// DRV_CTRL: Predriver Control Register (0x22, Reset = 0x0000)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DrvCtrl {
    pub parity_bit: bool,
    pub dlycmp_en: bool,
    pub tdead_ctrl: u8,
    pub slew_rate: u8,
}

impl DrvCtrl {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            dlycmp_en: (val & (1 << 7)) != 0,
            tdead_ctrl: ((val >> 4) & 0x07) as u8,
            slew_rate: (val & 0x03) as u8,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.dlycmp_en as u16) << 7)
            | (((self.tdead_ctrl & 0x07) as u16) << 4)
            | ((self.slew_rate & 0x03) as u16)
    }
}

/// CSA_CTRL: CSA Control Register (0x23, Reset = 0x0008)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CsaCtrl {
    pub parity_bit: bool,
    pub csa_en: bool,
    pub csa_gain: u8,
}

impl CsaCtrl {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            csa_en: (val & (1 << 3)) != 0,
            csa_gain: (val & 0x03) as u8,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | ((self.csa_en as u16) << 3)
            | ((self.csa_gain & 0x03) as u16)
    }
}

/// SYS_CTRL: System Control Register (0x3F, Reset = 0x0000)
///
/// NOTE: Bits 14..12 MUST be set to `0x5` (WRITE_KEY) on any write, otherwise the write is ignored.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SysCtrl {
    pub parity_bit: bool,
    pub write_key: u8,
    pub reg_lock: bool,
    pub spi_pen: bool,
}

impl SysCtrl {
    pub const fn from_bits(val: u16) -> Self {
        Self {
            parity_bit: (val & (1 << 15)) != 0,
            write_key: ((val >> 12) & 0x07) as u8,
            reg_lock: (val & (1 << 7)) != 0,
            spi_pen: (val & (1 << 6)) != 0,
        }
    }

    pub const fn to_bits(&self) -> u16 {
        ((self.parity_bit as u16) << 15)
            | (((self.write_key & 0x07) as u16) << 12)
            | ((self.reg_lock as u16) << 7)
            | ((self.spi_pen as u16) << 6)
    }

    /// Creates a SysCtrl payload with WRITE_KEY injected (0x5).
    pub const fn with_write_key(reg_lock: bool, spi_pen: bool) -> Self {
        Self {
            parity_bit: false,
            write_key: SYS_CTRL_WRITE_KEY,
            reg_lock,
            spi_pen,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dev_sts1_roundtrip() {
        let val = 0x0080; // Reset
        let sts = DevSts1::from_bits(val);
        assert!(sts.reset);
        assert!(!sts.fault);
        assert_eq!(sts.to_bits(), val);

        let sts_all = DevSts1 {
            parity_bit: true,
            otp_flt: true,
            reset: true,
            spi_flt: true,
            ocp: true,
            uvp: true,
            ot: true,
            fault: true,
        };
        assert_eq!(DevSts1::from_bits(sts_all.to_bits()), sts_all);
    }

    #[test]
    fn test_sys_ctrl_write_key() {
        let unlock = SysCtrl::with_write_key(REG_LOCK_UNLOCKED, true);
        assert_eq!(unlock.write_key, 0x5);
        assert!(!unlock.reg_lock);
        assert!(unlock.spi_pen);
        let bits = unlock.to_bits();
        assert_eq!((bits >> 12) & 0x07, 0x5);
        assert_eq!((bits >> 7) & 0x01, 0x0);
        assert_eq!((bits >> 6) & 0x01, 0x1);
    }

    #[test]
    fn test_pwm_state_roundtrip() {
        let ps = PwmState {
            parity_bit: false,
            pwmc_state: 7,
            pwmb_state: 6,
            pwma_state: 0,
        };
        let bits = ps.to_bits();
        assert_eq!(PwmState::from_bits(bits), ps);
    }

    #[test]
    fn test_csa_ctrl_roundtrip() {
        let csa = CsaCtrl {
            parity_bit: false,
            csa_en: true,
            csa_gain: 3, // 2.0 V/A
        };
        let bits = csa.to_bits();
        assert_eq!(CsaCtrl::from_bits(bits), csa);
    }
}
