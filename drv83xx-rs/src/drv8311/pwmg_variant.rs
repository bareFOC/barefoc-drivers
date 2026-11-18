//! # drv8311::pwmg_variant — PWM Generation & tSPI variant (DRV8311P)
//!
//! Controls the DRV8311P using 32-bit `tSPI` communication with multi-device
//! addressing (Device IDs 0..3 and broadcast 15) and internal PWM Generation (`PWMG`).
//!
//! References: Texas Instruments DRV8311 Datasheet (SLVSFN2B).

use crate::spi::{
    decode_tspi_response, encode_tspi_frame, verify_even_parity_u16,
};
use crate::{CsaGainProvider, Drv83xxDriver, DrvError};
use super::config::{
    Drv8311CounterMode, Drv8311CsaGain, Drv8311DeadTime, Drv8311PwmMode, Drv8311PwmSync,
    Drv8311SlewRate,
};
use super::registers::*;
use super::status::Drv8311Status;

/// PWM generation and tSPI multi-device variant of the DRV8311 (DRV8311P).
#[derive(Clone, Copy, Debug)]
pub struct Drv8311P<SPI> {
    pub spi: SPI,
    pub device_id: u8,
    pub csa_gain: Drv8311CsaGain,
    pub slew_rate: Drv8311SlewRate,
    pub dead_time: Drv8311DeadTime,
    pub parity_enabled: bool,
    pub is_locked: bool,
    pub period: u16,
}

impl<SPI> Drv8311P<SPI> {
    /// Construct a new `Drv8311P` driver for a given secondary device ID (0..3, or 15).
    pub const fn new(spi: SPI, device_id: u8) -> Self {
        Self {
            spi,
            device_id: device_id & 0x0F,
            csa_gain: Drv8311CsaGain::Gain0V25,
            slew_rate: Drv8311SlewRate::Slew35Vus,
            dead_time: Drv8311DeadTime::DeadTime200ns,
            parity_enabled: true,
            is_locked: true,
            period: 1000, // Default 1000 counts (~20 kHz at 20 MHz internal oscillator)
        }
    }

    /// Builder method to configure the secondary device ID.
    pub const fn with_device_id(mut self, device_id: u8) -> Self {
        self.device_id = device_id & 0x0F;
        self
    }

    /// Builder method to configure the CSA gain.
    pub const fn with_csa_gain(mut self, gain: Drv8311CsaGain) -> Self {
        self.csa_gain = gain;
        self
    }

    /// Builder method to configure the slew rate.
    pub const fn with_slew_rate(mut self, slew: Drv8311SlewRate) -> Self {
        self.slew_rate = slew;
        self
    }

    /// Builder method to configure dead time.
    pub const fn with_dead_time(mut self, dt: Drv8311DeadTime) -> Self {
        self.dead_time = dt;
        self
    }

    /// Builder method to configure SPI parity.
    pub const fn with_parity(mut self, enable: bool) -> Self {
        self.parity_enabled = enable;
        self
    }

    /// Builder method to configure PWM generator period counts (12-bit, up to 4095).
    pub const fn with_period(mut self, period: u16) -> Self {
        self.period = period & 0x0FFF;
        self
    }
}

// ---------------------------------------------------------------------------
// Synchronous embedded-hal implementation
// ---------------------------------------------------------------------------

impl<SPI> Drv8311P<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    /// Reads a register over 32-bit tSPI.
    pub fn read_reg(&mut self, addr: u8) -> Result<(DevSts1, u16), DrvError<SPI::Error>> {
        let tx = encode_tspi_frame(self.device_id, addr, 0x0000, true, self.parity_enabled);
        let mut rx = [0u8; 4];
        self.spi.transfer(&mut rx, &tx)?;
        let (status_byte, data_word) = decode_tspi_response(rx);
        let dev_sts1 = DevSts1::from_status_byte(status_byte);
        if dev_sts1.spi_flt {
            return Err(DrvError::SpiFault);
        }
        if self.parity_enabled && !verify_even_parity_u16(data_word) {
            return Err(DrvError::SpiFault);
        }
        Ok((dev_sts1, data_word & 0x7FFF))
    }

    /// Writes a register directly without checking `is_locked`.
    pub fn write_reg_raw(
        &mut self,
        addr: u8,
        data: u16,
    ) -> Result<(DevSts1, u16), DrvError<SPI::Error>> {
        let tx = encode_tspi_frame(self.device_id, addr, data, false, self.parity_enabled);
        let mut rx = [0u8; 4];
        self.spi.transfer(&mut rx, &tx)?;
        let (status_byte, data_word) = decode_tspi_response(rx);
        let dev_sts1 = DevSts1::from_status_byte(status_byte);
        if dev_sts1.spi_flt {
            return Err(DrvError::SpiFault);
        }
        Ok((dev_sts1, data_word & 0x7FFF))
    }

    /// Writes a register over 32-bit tSPI with lock check.
    pub fn write_reg(&mut self, addr: u8, data: u16) -> Result<(DevSts1, u16), DrvError<SPI::Error>> {
        if self.is_locked && addr != SYS_CTRL_ADDR {
            return Err(DrvError::RegistersLocked);
        }
        self.write_reg_raw(addr, data)
    }

    /// Unlocks control registers by writing `WRITE_KEY = 0x5` to `SYS_CTRL`.
    pub fn unlock_registers(&mut self) -> Result<(), DrvError<SPI::Error>> {
        let val = SysCtrl::with_write_key(REG_LOCK_UNLOCKED, self.parity_enabled).to_bits();
        self.write_reg_raw(SYS_CTRL_ADDR, val)?;
        self.is_locked = false;
        Ok(())
    }

    /// Locks control registers by writing `REG_LOCK = 1` and `WRITE_KEY = 0x5` to `SYS_CTRL`.
    pub fn lock_registers(&mut self) -> Result<(), DrvError<SPI::Error>> {
        let val = SysCtrl::with_write_key(REG_LOCK_LOCKED, self.parity_enabled).to_bits();
        self.write_reg_raw(SYS_CTRL_ADDR, val)?;
        self.is_locked = true;
        Ok(())
    }

    /// Enables internal PWM Generation (`PWMG`) mode.
    ///
    /// * `period`: 12-bit period count (0..4095).
    /// * `center_aligned`: `true` for Center-aligned (Up-Down), `false` for Edge-aligned (Up).
    pub fn enable_pwm_gen(
        &mut self,
        period: u16,
        center_aligned: bool,
    ) -> Result<(), DrvError<SPI::Error>> {
        self.period = period & 0x0FFF;
        self.unlock_registers()?;

        // Configure PWM_CTRL1 to PWM Generation mode (3)
        let pwm_ctrl1 = PwmCtrl1 {
            parity_bit: false,
            ssc_dis: true,
            pwm_mode: Drv8311PwmMode::PwmGen as u8,
        };
        self.write_reg(PWM_CTRL1_ADDR, pwm_ctrl1.to_bits())?;

        // Write period count
        self.write_reg(PWMG_PERIOD_ADDR, self.period)?;

        // Configure PWMG_CTRL: enable PWM, counter mode
        let cntr_mode = if center_aligned {
            Drv8311CounterMode::CenterAligned as u8
        } else {
            Drv8311CounterMode::EdgeAlignedUp as u8
        };
        let pwmg_ctrl = PwmgCtrl {
            parity_bit: false,
            pwm_en: true,
            pwm_cntr_mode: cntr_mode,
            pwm_osc_sync: 0,
            spiclk_freq_sync: 0,
            spisync_acrcy: 0,
        };
        self.write_reg(PWMG_CTRL_ADDR, pwmg_ctrl.to_bits())?;

        // Initialize duty cycles to 0
        self.set_duties(0, 0, 0)?;

        // Enable all 3 phases in PWM mode (0x0777)
        self.write_reg(PWM_STATE_ADDR, 0x0777)?;

        self.lock_registers()?;
        Ok(())
    }

    /// Configures PWM oscillator synchronization (e.g. via `PWM_SYNC` pin).
    pub fn set_pwm_sync(&mut self, sync: Drv8311PwmSync) -> Result<(), DrvError<SPI::Error>> {
        let was_locked = self.is_locked;
        if was_locked {
            self.unlock_registers()?;
        }
        let (_, cur_ctrl) = self.read_reg(PWMG_CTRL_ADDR)?;
        let mut ctrl = PwmgCtrl::from_bits(cur_ctrl);
        ctrl.pwm_osc_sync = sync as u8;
        self.write_reg(PWMG_CTRL_ADDR, ctrl.to_bits())?;
        if was_locked {
            self.lock_registers()?;
        }
        Ok(())
    }

    /// Streams 12-bit phase duty cycle counts (0..period) to the DRV8311.
    pub fn set_duties(
        &mut self,
        duty_a: u16,
        duty_b: u16,
        duty_c: u16,
    ) -> Result<(), DrvError<SPI::Error>> {
        // In PWM Gen mode, duty registers are frequently updated, bypass lock check
        self.write_reg_raw(PWMG_A_DUTY_ADDR, duty_a & 0x0FFF)?;
        self.write_reg_raw(PWMG_B_DUTY_ADDR, duty_b & 0x0FFF)?;
        self.write_reg_raw(PWMG_C_DUTY_ADDR, duty_c & 0x0FFF)?;
        Ok(())
    }

    /// Sets individual phase states (0 = Hi-Z, 1 = Low Forced, 7 = PWM).
    pub fn set_phase_states(
        &mut self,
        state_a: u8,
        state_b: u8,
        state_c: u8,
    ) -> Result<(), DrvError<SPI::Error>> {
        let val = (((state_c & 0x07) as u16) << 8)
            | (((state_b & 0x07) as u16) << 4)
            | ((state_a & 0x07) as u16);
        self.write_reg_raw(PWM_STATE_ADDR, val)?;
        Ok(())
    }

    /// Clears any latched device faults.
    pub fn clear_fault(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.write_reg_raw(FLT_CLR_ADDR, FLT_CLR_CMD)?;
        Ok(())
    }

    /// Reads all diagnostic status registers and returns `Drv8311Status`.
    pub fn get_status(&mut self) -> Result<Drv8311Status, DrvError<SPI::Error>> {
        let (dev_sts1, _) = self.read_reg(DEV_STS1_ADDR)?;
        let (_, ot_sts_bits) = self.read_reg(OT_STS_ADDR)?;
        let (_, sup_sts_bits) = self.read_reg(SUP_STS_ADDR)?;
        let (_, drv_sts_bits) = self.read_reg(DRV_STS_ADDR)?;
        let (_, sys_sts_bits) = self.read_reg(SYS_STS_ADDR)?;
        Ok(Drv8311Status::new(
            dev_sts1,
            OtSts::from_bits(ot_sts_bits),
            SupSts::from_bits(sup_sts_bits),
            DrvSts::from_bits(drv_sts_bits),
            SysSts::from_bits(sys_sts_bits),
        ))
    }

    /// Checks if a hardware fault condition is currently active.
    pub fn is_fault(&mut self) -> Result<bool, DrvError<SPI::Error>> {
        let (status, _) = self.read_reg(DEV_STS1_ADDR)?;
        Ok(status.fault)
    }

    /// Initializes the DRV8311P with configured settings.
    pub fn init(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.unlock_registers()?;

        let drv_ctrl = DrvCtrl {
            parity_bit: false,
            dlycmp_en: false,
            tdead_ctrl: self.dead_time as u8,
            slew_rate: self.slew_rate as u8,
        };
        self.write_reg(DRV_CTRL_ADDR, drv_ctrl.to_bits())?;

        let csa_ctrl = CsaCtrl {
            parity_bit: false,
            csa_en: true,
            csa_gain: self.csa_gain as u8,
        };
        self.write_reg(CSA_CTRL_ADDR, csa_ctrl.to_bits())?;

        self.clear_fault()?;
        self.lock_registers()?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Trait Implementations
// ---------------------------------------------------------------------------

impl<SPI> Drv83xxDriver for Drv8311P<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    type Error = DrvError<SPI::Error>;

    fn clear_fault(&mut self) -> Result<(), Self::Error> {
        self.clear_fault()
    }

    fn is_fault(&mut self) -> Result<bool, Self::Error> {
        self.is_fault()
    }
}

impl<SPI> CsaGainProvider for Drv8311P<SPI> {
    fn csa_gain_v_per_a(&self) -> f32 {
        self.csa_gain.v_per_a()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

    #[test]
    fn test_drv8311p_tspi_read_write() {
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(
                vec![0x91, 0x19, 0x00, 0x00],
                vec![0x00, 0x00, 0x80, 0x08],
            ),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut drv = Drv8311P::new(spi, 2);

        let (sts, data) = drv.read_reg(CSA_CTRL_ADDR).unwrap();
        assert!(!sts.fault);
        assert_eq!(data, 0x0008);
        drv.spi.done();
    }
}
