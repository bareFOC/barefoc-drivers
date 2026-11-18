//! # drv8311::spi_variant — SPI configuration variant (DRV8311S)
//!
//! Controls and monitors the DRV8311S via 24-bit SPI transactions.
//! Supports both synchronous (`embedded_hal::spi::SpiDevice`) and optional
//! asynchronous (`embedded_hal_async::spi::SpiDevice`) execution.
//!
//! References: Texas Instruments DRV8311 Datasheet (SLVSFN2B).

use crate::spi::{
    decode_spi24_response, encode_spi24_frame, verify_even_parity_u16,
};
use crate::{CsaGainProvider, Drv83xxDriver, DrvError};
use super::config::{Drv8311CsaGain, Drv8311DeadTime, Drv8311PwmMode, Drv8311SlewRate};
use super::registers::*;
use super::status::Drv8311Status;

/// SPI configuration variant of the DRV8311 (DRV8311S).
#[derive(Clone, Copy, Debug)]
pub struct Drv8311S<SPI> {
    pub spi: SPI,
    pub csa_gain: Drv8311CsaGain,
    pub pwm_mode: Drv8311PwmMode,
    pub slew_rate: Drv8311SlewRate,
    pub dead_time: Drv8311DeadTime,
    pub parity_enabled: bool,
    pub is_locked: bool,
}

impl<SPI> Drv8311S<SPI> {
    /// Create a new `Drv8311S` driver with standard defaults.
    pub const fn new(spi: SPI) -> Self {
        Self {
            spi,
            csa_gain: Drv8311CsaGain::Gain0V25,
            pwm_mode: Drv8311PwmMode::Pwm6x,
            slew_rate: Drv8311SlewRate::Slew35Vus,
            dead_time: Drv8311DeadTime::DeadTime200ns,
            parity_enabled: true,
            is_locked: true,
        }
    }

    /// Builder method to configure initial PWM mode.
    pub const fn with_pwm_mode(mut self, mode: Drv8311PwmMode) -> Self {
        self.pwm_mode = mode;
        self
    }

    /// Builder method to configure initial CSA gain.
    pub const fn with_csa_gain(mut self, gain: Drv8311CsaGain) -> Self {
        self.csa_gain = gain;
        self
    }

    /// Builder method to configure initial slew rate.
    pub const fn with_slew_rate(mut self, slew: Drv8311SlewRate) -> Self {
        self.slew_rate = slew;
        self
    }

    /// Builder method to configure initial dead time.
    pub const fn with_dead_time(mut self, dt: Drv8311DeadTime) -> Self {
        self.dead_time = dt;
        self
    }

    /// Builder method to configure SPI parity.
    pub const fn with_parity(mut self, enable: bool) -> Self {
        self.parity_enabled = enable;
        self
    }
}

// ---------------------------------------------------------------------------
// Synchronous embedded-hal implementation
// ---------------------------------------------------------------------------

impl<SPI> Drv8311S<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    /// Reads a register over 24-bit SPI.
    ///
    /// Returns `(DevSts1, data_word)`.
    pub fn read_reg(&mut self, addr: u8) -> Result<(DevSts1, u16), DrvError<SPI::Error>> {
        let tx = encode_spi24_frame(addr, 0x0000, true, self.parity_enabled);
        let mut rx = [0u8; 3];
        self.spi.transfer(&mut rx, &tx)?;
        let (status_byte, data_word) = decode_spi24_response(rx);
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
        let tx = encode_spi24_frame(addr, data, false, self.parity_enabled);
        let mut rx = [0u8; 3];
        self.spi.transfer(&mut rx, &tx)?;
        let (status_byte, data_word) = decode_spi24_response(rx);
        let dev_sts1 = DevSts1::from_status_byte(status_byte);
        if dev_sts1.spi_flt {
            return Err(DrvError::SpiFault);
        }
        Ok((dev_sts1, data_word & 0x7FFF))
    }

    /// Writes a register over 24-bit SPI.
    ///
    /// Enforces register write protection if `is_locked` is true.
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

    /// Enables or disables SPI dual parity checking in device and driver.
    pub fn set_parity_enable(&mut self, enable: bool) -> Result<(), DrvError<SPI::Error>> {
        let val = SysCtrl::with_write_key(self.is_locked, enable).to_bits();
        self.write_reg_raw(SYS_CTRL_ADDR, val)?;
        self.parity_enabled = enable;
        Ok(())
    }

    /// Clears any latched device faults.
    pub fn clear_fault(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.write_reg(FLT_CLR_ADDR, FLT_CLR_CMD)?;
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

    /// Initializes the DRV8311S with configured settings.
    pub fn init(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.unlock_registers()?;

        let pwm_ctrl1 = PwmCtrl1 {
            parity_bit: false,
            ssc_dis: true,
            pwm_mode: self.pwm_mode as u8,
        };
        self.write_reg(PWM_CTRL1_ADDR, pwm_ctrl1.to_bits())?;

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
// Asynchronous embedded-hal-async implementation
// ---------------------------------------------------------------------------

#[cfg(feature = "async")]
impl<SPI> Drv8311S<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice,
{
    /// Reads a register asynchronously over 24-bit SPI.
    pub async fn read_reg_async(&mut self, addr: u8) -> Result<(DevSts1, u16), DrvError<SPI::Error>> {
        let tx = encode_spi24_frame(addr, 0x0000, true, self.parity_enabled);
        let mut rx = [0u8; 3];
        self.spi.transfer(&mut rx, &tx).await?;
        let (status_byte, data_word) = decode_spi24_response(rx);
        let dev_sts1 = DevSts1::from_status_byte(status_byte);
        if dev_sts1.spi_flt {
            return Err(DrvError::SpiFault);
        }
        if self.parity_enabled && !verify_even_parity_u16(data_word) {
            return Err(DrvError::SpiFault);
        }
        Ok((dev_sts1, data_word & 0x7FFF))
    }

    /// Writes a register directly without checking `is_locked` asynchronously.
    pub async fn write_reg_raw_async(
        &mut self,
        addr: u8,
        data: u16,
    ) -> Result<(DevSts1, u16), DrvError<SPI::Error>> {
        let tx = encode_spi24_frame(addr, data, false, self.parity_enabled);
        let mut rx = [0u8; 3];
        self.spi.transfer(&mut rx, &tx).await?;
        let (status_byte, data_word) = decode_spi24_response(rx);
        let dev_sts1 = DevSts1::from_status_byte(status_byte);
        if dev_sts1.spi_flt {
            return Err(DrvError::SpiFault);
        }
        Ok((dev_sts1, data_word & 0x7FFF))
    }

    /// Writes a register asynchronously with lock check.
    pub async fn write_reg_async(
        &mut self,
        addr: u8,
        data: u16,
    ) -> Result<(DevSts1, u16), DrvError<SPI::Error>> {
        if self.is_locked && addr != SYS_CTRL_ADDR {
            return Err(DrvError::RegistersLocked);
        }
        self.write_reg_raw_async(addr, data).await
    }

    /// Unlocks control registers asynchronously.
    pub async fn unlock_registers_async(&mut self) -> Result<(), DrvError<SPI::Error>> {
        let val = SysCtrl::with_write_key(REG_LOCK_UNLOCKED, self.parity_enabled).to_bits();
        self.write_reg_raw_async(SYS_CTRL_ADDR, val).await?;
        self.is_locked = false;
        Ok(())
    }

    /// Locks control registers asynchronously.
    pub async fn lock_registers_async(&mut self) -> Result<(), DrvError<SPI::Error>> {
        let val = SysCtrl::with_write_key(REG_LOCK_LOCKED, self.parity_enabled).to_bits();
        self.write_reg_raw_async(SYS_CTRL_ADDR, val).await?;
        self.is_locked = true;
        Ok(())
    }

    /// Clears any latched device faults asynchronously.
    pub async fn clear_fault_async(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.write_reg_async(FLT_CLR_ADDR, FLT_CLR_CMD).await?;
        Ok(())
    }

    /// Reads all diagnostic status registers asynchronously.
    pub async fn get_status_async(&mut self) -> Result<Drv8311Status, DrvError<SPI::Error>> {
        let (dev_sts1, _) = self.read_reg_async(DEV_STS1_ADDR).await?;
        let (_, ot_sts_bits) = self.read_reg_async(OT_STS_ADDR).await?;
        let (_, sup_sts_bits) = self.read_reg_async(SUP_STS_ADDR).await?;
        let (_, drv_sts_bits) = self.read_reg_async(DRV_STS_ADDR).await?;
        let (_, sys_sts_bits) = self.read_reg_async(SYS_STS_ADDR).await?;
        Ok(Drv8311Status::new(
            dev_sts1,
            OtSts::from_bits(ot_sts_bits),
            SupSts::from_bits(sup_sts_bits),
            DrvSts::from_bits(drv_sts_bits),
            SysSts::from_bits(sys_sts_bits),
        ))
    }

    /// Initializes the DRV8311S asynchronously.
    pub async fn init_async(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.unlock_registers_async().await?;

        let pwm_ctrl1 = PwmCtrl1 {
            parity_bit: false,
            ssc_dis: true,
            pwm_mode: self.pwm_mode as u8,
        };
        self.write_reg_async(PWM_CTRL1_ADDR, pwm_ctrl1.to_bits()).await?;

        let drv_ctrl = DrvCtrl {
            parity_bit: false,
            dlycmp_en: false,
            tdead_ctrl: self.dead_time as u8,
            slew_rate: self.slew_rate as u8,
        };
        self.write_reg_async(DRV_CTRL_ADDR, drv_ctrl.to_bits()).await?;

        let csa_ctrl = CsaCtrl {
            parity_bit: false,
            csa_en: true,
            csa_gain: self.csa_gain as u8,
        };
        self.write_reg_async(CSA_CTRL_ADDR, csa_ctrl.to_bits()).await?;

        self.clear_fault_async().await?;
        self.lock_registers_async().await?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Trait Implementations
// ---------------------------------------------------------------------------

impl<SPI> Drv83xxDriver for Drv8311S<SPI>
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

impl<SPI> CsaGainProvider for Drv8311S<SPI> {
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
    fn test_drv8311s_read_reg() {
        // Read DEV_STS1 (addr 0x00) with parity: TX is [0x81, 0x00, 0x00]
        // Mock returns status 0x00, data 0x0080 (reset bit set, 1 one -> bit 15 is 1 -> 0x8080)
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(
                vec![0x81, 0x00, 0x00],
                vec![0x00, 0x80, 0x80],
            ),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut drv = Drv8311S::new(spi);

        let (sts, data) = drv.read_reg(DEV_STS1_ADDR).unwrap();
        assert!(!sts.fault);
        assert_eq!(data, 0x0080);
        drv.spi.done();
    }

    #[test]
    fn test_drv8311s_write_reg_locked() {
        let spi = SpiMock::<u8>::new(&[]);
        let mut drv = Drv8311S::new(spi);
        assert!(drv.is_locked);
        let err = drv.write_reg(CSA_CTRL_ADDR, 0x0008).unwrap_err();
        assert_eq!(err, DrvError::RegistersLocked);
        drv.spi.done();
    }

    #[test]
    fn test_drv8311s_unlock_and_write() {
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(
                vec![0x7E, 0xD0, 0x40],
                vec![0x00, 0x00, 0x00],
            ),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(
                vec![0x47, 0x80, 0x08],
                vec![0x00, 0x00, 0x00],
            ),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut drv = Drv8311S::new(spi);

        drv.unlock_registers().unwrap();
        assert!(!drv.is_locked);
        drv.write_reg(CSA_CTRL_ADDR, 0x0008).unwrap();
        drv.spi.done();
    }
}
