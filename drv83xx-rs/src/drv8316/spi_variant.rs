//! # drv8316::spi_variant — SPI configuration variant (DRV8316S)
//!
//! Controls and monitors the DRV8316S via 16-bit SPI transactions.
//! Supports both synchronous (`embedded_hal::spi::SpiDevice`) and optional
//! asynchronous (`embedded_hal_async::spi::SpiDevice`) execution.

use crate::spi::{decode_response_bytes, encode_frame_bytes};
use crate::{CsaGainProvider, Drv83xxDriver, DrvError};
use super::config::{Drv8316CsaGain, Drv8316PwmMode, Drv8316SlewRate};
use super::registers::*;
use super::status::Drv8316Status;

/// SPI configuration variant of the DRV8316 (DRV8316S).
#[derive(Clone, Copy, Debug)]
pub struct Drv8316S<SPI> {
    pub spi: SPI,
    pub csa_gain: Drv8316CsaGain,
    pub pwm_mode: Drv8316PwmMode,
    pub slew_rate: Drv8316SlewRate,
    pub is_locked: bool,
}

impl<SPI> Drv8316S<SPI> {
    /// Create a new `Drv8316S` driver with standard defaults.
    pub const fn new(spi: SPI) -> Self {
        Self {
            spi,
            csa_gain: Drv8316CsaGain::Gain0V15,
            pwm_mode: Drv8316PwmMode::Pwm6,
            slew_rate: Drv8316SlewRate::Slew150Vus,
            is_locked: true,
        }
    }

    /// Builder method to configure initial PWM mode.
    pub const fn with_pwm_mode(mut self, mode: Drv8316PwmMode) -> Self {
        self.pwm_mode = mode;
        self
    }

    /// Builder method to configure initial CSA gain.
    pub const fn with_csa_gain(mut self, gain: Drv8316CsaGain) -> Self {
        self.csa_gain = gain;
        self
    }

    /// Builder method to configure initial slew rate.
    pub const fn with_slew_rate(mut self, slew: Drv8316SlewRate) -> Self {
        self.slew_rate = slew;
        self
    }
}

// ---------------------------------------------------------------------------
// Synchronous embedded-hal implementation
// ---------------------------------------------------------------------------

impl<SPI> Drv8316S<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    /// Reads a register over SPI.
    ///
    /// Returns `(IcStatus, data_byte)`.
    pub fn read_reg(&mut self, addr: u8) -> Result<(IcStatus, u8), DrvError<SPI::Error>> {
        let tx = encode_frame_bytes(addr, 0x00, true);
        let mut rx = [0u8; 2];
        self.spi.transfer(&mut rx, &tx)?;
        let (status_byte, data_byte) = decode_response_bytes(rx);
        let ic_status = IcStatus::from_bits(status_byte);
        if ic_status.spi_flt {
            return Err(DrvError::SpiFault);
        }
        Ok((ic_status, data_byte))
    }

    /// Writes a register over SPI.
    ///
    /// Returns `(IcStatus, data_byte)` where `data_byte` is the register content
    /// before the write took effect.
    pub fn write_reg(&mut self, addr: u8, data: u8) -> Result<(IcStatus, u8), DrvError<SPI::Error>> {
        let tx = encode_frame_bytes(addr, data, false);
        let mut rx = [0u8; 2];
        self.spi.transfer(&mut rx, &tx)?;
        let (status_byte, data_byte) = decode_response_bytes(rx);
        let ic_status = IcStatus::from_bits(status_byte);
        if ic_status.spi_flt {
            return Err(DrvError::SpiFault);
        }
        Ok((ic_status, data_byte))
    }

    /// Unlocks DRV8316 control registers for writing.
    pub fn unlock_registers(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.write_reg(CONTROL_1_ADDR, REG_LOCK_UNLOCK)?;
        self.is_locked = false;
        Ok(())
    }

    /// Locks DRV8316 control registers to prevent accidental writes.
    pub fn lock_registers(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.write_reg(CONTROL_1_ADDR, REG_LOCK_LOCK)?;
        self.is_locked = true;
        Ok(())
    }

    /// Clears any latched device faults.
    pub fn clear_fault(&mut self) -> Result<(), DrvError<SPI::Error>> {
        let (_, c2_byte) = self.read_reg(CONTROL_2_ADDR)?;
        let mut c2 = Control2::from_bits(c2_byte);
        c2.clr_flt = true;
        self.write_reg(CONTROL_2_ADDR, c2.to_bits())?;
        Ok(())
    }

    /// Reads full diagnostic status (IC_Status, Status_1, Status_2).
    pub fn get_status(&mut self) -> Result<Drv8316Status, DrvError<SPI::Error>> {
        let (ic_status, _) = self.read_reg(IC_STATUS_ADDR)?;
        let (_, s1_byte) = self.read_reg(STATUS_1_ADDR)?;
        let (_, s2_byte) = self.read_reg(STATUS_2_ADDR)?;
        Ok(Drv8316Status::new(
            ic_status,
            Status1::from_bits(s1_byte),
            Status2::from_bits(s2_byte),
        ))
    }

    /// Sets the PWM mode (6-PWM, 3-PWM, etc.).
    pub fn set_pwm_mode(&mut self, mode: Drv8316PwmMode) -> Result<(), DrvError<SPI::Error>> {
        let (_, c2_byte) = self.read_reg(CONTROL_2_ADDR)?;
        let mut c2 = Control2::from_bits(c2_byte);
        c2.pwm_mode = mode as u8;
        self.write_reg(CONTROL_2_ADDR, c2.to_bits())?;
        self.pwm_mode = mode;
        Ok(())
    }

    /// Sets the MOSFET switching slew rate.
    pub fn set_slew_rate(&mut self, slew: Drv8316SlewRate) -> Result<(), DrvError<SPI::Error>> {
        let (_, c2_byte) = self.read_reg(CONTROL_2_ADDR)?;
        let mut c2 = Control2::from_bits(c2_byte);
        c2.slew = slew as u8;
        self.write_reg(CONTROL_2_ADDR, c2.to_bits())?;
        self.slew_rate = slew;
        Ok(())
    }

    /// Sets the Current Sense Amplifier (CSA) gain.
    pub fn set_csa_gain(&mut self, gain: Drv8316CsaGain) -> Result<(), DrvError<SPI::Error>> {
        let (_, c5_byte) = self.read_reg(CONTROL_5_ADDR)?;
        let mut c5 = Control5::from_bits(c5_byte);
        c5.csa_gain = gain as u8;
        self.write_reg(CONTROL_5_ADDR, c5.to_bits())?;
        self.csa_gain = gain;
        Ok(())
    }

    /// Enables or disables the output stage (DRV_OFF).
    ///
    /// When `off` is `true`, power MOSFETs are disabled (Hi-Z).
    pub fn set_driver_off(&mut self, off: bool) -> Result<(), DrvError<SPI::Error>> {
        let (_, c4_byte) = self.read_reg(CONTROL_4_ADDR)?;
        let mut c4 = Control4::from_bits(c4_byte);
        c4.drv_off = off;
        self.write_reg(CONTROL_4_ADDR, c4.to_bits())?;
        Ok(())
    }

    /// Enables or disables Active Synchronous Rectification (EN_ASR).
    pub fn set_active_synchronous_rectification(&mut self, enable: bool) -> Result<(), DrvError<SPI::Error>> {
        let (_, c5_byte) = self.read_reg(CONTROL_5_ADDR)?;
        let mut c5 = Control5::from_bits(c5_byte);
        c5.en_asr = enable;
        self.write_reg(CONTROL_5_ADDR, c5.to_bits())?;
        Ok(())
    }

    /// Enables or disables Active Asynchronous Rectification (EN_AAR).
    pub fn set_active_asynchronous_rectification(&mut self, enable: bool) -> Result<(), DrvError<SPI::Error>> {
        let (_, c5_byte) = self.read_reg(CONTROL_5_ADDR)?;
        let mut c5 = Control5::from_bits(c5_byte);
        c5.en_aar = enable;
        self.write_reg(CONTROL_5_ADDR, c5.to_bits())?;
        Ok(())
    }

    /// Initializes the DRV8316S device over SPI:
    /// - Unlocks registers
    /// - Clears any existing faults
    /// - Configures PWM mode, slew rate, and CSA gain
    /// - Enables Active Synchronous Rectification
    /// - Enables driver power stage
    pub fn init(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.unlock_registers()?;
        self.clear_fault()?;
        self.set_pwm_mode(self.pwm_mode)?;
        self.set_slew_rate(self.slew_rate)?;
        self.set_csa_gain(self.csa_gain)?;
        self.set_active_synchronous_rectification(true)?;
        self.set_driver_off(false)?;

        let status = self.get_status()?;
        if status.is_fault() {
            return Err(DrvError::HardwareFault);
        }

        Ok(())
    }
}

impl<SPI> Drv83xxDriver for Drv8316S<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    type Error = DrvError<SPI::Error>;

    fn clear_fault(&mut self) -> Result<(), Self::Error> {
        self.clear_fault()
    }

    fn is_fault(&mut self) -> Result<bool, Self::Error> {
        let (ic_status, _) = self.read_reg(IC_STATUS_ADDR)?;
        Ok(ic_status.fault)
    }
}

impl<SPI> CsaGainProvider for Drv8316S<SPI> {
    fn csa_gain_v_per_a(&self) -> f32 {
        self.csa_gain.v_per_a()
    }
}

// ---------------------------------------------------------------------------
// Asynchronous embedded-hal-async implementation
// ---------------------------------------------------------------------------

#[cfg(feature = "async")]
impl<SPI> Drv8316S<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice,
{
    /// Reads a register over SPI asynchronously.
    pub async fn read_reg_async(&mut self, addr: u8) -> Result<(IcStatus, u8), DrvError<SPI::Error>> {
        let tx = encode_frame_bytes(addr, 0x00, true);
        let mut rx = [0u8; 2];
        self.spi.transfer(&mut rx, &tx).await?;
        let (status_byte, data_byte) = decode_response_bytes(rx);
        let ic_status = IcStatus::from_bits(status_byte);
        if ic_status.spi_flt {
            return Err(DrvError::SpiFault);
        }
        Ok((ic_status, data_byte))
    }

    /// Writes a register over SPI asynchronously.
    pub async fn write_reg_async(&mut self, addr: u8, data: u8) -> Result<(IcStatus, u8), DrvError<SPI::Error>> {
        let tx = encode_frame_bytes(addr, data, false);
        let mut rx = [0u8; 2];
        self.spi.transfer(&mut rx, &tx).await?;
        let (status_byte, data_byte) = decode_response_bytes(rx);
        let ic_status = IcStatus::from_bits(status_byte);
        if ic_status.spi_flt {
            return Err(DrvError::SpiFault);
        }
        Ok((ic_status, data_byte))
    }

    /// Unlocks DRV8316 control registers for writing asynchronously.
    pub async fn unlock_registers_async(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.write_reg_async(CONTROL_1_ADDR, REG_LOCK_UNLOCK).await?;
        self.is_locked = false;
        Ok(())
    }

    /// Locks DRV8316 control registers asynchronously.
    pub async fn lock_registers_async(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.write_reg_async(CONTROL_1_ADDR, REG_LOCK_LOCK).await?;
        self.is_locked = true;
        Ok(())
    }

    /// Clears any latched device faults asynchronously.
    pub async fn clear_fault_async(&mut self) -> Result<(), DrvError<SPI::Error>> {
        let (_, c2_byte) = self.read_reg_async(CONTROL_2_ADDR).await?;
        let mut c2 = Control2::from_bits(c2_byte);
        c2.clr_flt = true;
        self.write_reg_async(CONTROL_2_ADDR, c2.to_bits()).await?;
        Ok(())
    }

    /// Reads full diagnostic status asynchronously.
    pub async fn get_status_async(&mut self) -> Result<Drv8316Status, DrvError<SPI::Error>> {
        let (ic_status, _) = self.read_reg_async(IC_STATUS_ADDR).await?;
        let (_, s1_byte) = self.read_reg_async(STATUS_1_ADDR).await?;
        let (_, s2_byte) = self.read_reg_async(STATUS_2_ADDR).await?;
        Ok(Drv8316Status::new(
            ic_status,
            Status1::from_bits(s1_byte),
            Status2::from_bits(s2_byte),
        ))
    }

    /// Sets PWM mode asynchronously.
    pub async fn set_pwm_mode_async(&mut self, mode: Drv8316PwmMode) -> Result<(), DrvError<SPI::Error>> {
        let (_, c2_byte) = self.read_reg_async(CONTROL_2_ADDR).await?;
        let mut c2 = Control2::from_bits(c2_byte);
        c2.pwm_mode = mode as u8;
        self.write_reg_async(CONTROL_2_ADDR, c2.to_bits()).await?;
        self.pwm_mode = mode;
        Ok(())
    }

    /// Sets slew rate asynchronously.
    pub async fn set_slew_rate_async(&mut self, slew: Drv8316SlewRate) -> Result<(), DrvError<SPI::Error>> {
        let (_, c2_byte) = self.read_reg_async(CONTROL_2_ADDR).await?;
        let mut c2 = Control2::from_bits(c2_byte);
        c2.slew = slew as u8;
        self.write_reg_async(CONTROL_2_ADDR, c2.to_bits()).await?;
        self.slew_rate = slew;
        Ok(())
    }

    /// Sets CSA gain asynchronously.
    pub async fn set_csa_gain_async(&mut self, gain: Drv8316CsaGain) -> Result<(), DrvError<SPI::Error>> {
        let (_, c5_byte) = self.read_reg_async(CONTROL_5_ADDR).await?;
        let mut c5 = Control5::from_bits(c5_byte);
        c5.csa_gain = gain as u8;
        self.write_reg_async(CONTROL_5_ADDR, c5.to_bits()).await?;
        self.csa_gain = gain;
        Ok(())
    }

    /// Sets DRV_OFF state asynchronously.
    pub async fn set_driver_off_async(&mut self, off: bool) -> Result<(), DrvError<SPI::Error>> {
        let (_, c4_byte) = self.read_reg_async(CONTROL_4_ADDR).await?;
        let mut c4 = Control4::from_bits(c4_byte);
        c4.drv_off = off;
        self.write_reg_async(CONTROL_4_ADDR, c4.to_bits()).await?;
        Ok(())
    }

    /// Initializes device over SPI asynchronously.
    pub async fn init_async(&mut self) -> Result<(), DrvError<SPI::Error>> {
        self.unlock_registers_async().await?;
        self.clear_fault_async().await?;
        self.set_pwm_mode_async(self.pwm_mode).await?;
        self.set_slew_rate_async(self.slew_rate).await?;
        self.set_csa_gain_async(self.csa_gain).await?;
        self.set_driver_off_async(false).await?;

        let status = self.get_status_async().await?;
        if status.is_fault() {
            return Err(DrvError::HardwareFault);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};
    extern crate std;
    use std::vec;

    #[test]
    fn test_drv8316s_read_reg() {
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x81, 0x00], vec![0x00, 0x55]),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut drv = Drv8316S::new(spi);
        let (status, data) = drv.read_reg(0x00).unwrap();
        assert!(!status.fault);
        assert_eq!(data, 0x55);
        drv.spi.done();
    }

    #[test]
    fn test_drv8316s_write_reg() {
        // Write Control_1 (0x03) with UNLOCK (0b011 = 0x03)
        // Tx: [0x06, 0x03]
        // Rx: [0x00, 0x00]
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x06, 0x03], vec![0x00, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut drv = Drv8316S::new(spi);
        drv.unlock_registers().unwrap();
        assert!(!drv.is_locked);
        drv.spi.done();
    }

    #[test]
    fn test_drv8316s_spi_fault() {
        // Device returns SPI_FLT (bit 5 in IC_STATUS = 0x20)
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::transfer(vec![0x81, 0x00], vec![0x20, 0x00]),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut drv = Drv8316S::new(spi);
        let err = drv.read_reg(0x00).unwrap_err();
        assert_eq!(err, DrvError::SpiFault);
        drv.spi.done();
    }
}
