//! # mt6701::blocking — Blocking SPI driver for MT6701
//!
//! Upstream reference:
//! - [`MagneticSensorMT6701SSI.h`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.h)
//! - [`MagneticSensorMT6701SSI.cpp`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.cpp)

use core::cell::{Cell, RefCell};
use embedded_hal::spi::SpiDevice;

use crate::error::Error;
use crate::mt6701::status::{Mt6701Status, Mt6701Telemetry};
use crate::mt6701::{Config, Mt6701Frame, SpiMode};
use crate::{MT6701_CPR, _2PI};

/// MT6701 14-bit magnetic rotary angle encoder driver (blocking).
pub struct Mt6701<SPI> {
    spi: RefCell<SPI>,
    config: Config,
    telemetry: Cell<Mt6701Telemetry>,
    last_status: Cell<Mt6701Status>,
    last_good_raw: Cell<u16>,
    #[cfg(feature = "barefoc")]
    pub(crate) base: barefoc::traits::sensor::SensorBase,
}

impl<SPI: SpiDevice> Mt6701<SPI> {
    /// Creates a new MT6701 driver instance with default configuration.
    ///
    /// Defaults:
    /// - Mode: [`SpiMode::Mode1`] (24 clocks, idle-low clock).
    /// - CRC Init: 0.
    /// - Max Retries: 3.
    pub fn new(spi: SPI) -> Self {
        Self::with_config(spi, Config::default())
    }

    /// Creates a new MT6701 driver with the specified configuration.
    pub fn with_config(spi: SPI, config: Config) -> Self {
        Self {
            spi: RefCell::new(spi),
            config,
            telemetry: Cell::new(Mt6701Telemetry::default()),
            last_status: Cell::new(Mt6701Status::default()),
            last_good_raw: Cell::new(0),
            #[cfg(feature = "barefoc")]
            base: barefoc::traits::sensor::SensorBase::new(),
        }
    }

    /// Configures the SPI framing mode (Mode 1 or Mode 2).
    pub fn with_spi_mode(mut self, mode: SpiMode) -> Self {
        self.config.spi_mode = mode;
        self
    }

    /// Configures the CRC-6 initialization seed (default 0).
    pub fn with_crc_init(mut self, crc_init: u8) -> Self {
        self.config.crc_init = crc_init;
        self
    }

    /// Configures the maximum number of read attempts per query (default 3).
    pub fn with_max_retries(mut self, retries: u8) -> Self {
        self.config.max_retries = retries;
        self
    }

    /// Returns the active driver configuration.
    pub fn config(&self) -> Config {
        self.config
    }

    /// Updates the driver configuration.
    pub fn set_config(&mut self, config: Config) {
        self.config = config;
    }

    /// Updates the SPI framing mode.
    pub fn set_spi_mode(&mut self, mode: SpiMode) {
        self.config.spi_mode = mode;
    }

    /// Returns the cumulative telemetry counters.
    pub fn telemetry(&self) -> Mt6701Telemetry {
        self.telemetry.get()
    }

    /// Returns the status nibble decoded from the most recent valid frame.
    pub fn last_status(&self) -> Mt6701Status {
        self.last_status.get()
    }

    /// Returns the last known good 14-bit raw angle ($0 \dots 16383$).
    pub fn last_good_raw(&self) -> u16 {
        self.last_good_raw.get()
    }

    /// Returns the last known good angle in radians $[0, 2\pi)$.
    pub fn last_good_angle(&self) -> f32 {
        (self.last_good_raw.get() as f32 / MT6701_CPR) * _2PI
    }

    /// Returns `true` if a persistent read error exhausted all retries and raised the sticky fault flag.
    pub fn has_fault(&self) -> bool {
        self.telemetry.get().fault_flag
    }

    /// Clears the sticky fault flag.
    pub fn clear_fault(&self) {
        let mut telem = self.telemetry.get();
        telem.clear_fault();
        self.telemetry.set(telem);
    }

    /// Resets all cumulative telemetry counters and clears the fault flag.
    pub fn reset_telemetry(&self) {
        let mut telem = self.telemetry.get();
        telem.reset();
        self.telemetry.set(telem);
    }

    /// Consumes the driver and returns the underlying SPI device.
    pub fn free(self) -> SPI {
        self.spi.into_inner()
    }

    /// Borrows the internal SPI device cell.
    pub fn spi(&self) -> &RefCell<SPI> {
        &self.spi
    }

    /// Performs a single SPI read attempt without retries.
    ///
    /// Reads either 3 bytes (Mode 1) or 4 bytes (Mode 2) via simplex `spi.read()`,
    /// verifies the 6-bit CRC checksum, updates internal telemetry counters, and returns the parsed frame.
    pub fn read_frame_attempt(&self) -> Result<Mt6701Frame, Error<SPI::Error>> {
        let frame = match self.config.spi_mode {
            SpiMode::Mode1 => {
                let mut buf = [0u8; 3];
                self.spi
                    .borrow_mut()
                    .read(&mut buf)
                    .map_err(Error::Communication)?;
                Mt6701Frame::parse(&buf, SpiMode::Mode1, self.config.crc_init)
            }
            SpiMode::Mode2 => {
                let mut buf = [0u8; 4];
                self.spi
                    .borrow_mut()
                    .read(&mut buf)
                    .map_err(Error::Communication)?;
                Mt6701Frame::parse(&buf, SpiMode::Mode2, self.config.crc_init)
            }
        };

        let mut telem = self.telemetry.get();
        if !frame.is_crc_valid() {
            telem.crc_failed = telem.crc_failed.saturating_add(1);
            self.telemetry.set(telem);
            return Err(Error::CrcError {
                received: frame.crc_received,
                expected: frame.crc_expected,
            });
        }

        telem.crc_ok = telem.crc_ok.saturating_add(1);
        self.telemetry.set(telem);
        self.last_status.set(frame.status);
        self.last_good_raw.set(frame.angle);

        Ok(frame)
    }

    /// Reads an MT6701 frame with bounded retries.
    ///
    /// Attempts up to `max_retries` (default 3) read operations:
    /// - If an attempt succeeds, returns `Ok(frame)`.
    /// - If all attempts fail, increments `crc_exhausted`, raises the sticky `fault_flag`,
    ///   and returns `Err(Error::CrcExhausted)`.
    pub fn read_frame_with_retry(&self) -> Result<Mt6701Frame, Error<SPI::Error>> {
        let attempts = self.config.max_retries.max(1);
        for _ in 0..attempts {
            match self.read_frame_attempt() {
                Ok(frame) => return Ok(frame),
                Err(Error::CrcError { .. }) => continue,
                Err(Error::Communication(_)) => continue,
                Err(Error::CrcExhausted) => unreachable!(),
            }
        }

        let mut telem = self.telemetry.get();
        telem.crc_exhausted = telem.crc_exhausted.saturating_add(1);
        telem.fault_flag = true;
        self.telemetry.set(telem);

        Err(Error::CrcExhausted)
    }

    /// Reads the 14-bit raw shaft angle ($0 \dots 16383$) with bounded retry.
    ///
    /// On success returns `Ok(angle)`. If retries are exhausted, returns `Err(Error::CrcExhausted)`.
    pub fn read_raw(&self) -> Result<u16, Error<SPI::Error>> {
        self.read_frame_with_retry().map(|f| f.angle)
    }

    /// Reads the 14-bit raw shaft angle, holding the last-known-good angle on persistent failure.
    ///
    /// If all retry attempts fail, increments `crc_exhausted`, raises `fault_flag`, and
    /// returns `self.last_good_raw()`.
    pub fn read_raw_angle(&self) -> u16 {
        match self.read_frame_with_retry() {
            Ok(f) => f.angle,
            Err(_) => self.last_good_raw.get(),
        }
    }

    /// Reads the shaft angle in radians $[0, 2\pi)$ with bounded retries, holding the last-known-good angle
    /// on persistent failure.
    ///
    /// Guarantees that the angle trajectory into FOC velocity loops remains smooth and never jumps
    /// to an artificial negative value (e.g. -1.0 rad).
    pub fn read_angle_radians(&self) -> f32 {
        (self.read_raw_angle() as f32 / MT6701_CPR) * _2PI
    }
}
