//! # mt6701::asynch — Asynchronous SPI driver for MT6701 using embedded-hal-async
//!
//! Upstream reference:
//! - [`MagneticSensorMT6701SSI.h`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.h)

use embedded_hal_async::spi::SpiDevice;

use crate::error::Error;
use crate::mt6701::status::{Mt6701Status, Mt6701Telemetry};
use crate::mt6701::{Config, Mt6701Frame, SpiMode};
use crate::{MT6701_CPR, _2PI};

/// MT6701 14-bit magnetic rotary angle encoder driver (async).
pub struct Mt6701Async<SPI> {
    spi: SPI,
    config: Config,
    telemetry: Mt6701Telemetry,
    last_status: Mt6701Status,
    last_good_raw: u16,
}

impl<SPI: SpiDevice> Mt6701Async<SPI> {
    /// Creates a new asynchronous MT6701 driver with default configuration.
    pub fn new(spi: SPI) -> Self {
        Self::with_config(spi, Config::default())
    }

    /// Creates a new asynchronous MT6701 driver with the specified configuration.
    pub fn with_config(spi: SPI, config: Config) -> Self {
        Self {
            spi,
            config,
            telemetry: Mt6701Telemetry::default(),
            last_status: Mt6701Status::default(),
            last_good_raw: 0,
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
        self.telemetry
    }

    /// Returns the status nibble decoded from the most recent valid frame.
    pub fn last_status(&self) -> Mt6701Status {
        self.last_status
    }

    /// Returns the last known good 14-bit raw angle ($0 \dots 16383$).
    pub fn last_good_raw(&self) -> u16 {
        self.last_good_raw
    }

    /// Returns the last known good angle in radians $[0, 2\pi)$.
    pub fn last_good_angle(&self) -> f32 {
        (self.last_good_raw as f32 / MT6701_CPR) * _2PI
    }

    /// Returns `true` if a persistent read error exhausted all retries and raised the sticky fault flag.
    pub fn has_fault(&self) -> bool {
        self.telemetry.fault_flag
    }

    /// Clears the sticky fault flag.
    pub fn clear_fault(&mut self) {
        self.telemetry.clear_fault();
    }

    /// Resets all cumulative telemetry counters and clears the fault flag.
    pub fn reset_telemetry(&mut self) {
        self.telemetry.reset();
    }

    /// Consumes the driver and returns the underlying SPI device.
    pub fn free(self) -> SPI {
        self.spi
    }

    /// Performs a single asynchronous SPI read attempt without retries.
    pub async fn read_frame_attempt(&mut self) -> Result<Mt6701Frame, Error<SPI::Error>> {
        let frame = match self.config.spi_mode {
            SpiMode::Mode1 => {
                let mut buf = [0u8; 3];
                self.spi
                    .read(&mut buf)
                    .await
                    .map_err(Error::Communication)?;
                Mt6701Frame::parse(&buf, SpiMode::Mode1, self.config.crc_init)
            }
            SpiMode::Mode2 => {
                let mut buf = [0u8; 4];
                self.spi
                    .read(&mut buf)
                    .await
                    .map_err(Error::Communication)?;
                Mt6701Frame::parse(&buf, SpiMode::Mode2, self.config.crc_init)
            }
        };

        if !frame.is_crc_valid() {
            self.telemetry.crc_failed = self.telemetry.crc_failed.saturating_add(1);
            return Err(Error::CrcError {
                received: frame.crc_received,
                expected: frame.crc_expected,
            });
        }

        self.telemetry.crc_ok = self.telemetry.crc_ok.saturating_add(1);
        self.last_status = frame.status;
        self.last_good_raw = frame.angle;

        Ok(frame)
    }

    /// Reads an MT6701 frame asynchronously with bounded retries.
    pub async fn read_frame_with_retry(&mut self) -> Result<Mt6701Frame, Error<SPI::Error>> {
        let attempts = self.config.max_retries.max(1);
        for _ in 0..attempts {
            match self.read_frame_attempt().await {
                Ok(frame) => return Ok(frame),
                Err(Error::CrcError { .. }) => continue,
                Err(Error::Communication(_)) => continue,
                Err(Error::CrcExhausted) => unreachable!(),
            }
        }

        self.telemetry.crc_exhausted = self.telemetry.crc_exhausted.saturating_add(1);
        self.telemetry.fault_flag = true;

        Err(Error::CrcExhausted)
    }

    /// Reads the 14-bit raw shaft angle ($0 \dots 16383$) asynchronously with bounded retry.
    pub async fn read_raw(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_frame_with_retry().await.map(|f| f.angle)
    }

    /// Reads the 14-bit raw shaft angle asynchronously, holding the last-known-good angle on failure.
    pub async fn read_raw_angle(&mut self) -> u16 {
        match self.read_frame_with_retry().await {
            Ok(f) => f.angle,
            Err(_) => self.last_good_raw,
        }
    }

    /// Reads the shaft angle in radians $[0, 2\pi)$ asynchronously with bounded retry and last-good hold.
    pub async fn read_angle_radians(&mut self) -> f32 {
        (self.read_raw_angle().await as f32 / MT6701_CPR) * _2PI
    }
}
