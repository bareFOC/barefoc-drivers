//! # mt6701 — MagnTek MT6701 14-bit SSI Magnetic Rotary Angle Encoder
//!
//! Upstream reference:
//! - [`MagneticSensorMT6701SSI.h`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.h)
//! - [`MagneticSensorMT6701SSI.cpp`](ref/Miniata/simplefoc-drivers/src/encoders/mt6701/MagneticSensorMT6701SSI.cpp)

pub mod blocking;
pub mod status;

#[cfg(feature = "async")]
pub mod asynch;

pub use blocking::Mt6701;
pub use status::{Mt6701Status, Mt6701Telemetry};

#[cfg(feature = "async")]
pub use asynch::Mt6701Async;

use crate::crc::compute_crc6;

/// SPI mode configuration determining clock polarity, phase, and frame slicing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiMode {
    /// SPI Mode 1 (CPOL = 0, CPHA = 1):
    /// Clock idles low. 24 clock cycles (3 bytes read via `spi.read`).
    /// The 24-bit frame occupies bits [23:0]. Recommended for shared buses with no idle-high window.
    #[default]
    Mode1,
    /// SPI Mode 2 (CPOL = 1, CPHA = 0):
    /// Clock idles high. 32 clock cycles (4 bytes read via `spi.read`).
    /// One leading stale bit; the 24-bit frame occupies bits [24:1].
    Mode2,
}

/// Configuration parameters for MT6701.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// SPI mode determining framing (Mode 1 or Mode 2).
    pub spi_mode: SpiMode,
    /// Initial value for the CRC-6 calculation (default 0).
    pub crc_init: u8,
    /// Maximum number of read attempts before declaring failure and holding last-known-good angle.
    pub max_retries: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            spi_mode: SpiMode::Mode1,
            crc_init: 0,
            max_retries: 3,
        }
    }
}

/// Unpacked MT6701 SSI frame data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Mt6701Frame {
    /// 14-bit raw shaft angle ($0 \dots 16383$).
    pub angle: u16,
    /// 4-bit status flags (`MGH`, `MGL`, `PUSH`, `LOS`).
    pub status: Mt6701Status,
    /// 6-bit CRC checksum received in the frame.
    pub crc_received: u8,
    /// 6-bit CRC checksum calculated over the 18 data bits.
    pub crc_expected: u8,
    /// Full 24-bit aligned frame (`[23:0]`).
    pub raw_frame: u32,
}

impl Mt6701Frame {
    /// Unpacks a 24-bit MT6701 frame from raw SPI buffer according to the given [`SpiMode`] and `crc_init`.
    pub fn parse(buf: &[u8], mode: SpiMode, crc_init: u8) -> Self {
        let raw_frame = match mode {
            SpiMode::Mode1 => {
                ((buf[0] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[2] as u32)
            }
            SpiMode::Mode2 => {
                let raw32 = ((buf[0] as u32) << 24)
                    | ((buf[1] as u32) << 16)
                    | ((buf[2] as u32) << 8)
                    | (buf[3] as u32);
                (raw32 >> 1) & 0x00FF_FFFF
            }
        };

        let data18 = (raw_frame >> 6) & 0x3FFFF;
        let angle = ((raw_frame >> 10) & 0x3FFF) as u16;
        let status_nibble = ((raw_frame >> 6) & 0x0F) as u8;
        let status = Mt6701Status::from_raw(status_nibble);
        let crc_received = (raw_frame & 0x3F) as u8;
        let crc_expected = compute_crc6(data18, crc_init);

        Self {
            angle,
            status,
            crc_received,
            crc_expected,
            raw_frame,
        }
    }

    /// Returns `true` if the received CRC matches the expected CRC checksum.
    #[inline]
    pub const fn is_crc_valid(&self) -> bool {
        self.crc_received == self.crc_expected
    }
}
