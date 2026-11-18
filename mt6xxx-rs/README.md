# mt6xxx-rs

High-performance, `#![no_std]` driver for MagnTek magnetic rotary angle encoders
(focusing on the 14-bit MT6701 SSI, extensible to MT6816 / MT6835).

Part of the BareFOC project. `#![no_std]` compatible.

## Features

- RX-only simplex SPI via `spi.read`, compatible with unrouted MOSI
- Dual SPI framing: `SpiMode::Mode1` (24 cycles) and `SpiMode::Mode2` (32 cycles)
- Hardware CRC-6 verification (polynomial X^6 + X + 1)
- Bounded retries with last-known-good angle holding on persistent error
- Async and blocking APIs (`embedded-hal` 1.0 / `embedded-hal-async` 1.0)
- Direct `barefoc::traits::sensor::Sensor` implementation

## Usage

```toml
[dependencies]
mt6xxx-rs = { path = "." }
```
