# tmag-rs

Production-grade, `#![no_std]` Texas Instruments TMAG5273 3D Hall-effect sensor driver
with Embassy async DMA support, high-speed 1-byte streaming, online fault recovery,
native SI radians/degrees conversions, and first-class BareFOC 1D/2D joint integration.

Part of the BareFOC project. `#![no_std]` compatible.

## Features

- Full register coverage for all 29 hardware registers (`0x00`..=`0x1C`)
- Dual sync/async APIs via `embedded-hal` 1.0 and `embedded-hal-async` 1.0 (feature `async`)
- High-throughput fast modes: 1-byte stream (`read_stream()`), 8-bit MSB fast mode
- Online fault recovery: `clear_faults()`, `clear_por()`
- First-class SI units: `Angle`, `MagneticField3D` with plane angles
- BareFOC adapters (feature `barefoc`): `Tmag5273Sensor`, `Tmag5273Sensor2D`

## Usage

```toml
[dependencies]
tmag-rs = { path = ".", features = ["barefoc", "async"] }
```
