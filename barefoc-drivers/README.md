# barefoc-drivers

Pure-Rust, `#![no_std]` port of the Arduino SimpleFOC-drivers library: extended
encoders, gate drivers, comms, voltage sense, and utilities for `embedded-hal` v1.0.

Part of the BareFOC project. `#![no_std]` compatible.

## Modules

- `encoders/` — AS5047P, AS5048A, AS5600, MT6701 SSI, generic SSI
- `drivers/` — gate driver re-exports (DRV83xx via `drv83xx-rs`)
- `comms/` — register protocol, motor config bridge
- `voltage/` — `VoltageSense`, `GenericVoltageSense`
- `utilities/` — `PreciseAngle`, `TrapezoidalPlanner`

## Usage

```toml
[dependencies]
barefoc-drivers = { path = "." }
```
