# barefoc-drivers

Peripheral drivers, smart gate driver abstractions, and rotary angle sensor implementations for the [BareFOC](https://github.com/bareFOC/bareFOC) ecosystem.

This workspace provides `#![no_std]` drivers ported from [Arduino-FOC-drivers](https://github.com/simplefoc/Arduino-FOC-drivers) and standalone silicon driver crates for embedded motion control applications using `embedded-hal` v1.0.

---

## Workspace Structure

```
barefoc-drivers/
├── barefoc-drivers/    # Extended drivers port (encoders, gate drivers, DC motors, utilities)
├── drv83xx-rs/         # Texas Instruments DRV8316 & DRV8311 smart gate driver implementations
├── mt6xxx-rs/          # MagnTek MT6701 magnetic rotary angle sensor driver
└── tmag-rs/            # Texas Instruments TMAG5273 3D Hall-effect sensor driver
```

---

## Crates Overview

### 1. `barefoc-drivers` (Extended Drivers Meta-Crate)
Port of core `Arduino-FOC-drivers` modules:
- **`encoders`**: Direct `Sensor` trait implementations for rotary angle ICs:
  - `AS5600`: 12-bit I2C magnetic encoder with status and gain diagnostics.
  - `AS5048A` / `AS5048B`: 14-bit SPI/I2C angle sensor with parity checking.
  - `AS5047P` / `AS5047D`: 14-bit SPI magnetic encoder with DAEC dynamic compensation.
  - `MT6701`: 14-bit SSI/I2C magnetic encoder with CRC-6 framing.
  - `GenericSSI`: Configurable SSI protocol reader.
- **`drivers`**: Gate driver re-exports and configuration helpers.
- **`dc`**: Brushless DC and brushed DC motor drivers (`DCDriver2PWM`), closed/open-loop DC motor control (`DCMotor`), and integrated current mirror sensing (`CurrentMirrorSense`).
- **`voltage`**: Bus voltage sensing abstractions (`VoltageSense`, `GenericVoltageSense`).
- **`utilities`**: Trajectory profiling (`TrapezoidalPlanner`) and continuous multi-turn tracking (`PreciseAngle`).

### 2. `drv83xx-rs` (Smart Gate Drivers)
Dedicated driver library for Texas Instruments smart gate driver ICs:
- **DRV8316**: 3-phase gate driver with integrated MOSFETs and current sense amplifiers over 16-bit SPI.
- **DRV8311**: 3-phase gate driver supporting 24-bit SPI and 32-bit tSPI modes, substrate cross-talk decoupling, and hardware PWMG configurations.
- Integrates with BareFOC driver bundles (`BLDCDriver3PWM`, `BLDCDriver6PWM`) and current sense calibration.

### 3. `mt6xxx-rs` (MagnTek Magnetic Encoders)
Optimized driver for the MagnTek MT6701 rotary encoder:
- Synchronous Serial Interface (SSI) simplex SPI reader with Mode 1 / Mode 2 framing.
- Hardware CRC-6 verification and bounded retry with hold of last-known-good angle.
- Implements `barefoc::traits::sensor::Sensor` with zero allocation.

### 4. `tmag-rs` (TI 3D Hall-Effect Sensor)
Dedicated driver for the Texas Instruments TMAG5273 3D Hall-effect sensor:
- Configurable $X$, $Y$, $Z$ magnetic axis sampling and plane angle computation.
- Support for async DMA streaming, fast 1-byte read mode (up to 20 kSPS), and wake-and-sleep modes.
- Provides BareFOC `Sensor` and `Sensor2D` trait adapters.

---

## Usage

Add the desired crate to your `Cargo.toml`:

```toml
[dependencies]
# Extended drivers meta-crate
barefoc-drivers = { version = "0.1", default-features = false }

# Or individual standalone drivers
drv83xx-rs = { version = "0.1", default-features = false, features = ["drv8311"] }
mt6xxx-rs = { version = "0.1", default-features = false, features = ["barefoc"] }
tmag-rs = { version = "0.1", default-features = false, features = ["barefoc"] }
```

---

## Testing

Host unit tests verify register configurations, CRC calculations, and sensor decoding:

```bash
cargo test -p barefoc-drivers
cargo test -p drv83xx-rs
cargo test -p mt6xxx-rs
cargo test -p tmag-rs
```

---

## License

This project is licensed under the MIT License. See individual crate directories for license files.
