# barefoc-encoders

Pure-Rust, `#![no_std]` drivers for rotary position encoders targeting embedded systems via the `embedded-hal` 1.0 trait ecosystem, built for seamless integration with [BareFOC](https://github.com/simplefoc/Arduino-FOC).

## Supported Sensors

| Sensor | Protocol | Resolution | Key Features |
|---|---|---|---|
| **AS5047P / AS5047D** | SPI Mode 1 | 14-bit (16384 CPR) | Dynamic Angle Error Compensation (DAEC), parity validation, `ERRFL` decoding |
| **AS5048A** | SPI Mode 1 | 14-bit (16384 CPR) | Even parity check, AGC & CORDIC diagnostic flags |
| **AS5600** | I²C (`0x36`) | 12-bit (4096 CPR) | Magnet placement detection (`MD`, `ML`, `MH`), AGC monitoring |
| **MT6701 SSI** | SSI Mode 2 | 14-bit (16384 CPR) | 24-bit frame unpacking, status decoding (`MGH`, `MGL`, `PUSH`, `LOS`), 6-bit CRC check |
| **Generic SSI** | SSI | 12..24 bit | Configurable industrial SSI absolute encoder reader with optional Gray code decoding |

All sensor structs implement [`barefoc::traits::sensor::Sensor`], providing zero-boilerplate integration into motor control loops.

---

## 1-Line Setup & Usage Examples

### 1. AS5047P (SPI)

```rust
use barefoc_encoders::As5047p;

// spi_dev is any embedded_hal::spi::SpiDevice in Mode 1
let mut sensor = As5047p::new(spi_dev);

// Link directly to BareFOC motor
motor.link_sensor(&mut sensor);

// Silicon diagnostics
let diags = sensor.read_diagnostics()?;
let errors = sensor.read_errfl()?;
```

### 2. AS5048A (SPI)

```rust
use barefoc_encoders::As5048a;

// spi_dev is any embedded_hal::spi::SpiDevice in Mode 1
let mut sensor = As5048a::new(spi_dev);

motor.link_sensor(&mut sensor);

// Query diagnostics
let diags = sensor.read_diagnostics()?;
```

### 3. AS5600 (I²C)

```rust
use barefoc_encoders::As5600;

// i2c_bus is any embedded_hal::i2c::I2c
let mut sensor = As5600::new(i2c_bus);

motor.link_sensor(&mut sensor);

// Check magnet placement before starting motor
if !sensor.is_magnet_detected()? {
    // Magnet is missing or misaligned
}
```

### 4. MT6701 (SSI / SPI Mode 2)

```rust
use barefoc_encoders::Mt6701Ssi;

// spi_dev is configured in SPI Mode 2 (CPOL=1, CPHA=0)
let mut sensor = Mt6701Ssi::new(spi_dev);

motor.link_sensor(&mut sensor);

// Inspect status flags
let status = sensor.last_status();
if status.los {
    // Tracking loss / loss of signal
}
```

### 5. Generic Industrial SSI (12..24 Bit)

```rust
use barefoc_encoders::GenericSsi;

// 16-bit industrial SSI encoder with Gray code decoding
let mut sensor = GenericSsi::new(spi_dev, 16)?
    .with_gray_code(true);

motor.link_sensor(&mut sensor);
```

---

## Cargo Features

| Feature | Description | Default |
|---|---|---|
| `std` | Enables standard library support | **Yes** |
| `async` | Enables `embedded-hal-async` integration | No |
| `defmt` | Implements `defmt::Format` on status and error types | No |

---

## License

MIT OR Apache-2.0
