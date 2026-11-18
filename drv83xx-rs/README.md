# drv83xx-rs

Pure-Rust, `#![no_std]` driver for Texas Instruments DRV83xx 3-phase smart gate drivers (DRV8316, DRV8311, etc.) with Embassy `async` support and seamless BareFOC integration.

---

## Supported Devices

- **DRV8316**: 40V, 8A peak smart gate driver with integrated 16-bit SPI and current sensing.
- **DRV8311**: 3V–20V, 5A peak smart driver with 24-bit SPI / 32-bit tSPI multi-device addressing, internal PWM generation (`PWMG`), and silicon substrate cross-talk decoupling.

---

## Features

- **`#![no_std]` & Zero Heap Allocation**: Safe for deeply embedded microcontrollers (STM32, ESP32, RP2040, etc.).
- **TI 16-Bit & 24-Bit/32-Bit SPI Framing Engines**:
  - 16-bit SPI with XNOR even parity (DRV8316).
  - 24-bit SPI with dual even parity: bit 16 header parity, bit 15 data parity (DRV8311).
  - 32-bit `tSPI` with secondary device ID addressing (0..3, broadcast 15) for multi-motor buses (DRV8311P).
- **Full Register Maps**:
  - DRV8316: 0x00 to 0x0C (`IC_STATUS`, `STATUS_1/2`, `CONTROL_1..6`, `CONTROL_10`).
  - DRV8311: 21 registers from 0x00 to 0x3F (`DEV_STS1`, `OT_STS`, `SUP_STS`, `DRV_STS`, `SYS_STS`, `PWM_SYNC_PRD`, `FLT_MODE`, `SYSF_CTRL`, `DRVF_CTRL`, `FLT_TCTRL`, `FLT_CLR`, `PWMG_PERIOD`, `PWMG_A/B/C_DUTY`, `PWM_STATE`, `PWMG_CTRL`, `PWM_CTRL1`, `DRV_CTRL`, `CSA_CTRL`, and `SYS_CTRL`).
- **Comprehensive Diagnostics**: Strongly-typed `Drv8316Status` and `Drv8311Status` decoders identifying per-FET overcurrents, thermal warning vs. shutdown, charge pump / LDO undervoltage, and SPI parity/framing errors.
- **Substrate Cross-Talk Decoupling Filter**: `Drv8311CurrentSenseFilter` implementing Texas Instruments 3-phase and 2-phase decoupling matrices (Datasheet Section 8.3.10.2).
- **Multi-Variant Support**:
  - DRV8316: `Drv8316S` (SPI), `Drv8316H` (HW pin-strapped), `Drv8316P` (PWM).
  - DRV8311: `Drv8311S` (24-bit SPI), `Drv8311H` (HW pin-strapped), `Drv8311P` (tSPI / PWMG).
- **BareFOC Integrated Bundles**:
  - `Drv8316Driver6PWM` / `Drv8316Driver3PWM` and `Drv8311Driver6PWM` / `Drv8311Driver3PWM` implementing `barefoc::traits::driver::BLDCDriver`.
  - `Drv8311DriverPwmGen`: Zero-timer-pin FOC driver bundle driving motor phase voltages over SPI into internal 12-bit PWM generator duty registers, with optional `PWM_SYNC` synchronization.
  - 1-line `create_current_sense()` helper that automatically calculates $A/V$ gain ($1 / G_{CSA}$) and zero-offset ($V_{REF} / 2$).

---

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
drv83xx-rs = { version = "0.1.0", default-features = false, features = ["barefoc"] }
```

### Feature Flags

| Feature | Description | Default |
|---|---|:---:|
| `barefoc` | Enables BareFOC driver bundles and `create_current_sense` | Yes |
| `std` | Enables standard library for host tests | Yes |
| `async` | Enables `embedded-hal-async` SPI support | No |
| `defmt` | Implements `defmt::Format` on enums and status structs | No |

---

## Usage Examples

### 1. DRV8311 Zero-Timer-Pin FOC Bundle (`Drv8311DriverPwmGen`)

Drive a BLDC motor entirely over SPI without configuring microcontroller PWM timer pins:

```rust,no_run
use drv83xx_rs::{Drv8311CsaGain, Drv8311DriverPwmGen, Drv8311P};

// 1. Create DRV8311P driver on tSPI bus (device ID 0)
let drv = Drv8311P::new(spi_device, 0)
    .with_csa_gain(Drv8311CsaGain::Gain0V25); // 0.25 V/A gain

// 2. Create zero-timer-pin FOC bundle with 1000-count period (~20 kHz)
let mut driver = Drv8311DriverPwmGen::new(drv)
    .with_period(1000)
    .with_center_aligned(true);

// 3. One-line current sensing setup: automatic sensitivity (4.0 A/V) & VREF/2 zero-offset
let current_sense = driver.create_current_sense::<_, _, _, AppClock>(
    3.3, // V_REF = 3.3V
    adc_a,
    adc_b,
    Some(adc_c),
);
```

### 2. DRV8311 Substrate Cross-Talk Decoupling Filter

Clean up shared substrate ground offsets in FOC feedback loops:

```rust,no_run
use drv83xx_rs::Drv8311CurrentSenseFilter;

// Sample raw ADC phase currents
let (raw_ia, raw_ib, raw_ic) = (0.52, -0.24, -0.28);

// Apply TI 3-phase cross-talk decoupling matrix
let (ia, ib, ic) = Drv8311CurrentSenseFilter::correct_3phase(raw_ia, raw_ib, raw_ic);
```

### 3. DRV8316 Integrated BareFOC 6-PWM Bundle

```rust,no_run
use barefoc::drivers::bldc_6pwm::BLDCDriver6PWM;
use drv83xx_rs::{Drv8316CsaGain, Drv8316Driver6PWM, Drv8316S};

let spi_dev = Drv8316S::new(spi_peripheral)
    .with_csa_gain(Drv8316CsaGain::Gain0V25);

let pwm_driver = BLDCDriver6PWM::new(pwm_ah, pwm_al, pwm_bh, pwm_bl, pwm_ch, pwm_cl, None);

let mut driver = Drv8316Driver6PWM::new(spi_dev, pwm_driver)
    .with_fault_pin(fault_pin);

let current_sense = driver.create_current_sense::<_, _, _, AppClock>(
    3.3,
    adc_a,
    adc_b,
    Some(adc_c),
);
```

### 4. Standalone DRV8311S SPI Diagnostics

```rust,no_run
use drv83xx_rs::{Drv8311CsaGain, Drv8311PwmMode, Drv8311S, Drv8311SlewRate};

let mut drv = Drv8311S::new(spi_device)
    .with_pwm_mode(Drv8311PwmMode::Pwm6x)
    .with_slew_rate(Drv8311SlewRate::Slew180Vus)
    .with_csa_gain(Drv8311CsaGain::Gain0V5);

drv.init().expect("DRV8311 init failed");

let status = drv.get_status().unwrap();
if status.is_fault() {
    if status.is_overcurrent_phase_a_high() {
        // Phase A High-side OCP triggered
    }
    if status.is_csaref_undervoltage() {
        // CSAREF pin undervoltage detected
    }
}
```

---

## License

Licensed under MIT.
