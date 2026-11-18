# barefoc-drivers

Official silicon sensor, smart gate driver, and communication transport crates for the [BareFOC](https://github.com/bareFOC/barefoc) motion control ecosystem.

A pure-Rust, `#![no_std]` port of [Arduino-FOC-drivers](https://github.com/simplefoc/Arduino-FOC-drivers) targeting `embedded-hal` v1.0.

## Crates in this Workspace

### Silicon Drivers
- **[`drv83xx-rs`](drv83xx-rs/)**: Texas Instruments DRV83xx smart 3-phase gate driver family (DRV8316 16-bit SPI, DRV8311 24-bit tSPI with cross-talk decoupling).
- **[`mt6xxx-rs`](mt6xxx-rs/)**: MagnTek magnetic angle encoder drivers (MT6701 Mode 1/2 SSI with hardware CRC-6).
- **[`tmag-rs`](tmag-rs/)**: Texas Instruments TMAG5273 3D Hall-effect magnetic sensor with async DMA, 20 kSPS 1-byte streaming, and 2-DOF plane angles.
- **[`barefoc-encoders`](barefoc-encoders/)**: Zero-boilerplate rotary angle sensor drivers (AS5047P, AS5048A, AS5600, MT6701, generic SSI).

### Communication & Protocols
- **[`barefoc-commander`](barefoc-commander/)**: Streaming SimpleFOC ASCII console command line processor over `embedded-io`.
- **[`barefoc-multibus`](barefoc-multibus/)**: Address-based memory-mapped register table server/client protocol (I2C/SPI/I3C).
- **[`barefoc-fibre`](barefoc-fibre/)**: Zero-allocation Fibre RPC protocol server (compatible with ODrive Web GUI).
- **[`barefoc-rtt`](barefoc-rtt/)**: SEGGER RTT byte-stream transport implementing `embedded-io` `Read`/`Write`.
- **[`barefoc-boot`](barefoc-boot/)**: Portable BareBoot/Katapult bootloader mailbox entry cookie helper.

### Meta Crate
- **[`barefoc-drivers`](barefoc-drivers/)**: Umbrella meta-crate providing SimpleFOC register mapping, binary serialization, and re-exporting drivers.
