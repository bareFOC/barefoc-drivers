# barefoc-multibus

Portable `#![no_std]`, zero-allocation **descriptor-driven register-map (control table)
framework** for address-based buses (I2C / I3C / SPI register model). This is the third
protocol lane of the BareFOC workspace, alongside `barefoc-commander` (ASCII stream lane)
and `barefoc-fibre` (RPC stream lane).

## Contents

| Module        | Purpose |
|---------------|---------|
| `table`       | `RegMap` server core: register descriptors (offset + width + access mode + write hook), bounded byte-bank storage, register-pointer latch / auto-increment wire semantics, width-checked partial writes, read-only enforcement, self-clearing command hooks. |
| `transport`   | `TargetTransport` trait capturing the address-bus event model (write phase / read phase) plus the ready-made `RegServer` adapter, so bus glue only forwards bytes. |
| `client`      | Controller-side `RegClientI2c` / `RegClientSpi` generic over `embedded-hal` 1.0 master traits (`i2c::I2c`, `spi::SpiDevice`), enabling full server↔client round-trip tests on the host. |

## Upstream Attribution

Register-semantics ported from the Miniata control-table firmware by the Miniata /
BareFOC team:

- `ref/Miniata/firmware/src/control_table.h` — register offsets, access semantics,
  `REG_SYSTEM_CMD` action-trigger convention (`SystemCmd` self-clearing command bytes).
- `ref/Miniata/firmware/src/control_table.cpp` — `writeRegister()` bounds/width
  validation, `checkAndClearCommand()` self-clear semantics, little-endian `float`
  register representation.

Protocol family naming per the Dinnge/Dynamixel *control table* convention
(also seen in CANopen *object dictionary*, Modbus *register map*, Linux *regmap*).
The wire protocol is the register-pointer idiom (pointer latch + auto-increment
read/write) from `ref/Miniata/firmware/src/comm_i2c.cpp`; the C++ `0xFF`
variable-width burst protocol of that file is **intentionally not ported** — it was an
Arduino `Wire` master-talks-to-itself workaround, and the clean pointer idiom covers all
its use cases (see `openspec/changes/add-barefoc-multibus/design.md` D3).

This crate carries **no platform dependencies** (design D1): the embassy/STM32 glue that
implements `TargetTransport` on real bus hardware lives in `barefoc-hal/barefoc-stm32`.

## Concurrency contract (design D4)

Bus IRQ / ISR code must only **buffer** incoming bytes and set an event flag. A single
motor-loop task drains the buffered events into the `RegMap` (via
`TargetTransport::on_write` / `on_read`) under `critical_section::with`. The table is
never touched from ISR context. Write hooks run synchronously inside that drain path:
they must be short and non-blocking, and they **cannot re-enter the map** — the hook
signature (`fn(&mut HandlerCtx, &[u8]) -> HookResult`) aliases neither the map nor the
bank.

## Naming

`RegMap` / `RegEntry` / `AccessMode` / `HookResult` follow the control-table family
naming; no `Foc` prefix is used. Register *offset profiles* (Miniata `REG_*` constants
and identity anchors) live in the application layer; this crate only provides the
generic machinery.
