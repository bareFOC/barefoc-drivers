# barefoc-fibre

Pure-Rust, `#![no_std]` implementation of the Fibre RPC protocol compatible
with the ODrive Web GUI and `odrivetool`.

Part of the BareFOC project. `#![no_std]` compatible.

## Modules

- `crc` — CRC-8/16 helpers
- `framing` — `PacketWrapper` / `PacketUnwrapper`
- `schema` — Endpoint 0 handler
- `server` — `FibreServer`, `Endpoint`, property handlers
- `odrive` — ODrive GUI mapper

## Usage

```toml
[dependencies]
barefoc-fibre = { path = "." }
```

