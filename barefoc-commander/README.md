# barefoc-commander

Pure-Rust, `#![no_std]` implementation of the SimpleFOC Commander ASCII serial console
tuning and inspection protocol.

Part of the BareFOC project. `#![no_std]` compatible.

## Modules

- `commander` — `Commander`, `CommandCallback`, `CallbackEntry`, verbose modes
- `io` — `LineProcessor`, `LineError`, `StreamError` (streaming over `embedded-io`)
- `motor` (feature `barefoc`) — `MotorCommander`, `MotorCommandError`

## Usage

```toml
[dependencies]
barefoc-commander = { path = ".", features = ["barefoc"] }
```

`Commander.cpp`, `commands.h`.
