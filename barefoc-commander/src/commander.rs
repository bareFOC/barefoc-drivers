//! # Commander callback dispatcher
//!
//! Ports [`Commander.h`](source/Arduino-FOC/src/communication/Commander.h)
//! and [`Commander.cpp`](source/Arduino-FOC/src/communication/Commander.cpp)
//! from Arduino-FOC.
//!
//! Provides a zero-allocation single-character command ID callback dispatcher
//! compatible with upstream SimpleFOC commands:
//! - `'?'` : Scan attached command nodes / print labels
//! - `'@'` : Configure verbose mode (`0`: nothing, `1`: on_request, `2`: user_friendly, `3`: machine_readable)
//! - `'#'` : Configure decimal places for floating-point formatting
//! - registered single-character IDs : Dispatched to matching callbacks

#![allow(non_camel_case_types)]

/// Single-character scan command.
pub const CMD_SCAN: char = '?';
/// Single-character verbose mode configuration.
pub const CMD_VERBOSE: char = '@';
/// Single-character decimal places configuration.
pub const CMD_DECIMAL: char = '#';

/// Verbose display mode matching upstream SimpleFOC `VerboseMode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum VerboseMode {
    /// Display nothing (good for telemetry / bandwidth constrained monitoring).
    nothing = 0x00,
    /// Display only numerical outputs on user request.
    on_request = 0x01,
    /// Display human-readable textual messages and prefixes.
    user_friendly = 0x02,
    /// Display machine-readable formatted output.
    machine_readable = 0x03,
}

impl Default for VerboseMode {
    fn default() -> Self {
        Self::user_friendly
    }
}

/// Command callback function pointer.
///
/// Receives the command argument/payload string (everything following the command ID).
pub type CommandCallback = fn(&str);

/// An entry in the Commander callback registry.
#[derive(Clone, Copy)]
pub struct CallbackEntry {
    /// Single ASCII character command ID.
    pub id: char,
    /// Callback function pointer.
    pub callback: CommandCallback,
    /// Static string label printed during scan (`'?'`).
    pub label: &'static str,
}

/// Zero-allocation command dispatcher matching upstream SimpleFOC `Commander`.
///
/// Generic over `N`, the maximum number of registered command callbacks (default 20).
pub struct Commander<const N: usize = 20> {
    callbacks: [Option<CallbackEntry>; N],
    count: usize,
    /// Verbose output mode.
    pub verbose: VerboseMode,
    /// Floating-point decimal places (default 3).
    pub decimal_places: u8,
    /// End-of-line sentinel character (default `'\n'`).
    pub eol: char,
    /// Echo flag (echo incoming characters).
    pub echo: bool,
    /// Set to true when an unknown command or syntax error occurs.
    pub last_error: bool,
}

impl<const N: usize> Default for Commander<N> {
    fn default() -> Self {
        Self {
            callbacks: [const { None }; N],
            count: 0,
            verbose: VerboseMode::user_friendly,
            decimal_places: 3,
            eol: '\n',
            echo: false,
            last_error: false,
        }
    }
}

impl<const N: usize> Commander<N> {
    /// Create a new Commander with default settings.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a command callback with a single-character ID and a label.
    ///
    /// Returns `true` if successfully registered, or `false` if the table is full.
    pub fn add(&mut self, id: char, callback: CommandCallback, label: &'static str) -> bool {
        if self.count < N {
            self.callbacks[self.count] = Some(CallbackEntry { id, callback, label });
            self.count += 1;
            true
        } else {
            false
        }
    }

    /// Number of registered callbacks.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Check if character is an end-of-line sentinel.
    pub fn is_sentinel(&self, ch: char) -> bool {
        ch == self.eol || ch == '\r'
    }

    /// Parse and execute a user command string.
    ///
    /// Unknown commands flag `self.last_error = true` without panicking.
    pub fn run(&mut self, user_input: &str) {
        let input = user_input.trim_matches(|c| c == '\r' || c == '\n');
        if input.is_empty() {
            return;
        }

        let mut chars = input.chars();
        let id = match chars.next() {
            Some(c) => c,
            None => return,
        };
        let payload = chars.as_str().trim_start();

        match id {
            CMD_SCAN => {
                self.last_error = false;
                // Scan command: iterated in run_with_writer when transport is provided
            }
            CMD_VERBOSE => {
                self.last_error = false;
                if !payload.is_empty() {
                    if let Ok(mode) = payload.parse::<u8>() {
                        self.verbose = match mode {
                            0 => VerboseMode::nothing,
                            1 => VerboseMode::on_request,
                            2 => VerboseMode::user_friendly,
                            3 => VerboseMode::machine_readable,
                            _ => self.verbose,
                        };
                    }
                }
            }
            CMD_DECIMAL => {
                self.last_error = false;
                if !payload.is_empty() {
                    if let Ok(places) = payload.parse::<u8>() {
                        self.decimal_places = places;
                    }
                }
            }
            _ => {
                for entry in self.callbacks.iter().flatten() {
                    if entry.id == id {
                        self.last_error = false;
                        (entry.callback)(payload);
                        return;
                    }
                }
                // Unknown command: flag error without panicking
                self.last_error = true;
            }
        }
    }

    /// Parse and execute a command string, writing human/machine-readable output to `writer`.
    #[cfg(feature = "embedded-io")]
    pub fn run_with_writer<W: embedded_io::Write>(
        &mut self,
        user_input: &str,
        writer: &mut W,
    ) -> Result<(), W::Error> {
        let input = user_input.trim_matches(|c| c == '\r' || c == '\n');
        if input.is_empty() {
            return Ok(());
        }

        let mut chars = input.chars();
        let id = match chars.next() {
            Some(c) => c,
            None => return Ok(()),
        };
        let payload = chars.as_str().trim_start();

        match id {
            CMD_SCAN => {
                self.last_error = false;
                for entry in self.callbacks.iter().flatten() {
                    if self.verbose == VerboseMode::machine_readable {
                        writer.write_all(b"?")?;
                    }
                    let mut id_buf = [0u8; 4];
                    let id_str = entry.id.encode_utf8(&mut id_buf);
                    writer.write_all(id_str.as_bytes())?;
                    writer.write_all(b":")?;
                    writer.write_all(entry.label.as_bytes())?;
                    writer.write_all(b"\r\n")?;
                }
            }
            CMD_VERBOSE => {
                self.last_error = false;
                if !payload.is_empty() {
                    if let Ok(mode) = payload.parse::<u8>() {
                        self.verbose = match mode {
                            0 => VerboseMode::nothing,
                            1 => VerboseMode::on_request,
                            2 => VerboseMode::user_friendly,
                            3 => VerboseMode::machine_readable,
                            _ => self.verbose,
                        };
                    }
                }
                if self.verbose != VerboseMode::nothing {
                    if self.verbose == VerboseMode::user_friendly {
                        writer.write_all(b"Verb:")?;
                    } else if self.verbose == VerboseMode::machine_readable {
                        writer.write_all(b"@")?;
                    }
                    match self.verbose {
                        VerboseMode::nothing => {}
                        VerboseMode::on_request | VerboseMode::user_friendly => {
                            writer.write_all(b"on!\r\n")?;
                        }
                        VerboseMode::machine_readable => {
                            writer.write_all(b"machine\r\n")?;
                        }
                    }
                }
            }
            CMD_DECIMAL => {
                self.last_error = false;
                if !payload.is_empty() {
                    if let Ok(places) = payload.parse::<u8>() {
                        self.decimal_places = places;
                    }
                }
                if self.verbose != VerboseMode::nothing {
                    if self.verbose == VerboseMode::user_friendly {
                        writer.write_all(b"Decimal:")?;
                    } else if self.verbose == VerboseMode::machine_readable {
                        writer.write_all(b"#")?;
                    }
                    let mut buf = [0u8; 10];
                    let s = write_u8(self.decimal_places, &mut buf);
                    writer.write_all(s.as_bytes())?;
                    writer.write_all(b"\r\n")?;
                }
            }
            _ => {
                for entry in self.callbacks.iter().flatten() {
                    if entry.id == id {
                        self.last_error = false;
                        if self.verbose == VerboseMode::machine_readable {
                            let mut id_buf = [0u8; 4];
                            let id_str = entry.id.encode_utf8(&mut id_buf);
                            writer.write_all(id_str.as_bytes())?;
                        }
                        (entry.callback)(payload);
                        return Ok(());
                    }
                }

                // Unknown command: output error notice without panicking
                self.last_error = true;
                if self.verbose != VerboseMode::nothing {
                    if self.verbose == VerboseMode::user_friendly {
                        writer.write_all(b"unknown cmd err\r\n")?;
                    } else {
                        writer.write_all(b"err\r\n")?;
                    }
                }
            }
        }
        Ok(())
    }
}

/// Helper function to format a `u8` into a byte buffer without std.
pub fn write_u8<'a>(mut val: u8, buf: &'a mut [u8; 10]) -> &'a str {
    if val == 0 {
        buf[0] = b'0';
        return core::str::from_utf8(&buf[0..1]).unwrap_or("0");
    }
    let mut i = 0;
    let mut temp = [0u8; 10];
    while val > 0 {
        temp[i] = b'0' + (val % 10);
        val /= 10;
        i += 1;
    }
    for j in 0..i {
        buf[j] = temp[i - 1 - j];
    }
    core::str::from_utf8(&buf[0..i]).unwrap_or("")
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicBool, Ordering};

    static CALLBACK_FIRED: AtomicBool = AtomicBool::new(false);

    fn test_callback(arg: &str) {
        if arg == "12.5" {
            CALLBACK_FIRED.store(true, Ordering::SeqCst);
        }
    }

    #[test]
    fn test_commander_defaults() {
        let cmd = Commander::<5>::new();
        assert_eq!(cmd.verbose, VerboseMode::user_friendly);
        assert_eq!(cmd.decimal_places, 3);
        assert_eq!(cmd.count(), 0);
        assert!(!cmd.last_error);
    }

    #[test]
    fn test_commander_add_and_dispatch() {
        let mut cmd = Commander::<5>::new();
        assert!(cmd.add('A', test_callback, "Test A"));
        assert_eq!(cmd.count(), 1);

        CALLBACK_FIRED.store(false, Ordering::SeqCst);
        cmd.run("A12.5");
        assert!(!cmd.last_error);
        assert!(CALLBACK_FIRED.load(Ordering::SeqCst));
    }

    #[test]
    fn test_commander_unknown_command() {
        let mut cmd = Commander::<5>::new();
        cmd.add('M', test_callback, "Motor");

        // Unknown command 'X'
        cmd.run("X100");
        assert!(cmd.last_error);
    }

    #[test]
    fn test_commander_config_commands() {
        let mut cmd = Commander::<5>::new();

        // Test verbose config '@'
        cmd.run("@3");
        assert_eq!(cmd.verbose, VerboseMode::machine_readable);

        cmd.run("@0");
        assert_eq!(cmd.verbose, VerboseMode::nothing);

        // Test decimal config '#'
        cmd.run("#5");
        assert_eq!(cmd.decimal_places, 5);
    }
}
