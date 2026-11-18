//! # Streaming line processor over embedded-io
//!
//! Provides zero-allocation, fixed-size line buffering and streaming execution
//! over `embedded_io::Read` and `embedded_io::Write`.

use core::str::Utf8Error;

/// Line buffering and decoding errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineError {
    /// Line buffer capacity exceeded before newline sentinel was received.
    BufferOverflow,
    /// Line contains invalid UTF-8 byte sequences.
    InvalidUtf8,
}

impl From<Utf8Error> for LineError {
    fn from(_: Utf8Error) -> Self {
        LineError::InvalidUtf8
    }
}

/// Errors occurring during stream read/write/dispatch operations.
#[derive(Debug)]
pub enum StreamError<RE, WE> {
    /// Read error from the transport stream.
    Read(RE),
    /// Write error to the transport stream.
    Write(WE),
    /// Line buffering or decoding error.
    Line(LineError),
}

/// Zero-allocation streaming line buffer and processor.
///
/// Default capacity is 64 bytes (`BUF_SIZE = 64`), which comfortably holds any
/// SimpleFOC serial command without heap allocation.
pub struct LineProcessor<const BUF_SIZE: usize = 64> {
    buf: [u8; BUF_SIZE],
    len: usize,
    /// Echo received characters back to the stream.
    pub echo: bool,
}

impl<const BUF_SIZE: usize> Default for LineProcessor<BUF_SIZE> {
    fn default() -> Self {
        Self {
            buf: [0u8; BUF_SIZE],
            len: 0,
            echo: false,
        }
    }
}

impl<const BUF_SIZE: usize> LineProcessor<BUF_SIZE> {
    /// Create a new line processor with empty buffer.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder method configuring character echo.
    pub fn with_echo(mut self, echo: bool) -> Self {
        self.echo = echo;
        self
    }

    /// Feed a single incoming byte into the line buffer.
    ///
    /// When a line terminator (`\n` or `\r`) is encountered, the completed line
    /// is passed to `on_line` and the internal buffer is reset.
    ///
    /// Returns `Ok(true)` if a line was completed, `Ok(false)` if buffering,
    /// or `Err(LineError)` on overflow or invalid UTF-8.
    pub fn feed_byte<F>(&mut self, byte: u8, mut on_line: F) -> Result<bool, LineError>
    where
        F: FnMut(&str),
    {
        if byte == b'\n' || byte == b'\r' {
            if self.len == 0 {
                // Ignore empty line (e.g. secondary byte of \r\n)
                return Ok(false);
            }
            let slice = &self.buf[..self.len];
            let line = core::str::from_utf8(slice)?;
            on_line(line);
            self.len = 0;
            Ok(true)
        } else {
            if self.len >= BUF_SIZE {
                self.len = 0;
                return Err(LineError::BufferOverflow);
            }
            self.buf[self.len] = byte;
            self.len += 1;
            Ok(false)
        }
    }

    /// Feed a slice of bytes, invoking `on_line` for each completed line.
    ///
    /// Returns the number of completed lines processed.
    pub fn feed_slice<F>(&mut self, bytes: &[u8], mut on_line: F) -> Result<usize, LineError>
    where
        F: FnMut(&str),
    {
        let mut count = 0;
        for &b in bytes {
            if self.feed_byte(b, |line| on_line(line))? {
                count += 1;
            }
        }
        Ok(count)
    }

    /// Poll incoming bytes from `reader`, process lines, and invoke `handler`.
    ///
    /// Reads up to 16 bytes per poll call. Echoes characters if `self.echo` is true.
    #[cfg(feature = "embedded-io")]
    pub fn process_stream<R, W, H>(
        &mut self,
        reader: &mut R,
        writer: &mut W,
        mut handler: H,
    ) -> Result<usize, StreamError<R::Error, W::Error>>
    where
        R: embedded_io::Read,
        W: embedded_io::Write,
        H: FnMut(&str, &mut W) -> Result<(), W::Error>,
    {
        let mut chunk = [0u8; 16];
        let bytes_read = reader.read(&mut chunk).map_err(StreamError::Read)?;
        if bytes_read == 0 {
            return Ok(0);
        }

        let mut lines_processed = 0;
        for i in 0..bytes_read {
            let b = chunk[i];
            if self.echo {
                writer.write_all(&[b]).map_err(StreamError::Write)?;
            }

            let mut handler_err = None;
            let res = self.feed_byte(b, |line| {
                if let Err(e) = handler(line, writer) {
                    handler_err = Some(e);
                }
            });

            if let Some(err) = handler_err {
                return Err(StreamError::Write(err));
            }

            match res {
                Ok(true) => lines_processed += 1,
                Ok(false) => {}
                Err(e) => return Err(StreamError::Line(e)),
            }
        }

        Ok(lines_processed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_feed_single_line() {
        let mut proc = LineProcessor::<32>::new();
        let mut matched = false;

        let input = b"VP0.5\n";
        let res = proc.feed_slice(input, |line| {
            if line == "VP0.5" {
                matched = true;
            }
        });

        assert_eq!(res, Ok(1));
        assert!(matched);
    }

    #[test]
    fn test_crlf_handling() {
        let mut proc = LineProcessor::<32>::new();
        let mut matched = false;

        // \r\n should only trigger one callback invocation
        let input = b"T10.0\r\n";
        let res = proc.feed_slice(input, |line| {
            if line == "T10.0" {
                matched = true;
            }
        });

        assert_eq!(res, Ok(1));
        assert!(matched);
    }

    #[test]
    fn test_buffer_overflow() {
        let mut proc = LineProcessor::<8>::new();
        let input = b"1234567890\n";
        let res = proc.feed_slice(input, |_| {});
        assert_eq!(res, Err(LineError::BufferOverflow));
    }

    #[cfg(feature = "embedded-io")]
    struct TestStream<'a> {
        read_data: &'a [u8],
        read_pos: usize,
        write_buf: [u8; 64],
        write_pos: usize,
    }

    #[cfg(feature = "embedded-io")]
    impl<'a> embedded_io::ErrorType for TestStream<'a> {
        type Error = embedded_io::ErrorKind;
    }

    #[cfg(feature = "embedded-io")]
    impl<'a> embedded_io::Read for TestStream<'a> {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.read_pos >= self.read_data.len() {
                return Ok(0);
            }
            let remaining = self.read_data.len() - self.read_pos;
            let to_read = remaining.min(buf.len());
            buf[..to_read].copy_from_slice(&self.read_data[self.read_pos..self.read_pos + to_read]);
            self.read_pos += to_read;
            Ok(to_read)
        }
    }

    #[cfg(feature = "embedded-io")]
    impl<'a> embedded_io::Write for TestStream<'a> {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let remaining = self.write_buf.len() - self.write_pos;
            let to_write = remaining.min(buf.len());
            self.write_buf[self.write_pos..self.write_pos + to_write].copy_from_slice(&buf[..to_write]);
            self.write_pos += to_write;
            Ok(to_write)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    #[cfg(feature = "embedded-io")]
    fn test_process_stream_with_motor_command() {
        use barefoc::motor::base::{MotorConfig, NullClock};
        use crate::commander::VerboseMode;
        use crate::motor::MotorCommander;

        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);
        let mut proc = LineProcessor::<64>::new();

        let mut reader = TestStream {
            read_data: b"T25.5\n",
            read_pos: 0,
            write_buf: [0u8; 64],
            write_pos: 0,
        };
        let mut writer = TestStream {
            read_data: &[],
            read_pos: 0,
            write_buf: [0u8; 64],
            write_pos: 0,
        };

        let processed = proc.process_stream(&mut reader, &mut writer, |line, w| {
            MotorCommander::handle_with_writer(&mut config, line, w, VerboseMode::user_friendly, 2)
                .map_err(|_| embedded_io::ErrorKind::Other)
        }).unwrap();

        assert_eq!(processed, 1);
        assert_eq!(config.target, 25.5);
        let out = core::str::from_utf8(&writer.write_buf[..writer.write_pos]).unwrap();
        assert_eq!(out, "Target: 25.50\r\n");
    }
}
