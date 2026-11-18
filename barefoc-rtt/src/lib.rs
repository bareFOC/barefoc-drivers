//! # barefoc-rtt
//!
//! Reusable `#![no_std]` byte-stream transport over [SEGGER RTT](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/)
//! up/down channels for the BareFOC protocol lanes.
//!
//! [`RttStream`] implements [`embedded_io::Read`] and [`embedded_io::Write`], so it can be
//! handed directly to transport-agnostic consumers such as
//! `barefoc_commander::LineProcessor::process_stream` without any application-level byte
//! plumbing. The adapter is generic over a writer and a reader, which makes it testable on
//! the host with in-memory mocks and keeps the core crate free of hardware dependencies.
//!
//! ## Application-owned channel setup
//!
//! This crate **never** calls `rtt_init!` and installs no global singleton. The application
//! calls `rtt_init!` exactly once and passes the resulting channel handles to
//! [`RttStream::from_channels`] (feature `rtt-target`) or to [`RttStream::new`] with its own
//! adapters. This keeps coexistence with `panic-rtt-target` under the application's control.
//!
//! ## RTT vs `defmt-rtt` (mutual exclusion)
//!
//! `rtt-target` and `defmt-rtt` both own the global RTT control block and must not be linked
//! into the same firmware image. Pick one per image: this crate is for images using
//! `rtt-target`; images using `defmt-rtt` for logging should not enable the `rtt-target`
//! feature here.
//!
//! ## Separate up channels for console and telemetry
//!
//! RTT up channels in `NoBlockTrim` mode silently drop bytes when the host is not draining
//! them. Route interactive console replies through a dedicated up channel (via
//! [`RttStream`]) and keep periodic `rprintln!`/telemetry on the print up channel so that
//! chatty telemetry cannot truncate command replies.
//!
//! ## Non-blocking read contract
//!
//! [`RttStream::read`] never blocks. When the down channel is empty it returns `Ok(0)`,
//! meaning "no data yet, try again" — **not** end-of-file. Callers should poll it from their
//! control loop. Do not feed a zero-length read into a stateful decoder that treats it as
//! `UnexpectedEof` (Fibre framing needs a separate non-blocking `poll_frame` helper).
//!
//! ## Attribution / provenance
//!
//! No C++ source is ported here; this crate provides the transport layer used by the
//! BareFOC port of [`Commander`](https://github.com/simplefoc/Arduino-FOC/blob/master/src/communication/Commander.cpp).
//! The concrete-channel integration mirrors the ad-hoc RTT console in
//! `examples/stm32h503_dc_servo/src/main.rs`.

#![no_std]
#![deny(missing_docs)]

#[cfg(feature = "embedded-io")]
use embedded_io::{ErrorType, Read, Write};

/// A byte stream over an RTT up channel (writer) and down channel (reader).
///
/// `Up` and `Down` are any types implementing [`embedded_io::Write`] and
/// [`embedded_io::Read`] respectively. In host tests these can be in-memory mocks; on
/// target they are typically the `UpChannel`/`DownChannel` adapters supplied by
/// [`RttStream::from_channels`].
///
/// Constructed **by value**; the application owns `rtt_init!` and channel setup.
#[cfg(feature = "embedded-io")]
pub struct RttStream<Up, Down> {
    up: Up,
    down: Down,
}

#[cfg(feature = "embedded-io")]
impl<Up, Down> RttStream<Up, Down> {
    /// Create an RTT byte stream from a writer and reader supplied by the application.
    ///
    /// The application is responsible for calling `rtt_init!` once (if using `rtt-target`)
    /// and for wrapping its channels in types implementing the `embedded-io` traits.
    pub const fn new(up: Up, down: Down) -> Self {
        Self { up, down }
    }

    /// Borrow both channels as disjoint mutable references.
    ///
    /// Useful when a protocol consumer needs a separate reader and writer (for example
    /// `LineProcessor::process_stream(&mut down, &mut up, handler)`).
    pub fn split(&mut self) -> (&mut Down, &mut Up) {
        (&mut self.down, &mut self.up)
    }

    /// Borrow the up (writer) channel.
    pub fn up(&mut self) -> &mut Up {
        &mut self.up
    }

    /// Borrow the down (reader) channel.
    pub fn down(&mut self) -> &mut Down {
        &mut self.down
    }

    /// Consume the stream and return the underlying channels.
    pub fn into_inner(self) -> (Up, Down) {
        (self.up, self.down)
    }
}

#[cfg(feature = "embedded-io")]
impl<Up, Down> ErrorType for RttStream<Up, Down>
where
    Up: ErrorType,
    Down: ErrorType<Error = Up::Error>,
{
    type Error = Up::Error;
}

#[cfg(feature = "embedded-io")]
impl<Up, Down> Write for RttStream<Up, Down>
where
    Up: Write,
    Down: ErrorType<Error = Up::Error>,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.up.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.up.flush()
    }
}

#[cfg(feature = "embedded-io")]
impl<Up, Down> Read for RttStream<Up, Down>
where
    Up: ErrorType,
    Down: Read<Error = Up::Error>,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // RTT down reads never block; an empty buffer yields Ok(0) meaning
        // "no data yet, try again", not EOF.
        self.down.read(buf)
    }
}

/// Concrete `rtt-target` integration: adapters and constructor.
#[cfg(feature = "rtt-target")]
pub mod target {
    use super::RttStream;
    use embedded_io::{Error, ErrorKind, ErrorType, Read, Write};
    use rtt_target::{DownChannel, UpChannel};

    /// [`embedded_io::Error`] returned by the RTT channel adapters.
    ///
    /// RTT channel reads and writes never fail in a way the `embedded-io` traits can
    /// express; short writes are reported as the number of bytes actually written.
    #[derive(Debug)]
    pub enum RttError {
        /// An RTT channel operation could not be completed.
        Other,
    }

    impl Error for RttError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    /// [`embedded_io::Write`] adapter over a concrete `rtt_target::UpChannel`.
    pub struct RttUp {
        channel: UpChannel,
    }

    impl ErrorType for RttUp {
        type Error = RttError;
    }

    impl Write for RttUp {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            // `UpChannel::write` is non-blocking under NoBlockTrim/NoBlockSkip and
            // returns the number of bytes actually accepted.
            Ok(self.channel.write(buf))
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            // RTT is memory-mapped; there is nothing to flush.
            Ok(())
        }
    }

    /// [`embedded_io::Read`] adapter over a concrete `rtt_target::DownChannel`.
    pub struct RttDown {
        channel: DownChannel,
    }

    impl ErrorType for RttDown {
        type Error = RttError;
    }

    impl Read for RttDown {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            // `DownChannel::read` never blocks and yields 0 when no bytes are available.
            Ok(self.channel.read(buf))
        }
    }

    /// Convenience alias for an [`RttStream`] built from `rtt-target` channels.
    pub type RttChannels = RttStream<RttUp, RttDown>;

    impl RttStream<RttUp, RttDown> {
        /// Build an [`RttStream`] from concrete channels created by the application's
        /// single `rtt_init!` invocation.
        ///
        /// This method does **not** call `rtt_init!` or install any global RTT state.
        ///
        /// ```
        /// # #[cfg(feature = "rtt-target")]
        /// # fn example() {
        /// let channels = rtt_target::rtt_init! {
        ///     up: { 0: { size: 128, name: "Console" } }
        ///     down: { 0: { size: 128, name: "Terminal" } }
        /// };
        /// let stream = barefoc_rtt::RttStream::from_channels(channels.up.0, channels.down.0);
        /// # let _ = stream;
        /// # }
        /// ```
        pub fn from_channels(up: UpChannel, down: DownChannel) -> Self {
            RttStream::new(RttUp { channel: up }, RttDown { channel: down })
        }
    }

    impl RttUp {
        /// Wrap a concrete `UpChannel` in the `embedded-io` adapter.
        pub const fn new(channel: UpChannel) -> Self {
            Self { channel }
        }
    }

    impl RttDown {
        /// Wrap a concrete `DownChannel` in the `embedded-io` adapter.
        pub const fn new(channel: DownChannel) -> Self {
            Self { channel }
        }
    }
}

#[cfg(feature = "rtt-target")]
pub use target::{RttChannels, RttDown, RttError, RttUp};

#[cfg(all(test, feature = "embedded-io"))]
mod tests {
    use super::*;
    use embedded_io::{ErrorKind, Read, Write};

    /// In-memory channel implementing both `Read` (down) and `Write` (up).
    struct MockChannel {
        rx: [u8; 64],
        rx_len: usize,
        rx_pos: usize,
        tx: [u8; 64],
        tx_len: usize,
    }

    impl Default for MockChannel {
        fn default() -> Self {
            Self {
                rx: [0u8; 64],
                rx_len: 0,
                rx_pos: 0,
                tx: [0u8; 64],
                tx_len: 0,
            }
        }
    }

    impl MockChannel {
        fn with_input(bytes: &[u8]) -> Self {
            let mut ch = Self::default();
            ch.rx[..bytes.len()].copy_from_slice(bytes);
            ch.rx_len = bytes.len();
            ch
        }
    }

    impl ErrorType for MockChannel {
        type Error = ErrorKind;
    }

    impl Read for MockChannel {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let available = self.rx_len - self.rx_pos;
            let n = available.min(buf.len());
            buf[..n].copy_from_slice(&self.rx[self.rx_pos..self.rx_pos + n]);
            self.rx_pos += n;
            Ok(n)
        }
    }

    impl Write for MockChannel {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let free = self.tx.len() - self.tx_len;
            let n = free.min(buf.len());
            self.tx[self.tx_len..self.tx_len + n].copy_from_slice(&buf[..n]);
            self.tx_len += n;
            Ok(n)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn round_trip() {
        let mut stream = RttStream::new(MockChannel::default(), MockChannel::with_input(b"abc"));
        let mut buf = [0u8; 8];
        assert_eq!(stream.read(&mut buf).unwrap(), 3);
        assert_eq!(&buf[..3], b"abc");
        stream.write_all(b"reply\n").unwrap();
        let up = stream.up();
        assert_eq!(&up.tx[..up.tx_len], b"reply\n");
    }

    #[test]
    fn empty_read_returns_zero() {
        let mut stream = RttStream::new(MockChannel::default(), MockChannel::default());
        let mut buf = [0u8; 4];
        assert_eq!(stream.read(&mut buf).unwrap(), 0);
    }

    #[test]
    fn partial_reads() {
        let mut stream = RttStream::new(
            MockChannel::default(),
            MockChannel::with_input(b"0123456789"),
        );
        let mut buf = [0u8; 4];
        assert_eq!(stream.read(&mut buf).unwrap(), 4);
        assert_eq!(&buf, b"0123");
        assert_eq!(stream.read(&mut buf).unwrap(), 4);
        assert_eq!(&buf, b"4567");
        assert_eq!(stream.read(&mut buf).unwrap(), 2);
        assert_eq!(&buf[..2], b"89");
        assert_eq!(stream.read(&mut buf).unwrap(), 0);
    }

    #[test]
    fn empty_write_buf_is_noop() {
        let mut stream = RttStream::new(MockChannel::default(), MockChannel::default());
        assert_eq!(stream.write(b"").unwrap(), 0);
    }
}
