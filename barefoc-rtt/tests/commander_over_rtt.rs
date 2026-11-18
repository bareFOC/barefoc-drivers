//! Integration test: `barefoc_commander::LineProcessor::process_stream` over [`RttStream`].

use barefoc_rtt::RttStream;
use embedded_io::{ErrorKind, ErrorType, Read, Write};

struct MockChannel {
    rx: [u8; 128],
    rx_len: usize,
    rx_pos: usize,
    tx: [u8; 128],
    tx_len: usize,
}

impl Default for MockChannel {
    fn default() -> Self {
        Self {
            rx: [0u8; 128],
            rx_len: 0,
            rx_pos: 0,
            tx: [0u8; 128],
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
        let n = (self.rx_len - self.rx_pos).min(buf.len());
        buf[..n].copy_from_slice(&self.rx[self.rx_pos..self.rx_pos + n]);
        self.rx_pos += n;
        Ok(n)
    }
}

impl Write for MockChannel {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let n = (self.tx.len() - self.tx_len).min(buf.len());
        self.tx[self.tx_len..self.tx_len + n].copy_from_slice(&buf[..n]);
        self.tx_len += n;
        Ok(n)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn commander_processes_line_over_rtt_stream() {
    let mut stream = RttStream::new(MockChannel::default(), MockChannel::with_input(b"T25.5\n"));
    let mut processor = barefoc_commander::LineProcessor::<64>::new();

    let (down, up) = stream.split();
    let lines = processor
        .process_stream(down, up, |line, writer| {
            if let Some(value) = line.strip_prefix('T') {
                let parsed: f32 = value.parse().map_err(|_| ErrorKind::InvalidData)?;
                assert!((parsed - 25.5).abs() < f32::EPSILON);
                writer
                    .write_all(b"Target: 25.50\r\n")
                    .map_err(|_| ErrorKind::Other)?;
            }
            Ok::<(), ErrorKind>(())
        })
        .expect("process_stream failed");

    assert_eq!(lines, 1);
    let up = stream.up();
    let reply = core::str::from_utf8(&up.tx[..up.tx_len]).unwrap();
    assert_eq!(reply, "Target: 25.50\r\n");
}
