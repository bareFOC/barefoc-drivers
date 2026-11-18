//! SPI encoder drivers.

pub mod as5047p;
pub mod as5048a;

pub use as5047p::{As5047p, As5047pDiagnostics, As5047pErrorFlags};
pub use as5048a::{As5048a, As5048aDiagnostics, As5048aErrorFlags};
