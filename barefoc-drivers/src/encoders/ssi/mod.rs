//! SSI (Synchronous Serial Interface) encoder drivers over SPI.

pub mod generic_ssi;
pub mod mt6701;

pub use generic_ssi::GenericSsi;
pub use mt6701::{Mt6701Ssi, Mt6701Status};
