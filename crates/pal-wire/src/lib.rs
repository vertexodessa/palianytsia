//! mabur wire formats, both directions, so a future Rust drone can share them.
//! Every module is checked against mabur's golden vectors
//! (`third_party/mabur/tests/vectors/*.json`).
#![no_std]

extern crate alloc;

pub mod crc16;
pub mod dot11;
pub mod frame_wire;
pub mod probe;
pub mod profile;
pub mod rc;
pub mod sbi;
pub mod siphash;
pub mod sw;
