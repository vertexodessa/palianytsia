//! mabur wire formats, both directions, so a future Rust drone can share them.
//! Every module is checked against mabur's golden vectors
//! (`third_party/mabur/tests/vectors/*.json`).
#![no_std]

pub mod crc16;
pub mod dot11;
pub mod profile;
