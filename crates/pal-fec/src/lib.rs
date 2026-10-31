//! mabur sliding-window FEC for the ground side: GF(2^8) 0x11D arithmetic,
//! the streaming decoder and the arrival-time loss tracker.
pub mod arrival;
pub mod decoder;
pub mod gf256;

pub use decoder::{Boundary, LossEpisode, SourceState, SwConfig, SwDecoder};
