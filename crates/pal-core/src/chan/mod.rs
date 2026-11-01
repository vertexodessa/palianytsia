//! mabur's channel search (ChannelCore and its parts), ported module by
//! module with each module's mabur tests. Plan and status:
//! `docs/channel-core-port.md`.
pub mod boot;
pub mod gate;
pub mod hop;
pub mod hop_ranker;
pub mod pair;
pub mod plan;
pub mod radio;
pub mod ranker;
pub mod scout_pick;
pub mod verdict;
