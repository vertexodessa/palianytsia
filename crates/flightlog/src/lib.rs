//! Session files (spec section 7): the raw capture (`bodies.bin`) and the
//! session directory with stats, events and the AU log.
pub mod capture;
pub mod decoder;
pub mod keyframes;
pub mod latency;
pub mod retain;
pub mod session;
