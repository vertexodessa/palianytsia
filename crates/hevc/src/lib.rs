//! H.265 helpers ported from mabur `common/src/hevc_*.cpp` (pin 51cea32f):
//! Annex-B NAL walk, the SPS/PPS subset slice salvage needs, slice-segment
//! header parse/rewrite, and the CABAC skip-slice fill.
pub mod bits;
pub mod cabac;
pub mod nal;
pub mod ps;
pub mod skip_slice;
pub mod slice;
