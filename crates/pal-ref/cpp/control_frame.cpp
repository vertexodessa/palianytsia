// mabur's build_control_frame behind a flat C ABI: the oracle for pal-wire
// dot11::control_frame, driven from tests/control_frame.rs.
#include <cstring>

#include "dot11.h"

// Bytes written to out (radiotap included), 0 when cap is too small.
extern "C" size_t pal_mabur_control_frame(uint16_t seq, const uint8_t* body, size_t len,
                                          uint8_t* out, size_t cap) {
  const auto f = maburgs::build_control_frame(seq, body, len);
  if (f.size() > cap) return 0;
  std::memcpy(out, f.data(), f.size());
  return f.size();
}
