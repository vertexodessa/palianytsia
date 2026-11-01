// C ABI over mabur's webgs::WebGs (and, for M0 test input only,
// maburgs::FrameFileSource). Test oracle for Palianytsia; never shipped.
// No C++ exception crosses this boundary: every entry point catches and
// reports through its return value.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pal_rx_body {
  uint8_t card_id;
  uint64_t mono_us;
  uint8_t rssi[2];
  int8_t snr[2];
  int8_t evm[2];
  uint8_t phy_valid;
  uint8_t crc_ok;
  uint16_t mac_seq;
  uint32_t tsfl;
  uint8_t mcs;
  uint8_t rx_channel;
  const uint8_t* body;
  size_t body_len;
} pal_rx_body;

typedef void (*pal_au_cb)(void* user, uint8_t sid, uint8_t flags, uint8_t complete,
                          uint8_t salvaged, uint32_t pts_us, uint32_t skipped_before,
                          uint8_t vcl_nal, const uint8_t* data, size_t len);
typedef void (*pal_send_cb)(void* user, const uint8_t* data, size_t len);

typedef struct pal_ref pal_ref;

// mode: 0 = Gs, 1 = Spotter. adaptive_gap: 0 = fixed frame_gap_timeout_ms.
// rz_nonce: 0 = random, else pinned (reproducible DISC nonce and RCF tags).
// channel: the channel the radio is actually tuned to; 0 = the config's first
// radio.channels member. A wrong value makes the core steer the drone to a
// channel the card never listens on.
// The callbacks get `user` and are called only from inside on_rx/tick.
// Returns NULL on failure with the reason in err (always NUL-terminated).
pal_ref* pal_ref_new(const char* config_path, int mode, int adaptive_gap, uint32_t rz_nonce,
                     uint8_t channel,
                     void* user, pal_au_cb on_au, pal_send_cb send, char* err, size_t err_cap);
void pal_ref_free(pal_ref* r);
// 0 = ok, -1 = the core threw (the message goes to stderr).
int pal_ref_on_rx(pal_ref* r, const pal_rx_body* b);
int pal_ref_tick(pal_ref* r, uint64_t now_us);
int pal_ref_inject_disc_ack_for_replay(pal_ref* r, uint64_t now_us);
uint32_t pal_ref_frame_gap_timeout_ms(const pal_ref* r);
// One inbound MAVLink packet from the drone into out; its length, 0 = none.
size_t pal_ref_mav_take(pal_ref* r, uint8_t* out, size_t cap);
// Local MAVLink peer bytes toward the drone (any boundary).
void pal_ref_mav_push(pal_ref* r, const uint8_t* p, size_t n);
// WebGs::set_idr_requests: the RCF idr_epoch byte becomes (uint8_t)n.
void pal_ref_set_idr_requests(pal_ref* r, uint32_t n);
// webgs::params_json(gs, now_us / 1000); same buffer contract as stats_json.
size_t pal_ref_params_json(const pal_ref* r, uint64_t now_us, char* buf, size_t cap);
// WebGs::param_set: 1 on success, 0 with the reason in why.
int pal_ref_param_set(pal_ref* r, size_t row, const char* text, char* why, size_t cap);
void pal_ref_param_refresh(pal_ref* r);
// Writes the stats JSON line; returns its full length (excluding NUL). The
// output is truncated (and NUL-terminated) when cap is too small.
size_t pal_ref_stats_json(const pal_ref* r, char* buf, size_t cap);

// A card devourer-sys opened, lent to the core as a maburgs::LinkCard
// (pal_card.cpp). With it the core is mabur's full GS: the ChannelCore picks,
// relocates and hops the channel itself and transmits through the card.
typedef struct pal_card pal_card;
typedef struct pal_card_rx {
  uint8_t crc_err;
  uint8_t physt;
  uint16_t data_rate;
  uint8_t rssi[2];
  int8_t snr[2];
  int8_t evm[2];
  uint32_t tsfl;
  uint8_t bw40;
  uint8_t stbc;
  uint8_t sgi;
} pal_card_rx;
// send: transmit one RC body as a GS control frame, nonzero = sent (the host
// owns the frame builder: devourer-sys dv_send_control). Called under the
// card's control lock, from the core's thread.
typedef int (*pal_card_send_cb)(void* user, const uint8_t* body, size_t len);
// radio: devourer's IRtlRadio* (dv_radio), already started at channel/width.
// It must outlive the card, and the card every pal_ref built on it.
pal_card* pal_card_new(void* radio, uint8_t channel, uint8_t width_mhz, void* user,
                       pal_card_send_cb send, char* err, size_t err_cap);
void pal_card_free(pal_card* c);
// From the RX thread, every frame, with the host's maburgs::fill_rx_body
// verdict (pal-wire dot11::parse): 0 = body, 1 = short, 2 = foreign.
// Returns -1 = not a body for the core, else the rx_channel to stamp on it
// (0 = unknown, mid-retune).
int pal_card_on_rx(pal_card* c, int verdict, size_t len, const pal_card_rx* a);
// The channel the card is tuned to now.
uint8_t pal_card_channel(const pal_card* c);
// pal_ref_new in Gs mode with the card as its roster (no send callback: the
// core transmits through the card). Its clock is pal_ref_now_us: stamp
// on_rx and tick with it, since the core's scout thread reads it directly.
pal_ref* pal_ref_new_with_card(const char* config_path, pal_card* card, void* user,
                               pal_au_cb on_au, char* err, size_t err_cap);
uint64_t pal_ref_now_us(void);
// The channel the core asked to remember for the next start (ChannelCore's
// store), 0 = none yet.
uint8_t pal_ref_stored_channel(const pal_ref* r);

// maburd --dry-run --out frame file as N virtual cards with deterministic loss.
typedef struct pal_ffs pal_ffs;
pal_ffs* pal_ffs_open(const char* path, int cards, int drop_pct, uint32_t seed);
void pal_ffs_free(pal_ffs* f);
// 1 = body written to *out (out->body valid until the next call or free),
// 0 = end of file.
int pal_ffs_next(pal_ffs* f, pal_rx_body* out);
uint64_t pal_ffs_frames_read(const pal_ffs* f);
uint64_t pal_ffs_dropped(const pal_ffs* f);

#ifdef __cplusplus
}
#endif
