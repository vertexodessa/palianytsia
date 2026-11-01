/* C ABI over devourer for palianytsia: one card, RX callback, control TX.
 * Bring-up and RX config mirror mabur's RadioFrontend::open_and_start
 * (gs/src/radio_frontend.cpp) so the GS radio behaves as maburgs does. */
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dv_card dv_card;

/* devourer RxAtrib fields the GS reads, plus the raw dot11 frame (FCS kept,
 * as maburgs keeps it). Valid only during the callback. */
typedef struct {
  const uint8_t *frame;
  size_t len;
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
} dv_rx;

typedef void (*dv_rx_cb)(void *user, const dv_rx *rx);

typedef struct {
  uint16_t vid;       /* 0 = scan the default list below */
  uint16_t pid;
  int index;          /* ordinal among matches */
  int fd;             /* >= 0: wrap this fd (Android); vid/pid/index ignored */
  uint8_t channel;
  uint8_t width_mhz;  /* 20 or 40 (secondary = mabur::ht40_offset) */
} dv_cfg;

/* NULL on failure; err gets "no device", "claim failed rc=N", "unsupported chip". */
dv_card *dv_open(const dv_cfg *cfg, char *err, size_t err_cap);
/* Starts the RX thread; cb runs on it. */
int dv_start_rx(dv_card *c, dv_rx_cb cb, void *user);
/* GS uplink control frame: MAX_RANGE radiotap + probe-req dot11 (canonical SA). */
int dv_send_control(dv_card *c, const uint8_t *body, size_t len);
/* Moves the running card to ch / width_mhz (20 or 40, secondary =
 * mabur::ht40_offset). Same width and offset: FastRetune (RF switch only,
 * ~1-3 ms); otherwise the full SetMonitorChannel. 0 on success, -2 for a
 * 40 MHz channel without a pair, -1 on a device error. A control-plane
 * call: never from the RX callback, never concurrent with a send. */
int dv_set_channel(dv_card *c, uint8_t ch, uint8_t width_mhz);
/* The card's IRtlRadio (devourer), for C++ in the same binary that drives
 * the control plane itself (pal-ref's LinkCard). Valid until dv_close. */
void *dv_radio(dv_card *c);
/* Chip name, NULL if unknown. */
const char *dv_chip(const dv_card *c);
void dv_close(dv_card *c);

#ifdef __cplusplus
}
#endif
