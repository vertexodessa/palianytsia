#include "dv.h"

#include <libusb.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "AdapterCaps.h"
#include "IRtlRadio.h"
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "TxMode.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"

namespace {
constexpr uint16_t kScanPids[] = {0xa81a, 0x881a, 0x8812};
constexpr uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

// mabur::ht40_offset (common/include/mabur/ht40.h).
uint8_t ht40_offset(uint8_t ch) {
  if (ch >= 36 && ch <= 144 && (ch - 36) % 4 == 0) return ((ch - 36) / 4) % 2 == 0 ? 1 : 2;
  if (ch >= 149 && ch <= 161 && (ch - 149) % 4 == 0) return ((ch - 149) / 4) % 2 == 0 ? 1 : 2;
  return 0;
}

void set_err(char *err, size_t cap, const std::string &s) {
  if (err && cap) std::snprintf(err, cap, "%s", s.c_str());
}
}  // namespace

struct dv_card {
  libusb_context *ctx = nullptr;
  libusb_device_handle *h = nullptr;
  std::shared_ptr<Logger> logger;
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  std::shared_ptr<WiFiDriver> driver;
  std::shared_ptr<IRtlRadio> dev;
  std::thread rx;
  std::atomic<bool> alive{false};
  uint16_t tx_seq = 0;
  std::string chip;
  std::vector<uint8_t> radiotap;
};

extern "C" dv_card *dv_open(const dv_cfg *cfg, char *err, size_t err_cap) try {
  auto c = std::make_unique<dv_card>();
  if (cfg->fd >= 0) libusb_set_option(nullptr, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
  if (libusb_init(&c->ctx) != 0) { set_err(err, err_cap, "libusb_init"); return nullptr; }
  if (cfg->fd >= 0) {
    if (libusb_wrap_sys_device(c->ctx, static_cast<intptr_t>(cfg->fd), &c->h) != 0) c->h = nullptr;
  } else {
    libusb_device **list = nullptr;
    const ssize_t n = libusb_get_device_list(c->ctx, &list);
    int match = 0;
    libusb_device *d = nullptr;
    for (ssize_t i = 0; i < n && !d; ++i) {
      libusb_device_descriptor dd;
      if (libusb_get_device_descriptor(list[i], &dd) != 0) continue;
      bool ok = false;
      if (cfg->vid != 0) ok = dd.idVendor == cfg->vid && (cfg->pid == 0 || dd.idProduct == cfg->pid);
      else if (dd.idVendor == 0x0bda) for (uint16_t p : kScanPids) ok |= dd.idProduct == p;
      if (ok && match++ == cfg->index) d = list[i];
    }
    if (d && libusb_open(d, &c->h) != 0) c->h = nullptr;
    if (list) libusb_free_device_list(list, 1);
  }
  if (!c->h) { set_err(err, err_cap, "no device"); dv_close(c.release()); return nullptr; }

  c->logger = std::make_shared<Logger>();
  const int rc = devourer::claim_interface_then_reset(c->h, 0, c->logger, /*do_reset=*/true, c->lock);
  if (rc != 0) {
    set_err(err, err_cap, "claim failed rc=" + std::to_string(rc));
    dv_close(c.release());
    return nullptr;
  }

  // As maburgs (radio_frontend.cpp:76-129): duplex (mandatory on 8822E),
  // keep FCS-failed frames for SBI salvage, absolute noise floor, carrier
  // sense ON (must match maburd), no multi-packet cancel, heap RX path.
  devourer::DeviceConfig dc;
  dc.rx.enable_with_tx = true;
  dc.rx.keep_corrupted = true;
  dc.rx.abs_noise_floor = true;
  dc.tuning.disable_cca = false;
  dc.tx.no_cancel_multipkt = true;
  dc.usb.rx_zerocopy = false;
  c->driver = std::make_shared<WiFiDriver>(c->logger);
  if (auto r = c->driver->CreateRadio(c->h, c->ctx, c->lock, dc); r && dynamic_cast<IRtlRadio *>(r.get()))
    c->dev.reset(static_cast<IRtlRadio *>(r.release()));
  if (!c->dev) { set_err(err, err_cap, "unsupported chip"); dv_close(c.release()); return nullptr; }

  c->dev->InitWrite(cfg->width_mhz == 40
                        ? SelectedChannel{cfg->channel, ht40_offset(cfg->channel), CHANNEL_WIDTH_40}
                        : SelectedChannel{cfg->channel, 0, CHANNEL_WIDTH_20});
  const devourer::AdapterCaps ac = c->dev->GetAdapterCaps();
  if (ac.chip_name) c->chip = ac.chip_name;

  devourer::TxMode m;
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 0;
  m.bw_mhz = 20;
  m.ldpc = true;
  m.stbc = true;
  c->radiotap = devourer::build_stream_radiotap(m);
  return c.release();
} catch (const std::exception &e) {
  set_err(err, err_cap, std::string("exception: ") + e.what());
  return nullptr;
}

extern "C" int dv_start_rx(dv_card *c, dv_rx_cb cb, void *user) {
  if (!c || !c->dev || c->rx.joinable()) return -1;
  c->alive = true;
  c->rx = std::thread([c, cb, user] {
    c->dev->StartRxLoop([cb, user](const Packet &p) {
      const auto &a = p.RxAtrib;
      dv_rx r;
      r.frame = p.Data.data();
      r.len = p.Data.size();
      r.crc_err = a.crc_err;
      r.physt = a.physt;
      r.data_rate = a.data_rate;
      r.rssi[0] = a.rssi[0];
      r.rssi[1] = a.rssi[1];
      r.snr[0] = a.snr[0];
      r.snr[1] = a.snr[1];
      r.evm[0] = a.evm[0];
      r.evm[1] = a.evm[1];
      r.tsfl = a.tsfl;
      r.bw40 = a.bw == 1;
      cb(user, &r);
    });
    c->alive = false;
  });
  return 0;
}

extern "C" int dv_send_control(dv_card *c, const uint8_t *body, size_t len) try {
  if (!c || !c->dev) return -1;
  // As maburgs build_control_frame (gs/src/dot11.cpp).
  std::vector<uint8_t> f(c->radiotap.size() + 24 + len);
  std::memcpy(f.data(), c->radiotap.data(), c->radiotap.size());
  uint8_t *d = f.data() + c->radiotap.size();
  d[0] = 0x40;
  std::memset(d + 4, 0xff, 6);
  std::memcpy(d + 10, kSa, 6);
  std::memcpy(d + 16, kSa, 6);
  const uint16_t sc = static_cast<uint16_t>(c->tx_seq << 4);
  d[22] = static_cast<uint8_t>(sc & 0xff);
  d[23] = static_cast<uint8_t>(sc >> 8);
  if (len) std::memcpy(d + 24, body, len);
  c->tx_seq = static_cast<uint16_t>((c->tx_seq + 1) & 0xFFF);
  return c->dev->send_packet(f.data(), f.size()) ? 0 : -1;
} catch (...) {
  return -1;
}

extern "C" const char *dv_chip(const dv_card *c) {
  return c && !c->chip.empty() ? c->chip.c_str() : nullptr;
}

extern "C" void dv_close(dv_card *c) {
  if (!c) return;
  try {
    if (c->dev && c->alive) c->dev->StopRxLoop();
    if (c->rx.joinable()) c->rx.join();
    if (c->dev) c->dev->Stop();
  } catch (...) {
  }
  c->dev.reset();
  c->driver.reset();
  if (c->h) { libusb_release_interface(c->h, 0); libusb_close(c->h); }
  c->lock.reset();  // or a reopen refuses against our own stale lock
  if (c->ctx) libusb_exit(c->ctx);
  delete c;
}
