#include "pal_ref.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config.h"
#include "frame_file_source.h"
#include "link_card.h"
#include "mabur/node.h"
#include "web_gs.h"

namespace {

void copy_err(char* err, size_t cap, const char* msg) {
  if (!err || cap == 0) return;
  std::snprintf(err, cap, "%s", msg);
}

mabur::node::RxBody to_cpp(const pal_rx_body& b) {
  mabur::node::RxBody m;
  m.card_id = b.card_id;
  m.mono_us = b.mono_us;
  m.rssi[0] = b.rssi[0];
  m.rssi[1] = b.rssi[1];
  m.snr[0] = b.snr[0];
  m.snr[1] = b.snr[1];
  m.evm[0] = b.evm[0];
  m.evm[1] = b.evm[1];
  m.phy_valid = b.phy_valid != 0;
  m.crc_ok = b.crc_ok != 0;
  m.mac_seq = b.mac_seq;
  m.tsfl = b.tsfl;
  m.mcs = b.mcs;
  m.rx_channel = b.rx_channel;
  if (b.body_len) m.body.assign(b.body, b.body + b.body_len);
  return m;
}

void from_cpp(const mabur::node::RxBody& m, pal_rx_body* out) {
  out->card_id = m.card_id;
  out->mono_us = m.mono_us;
  out->rssi[0] = m.rssi[0];
  out->rssi[1] = m.rssi[1];
  out->snr[0] = m.snr[0];
  out->snr[1] = m.snr[1];
  out->evm[0] = m.evm[0];
  out->evm[1] = m.evm[1];
  out->phy_valid = m.phy_valid ? 1 : 0;
  out->crc_ok = m.crc_ok ? 1 : 0;
  out->mac_seq = m.mac_seq;
  out->tsfl = m.tsfl;
  out->mcs = m.mcs;
  out->rx_channel = m.rx_channel;
  out->body = m.body.data();
  out->body_len = m.body.size();
}

}  // namespace

struct pal_ref {
  maburgs::Config cfg;
  std::atomic<uint8_t> stored_ch{0};
  std::unique_ptr<webgs::WebGs> gs;   // last: its threads die before the rest
};

maburgs::LinkCard* pal_card_link(pal_card* c);   // pal_card.cpp

struct pal_ffs {
  std::unique_ptr<maburgs::FrameFileSource> src;
  mabur::node::RxBody cur;  // owns the body bytes handed out by pal_ffs_next
};

extern "C" {

pal_ref* pal_ref_new(const char* config_path, int mode, int adaptive_gap, uint32_t rz_nonce,
                     uint8_t channel,
                     void* user, pal_au_cb on_au, pal_send_cb send, char* err, size_t err_cap) {
  if (!config_path || !on_au) {
    copy_err(err, err_cap, "config_path and on_au are required");
    return nullptr;
  }
  try {
    auto r = std::make_unique<pal_ref>();
    r->cfg = maburgs::load_config(config_path, nullptr, "");
    if (r->cfg.radio.channels.empty()) {
      copy_err(err, err_cap, "config has no radio.channels");
      return nullptr;
    }
    webgs::Io io;
    io.on_au = [user, on_au](webgs::Au&& a) {
      on_au(user, a.sid, a.flags, a.complete ? 1 : 0, a.salvaged ? 1 : 0, a.pts_us,
            a.skipped_before, a.vcl_nal, a.data.data(), a.data.size());
    };
    const webgs::Mode m = mode == 0 ? webgs::Mode::Gs : webgs::Mode::Spotter;
    if (m == webgs::Mode::Gs) {
      // Gs needs a send path even without a radio (the ctor throws otherwise).
      io.send = [user, send](const std::vector<uint8_t>& body) {
        if (send) send(user, body.data(), body.size());
      };
    }
    webgs::Opts o;
    o.adaptive_gap = adaptive_gap != 0;
    o.rz_nonce = rz_nonce;
    const uint8_t ch = channel ? channel : r->cfg.radio.channels.front();
    r->gs = std::make_unique<webgs::WebGs>(r->cfg, m, ch,
                                           maburgs::derived_width(r->cfg.link),
                                           std::vector<maburgs::LinkCard*>{}, 0, std::move(io), o);
    return r.release();
  } catch (const std::exception& e) {
    copy_err(err, err_cap, e.what());
  } catch (...) {
    copy_err(err, err_cap, "unknown C++ exception");
  }
  return nullptr;
}

pal_ref* pal_ref_new_with_card(const char* config_path, pal_card* card, void* user,
                               pal_au_cb on_au, char* err, size_t err_cap) {
  if (!config_path || !on_au || !card) {
    copy_err(err, err_cap, "config_path, card and on_au are required");
    return nullptr;
  }
  try {
    auto r = std::make_unique<pal_ref>();
    r->cfg = maburgs::load_config(config_path, nullptr, "");
    if (r->cfg.radio.channels.empty()) {
      copy_err(err, err_cap, "config has no radio.channels");
      return nullptr;
    }
    webgs::Io io;
    io.on_au = [user, on_au](webgs::Au&& a) {
      on_au(user, a.sid, a.flags, a.complete ? 1 : 0, a.salvaged ? 1 : 0, a.pts_us,
            a.skipped_before, a.vcl_nal, a.data.data(), a.data.size());
    };
    pal_ref* raw = r.get();
    io.on_channel_store = [raw](uint8_t ch) { raw->stored_ch.store(ch, std::memory_order_relaxed); };
    io.on_log = [](const std::string& line) { std::fprintf(stderr, "%s\n", line.c_str()); };
    webgs::Opts o;   // live: adaptive gap, random nonce, the scout thread
    maburgs::LinkCard* lc = pal_card_link(card);
    r->gs = std::make_unique<webgs::WebGs>(r->cfg, webgs::Mode::Gs, lc->channel(), lc->width(),
                                           std::vector<maburgs::LinkCard*>{lc}, 1, std::move(io),
                                           o);
    return r.release();
  } catch (const std::exception& e) {
    copy_err(err, err_cap, e.what());
  } catch (...) {
    copy_err(err, err_cap, "unknown C++ exception");
  }
  return nullptr;
}

// WebGs::now_us_for_core_ with core_threads.
uint64_t pal_ref_now_us(void) {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

uint8_t pal_ref_stored_channel(const pal_ref* r) {
  return r ? r->stored_ch.load(std::memory_order_relaxed) : 0;
}

void pal_ref_free(pal_ref* r) { delete r; }

int pal_ref_on_rx(pal_ref* r, const pal_rx_body* b) {
  try {
    r->gs->on_rx(to_cpp(*b));
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "pal_ref_on_rx: %s\n", e.what());
  } catch (...) {
    std::fprintf(stderr, "pal_ref_on_rx: unknown C++ exception\n");
  }
  return -1;
}

int pal_ref_tick(pal_ref* r, uint64_t now_us) {
  try {
    r->gs->tick(now_us);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "pal_ref_tick: %s\n", e.what());
  } catch (...) {
    std::fprintf(stderr, "pal_ref_tick: unknown C++ exception\n");
  }
  return -1;
}

int pal_ref_inject_disc_ack_for_replay(pal_ref* r, uint64_t now_us) {
  try {
    r->gs->inject_disc_ack_for_replay(now_us);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "pal_ref_inject_disc_ack_for_replay: %s\n", e.what());
  } catch (...) {
    std::fprintf(stderr, "pal_ref_inject_disc_ack_for_replay: unknown C++ exception\n");
  }
  return -1;
}

size_t pal_ref_mav_take(pal_ref* r, uint8_t* out, size_t cap) {
  return r->gs->mav_take_inbound(out, cap);
}

void pal_ref_mav_push(pal_ref* r, const uint8_t* p, size_t n) {
  r->gs->mav_push_outbound(p, n);
}

void pal_ref_set_idr_requests(pal_ref* r, uint32_t n) { r->gs->set_idr_requests(n); }

size_t pal_ref_params_json(const pal_ref* r, uint64_t now_us, char* buf, size_t cap) {
  std::string s;
  try {
    s = webgs::params_json(*r->gs, now_us / 1000);
  } catch (...) {
    s = "{}";
  }
  if (buf && cap) {
    const size_t n = s.size() < cap - 1 ? s.size() : cap - 1;
    std::memcpy(buf, s.data(), n);
    buf[n] = '\0';
  }
  return s.size();
}

int pal_ref_param_set(pal_ref* r, size_t row, const char* text, char* why, size_t cap) {
  std::string w;
  bool ok = false;
  try {
    ok = r->gs->param_set(row, text, &w);
  } catch (...) {
    w = "exception";
  }
  if (why && cap) std::snprintf(why, cap, "%s", w.c_str());
  return ok ? 1 : 0;
}

void pal_ref_param_refresh(pal_ref* r) { r->gs->param_refresh(); }

uint32_t pal_ref_frame_gap_timeout_ms(const pal_ref* r) {
  return static_cast<uint32_t>(r->cfg.video.frame_gap_timeout_ms);
}

size_t pal_ref_stats_json(const pal_ref* r, char* buf, size_t cap) {
  std::string s;
  try {
    s = webgs::stats_json(r->gs->stats());
  } catch (...) {
    s = "{}";
  }
  if (buf && cap) {
    const size_t n = s.size() < cap - 1 ? s.size() : cap - 1;
    std::memcpy(buf, s.data(), n);
    buf[n] = '\0';
  }
  return s.size();
}

void pal_ref_link_op(const pal_ref* r, int* mcs, int* bw) {
  try {
    const auto s = r->gs->stats();
    *mcs = s.mcs;
    *bw = s.bw;
  } catch (...) {
  }
}

pal_ffs* pal_ffs_open(const char* path, int cards, int drop_pct, uint32_t seed) {
  try {
    auto f = std::make_unique<pal_ffs>();
    maburgs::FrameFileSource::Options o;
    o.cards = cards;
    o.drop_pct = drop_pct;
    o.seed = seed;
    f->src = std::make_unique<maburgs::FrameFileSource>(path, o);
    if (!f->src->ok()) return nullptr;
    return f.release();
  } catch (...) {
    return nullptr;
  }
}

void pal_ffs_free(pal_ffs* f) { delete f; }

int pal_ffs_next(pal_ffs* f, pal_rx_body* out) {
  std::optional<mabur::node::RxBody> m = f->src->next();
  if (!m) return 0;
  f->cur = std::move(*m);
  from_cpp(f->cur, out);
  return 1;
}

uint64_t pal_ffs_frames_read(const pal_ffs* f) { return f->src->frames_read(); }
uint64_t pal_ffs_dropped(const pal_ffs* f) { return f->src->dropped(); }

}  // extern "C"
