#include "pal_ref.h"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
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

// The flat dump of a loaded Config. Every struct is taken apart with a
// structured binding, which must name each data member: a field a pin adds
// to any of these structs stops this file compiling until the dump (and the
// Rust twin, pal_core::config::dump) carries it.
class ConfigDump {
 public:
  std::string out;

  void put(const std::string& k, const std::string& v) { out += k + "=" + v + "\n"; }
  void i(const std::string& k, long long v) { put(k, std::to_string(v)); }
  void b(const std::string& k, bool v) { put(k, v ? "true" : "false"); }
  void d(const std::string& k, double v) {
    uint64_t u;
    std::memcpy(&u, &v, sizeof u);
    char buf[32];
    std::snprintf(buf, sizeof buf, "f64:%016llx", static_cast<unsigned long long>(u));
    put(k, buf);
  }
  void s(const std::string& k, const std::string& v) { put(k, v); }

  void config(const maburgs::Config& c) {
    const auto& [radio, fec, link, video, msp, stats, au_ring, debug_log, hop, mavlink] = c;
    this->radio(radio);
    this->fec(fec);
    this->link(link);
    this->video(video);
    this->msp(msp);
    this->stats(stats);
    this->au_ring(au_ring);
    this->debug_log(debug_log);
    this->hop(hop);
    this->mavlink(mavlink);
    i("derived.rx_width", maburgs::derived_width(c.link));
    s("derived.rx_width_reason", maburgs::derived_width_reason(c.link));
    const auto layers = c.uep_layers();
    for (size_t n = 0; n < layers.size(); ++n) {
      const auto& [fec_cfg, blocks_per_body] = layers[n];
      const auto& [symbol_size, window, overhead] = fec_cfg;
      const std::string p = "derived.uep_layers[" + std::to_string(n) + "].";
      i(p + "fec.symbol_size", symbol_size);
      i(p + "fec.window", window);
      d(p + "fec.overhead", overhead);
      i(p + "blocks_per_body", blocks_per_body);
    }
  }

 private:
  void radio(const maburgs::RadioCfg& r) {
    const auto& [channels, pin, cards, auto_scan, tx_card, relays, scan] = r;
    std::string chs;
    for (uint8_t ch : channels) chs += (chs.empty() ? "" : ",") + std::to_string(ch);
    s("radio.channels", chs);
    s("radio.pin", pin ? std::to_string(*pin) : "none");
    i("radio.cards.len", static_cast<long long>(cards.size()));
    for (size_t n = 0; n < cards.size(); ++n) {
      const auto& [usb_vid, usb_pid, index] = cards[n];
      const std::string p = "radio.cards[" + std::to_string(n) + "].";
      i(p + "usb_vid", usb_vid);
      i(p + "usb_pid", usb_pid);
      i(p + "index", index);
    }
    b("radio.auto_scan", auto_scan);
    i("radio.tx_card", tx_card);
    i("radio.relays.len", static_cast<long long>(relays.size()));
    for (size_t n = 0; n < relays.size(); ++n)
      s("radio.relays[" + std::to_string(n) + "]", relays[n]);
    const auto& [dwell_ms, settle_ms, min_rounds, search_ms, op_window_ms, search_after_ms,
                 pick_margin_pct, one_card_ms, max_ms, busy] = scan;
    i("radio.scan.dwell_ms", dwell_ms);
    i("radio.scan.settle_ms", settle_ms);
    i("radio.scan.min_rounds", min_rounds);
    i("radio.scan.search_ms", search_ms);
    i("radio.scan.op_window_ms", op_window_ms);
    i("radio.scan.search_after_ms", search_after_ms);
    i("radio.scan.pick_margin_pct", pick_margin_pct);
    i("radio.scan.one_card_ms", one_card_ms);
    i("radio.scan.max_ms", max_ms);
    const auto& [busy_dbm, blocked_pct] = busy;
    i("radio.scan.busy.busy_dbm", busy_dbm);
    d("radio.scan.busy.blocked_pct", blocked_pct);
  }

  void fec(const maburgs::FecCfg& f) {
    const auto& [symbol_size, seq_horizon] = f;
    i("fec.symbol_size[0]", symbol_size[0]);
    i("fec.symbol_size[1]", symbol_size[1]);
    i("fec.seq_horizon", seq_horizon);
  }

  void link(const maburgs::LinkCfg& l) {
    const auto& [feedback_ms, beacon_keepalive_ms, rcf_slot_hold_ms, arrival_guard_syms, nack,
                 nack_residual, static_mcs, static_overhead_base, static_overhead_enh, static_bw,
                 ladder_cfg, key_file, key, key_is_default, key_source] = l;
    i("link.feedback_ms", feedback_ms);
    i("link.beacon_keepalive_ms", beacon_keepalive_ms);
    i("link.rcf_slot_hold_ms", rcf_slot_hold_ms);
    i("link.arrival_guard_syms", arrival_guard_syms);
    const auto& [n_enable, lookback, repeat_ms, max_tries, min_lead_ms, settle_min_ms,
                 settle_max_ms, settle_seed_ms, settle_window_ms, settle_min_samples] = nack;
    b("link.nack.enable", n_enable);
    i("link.nack.lookback", lookback);
    i("link.nack.repeat_ms", repeat_ms);
    i("link.nack.max_tries", max_tries);
    i("link.nack.min_lead_ms", min_lead_ms);
    i("link.nack.settle_min_ms", settle_min_ms);
    i("link.nack.settle_max_ms", settle_max_ms);
    i("link.nack.settle_seed_ms", settle_seed_ms);
    i("link.nack.settle_window_ms", settle_window_ms);
    i("link.nack.settle_min_samples", settle_min_samples);
    s("link.nack_residual", maburgs::nack_residual_name(nack_residual));
    i("link.static_mcs", static_mcs);
    d("link.static_overhead_base", static_overhead_base);
    d("link.static_overhead_enh", static_overhead_enh);
    i("link.static_bw", static_bw);
    ladder(ladder_cfg);
    s("link.key_file", key_file);
    s("link.key", mabur::key_to_hex(key));
    b("link.key_is_default", key_is_default);
    s("link.key_source", key_source);
  }

  void ladder(const maburgs::LadderCfg& lc) {
    const auto& [rungs, down_util, up_util, confirm_ms, clean_ms, probation_ms, penalty_base_ms,
                 penalty_max_ms, lockout_ms, lockout_latch, hold_after_down_ms, min_between_changes_ms, feedback_timeout_ms,
                 starved_confirm_ms, probe, s3_demote, s3_down_util, s3_min_syms, s3_settle_ms,
                 rung_stats, fade] = lc;
    const std::string p = "link.ladder_cfg.";
    i(p + "ladder.len", static_cast<long long>(rungs.size()));
    for (size_t n = 0; n < rungs.size(); ++n) {
      const auto& [mcs, overhead_base, overhead_enh, bw] = rungs[n];
      const std::string q = p + "ladder[" + std::to_string(n) + "].";
      i(q + "mcs", mcs);
      d(q + "overhead_base", overhead_base);
      d(q + "overhead_enh", overhead_enh);
      i(q + "bw", bw);
    }
    d(p + "down_util", down_util);
    d(p + "up_util", up_util);
    i(p + "confirm_ms", confirm_ms);
    i(p + "clean_ms", clean_ms);
    i(p + "probation_ms", probation_ms);
    i(p + "penalty_base_ms", penalty_base_ms);
    i(p + "penalty_max_ms", penalty_max_ms);
    i(p + "lockout_ms", lockout_ms);
    i(p + "lockout_latch", lockout_latch);
    i(p + "hold_after_down_ms", hold_after_down_ms);
    i(p + "min_between_changes_ms", min_between_changes_ms);
    i(p + "feedback_timeout_ms", feedback_timeout_ms);
    i(p + "starved_confirm_ms", starved_confirm_ms);
    const auto& [p_enable, rung_offset, clean_bodies, max_util, min_syms, silence_ms, pin_mcs] =
        probe;
    b(p + "probe.enable", p_enable);
    i(p + "probe.rung_offset", rung_offset);
    i(p + "probe.clean_bodies", clean_bodies);
    d(p + "probe.max_util", max_util);
    i(p + "probe.min_syms", min_syms);
    i(p + "probe.silence_ms", silence_ms);
    i(p + "probe.pin_mcs", pin_mcs);
    b(p + "s3_demote", s3_demote);
    d(p + "s3_down_util", s3_down_util);
    i(p + "s3_min_syms", s3_min_syms);
    i(p + "s3_settle_ms", s3_settle_ms);
    const auto& [half_life_samples] = rung_stats;
    i(p + "rung_stats.half_life_samples", half_life_samples);
    const auto& [cascade, predict, hold_ms, f_confirm_ms, rssi_db, snr_db, trigger_ms, min_rung] =
        fade;
    b(p + "fade.cascade", cascade);
    b(p + "fade.predict", predict);
    i(p + "fade.hold_ms", hold_ms);
    i(p + "fade.confirm_ms", f_confirm_ms);
    d(p + "fade.rssi_db", rssi_db);
    d(p + "fade.snr_db", snr_db);
    i(p + "fade.trigger_ms", trigger_ms);
    i(p + "fade.min_rung", min_rung);
  }

  void video(const maburgs::VideoCfg& v) {
    const auto& [frame_gap_timeout_ms, frame_gap_timeout_max_ms, frame_lookahead] = v;
    i("video.frame_gap_timeout_ms", frame_gap_timeout_ms);
    i("video.frame_gap_timeout_max_ms", frame_gap_timeout_max_ms);
    i("video.frame_lookahead", frame_lookahead);
  }

  void msp(const maburgs::MspCfg& m) {
    const auto& [enable, out_host, out_port, symbol_size, window] = m;
    b("msp.enable", enable);
    s("msp.out_host", out_host);
    i("msp.out_port", out_port);
    i("msp.symbol_size", symbol_size);
    i("msp.window", window);
  }

  void stats(const maburgs::StatsCfg& st) {
    const auto& [enable, interval_ms, outs] = st;
    b("stats.enable", enable);
    i("stats.interval_ms", interval_ms);
    i("stats.out.len", static_cast<long long>(outs.size()));
    for (size_t n = 0; n < outs.size(); ++n) {
      const auto& [host, port] = outs[n];
      const std::string p = "stats.out[" + std::to_string(n) + "].";
      s(p + "host", host);
      i(p + "port", port);
    }
  }

  void au_ring(const maburgs::AuRingOutCfg& a) {
    const auto& [enable, path, socket, slot_kb, slot_count] = a;
    b("au_ring.enable", enable);
    s("au_ring.path", path);
    s("au_ring.socket", socket);
    i("au_ring.slot_kb", slot_kb);
    i("au_ring.slot_count", slot_count);
  }

  void debug_log(const maburgs::DebugLogCfg& dl) {
    const auto& [enable, dir, ctl_period_ms, rung_period_s] = dl;
    b("debug_log.enable", enable);
    s("debug_log.dir", dir);
    i("debug_log.ctl_period_ms", ctl_period_ms);
    i("debug_log.rung_period_s", rung_period_s);
  }

  void hop(const maburgs::HopCfg& h) {
    const auto& [window_ms, persist, dwell_observe_ms, dwell_period_ms, burst_period_ms,
                 rank_visits, rank_max_age_ms, confirm_ms, confirm_extend_ms, verify_ms,
                 cooldown_ms, max_hops_per_min, backoff_ms, one_card_repeats,
                 relay_burst_period_ms, lease_sweep_period_ms, verdict] = h;
    i("hop.window_ms", window_ms);
    i("hop.persist", persist);
    i("hop.dwell_observe_ms", dwell_observe_ms);
    i("hop.dwell_period_ms", dwell_period_ms);
    i("hop.burst_period_ms", burst_period_ms);
    i("hop.rank_visits", rank_visits);
    i("hop.rank_max_age_ms", rank_max_age_ms);
    i("hop.confirm_ms", confirm_ms);
    i("hop.confirm_extend_ms", confirm_extend_ms);
    i("hop.verify_ms", verify_ms);
    i("hop.cooldown_ms", cooldown_ms);
    i("hop.max_hops_per_min", max_hops_per_min);
    i("hop.backoff_ms", backoff_ms);
    i("hop.one_card_repeats", one_card_repeats);
    i("hop.relay_burst_period_ms", relay_burst_period_ms);
    i("hop.lease_sweep_period_ms", lease_sweep_period_ms);
    const auto& [loss_pct, recovered_x, recovered_min, weak_rssi_dbm, weak_snr_db,
                 occupied_windows, detect_pct, detect_windows, detect_impaired] = verdict;
    d("hop.verdict.loss_pct", loss_pct);
    d("hop.verdict.recovered_x", recovered_x);
    i("hop.verdict.recovered_min", recovered_min);
    i("hop.verdict.weak_rssi_dbm", weak_rssi_dbm);
    i("hop.verdict.weak_snr_db", weak_snr_db);
    i("hop.verdict.occupied_windows", occupied_windows);
    d("hop.verdict.detect_pct", detect_pct);
    i("hop.verdict.detect_windows", detect_windows);
    i("hop.verdict.detect_impaired", detect_impaired);
  }

  void mavlink(const maburgs::MavlinkCfg& m) {
    const auto& [enable, bind, port] = m;
    b("mavlink.enable", enable);
    s("mavlink.bind", bind);
    i("mavlink.port", port);
  }
};

char* dup_c(const std::string& s) {
  char* p = static_cast<char*>(std::malloc(s.size() + 1));
  if (p) std::memcpy(p, s.c_str(), s.size() + 1);
  return p;
}

// load_config keeps its defaulted list and file name in globals for the
// duration of a call; test threads call this concurrently.
std::mutex g_config_mu;

}  // namespace

struct pal_ref {
  maburgs::Config cfg;
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

pal_ffs* pal_ffs_open(const char* path, int drop_pct, uint32_t seed) {
  try {
    auto f = std::make_unique<pal_ffs>();
    maburgs::FrameFileSource::Options o;
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

void pal_ref_config_check(const char* toml_text, pal_config_check* out) {
  out->ok = 0;
  out->text = nullptr;
  out->defaulted = nullptr;
  std::string text, defaulted_text;
  const char* tmp = std::getenv("TMPDIR");
  std::string tmpl = std::string(tmp && *tmp ? tmp : "/tmp") + "/pal-config-check-XXXXXX";
  std::vector<char> path_buf(tmpl.begin(), tmpl.end());
  path_buf.push_back('\0');
  char* path = path_buf.data();
  const int fd = mkstemp(path);
  if (fd < 0) {
    out->text = dup_c("pal_ref_config_check: mkstemp failed");
    out->defaulted = dup_c("");
    return;
  }
  const std::string in = toml_text ? toml_text : "";
  const bool written = write(fd, in.data(), in.size()) == static_cast<ssize_t>(in.size());
  close(fd);
  if (!written) {
    text = "pal_ref_config_check: short write";
  } else {
    std::vector<std::string> defaulted;
    try {
      std::lock_guard<std::mutex> lock(g_config_mu);
      const maburgs::Config c = maburgs::load_config(path, &defaulted, "");
      ConfigDump dump;
      dump.config(c);
      text = dump.out;
      out->ok = 1;
    } catch (const std::exception& e) {
      text = e.what();
      // "config: <file>:<line>: <field>: <why>": the file is ours and the
      // line has no Rust twin (pal-core's toml table keeps no spans).
      const std::string head = std::string("config: ") + path + ":";
      if (text.compare(0, head.size(), head) == 0) {
        const size_t rest = text.find(": ", head.size());
        if (rest != std::string::npos) text = "config: " + text.substr(rest + 2);
      }
    } catch (...) {
      text = "unknown C++ exception";
    }
    for (const auto& k : defaulted) defaulted_text += k + "\n";
  }
  unlink(path);
  out->text = dup_c(text);
  out->defaulted = dup_c(defaulted_text);
}

void pal_ref_config_check_free(pal_config_check* out) {
  if (!out) return;
  std::free(out->text);
  std::free(out->defaulted);
  out->text = nullptr;
  out->defaulted = nullptr;
}

uint64_t pal_ffs_frames_read(const pal_ffs* f) { return f->src->frames_read(); }
uint64_t pal_ffs_dropped(const pal_ffs* f) { return f->src->dropped(); }

}  // extern "C"
