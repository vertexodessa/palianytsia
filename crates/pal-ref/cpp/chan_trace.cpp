// The C++ half of the ChannelCore trace harness (crates/pal-ref/tests/
// chan_trace.rs): mabur's ChannelCore, unthreaded, over cards whose every
// call goes back into Rust (one pal_tcard_vt per card), with a sink that
// writes each record as one text line. The Rust port runs the same script
// over the same fake card type and writes the same lines; the test diffs
// them. The line shapes are defined twice, here and in the test, and must
// stay identical.
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "aggregator.h"
#include "channel_core.h"
#include "config.h"
#include "mabur/rc_proto.h"
#include "vrx_cfg.h"
#include "vrx_controller.h"

extern "C" {

struct pal_tenergy {
  uint8_t fa_valid, igi_valid, nhm_valid, floor_valid;
  uint32_t cca_ofdm, fa_ofdm;
  uint8_t igi;
  int8_t floor_dbm;
};

struct pal_tnhm {
  uint8_t valid;
  uint8_t buckets[12];
  uint16_t duration, period;
};

// One fake card on the Rust side. Every call is the LinkCard member of the
// same name; bools are ints.
struct pal_tcard_vt {
  void* user;
  int (*retune)(void*, uint8_t);
  int (*retune_width)(void*, uint8_t, uint8_t);
  void (*read_energy)(void*, int, pal_tenergy*);
  void (*read_energy_scout)(void*, pal_tenergy*);
  void (*frames)(void*, uint64_t*);   // own, foreign, own_air_us
  int (*arm_nhm_busy)(void*, uint16_t);
  void (*read_nhm_busy)(void*, pal_tnhm*);
  int (*ready)(void*);
  int (*alive)(void*);
  int (*snr_ok)(void*);
  uint64_t (*rx_frames)(void*);
  uint64_t (*foreign)(void*);
  int (*send_control)(void*, const uint8_t*, size_t);
  uint64_t (*tx_frames)(void*);
  uint64_t (*tx_fail)(void*);
  int (*set_width)(void*, uint8_t, uint8_t);
  uint8_t (*channel)(void*);
  uint8_t (*width)(void*);
  int (*tuned_central)(void*);
  int (*can_scout)(void*);
};

struct pal_tenv {
  void* user;
  uint64_t (*now_ms)(void*);
  uint64_t (*now_us)(void*);
  void (*sleep_ms)(void*, int);
  void (*line)(void*, const char*, size_t);
};

struct pal_tcfg {
  const char* config_path;
  const uint8_t* channels;
  size_t n_channels;
  uint8_t width;
  uint8_t pin;       // 0 = auto
  int32_t max_ms;    // < 0 = keep the file's
  uint8_t start_ch;
  uint32_t rz_nonce;
};

}  // extern "C"

namespace {

using namespace maburgs;

maburgs::ScoutEnergy from(const pal_tenergy& e) {
  maburgs::ScoutEnergy o;
  o.fa_valid = e.fa_valid;
  o.cca_ofdm = e.cca_ofdm;
  o.fa_ofdm = e.fa_ofdm;
  o.igi_valid = e.igi_valid;
  o.igi = e.igi;
  o.nhm_valid = e.nhm_valid;
  o.floor_valid = e.floor_valid;
  o.floor_dbm = e.floor_dbm;
  return o;
}

class TraceCard final : public LinkCard {
 public:
  explicit TraceCard(const pal_tcard_vt& vt) : v_(vt) {}
  bool retune(uint8_t ch) override { return v_.retune(v_.user, ch) != 0; }
  bool retune_width(uint8_t ch, uint8_t w) override { return v_.retune_width(v_.user, ch, w) != 0; }
  ScoutEnergy read_energy(bool with_nhm) override {
    pal_tenergy e{};
    v_.read_energy(v_.user, with_nhm ? 1 : 0, &e);
    return from(e);
  }
  ScoutEnergy read_energy_scout() override {
    pal_tenergy e{};
    v_.read_energy_scout(v_.user, &e);
    return from(e);
  }
  ScoutFrames frames() const override {
    uint64_t f[3] = {};
    v_.frames(v_.user, f);
    return ScoutFrames{f[0], f[1], f[2]};
  }
  bool arm_nhm_busy(uint16_t p) override { return v_.arm_nhm_busy(v_.user, p) != 0; }
  NhmBusyRead read_nhm_busy() override {
    pal_tnhm r{};
    v_.read_nhm_busy(v_.user, &r);
    NhmBusyRead o;
    o.valid = r.valid;
    for (int i = 0; i < 12; ++i) o.buckets[i] = r.buckets[i];
    o.duration = r.duration;
    o.period = r.period;
    return o;
  }
  bool open_and_start() override { return ready(); }
  void stop() override {}
  bool ready() const override { return v_.ready(v_.user) != 0; }
  bool alive() const override { return v_.alive(v_.user) != 0; }
  CardCaps caps() const override {
    CardCaps c;
    c.valid = true;
    c.snr_ok = v_.snr_ok(v_.user) != 0;
    return c;
  }
  uint64_t rx_frames() const override { return v_.rx_frames(v_.user); }
  uint64_t foreign() const override { return v_.foreign(v_.user); }
  bool send_control(const std::vector<uint8_t>& b) override {
    return v_.send_control(v_.user, b.data(), b.size()) != 0;
  }
  uint64_t tx_frames() const override { return v_.tx_frames(v_.user); }
  uint64_t tx_fail() const override { return v_.tx_fail(v_.user); }
  bool set_width(uint8_t ch, uint8_t w) override { return v_.set_width(v_.user, ch, w) != 0; }
  uint8_t channel() const override { return v_.channel(v_.user); }
  uint8_t width() const override { return v_.width(v_.user); }
  int tuned_central() override { return v_.tuned_central(v_.user); }
  bool can_scout() const override { return v_.can_scout(v_.user) != 0; }

 private:
  pal_tcard_vt v_;
};

std::string fmt(const char* f, ...) {
  char b[1024];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(b, sizeof b, f, ap);
  va_end(ap);
  return b;
}

// Readable, then the exact bits: equal lines mean bit-equal doubles.
std::string num(double x) {
  if (std::isnan(x)) return "nan";
  uint64_t u;
  std::memcpy(&u, &x, sizeof u);
  return fmt("%.3f~%llx", x, static_cast<unsigned long long>(u));
}

class TraceSink final : public ChannelSink {
 public:
  explicit TraceSink(const pal_tenv& env) : env_(env) {}
  void emit(const std::string& s) { env_.line(env_.user, s.data(), s.size()); }
  void dwell(double t, int card, const ScoutDwell& d) override {
    const auto& s = d.survey;
    emit(fmt("D t=%s card=%d seq=%llu ch=%u w=%u off=%u round=%llu ts=%lld te=%lld rt=%lld st=%d "
             "ob=%lld vfa=%d fa=%u cca=%u vigi=%d igi=%u vnhm=%d fl=%u own=%u fr=%u flv=%d "
             "fld=%d ins=%d to=%lld rd=%lld bk=%lld bv=%d bp=%s",
             num(t).c_str(), card, static_cast<unsigned long long>(s.seq), s.def.primary,
             s.def.width == CHANNEL_WIDTH_40 ? 40u : 20u, s.def.offset,
             static_cast<unsigned long long>(s.round), static_cast<long long>(s.t_start_ms),
             static_cast<long long>(s.t_end_ms), static_cast<long long>(s.retune_us), s.settle_ms,
             static_cast<long long>(s.observe_ms), s.valid_fa, s.fa_ofdm, s.cca_ofdm, s.valid_igi,
             s.igi, s.valid_nhm, s.flags, s.dvr_frames, s.frames, d.floor_valid, d.floor_dbm,
             d.in_session, static_cast<long long>(d.to_us), static_cast<long long>(d.read_us),
             static_cast<long long>(d.back_us), d.busy_valid, num(d.busy_pct).c_str()));
  }
  void pick(double t, std::optional<uint8_t> p, uint64_t rounds, const std::vector<RankEntry>& all,
            int mr) override {
    std::string s = fmt("K t=%s pick=%s rounds=%llu mr=%d all=", num(t).c_str(),
                        p ? fmt("%u", *p).c_str() : "-", static_cast<unsigned long long>(rounds), mr);
    for (const auto& e : all)
      s += fmt("%u:%u:%u:%d:%d:%d:%s;", e.ch, e.worst_busy, e.visits, e.floor_valid, e.floor_dbm,
               e.busy_valid, num(e.worst_busy_pct).c_str());
    emit(s);
  }
  void move(const MoveEvent& e) override {
    emit(fmt("M t=%s card=%d %u->%u %s", num(e.t_ms).c_str(), e.card, e.from, e.to,
             to_string(e.reason)));
  }
  void verdict(double t, const VerdictOut& o, const std::vector<VerdictCardIn>& cards,
               const VerdictLinkIn& l) override {
    std::string s = fmt("V t=%s v=%s ev=%u rr=%d trig=%d rref=%s dr=%s ts=%s tw=%s frz=%d first=%d "
                        "link=%s:%u:%d:%u cards=",
                        num(t).c_str(), to_string(o.v), o.evidence, o.ref_rung, o.trigger,
                        num(o.ref_rssi_dbm).c_str(), num(o.d_rssi_db).c_str(),
                        num(o.t_start_ms).c_str(), num(o.t_ms).c_str(), o.ref_frozen,
                        o.first_interfered, num(l.pre_fec_loss).c_str(), l.recovered, l.starved,
                        l.au_count);
    for (const auto& c : cards)
      s += fmt("%d:%u:%u:%u:%u:%s:%s:%d:%d:%s:%s;", c.valid, c.foreign, c.crc_fail, c.fa, c.cca,
               num(c.rssi_dbm).c_str(), num(c.snr_db).c_str(), c.snr_valid, c.busy_valid,
               num(c.nhm_busy_pct).c_str(), num(c.own_air_pct).c_str());
    emit(s);
  }
  void hop(const HopEvent& e) override {
    emit(fmt("H t=%s %s epoch=%u target=%u score=%u el=%s reason=%s", num(e.t_ms).c_str(),
             e.kind.c_str(), e.epoch, e.target, e.score, num(e.elapsed_ms).c_str(),
             e.reason.c_str()));
  }
  void log(const std::string& line) override { emit("L " + line); }

 private:
  pal_tenv env_;
};

std::string opt_d(const std::optional<double>& x) { return x ? num(*x) : "-"; }

}  // namespace

struct pal_ctrace {
  pal_tenv env;
  Config cfg;
  std::vector<std::unique_ptr<TraceCard>> cards;
  std::vector<LinkCard*> ptrs;
  std::unique_ptr<TraceSink> sink;
  std::unique_ptr<VrxController> vrx;
  std::unique_ptr<Aggregator> agg;
  std::unique_ptr<ChannelCore> core;
};

extern "C" {

pal_ctrace* pal_ctrace_new(const pal_tcfg* c, const pal_tcard_vt* cards, size_t n,
                           const pal_tenv* env, char* err, size_t err_cap) {
  try {
    auto t = std::make_unique<pal_ctrace>();
    t->env = *env;
    t->cfg = load_config(c->config_path);
    t->cfg.radio.channels.assign(c->channels, c->channels + c->n_channels);
    t->cfg.radio.width = c->width;
    if (c->pin) t->cfg.radio.pin = c->pin;
    if (c->max_ms >= 0) t->cfg.radio.scan.max_ms = c->max_ms;
    for (size_t i = 0; i < n; ++i) {
      t->cards.push_back(std::make_unique<TraceCard>(cards[i]));
      t->ptrs.push_back(t->cards.back().get());
    }
    t->sink = std::make_unique<TraceSink>(t->env);
    VrxCfg vc = vrx_cfg_from(t->cfg, c->start_ch);
    vc.rz_nonce = c->rz_nonce;
    t->vrx = std::make_unique<VrxController>(vc);
    t->agg = std::make_unique<Aggregator>(t->cfg.uep_layers(), 32, static_cast<int>(n), 0);
    ChannelCoreCfg cc;
    cc.radio = t->cfg.radio;
    cc.hop = t->cfg.hop;
    cc.key = t->cfg.link.key;
    cc.start_ch = c->start_ch;
    cc.n_usb = static_cast<int>(n);
    cc.threaded = false;
    cc.store_name = "the-store";
    pal_ctrace* raw = t.get();
    t->core = std::make_unique<ChannelCore>(
        cc, t->ptrs, *t->vrx, *t->sink,
        [raw](uint8_t ch) {
          raw->sink->emit(fmt("S %u", ch));
          return true;
        },
        [raw] { return raw->env.now_ms(raw->env.user); },
        [raw] { return raw->env.now_us(raw->env.user); },
        [raw](int ms) { raw->env.sleep_ms(raw->env.user, ms); });
    return t.release();
  } catch (const std::exception& e) {
    if (err && err_cap) std::snprintf(err, err_cap, "%s", e.what());
  } catch (...) {
    if (err && err_cap) std::snprintf(err, err_cap, "unknown C++ exception");
  }
  return nullptr;
}

void pal_ctrace_free(pal_ctrace* t) {
  if (t && t->core) t->core->shutdown();
  delete t;
}

void pal_ctrace_tick(pal_ctrace* t, double now_ms, int session, int tx, int* frozen, int* busy) {
  ChannelTickIn in;
  in.now_ms = now_ms;
  in.in_session = session != 0;
  in.tx_card = tx;
  in.agg = t->agg.get();
  const auto o = t->core->tick(in);
  *frozen = o.tx_frozen;
  *busy = o.dwell_busy;
}

void pal_ctrace_vrx_rc(pal_ctrace* t, double now_ms, const uint8_t* b, size_t n) {
  t->vrx->on_rc_frame(b, n, now_ms);
}
void pal_ctrace_vrx_video(pal_ctrace* t, double now_ms) { t->vrx->on_video(now_ms); }
// One vrx.step() with a healthy window; returns the frame length (0 = none).
size_t pal_ctrace_vrx_step(pal_ctrace* t, double now_ms, uint8_t* out, size_t cap, int* is_disc) {
  LinkHealth h;
  h.sample_valid = true;
  const auto o = t->vrx->step(now_ms, h);
  if (!o || o->frame.size() > cap) return 0;
  std::copy(o->frame.begin(), o->frame.end(), out);
  *is_disc = o->is_disc;
  return o->frame.size();
}
int pal_ctrace_vrx_session(const pal_ctrace* t) {
  return t->vrx->link_state() == VrxState::SESSION;
}
uint8_t pal_ctrace_vrx_agreed(const pal_ctrace* t) { return t->vrx->agreed_channel(); }
uint8_t pal_ctrace_vrx_proposal(const pal_ctrace* t) { return t->vrx->proposal(); }
void pal_ctrace_vrx_move_edge(pal_ctrace* t) { t->vrx->test_set_move_edge(); }

void pal_ctrace_on_rc_body(pal_ctrace* t, uint8_t ch) { t->core->on_rc_body(ch); }
void pal_ctrace_on_session_opened(pal_ctrace* t, const uint8_t* b, size_t n, double now_ms) {
  if (const auto a = mabur::rc::parse_disc_ack(b, n)) t->core->on_session_opened(*a, now_ms);
}
void pal_ctrace_note_video(pal_ctrace* t, uint8_t ch) { t->core->note_video(ch); }
void pal_ctrace_note_sent(pal_ctrace* t, int ok, int rcf) { t->core->note_sent(ok, rcf); }
void pal_ctrace_note_au_end(pal_ctrace* t) { t->core->note_au_end(); }
void pal_ctrace_run_scout_step(pal_ctrace* t) { t->core->run_scout_step(); }
void pal_ctrace_run_inflight_step(pal_ctrace* t) { t->core->run_inflight_step(); }
void pal_ctrace_run_sampler_step(pal_ctrace* t) { t->core->run_sampler_step(); }
void pal_ctrace_on_card_died(pal_ctrace* t, int card) { t->core->on_card_died(card); }
void pal_ctrace_on_card_reopened(pal_ctrace* t, int card) { t->core->on_card_reopened(card); }
int pal_ctrace_may_send(pal_ctrace* t, int card) { return t->core->may_send(card); }
uint8_t pal_ctrace_op(const pal_ctrace* t) { return t->core->op(); }
int pal_ctrace_pick_open(const pal_ctrace* t) { return t->core->pick_open(); }

size_t pal_ctrace_disc_targets(const pal_ctrace* t, int tx, int* out, size_t cap) {
  const auto v = t->core->disc_targets(tx);
  for (size_t i = 0; i < v.size() && i < cap; ++i) out[i] = v[i];
  return v.size();
}

size_t pal_ctrace_disc_for_card(const pal_ctrace* t, const uint8_t* f, size_t n, int card,
                                uint8_t* out, size_t cap) {
  const auto v = t->core->disc_for_card(std::vector<uint8_t>(f, f + n), card);
  if (v.size() > cap) return 0;
  std::copy(v.begin(), v.end(), out);
  return v.size();
}

// The snapshot as one line ("P ..."), through the env's line callback.
void pal_ctrace_snapshot(pal_ctrace* t, int tx) {
  const auto s = t->core->snapshot(tx);
  const auto& h = s.hop;
  std::string l = fmt(
      "P ch=%d state=%s rounds=%llu pick=%s v=%s ev=%d rr=%s epoch=%d hs=%s target=%s hops=%u "
      "holds=%u last=%s drop=%llu gated=%llu energy=",
      s.channel, s.scan_state, static_cast<unsigned long long>(s.scan_rounds),
      s.scan_pick ? fmt("%d", *s.scan_pick).c_str() : "-", h.verdict, h.evidence,
      h.ref_rung ? fmt("%d", *h.ref_rung).c_str() : "-", h.epoch, h.state,
      h.target ? fmt("%d", *h.target).c_str() : "-", h.hops, h.holds,
      h.last_ms ? fmt("%llu", static_cast<unsigned long long>(*h.last_ms)).c_str() : "-",
      static_cast<unsigned long long>(h.sampler_dropped),
      static_cast<unsigned long long>(s.scout_gated_sends));
  for (const auto& e : s.energy) {
    if (!e) {
      l += "-;";
      continue;
    }
    l += fmt("%u:%u:%llu:%llu:%s:%s;", e->cca, e->fa, static_cast<unsigned long long>(e->own),
             static_cast<unsigned long long>(e->foreign), opt_d(e->busy_pct).c_str(),
             opt_d(e->own_air_pct).c_str());
  }
  l += " dwell=";
  for (const auto& d : s.dwell)
    l += d ? fmt("%u:%u:%u;", d->visits, d->score, d->cost_us) : std::string("-;");
  t->sink->emit(l);
}

}  // extern "C"
