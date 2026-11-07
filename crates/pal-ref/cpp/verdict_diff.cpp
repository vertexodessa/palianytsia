// mabur's HopVerdict behind a flat C ABI: the oracle for the Rust port
// (pal-core chan::verdict), driven window by window from tests/verdict_diff.rs.
#include <vector>

#include "hop_verdict.h"

using namespace maburgs;

extern "C" {

struct pal_vcfg {
  int window_ms, persist;
  double blocked_pct;
  double loss_pct, recovered_x;
  int recovered_min, weak_rssi_dbm, weak_snr_db, occupied_windows;
  double detect_pct;
  int detect_windows, detect_impaired;
};

struct pal_vcard {
  int valid, snr_valid, busy_valid;
  double rssi_dbm, snr_db, nhm_busy_pct, own_air_pct;
};

struct pal_vout {
  int v, evidence, ref_rung, trigger, ref_frozen, first_blocked;
  double t_start_ms, t_ms;
};

struct pal_verdict {
  HopVerdict v;
};

pal_verdict* pal_verdict_new(const pal_vcfg* c) {
  HopCfg h;
  h.window_ms = c->window_ms;
  h.persist = c->persist;
  h.verdict.loss_pct = c->loss_pct;
  h.verdict.recovered_x = c->recovered_x;
  h.verdict.recovered_min = c->recovered_min;
  h.verdict.weak_rssi_dbm = c->weak_rssi_dbm;
  h.verdict.weak_snr_db = c->weak_snr_db;
  h.verdict.occupied_windows = c->occupied_windows;
  h.verdict.detect_pct = c->detect_pct;
  h.verdict.detect_windows = c->detect_windows;
  h.verdict.detect_impaired = c->detect_impaired;
  BusyCfg b;
  b.blocked_pct = c->blocked_pct;
  return new pal_verdict{HopVerdict(h, b)};
}

void pal_verdict_free(pal_verdict* p) { delete p; }
void pal_verdict_reset(pal_verdict* p) { p->v.reset(); }
void pal_verdict_new_session(pal_verdict* p) { p->v.new_session(); }
void pal_verdict_landed(pal_verdict* p, double now_ms) { p->v.landed(now_ms); }
void pal_verdict_remap(pal_verdict* p, int rung) { p->v.remap_ref_rung(rung); }

void pal_verdict_window(pal_verdict* p, double now_ms, const pal_vcard* cards, size_t n,
                        double loss, uint32_t recovered, int rung, pal_vout* out) {
  std::vector<VerdictCardIn> in(n);
  for (size_t i = 0; i < n; ++i) {
    in[i].valid = cards[i].valid != 0;
    in[i].snr_valid = cards[i].snr_valid != 0;
    in[i].busy_valid = cards[i].busy_valid != 0;
    in[i].rssi_dbm = cards[i].rssi_dbm;
    in[i].snr_db = cards[i].snr_db;
    in[i].nhm_busy_pct = cards[i].nhm_busy_pct;
    in[i].own_air_pct = cards[i].own_air_pct;
  }
  VerdictLinkIn l;
  l.pre_fec_loss = loss;
  l.recovered = recovered;
  const VerdictOut o = p->v.window(now_ms, in, l, rung);
  out->v = static_cast<int>(o.v);
  out->evidence = o.evidence;
  out->ref_rung = o.ref_rung;
  out->trigger = o.trigger;
  out->ref_frozen = o.ref_frozen;
  out->first_blocked = o.first_blocked;
  out->t_start_ms = o.t_start_ms;
  out->t_ms = o.t_ms;
}

}  // extern "C"
