//! The scorecard numbers a flight log holds (pal-ttb.1), one JSON line per
//! session for `pal log DIR --kpi`, so a release's numbers are read off the
//! logs and never typed by hand.
//!
//! - time to picture: from the first record of the session (the link
//!   starts) to the drone being found (`session` true) and to the first
//!   complete keyframe AU (the first picture the decoder can show);
//! - outage recovery: every gap of `OUTAGE_US` or more between base AUs
//!   after the first picture. Heal: when the GS asked for a keyframe
//!   (`idr_req`) from the gap's start to `REQ_AFTER_US` after its end, from
//!   the first AU after the gap to the next `idr_heal`; with no request the
//!   reference chain held (or intra refresh mends it) and the heal is 0;
//!   a request with no heal before the next outage is unhealed;
//! - complete AUs %, decoder frames shown %, uplink % (median of the
//!   stats' `uplink_pct`);
//! - the GS stages per frame (spec 13.2) from the `frames` events: queue
//!   to codec input (t4 to t5) and codec input to output (t5 to t6), p50
//!   and p99 in µs.
//!
//! G2G, RCF heard at a fixed attenuation and GS load need the bench, not
//! a log.

use serde_json::{json, Value};

/// A gap between base AUs this long counts as an outage.
pub const OUTAGE_US: u64 = 500_000;
/// A keyframe request this soon after an outage's end belongs to it.
pub const REQ_AFTER_US: u64 = 1_000_000;

#[derive(Debug, Default, Clone)]
pub struct Kpi {
    t0: Option<u64>,
    found_us: Option<u64>,
    picture_us: Option<u64>,
    last_base: Option<u64>,
    /// (last base AU before, first base AU after) of every outage.
    gaps: Vec<(u64, u64)>,
    reqs: Vec<u64>,
    heals: Vec<u64>,
    /// (gap ms, heal ms or None), filled by `finish`.
    outages: Vec<(u64, Option<u64>)>,
    aus: u64,
    complete: u64,
    dec_in: u64,
    dec_shown: u64,
    uplink: Vec<u64>,
    /// Per frame out of the codec: t4 to t5 and t5 to t6, µs.
    wait: Vec<u64>,
    dec: Vec<u64>,
}

fn pct(v: &mut [u64], q: usize) -> Option<u64> {
    v.sort_unstable();
    (!v.is_empty()).then(|| v[(v.len() - 1) * q / 100])
}

/// The first entry of sorted `v` at or after `from`.
fn first_at(v: &[u64], from: u64) -> Option<u64> {
    v.get(v.partition_point(|&x| x < from)).copied()
}

impl Kpi {
    fn seen(&mut self, t: u64) {
        self.t0 = Some(self.t0.map_or(t, |t0| t0.min(t)));
    }

    pub(crate) fn au(&mut self, t: u64, sid: u64, idr: bool, complete: bool) {
        self.seen(t);
        self.aus += 1;
        self.complete += u64::from(complete);
        if idr && complete && self.picture_us.is_none() {
            self.picture_us = Some(t);
        }
        if sid != 0 {
            return;
        }
        if let (Some(_), Some(last)) = (self.picture_us, self.last_base) {
            if t.saturating_sub(last) >= OUTAGE_US {
                self.gaps.push((last, t));
            }
        }
        self.last_base = Some(t);
    }

    pub(crate) fn stats(&mut self, t: u64, v: &Value) {
        self.seen(t);
        if let Some(p) = v.get("uplink_pct").and_then(Value::as_u64) {
            self.uplink.push(p);
        }
    }

    pub(crate) fn event(&mut self, ev: &str, v: &Value) {
        let Some(t) = v.get("t_us").and_then(Value::as_u64) else {
            return;
        };
        self.seen(t);
        match ev {
            "session" if v.get("to") == Some(&Value::Bool(true)) => {
                self.found_us.get_or_insert(t);
            }
            "idr_req" => self.reqs.push(t),
            "idr_heal" => self.heals.push(t),
            "decoder" => {
                let u = |k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
                self.dec_in += u("aus_in");
                self.dec_shown += u("frames");
            }
            "frames" => {
                for r in v
                    .get("rows")
                    .and_then(Value::as_array)
                    .into_iter()
                    .flatten()
                {
                    let c = |i: usize| r.get(i).and_then(Value::as_u64);
                    if let (Some(w), Some(d)) = (c(1), c(2)) {
                        self.wait.push(w);
                        self.dec.push(d);
                    }
                }
            }
            _ => {}
        }
    }

    pub(crate) fn finish(&mut self) {
        self.reqs.sort_unstable();
        self.heals.sort_unstable();
        self.outages.clear();
        for (i, &(before, after)) in self.gaps.iter().enumerate() {
            let next = self.gaps.get(i + 1).map_or(u64::MAX, |g| g.0);
            let asked = first_at(&self.reqs, before).is_some_and(|r| r <= after + REQ_AFTER_US);
            let heal = if asked {
                first_at(&self.heals, before)
                    .filter(|&h| h <= next)
                    .map(|h| h.saturating_sub(after) / 1000)
            } else {
                Some(0)
            };
            self.outages
                .push((after.saturating_sub(before) / 1000, heal));
        }
    }

    pub fn json(&self) -> Value {
        let t0 = self.t0.unwrap_or(0);
        let since = |t: Option<u64>| t.map(|t| t.saturating_sub(t0) / 1000);
        let mut gaps: Vec<u64> = self.outages.iter().map(|o| o.0).collect();
        let mut heals: Vec<u64> = self.outages.iter().filter_map(|o| o.1).collect();
        let share = |a: u64, b: u64| (b > 0).then(|| (a as f64 * 1000.0 / b as f64).round() / 10.0);
        json!({
            "found_ms": since(self.found_us),
            "picture_ms": since(self.picture_us),
            "outages": self.outages.len(),
            "outage_gap_ms_p50": pct(&mut gaps, 50),
            "outage_gap_ms_max": pct(&mut gaps, 100),
            "heal_ms_p50": pct(&mut heals, 50),
            "heal_ms_max": pct(&mut heals, 100),
            "unhealed": self.outages.iter().filter(|o| o.1.is_none()).count(),
            "complete_au_pct": share(self.complete, self.aus),
            "shown_pct": share(self.dec_shown, self.dec_in),
            "uplink_pct_p50": pct(&mut self.uplink.clone(), 50),
            "frames": self.wait.len(),
            "queue_us_p50": pct(&mut self.wait.clone(), 50),
            "queue_us_p99": pct(&mut self.wait.clone(), 99),
            "decode_us_p50": pct(&mut self.dec.clone(), 50),
            "decode_us_p99": pct(&mut self.dec.clone(), 99),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_gs_stages_split_queue_from_decode() {
        let mut k = Kpi::default();
        k.event(
            "frames",
            &json!({"t_us": 1, "rows": [[1, 200, 3000, 1], [2, 400, 3500, 1], [3, 9000, 4000, 0]]}),
        );
        k.event(
            "frames",
            &json!({"t_us": 2, "rows": [[4, 300, 3200, 1], "junk"]}),
        );
        let j = k.json();
        assert_eq!(j["frames"], 4);
        assert_eq!(j["queue_us_p50"], 300);
        assert_eq!(j["queue_us_p99"], 400);
        assert_eq!(j["decode_us_p50"], 3200);
        assert_eq!(j["decode_us_p99"], 3500);
    }

    #[test]
    fn picture_time_and_outages_healed_by_request_or_needing_none() {
        let mut k = Kpi::default();
        k.event("cards", &json!({"t_us": 1_000_000}));
        k.event("session", &json!({"t_us": 3_000_000, "to": true}));
        // A broken keyframe, then the first whole one at 3.5 s.
        k.au(3_400_000, 0, true, false);
        k.au(3_500_000, 0, true, true);
        k.au(3_511_000, 1, false, true);
        k.au(3_522_000, 0, false, true);
        // 2 s without base AUs; the GS asks at 4.2 s, healed 300 ms after
        // the link is back.
        k.event("idr_req", &json!({"t_us": 4_200_000}));
        k.au(5_522_000, 0, false, false);
        k.au(5_822_000, 0, true, true);
        k.event("idr_heal", &json!({"t_us": 5_822_000}));
        // An outage with no request (nothing broke the chain): heal 0.
        k.au(6_500_000, 0, false, true);
        // One the log ends inside: asked, never healed.
        k.au(7_500_000, 0, false, true);
        k.event("idr_req", &json!({"t_us": 7_600_000}));
        k.finish();
        let j = k.json();
        assert_eq!(j["found_ms"], 2000);
        assert_eq!(j["picture_ms"], 2500);
        assert_eq!(j["outages"], 3);
        assert_eq!(j["outage_gap_ms_max"], 2000);
        assert_eq!(j["heal_ms_max"], 300);
        assert_eq!(j["heal_ms_p50"], 0);
        assert_eq!(j["unhealed"], 1);
        assert_eq!(j["complete_au_pct"], 75.0);
    }
}
