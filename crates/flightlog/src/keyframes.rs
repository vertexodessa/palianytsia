//! Keyframe traffic of one session, from what the session already holds:
//! `idr_req` / `idr_heal` events, the IRAP flag and size in au.log and the
//! rung in the stats rows. A keyframe storm shows here without reading the
//! events by hand, and the bytes per rung are what a keyframe budget needs.

use serde_json::Value;

#[derive(Debug, Default)]
pub struct Keyframes {
    /// First tries of a request, by reason, in first-seen order.
    pub by_reason: Vec<(String, u64)>,
    /// Retries of a request (tries after the first).
    pub retries: u64,
    /// The highest retry number one request reached.
    pub max_retry: u64,
    /// Requests their retries did not heal (`idr_stuck`).
    pub stuck: u64,
    /// Request to clearing IRAP, per heal, sorted.
    pub heal_ms: Vec<u64>,
    /// IRAP sizes by the rung of the stats row before them, sorted by rung;
    /// rung -1 = before the first stats row or with no ladder.
    pub bytes_by_rung: Vec<(i64, Vec<u64>)>,
    /// The most first tries inside any 10 s, and when that window began (µs).
    pub peak_10s: (u64, u64),
    /// Runs of `RUN_MIN` or more base AUs in a row that are all keyframes.
    pub runs: Vec<KeyRun>,
    /// Complete P frames (both layers) while the drone was not in low power,
    /// by video mode in first-seen order: what an encoder P-frame ceiling has
    /// to let through.
    pub p_by_mode: Vec<(String, Vec<u64>)>,
    idr_at: Vec<(u64, u64)>,
    rung_at: Vec<(u64, i64)>,
    req_at: Vec<u64>,
    /// Every idr_req try, first or retry.
    req_all: Vec<u64>,
    rtt_ms: Vec<f64>,
    lp_at: Vec<(u64, bool)>,
    mode_at: Vec<(u64, String)>,
    p_at: Vec<(u64, u64)>,
    run: Option<KeyRun>,
}

/// Base keyframes in a row from this many on make a run.
pub const RUN_MIN: u64 = 3;
/// An unrequested run this long is an encoder wedge (a P-frame ceiling below
/// an ordinary P frame turns every frame into a keyframe), not a heal.
pub const WEDGE_MIN: u64 = 10;
/// kB per megapixel the drone's P-frame ceiling never goes below
/// (`venc.superframe_p_floor_kb`, mabur default since v0.16.2).
pub const P_FLOOR_KB: u64 = 14;

/// Consecutive base keyframes.
#[derive(Clone, Debug, PartialEq)]
pub struct KeyRun {
    pub start_us: u64,
    pub n: u64,
    pub complete: u64,
    /// The GS asked for a keyframe shortly before the run began.
    pub requested: bool,
}

impl KeyRun {
    pub fn wedge(&self) -> bool {
        !self.requested && self.n >= WEDGE_MIN
    }
}

/// Width x height of a VENC_MODE like "120@1728x816/binned".
pub fn mode_pixels(mode: &str) -> Option<(u64, u64)> {
    let size = mode.split('@').nth(1)?.split('/').next()?;
    let (w, h) = size.split_once('x')?;
    Some((w.parse().ok()?, h.parse().ok()?))
}

/// The value at percentile `q` (0..=100) of sorted `v`, nearest rank below.
pub fn pct(v: &[u64], q: usize) -> u64 {
    if v.is_empty() {
        return 0;
    }
    v[(v.len() - 1) * q / 100]
}

impl Keyframes {
    pub fn requests(&self) -> u64 {
        self.by_reason.iter().map(|(_, n)| n).sum()
    }

    pub(crate) fn idr(&mut self, t_us: u64, bytes: u64) {
        self.idr_at.push((t_us, bytes));
    }

    /// One au.log row, in file order.
    pub(crate) fn au(&mut self, t_us: u64, sid: u64, bytes: u64, idr: bool, complete: bool) {
        if !idr && complete {
            self.p_at.push((t_us, bytes));
        }
        if sid != 0 {
            return;
        }
        if idr {
            let r = self.run.get_or_insert(KeyRun {
                start_us: t_us,
                n: 0,
                complete: 0,
                requested: false,
            });
            r.n += 1;
            r.complete += u64::from(complete);
        } else {
            self.end_run();
        }
    }

    fn end_run(&mut self) {
        if let Some(r) = self.run.take().filter(|r| r.n >= RUN_MIN) {
            self.runs.push(r);
        }
    }

    pub(crate) fn stats(&mut self, t_us: u64, v: &Value) {
        if let Some(r) = v.get("rung").and_then(Value::as_i64) {
            self.rung_at.push((t_us, r));
        }
        if let Some(r) = v.get("rtt_ms").and_then(Value::as_f64) {
            self.rtt_ms.push(r);
        }
        if let Some(lp) = v.get("drone_lp").and_then(Value::as_bool) {
            self.lp_at.push((t_us, lp));
        }
    }

    pub(crate) fn event(&mut self, ev: &str, v: &Value) {
        let u = |k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
        match ev {
            "params" => {
                if let Some(m) = v.pointer("/values/VENC_MODE").and_then(Value::as_str) {
                    self.mode_at.push((u("t_us"), m.into()));
                }
            }
            "param_set" if v.get("id").and_then(Value::as_str) == Some("VENC_MODE") => {
                if let Some(m) = v.get("value").and_then(Value::as_str) {
                    self.mode_at.push((u("t_us"), m.into()));
                }
            }
            "idr_req" => {
                self.req_all.push(u("t_us"));
                let retry = u("retry");
                if retry == 0 {
                    let r = v.get("reason").and_then(Value::as_str).unwrap_or("?");
                    match self.by_reason.iter_mut().find(|(k, _)| k == r) {
                        Some((_, n)) => *n += 1,
                        None => self.by_reason.push((r.into(), 1)),
                    }
                    self.req_at.push(u("t_us"));
                } else {
                    self.retries += 1;
                    self.max_retry = self.max_retry.max(retry);
                }
            }
            "idr_heal" => self.heal_ms.push(u("heal_ms")),
            "idr_stuck" => self.stuck += 1,
            _ => {}
        }
    }

    pub(crate) fn finish(&mut self) {
        self.end_run();
        self.mark_requested();
        self.p_sizes();
        self.heal_ms.sort_unstable();
        // au.log and the stats rows are both in time order.
        let mut ri = 0;
        let mut rung = -1;
        for &(t, bytes) in &self.idr_at {
            while ri < self.rung_at.len() && self.rung_at[ri].0 <= t {
                rung = self.rung_at[ri].1;
                ri += 1;
            }
            match self.bytes_by_rung.iter_mut().find(|(r, _)| *r == rung) {
                Some((_, v)) => v.push(bytes),
                None => self.bytes_by_rung.push((rung, vec![bytes])),
            }
        }
        self.bytes_by_rung.sort_by_key(|(r, _)| *r);
        for (_, v) in &mut self.bytes_by_rung {
            v.sort_unstable();
        }
        self.req_at.sort_unstable();
        let mut lo = 0;
        for hi in 0..self.req_at.len() {
            while self.req_at[hi] - self.req_at[lo] >= 10_000_000 {
                lo += 1;
            }
            let n = (hi - lo + 1) as u64;
            if n > self.peak_10s.0 {
                self.peak_10s = (n, self.req_at[lo]);
            }
        }
        self.idr_at = Vec::new();
        self.rung_at = Vec::new();
        self.req_at = Vec::new();
    }

    /// A run is requested when an idr_req went out within the round trip
    /// plus two frames plus 200 ms (encode and air of a keyframe) before it.
    fn mark_requested(&mut self) {
        self.req_all.sort_unstable();
        self.rtt_ms.sort_by(f64::total_cmp);
        let rtt = self
            .rtt_ms
            .get(self.rtt_ms.len() / 2)
            .copied()
            .unwrap_or(10.0);
        let win = ((rtt + 2.0 * 1000.0 / 60.0 + 200.0) * 1000.0) as u64;
        for r in &mut self.runs {
            let i = self.req_all.partition_point(|&t| t <= r.start_us);
            r.requested = i > 0 && r.start_us - self.req_all[i - 1] <= win;
        }
        self.req_all = Vec::new();
        self.rtt_ms = Vec::new();
    }

    fn p_sizes(&mut self) {
        self.mode_at.sort_by_key(|(t, _)| *t);
        let (mut mi, mut li) = (0, 0);
        let (mut mode, mut lp): (Option<&str>, bool) = (None, false);
        for &(t, bytes) in &self.p_at {
            while mi < self.mode_at.len() && self.mode_at[mi].0 <= t {
                mode = Some(&self.mode_at[mi].1);
                mi += 1;
            }
            while li < self.lp_at.len() && self.lp_at[li].0 <= t {
                lp = self.lp_at[li].1;
                li += 1;
            }
            let Some(m) = mode.filter(|_| !lp) else {
                continue;
            };
            match self.p_by_mode.iter_mut().find(|(k, _)| k == m) {
                Some((_, v)) => v.push(bytes),
                None => self.p_by_mode.push((m.to_string(), vec![bytes])),
            }
        }
        for (_, v) in &mut self.p_by_mode {
            v.sort_unstable();
        }
        self.p_at = Vec::new();
        self.lp_at = Vec::new();
        self.mode_at = Vec::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn requests_heals_and_sizes_are_counted_per_reason_and_rung() {
        let mut k = Keyframes::default();
        k.idr(500_000, 40_000);
        k.stats(1_000_000, &json!({"rung": 0}));
        k.stats(2_000_000, &json!({"rung": 2}));
        k.event(
            "idr_req",
            &json!({"t_us": 1_100_000, "reason": "lost", "retry": 0}),
        );
        k.event(
            "idr_req",
            &json!({"t_us": 1_400_000, "reason": "lost", "retry": 1}),
        );
        k.event(
            "idr_req",
            &json!({"t_us": 1_900_000, "reason": "lost", "retry": 2}),
        );
        k.event("idr_stuck", &json!({"reason": "lost", "retries": 4}));
        k.event("idr_heal", &json!({"heal_ms": 900}));
        k.idr(1_950_000, 30_000);
        k.idr(2_500_000, 60_000);
        k.idr(2_600_000, 50_000);
        k.event(
            "idr_req",
            &json!({"t_us": 12_000_000, "reason": "hole", "retry": 0}),
        );
        k.event(
            "idr_req",
            &json!({"t_us": 12_500_000, "reason": "lost", "retry": 0}),
        );
        k.event("idr_heal", &json!({"heal_ms": 100}));
        k.finish();
        assert_eq!(k.by_reason, vec![("lost".into(), 2), ("hole".into(), 1)]);
        assert_eq!(
            (k.requests(), k.retries, k.max_retry, k.stuck),
            (3, 2, 2, 1)
        );
        assert_eq!(k.heal_ms, vec![100, 900]);
        assert_eq!(
            k.bytes_by_rung,
            vec![
                (-1, vec![40_000]),
                (0, vec![30_000]),
                (2, vec![50_000, 60_000])
            ]
        );
        // 1.1 s and 12.0 s are 10.9 s apart: the busiest 10 s holds two.
        assert_eq!(k.peak_10s, (2, 12_000_000));
    }

    #[test]
    fn keyframe_runs_are_split_by_p_frames_and_marked_requested_or_not() {
        let mut k = Keyframes::default();
        k.stats(0, &json!({"rtt_ms": 10.0, "drone_lp": false}));
        k.event(
            "params",
            &json!({"t_us": 0, "values": {"VENC_MODE": "120@1728x816/binned"}}),
        );
        // A requested heal: two keyframes (below RUN_MIN), then P frames.
        k.event("idr_req", &json!({"t_us": 1_000_000, "retry": 0}));
        k.au(1_100_000, 0, 40_000, true, true);
        k.au(1_108_000, 0, 40_000, true, true);
        for i in 0..100 {
            k.au(1_200_000 + i * 8_000, 0, 6_000 + i, false, true);
            k.au(1_204_000 + i * 8_000, 1, 3_000, false, true);
        }
        // A wedge: 12 keyframes nobody asked for, three of them cut short.
        for i in 0..12 {
            k.au(5_000_000 + i * 8_000, 0, 9_000, true, i % 4 != 0);
            k.au(5_004_000 + i * 8_000, 1, 2_000, true, true);
        }
        k.au(5_200_000, 0, 6_000, false, true);
        // A requested run of three: an idr_req 100 ms before.
        k.event("idr_req", &json!({"t_us": 7_900_000, "retry": 1}));
        for i in 0..3 {
            k.au(8_000_000 + i * 8_000, 0, 30_000, true, true);
        }
        // Low power: these P frames are not full-power sizes.
        k.stats(9_000_000, &json!({"drone_lp": true}));
        k.au(9_100_000, 0, 99_000, false, true);
        k.finish();
        assert_eq!(
            k.runs,
            vec![
                KeyRun {
                    start_us: 5_000_000,
                    n: 12,
                    complete: 9,
                    requested: false
                },
                KeyRun {
                    start_us: 8_000_000,
                    n: 3,
                    complete: 3,
                    requested: true
                },
            ]
        );
        assert!(k.runs[0].wedge() && !k.runs[1].wedge());
        let (m, p) = &k.p_by_mode[0];
        assert_eq!(
            (m.as_str(), p.len(), pct(p, 100)),
            ("120@1728x816/binned", 201, 6_099)
        );
        assert_eq!(mode_pixels(m), Some((1728, 816)));
    }
}
