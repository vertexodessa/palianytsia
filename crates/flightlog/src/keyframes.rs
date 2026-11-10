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
    /// Request to clearing IRAP, per heal, sorted.
    pub heal_ms: Vec<u64>,
    /// IRAP sizes by the rung of the stats row before them, sorted by rung;
    /// rung -1 = before the first stats row or with no ladder.
    pub bytes_by_rung: Vec<(i64, Vec<u64>)>,
    /// The most first tries inside any 10 s, and when that window began (µs).
    pub peak_10s: (u64, u64),
    idr_at: Vec<(u64, u64)>,
    rung_at: Vec<(u64, i64)>,
    req_at: Vec<u64>,
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

    pub(crate) fn stats(&mut self, t_us: u64, v: &Value) {
        if let Some(r) = v.get("rung").and_then(Value::as_i64) {
            self.rung_at.push((t_us, r));
        }
    }

    pub(crate) fn event(&mut self, ev: &str, v: &Value) {
        let u = |k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
        match ev {
            "idr_req" => {
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
            _ => {}
        }
    }

    pub(crate) fn finish(&mut self) {
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
        assert_eq!((k.requests(), k.retries, k.max_retry), (3, 2, 2));
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
}
