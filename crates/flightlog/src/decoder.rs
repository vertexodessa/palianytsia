//! The `decoder` records of a session (one per second with video, written
//! by the Android decode loop) summed up: whether the decoder was the
//! latency tail or dropped frames can be read from the session directory.

use serde_json::Value;

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct DecoderTotals {
    /// Records (seconds with video).
    pub seconds: u64,
    pub aus_in: u64,
    pub frames: u64,
    pub stale: u64,
    pub no_input: u64,
    pub oversize: u64,
    pub errors: u64,
    pub au_drops: u64,
    /// The longest an AU waited for the decode thread (µs).
    pub wait_max_us: u64,
    /// The seconds' queue-to-output p50s, median over the session (µs).
    pub q_p50_us: u64,
    /// The worst second's p99 and the session's max (µs).
    pub q_p99_max_us: u64,
    pub q_max_us: u64,
    p50s: Vec<u64>,
}

impl DecoderTotals {
    pub(crate) fn event(&mut self, ev: &str, v: &Value) {
        if ev != "decoder" {
            return;
        }
        let u = |k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
        self.seconds += 1;
        self.aus_in += u("aus_in");
        self.frames += u("frames");
        self.stale += u("stale");
        self.no_input += u("no_input");
        self.oversize += u("oversize");
        self.errors += u("errors");
        self.au_drops += u("au_drops");
        self.wait_max_us = self.wait_max_us.max(u("wait_max_us"));
        if u("q_n") > 0 {
            self.p50s.push(u("q_p50_us"));
            self.q_p99_max_us = self.q_p99_max_us.max(u("q_p99_us"));
            self.q_max_us = self.q_max_us.max(u("q_max_us"));
        }
    }

    pub(crate) fn finish(&mut self) {
        self.p50s.sort_unstable();
        self.q_p50_us = crate::keyframes::pct(&self.p50s, 50);
        self.p50s = Vec::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn records_add_up_and_the_latency_keeps_its_tail() {
        let mut d = DecoderTotals::default();
        let rec = |p50, p99, stale| {
            json!({"aus_in": 90, "frames": 88, "stale": stale, "no_input": 0, "oversize": 0,
                   "errors": 0, "au_drops": 1, "wait_max_us": 300 * p50, "q_n": 88,
                   "q_p50_us": p50, "q_p99_us": p99, "q_max_us": p99 + 1000})
        };
        d.event("decoder", &rec(9_000, 14_000, 2));
        d.event("video_stall", &rec(1, 1, 1));
        d.event("decoder", &rec(11_000, 40_000, 0));
        d.event("decoder", &rec(10_000, 15_000, 0));
        d.finish();
        assert_eq!(
            (d.seconds, d.aus_in, d.frames, d.stale, d.au_drops),
            (3, 270, 264, 2, 3)
        );
        assert_eq!(
            (d.q_p50_us, d.q_p99_max_us, d.q_max_us),
            (10_000, 40_000, 41_000)
        );
        assert_eq!(d.wait_max_us, 3_300_000);
    }
}
