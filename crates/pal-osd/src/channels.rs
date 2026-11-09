//! The panel's channel choice as values: which channels a width offers,
//! which are chosen, what one click makes of it. The link applies the
//! result (`pal_core::config::with_channels` at start, `Cmd::Channels`
//! live); here only the rules the operator sees.
use pal_wire::ht40::ht40_offset;
use serde_json::Value;

use crate::json::{bool_or, opt_f64, str_or, u64_or};

/// The core's channel scan (`"chscan"` in the UI line).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ScanView {
    pub running: bool,
    pub pass: u64,
    pub passes: u64,
    pub rows: Vec<ScanRowView>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ScanRowView {
    pub ch: u8,
    /// Mean busy share of the air, percent; None before a reading.
    pub busy: Option<f64>,
    /// Foreign frames per second heard on it.
    pub foreign: Option<f64>,
    /// The link's channel: its reading includes the link itself.
    pub op: bool,
}

impl ScanView {
    pub fn parse(v: &Value) -> Option<Self> {
        let rows = v
            .get("rows")?
            .as_array()?
            .iter()
            .filter_map(|r| {
                Some(ScanRowView {
                    ch: u8::try_from(r.get("ch")?.as_u64()?).ok()?,
                    busy: opt_f64(r, "busy"),
                    foreign: opt_f64(r, "foreign"),
                    op: bool_or(r, "op", false),
                })
            })
            .collect();
        Some(Self {
            running: str_or(v, "state", "") == "running",
            pass: u64_or(v, "pass", 0),
            passes: u64_or(v, "passes", 0),
            rows,
        })
    }

    pub fn row(&self, ch: u8) -> Option<&ScanRowView> {
        self.rows.iter().find(|r| r.ch == ch)
    }
}

/// The set's channels a link at `bw` MHz can use: at 40 only those with a
/// standard HT40 pair.
pub fn offered(set: &[u8], bw: u8) -> Vec<u8> {
    set.iter()
        .copied()
        .filter(|&c| bw != 40 || ht40_offset(c) != 0)
        .collect()
}

/// The saved choice (`HudPrefs::link_channels`) as it applies: the saved
/// channels still offered, in set order; all of them when none is.
pub fn chosen(saved: &[u8], offered: &[u8]) -> Vec<u8> {
    let c: Vec<u8> = offered
        .iter()
        .copied()
        .filter(|ch| saved.contains(ch))
        .collect();
    if c.is_empty() {
        offered.to_vec()
    } else {
        c
    }
}

/// One click on `ch`: the new saved choice (empty = all), or None when the
/// click changes nothing (the last chosen channel stays). At 40 MHz every
/// chosen channel must sit on the same side of its pair (one HT40 offset
/// for the whole set, docs/bw40.md), so a channel from the other side
/// starts a new choice.
pub fn toggle(saved: &[u8], offered: &[u8], bw: u8, ch: u8) -> Option<Vec<u8>> {
    let mut cur = chosen(saved, offered);
    if cur.contains(&ch) {
        if cur.len() == 1 {
            return None;
        }
        cur.retain(|&c| c != ch);
    } else {
        if bw == 40 && cur.iter().any(|&c| ht40_offset(c) != ht40_offset(ch)) {
            cur.clear();
        }
        cur.push(ch);
    }
    Some(normal(cur, offered))
}

/// Set order; every offered channel = empty (all).
fn normal(cur: Vec<u8>, offered: &[u8]) -> Vec<u8> {
    if offered.iter().all(|c| cur.contains(c)) {
        return Vec::new();
    }
    offered
        .iter()
        .copied()
        .filter(|c| cur.contains(c))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const SET: [u8; 5] = [40, 64, 112, 144, 165];

    #[test]
    fn scan_view_reads_the_core_line() {
        let j = r#"{"pass":2,"passes":4,"state":"running","rows":[
            {"busy":12.5,"ch":40,"foreign":30.0,"n":2,"op":true},
            {"busy":null,"ch":64,"foreign":null,"n":0,"op":false}]}"#;
        let v = ScanView::parse(&serde_json::from_str(j).unwrap()).unwrap();
        assert!(v.running && v.pass == 2 && v.passes == 4);
        assert_eq!(v.row(40).unwrap().busy, Some(12.5));
        assert!(v.row(40).unwrap().op);
        assert_eq!(v.row(64).unwrap().busy, None);
        assert!(v.row(112).is_none());
    }

    #[test]
    fn forty_offers_only_pair_channels() {
        assert_eq!(offered(&SET, 20), SET.to_vec());
        assert_eq!(offered(&SET, 40), vec![40, 64, 112, 144]);
    }

    #[test]
    fn chosen_falls_back_to_all() {
        let o = offered(&SET, 40);
        assert_eq!(chosen(&[], &o), o);
        assert_eq!(chosen(&[144, 64], &o), vec![64, 144]);
        assert_eq!(chosen(&[165], &o), o, "165 is not offered at 40");
    }

    #[test]
    fn clicks_narrow_widen_and_keep_one() {
        let o = offered(&SET, 20);
        assert_eq!(toggle(&[], &o, 20, 64), Some(vec![40, 112, 144, 165]));
        assert_eq!(toggle(&[64], &o, 20, 64), None, "the last one stays");
        assert_eq!(toggle(&[64], &o, 20, 144), Some(vec![64, 144]));
        assert_eq!(toggle(&[64, 144], &o, 20, 64), Some(vec![144]));
        assert_eq!(
            toggle(&[40, 112, 144, 165], &o, 20, 64),
            Some(vec![]),
            "every channel again = all"
        );
    }

    #[test]
    fn forty_keeps_one_side_of_the_pair_grid() {
        // 36 is a lower primary (offset 1), 40 an upper one (offset 2).
        let o = offered(&[36, 40, 44, 48], 40);
        assert_eq!(toggle(&[40], &o, 40, 48), Some(vec![40, 48]));
        assert_eq!(toggle(&[40, 48], &o, 40, 36), Some(vec![36]));
    }
}
