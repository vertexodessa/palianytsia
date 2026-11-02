//! The panel's channel choice as values: which channels a width offers,
//! which are chosen, what one click makes of it. The link applies the
//! result (`pal_core::config::with_channels` at start, `Cmd::Channels`
//! live); here only the rules the operator sees.
use pal_wire::ht40::ht40_offset;

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
