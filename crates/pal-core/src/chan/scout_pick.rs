//! mabur `scout_pick.h`: which card scouts, bursts, leads a hop, and which
//! cards carry DISC while the scout owns one. Pure. USB cards come first in
//! the roster; relays (`can_sweep`) follow.

/// The boot scout: the last scout-capable card (the spare USB card on an
/// all-USB GS, the only one on a one-card GS).
pub fn pick_boot_scout(can_scout: &[bool]) -> Option<usize> {
    can_scout.iter().rposition(|&c| c)
}

/// The in-flight scout dwells on a card that is NOT transmitting (an
/// off-channel TX card would lose every RCF). None = skip this period.
pub fn pick_inflight_scout(can_scout: &[bool], tx: usize) -> Option<usize> {
    if can_scout.len() < 2 {
        return None;
    }
    (0..can_scout.len())
        .rev()
        .find(|&i| i != tx && can_scout[i])
}

/// The card for the hop freshness burst, first match: a non-TX scout card;
/// a non-TX relay; the scout-capable TX card; the relay TX card. A short
/// `can_sweep` reads as false past its end.
pub fn pick_burst_card(can_scout: &[bool], can_sweep: &[bool], tx: usize) -> Option<usize> {
    let n = can_scout.len();
    let last = |cap: &[bool], non_tx: bool| {
        (0..n)
            .rev()
            .find(|&i| i < cap.len() && cap[i] && (!non_tx || i != tx))
    };
    last(can_scout, true)
        .or_else(|| last(can_sweep, true))
        .or_else(|| last(can_scout, false))
        .or_else(|| last(can_sweep, false))
}

/// DISC targets while the scout owns a card. Two USB: the link card always,
/// plus the scout card while it beacons. One USB: the scout card while
/// beaconing. Plus every ready relay (with no USB card the scout is a relay
/// and joins only while beaconing).
pub fn scan_disc_targets(
    n_usb: usize,
    n_cards: usize,
    scout_card: Option<usize>,
    scout_beaconing: bool,
    ready: &[bool],
) -> Vec<usize> {
    let mut out = Vec::new();
    if n_cards == 0 {
        return out;
    }
    if n_usb >= 2 {
        out.push(if scout_card == Some(0) { 1 } else { 0 });
        if scout_beaconing {
            if let Some(s) = scout_card {
                out.push(s);
            }
        }
    } else if n_usb == 1 && scout_beaconing {
        if let Some(s) = scout_card {
            out.push(s);
        }
    }
    for (i, &r) in ready.iter().enumerate().take(n_cards).skip(n_usb) {
        if !r {
            continue;
        }
        if n_usb == 0 && Some(i) == scout_card {
            if scout_beaconing {
                out.push(i);
            }
            continue;
        }
        out.push(i);
    }
    out
}

/// The in-flight hop lead: the first ready card that is not transmitting;
/// None runs the one-card hop path instead of leading on a dead card.
pub fn pick_hop_lead(ready: &[bool], tx: usize) -> Option<usize> {
    (0..ready.len()).find(|&i| i != tx && ready[i])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boot_scout_is_last_scout_capable_card() {
        assert_eq!(pick_boot_scout(&[true]), Some(0));
        assert_eq!(pick_boot_scout(&[true, true]), Some(1));
        assert_eq!(pick_boot_scout(&[true, false]), Some(0));
        assert_eq!(pick_boot_scout(&[true, true, false]), Some(1));
        assert_eq!(pick_boot_scout(&[false]), None);
        assert_eq!(pick_boot_scout(&[]), None);
    }

    #[test]
    fn inflight_scout_is_last_scout_capable_non_tx_card() {
        assert_eq!(pick_inflight_scout(&[true, true], 0), Some(1));
        assert_eq!(pick_inflight_scout(&[true, true], 1), Some(0));
        assert_eq!(pick_inflight_scout(&[true, false], 1), Some(0));
        assert_eq!(pick_inflight_scout(&[true, false], 0), None);
        assert_eq!(pick_inflight_scout(&[true, true, false], 0), Some(1));
        assert_eq!(pick_inflight_scout(&[true], 0), None);
    }

    #[test]
    fn burst_card_prefers_non_tx_scout_capable_then_tx_scout_capable() {
        let no = |n| vec![false; n];
        assert_eq!(pick_burst_card(&[true, true], &no(2), 0), Some(1));
        assert_eq!(pick_burst_card(&[true, true], &no(2), 1), Some(0));
        assert_eq!(pick_burst_card(&[true], &no(1), 0), Some(0));
        assert_eq!(pick_burst_card(&[true, false], &no(2), 1), Some(0));
        assert_eq!(pick_burst_card(&[true, false], &no(2), 0), Some(0));
        assert_eq!(pick_burst_card(&[true, true, false], &no(3), 0), Some(1));
        assert_eq!(pick_burst_card(&[false], &no(1), 0), None);
        assert_eq!(pick_burst_card(&[false, false], &no(2), 1), None);
    }

    #[test]
    fn burst_card_prefers_non_tx_then_scout_then_sweep() {
        let (scout, sweep) = ([true, false], [false, true]);
        assert_eq!(pick_burst_card(&scout, &sweep, 1), Some(0));
        assert_eq!(pick_burst_card(&scout, &sweep, 0), Some(1));
        assert_eq!(pick_burst_card(&[false], &[true], 0), Some(0));
        assert_eq!(pick_burst_card(&[true], &[false], 0), Some(0));
        assert_eq!(pick_burst_card(&[false], &[false], 0), None);
    }

    #[test]
    fn scan_disc_targets_never_rely_on_the_relay_alone() {
        assert_eq!(
            scan_disc_targets(2, 2, Some(1), false, &[true, true]),
            vec![0]
        );
        assert_eq!(
            scan_disc_targets(2, 2, Some(1), true, &[true, true]),
            vec![0, 1]
        );
        assert_eq!(
            scan_disc_targets(2, 3, Some(1), false, &[true; 3]),
            vec![0, 2]
        );
        assert_eq!(
            scan_disc_targets(2, 3, Some(1), false, &[true, true, false]),
            vec![0]
        );
        assert_eq!(
            scan_disc_targets(1, 2, Some(0), true, &[true, true]),
            vec![0, 1]
        );
        assert_eq!(
            scan_disc_targets(1, 2, Some(0), false, &[true, true]),
            vec![1]
        );
        assert!(scan_disc_targets(1, 2, Some(0), false, &[true, false]).is_empty());
        assert_eq!(scan_disc_targets(1, 1, Some(0), true, &[true]), vec![0]);
        assert!(scan_disc_targets(1, 1, Some(0), false, &[true]).is_empty());
        assert!(scan_disc_targets(0, 0, None, true, &[]).is_empty());
    }

    #[test]
    fn hop_lead_is_first_ready_non_tx_card() {
        assert_eq!(pick_hop_lead(&[true, true], 0), Some(1));
        assert_eq!(pick_hop_lead(&[true, false], 0), None);
        assert_eq!(pick_hop_lead(&[false, true], 1), None);
        assert_eq!(pick_hop_lead(&[true, true, true], 1), Some(0));
        assert_eq!(pick_hop_lead(&[true], 0), None);
    }

    #[test]
    fn scan_disc_targets_relay_only_roster_uses_scout_relay_while_beaconing() {
        assert_eq!(
            scan_disc_targets(0, 2, Some(1), true, &[true, true]).len(),
            2
        );
        assert_eq!(
            scan_disc_targets(0, 2, Some(1), false, &[true, true]),
            vec![0]
        );
        assert!(scan_disc_targets(0, 2, Some(1), true, &[false, false]).is_empty());
    }
}
