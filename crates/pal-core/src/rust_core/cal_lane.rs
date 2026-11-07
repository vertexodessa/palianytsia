//! The TX-power calibration lane (maburgs' CalSession wiring): the session,
//! what a run may sweep (the GS's width, the drone's agreed width, the
//! ladder), and the MAVLink messages it puts on the tunnel. The core hands
//! it the peer, the channel core (the picker) and the tunnel; the lane reads
//! nothing else of the core.
use crate::cal::log::CalLog;
use crate::cal::picker::{CalPicker, FixedPicker, Mode as PickMode};
use crate::cal::plan::{default_sweeps, effective_width};
use crate::cal::session::{sweep_token, CalSession, CalSessionCfg};
use crate::cal::status_json;
use crate::chan::core::ChannelCore;
use crate::chan::gate::cal_cmd_clear;
use crate::config::Config;
use crate::mav::MavTunnel;
use crate::vrx::random_nonce;
use pal_wire::cal::{self, SweepSpec};

/// maburgs' MAVLink identity for the calibration kit.
const CAL_SYSID: u8 = 255;
const CAL_COMPID: u8 = 190;

pub(super) struct CalLane {
    /// Idle unless started.
    pub session: CalSession,
    /// The GS's own radio.width: the widest sweep a run may ask for.
    radio_width: u8,
    ladder: Vec<crate::config::Rung>,
    /// The linked drone's DiscAck agreed_width (0 = none heard yet).
    pub peer_width: u8,
    /// MAVLink packet seq of the cal packets.
    mav_seq: u8,
}

impl CalLane {
    pub fn new(cfg: &Config) -> Self {
        Self {
            // maburgs' defaults: margin 1 dB only predicts the verify
            // parks, and verify tallies by rate whatever the drone's own
            // margin parks at.
            session: CalSession::new(CalSessionCfg::default(), CalLog::none()),
            radio_width: cfg.rx_width(),
            ladder: cfg.link.ladder_cfg.ladder.clone(),
            peer_width: 0,
            mav_seq: 0,
        }
    }

    /// A run: `sweeps` empty = the default plan for the width the link
    /// flies. The channel core picks the channel first when it can measure
    /// (auto, a spare card); without one the run sweeps on `channel`. The
    /// caller has set the session's peer.
    pub fn start(
        &mut self,
        sweeps: &[SweepSpec],
        linked: bool,
        chan: Option<&mut ChannelCore>,
        n_cards: usize,
        channel: u8,
        now_ms: u64,
    ) -> Result<(), String> {
        let width = effective_width(self.radio_width, linked, self.peer_width);
        let sweeps = if sweeps.is_empty() {
            default_sweeps(width, &self.ladder)
        } else {
            if sweeps.iter().any(|s| s.bw > width) {
                return Err(format!("refused: the link flies {width} MHz at most"));
            }
            sweeps.to_vec()
        };
        let mut fixed = FixedPicker {
            mode: if n_cards >= 2 {
                PickMode::Pinned
            } else {
                PickMode::OneCard
            },
            op: channel,
        };
        let picker: &mut dyn CalPicker = match chan {
            Some(c) => c,
            None => &mut fixed,
        };
        self.session
            .start(random_nonce(), sweeps, now_ms, Some(picker))?;
        let snap = self.session.snapshot();
        eprintln!(
            "pal cal: started ch {} ({}) sweeps {:?}",
            snap.channel,
            snap.pick_note,
            self.session
                .sweeps()
                .iter()
                .map(sweep_token)
                .collect::<Vec<_>>()
        );
        Ok(())
    }

    /// Per tick, after the session's peer is set: the channel pick, then
    /// the due message onto the tunnel's cal lane. The one hold: a
    /// SweepCmd while a scout dwell begun just before the start is still
    /// off-channel (repeated every 200 ms, so only late).
    pub fn tick(&mut self, now_ms: u64, chan: Option<&mut ChannelCore>, mav: &mut MavTunnel) {
        let mut chan = chan;
        if let Some(c) = chan.as_deref_mut() {
            self.session.step_pick(now_ms, c);
        }
        if let Some(m) = self.session.due_mav(now_ms) {
            let dwell_busy = chan.is_some_and(|c| c.dwell_busy());
            let held = matches!(m, cal::CalMsg::Sweep(_))
                && !cal_cmd_clear(self.session.radio_silent(now_ms), dwell_busy);
            if !held {
                let pkt = cal::encode(&m, CAL_SYSID, CAL_COMPID, self.mav_seq);
                self.mav_seq = self.mav_seq.wrapping_add(1);
                mav.push_cal(pkt, self.session.tag_ctx());
            }
        }
    }

    pub fn abort(&mut self, chan: Option<&mut ChannelCore>) {
        if self.session.running() || self.session.picking() {
            eprintln!("pal cal: aborted by the operator");
        }
        let picker = chan.map(|c| c as &mut dyn CalPicker);
        self.session.abort("aborted by the operator", picker);
    }

    pub fn json(&self, capable: bool) -> String {
        status_json(&self.session.snapshot(), self.session.sweeps(), capable)
    }
}
