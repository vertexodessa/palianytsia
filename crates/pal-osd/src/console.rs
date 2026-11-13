//! The UI state every GS front end keeps (pal-desktop, the phone, the
//! Quest): stats history, MAVLink telemetry, the drone settings table and
//! its edits, HUD prefs, the settings panel, the fps meter. Each front end
//! used to hold and feed its own copy, so a panel feature touched all
//! three. A front end now owns the surface and the input, hands the link's
//! channels to `Console`, and forwards what `show_panel` returns.
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, Sender};
use std::time::Instant;

use crate::panel::{Panel, PanelIn, PanelOut};
use crate::params::{Cmd, Edits, Table};
use crate::prefs::HudPrefs;
use crate::stall::{FrameAge, Stall};
use crate::telem::Telemetry;
use crate::{History, Snapshot};

/// The link's channels to the UI other than the stats lines, which each
/// front end receives at its own pace.
pub struct Feeds<'a> {
    pub mav: &'a Receiver<Vec<u8>>,
    pub params: &'a Receiver<String>,
    pub set_res: &'a Receiver<(usize, Result<(), String>)>,
}

/// What a front end hands the flight overlay for one frame.
pub struct OverlayIn {
    /// Where the video is on the surface.
    pub video: egui::Rect,
    /// The part of it the HUD may use: all of it, or what a docked panel
    /// leaves free ([`crate::panel::hud_rect`]). The front end knows where
    /// its panel is; the HUD does not ask.
    pub hud: egui::Rect,
    /// Anchor the chips in the HUD's rect; false: in the video's.
    pub chips_in_hud: bool,
    pub top_offset: f32,
    /// A line above the chips, styled by the front end.
    pub status: Option<egui::RichText>,
    pub fps: f64,
    pub kbps: f64,
    pub dec_errors: u64,
}

pub struct Console {
    pub hist: History,
    pub telem: Telemetry,
    pub table: Option<Table>,
    pub edits: Edits,
    pub prefs: HudPrefs,
    pub panel: Panel,
    prefs_path: PathBuf,
    /// Decoded frames per second over the last full second, the frame
    /// count it was taken at, and when.
    fps: (f64, u64, Instant),
    /// The age of the picture on the glass. It survives `new_session`: a
    /// relink leaves the old picture up, and that picture is still stale.
    age: FrameAge,
    stall: Stall,
}

impl Console {
    /// Prefs from `prefs_path` (defaults when missing); `panel` is the
    /// front end's starting panel (touch rows, open on a tab).
    pub fn new(prefs_path: PathBuf, panel: Panel) -> Self {
        Self {
            hist: History::default(),
            telem: Telemetry::default(),
            table: None,
            edits: Edits::default(),
            prefs: HudPrefs::load(&prefs_path),
            panel,
            prefs_path,
            fps: (0.0, 0, Instant::now()),
            age: FrameAge::default(),
            stall: Stall::None,
        }
    }

    /// A new link is a new session: fresh history and telemetry. The
    /// settings table, the edits and the panel stay.
    pub fn new_session(&mut self) {
        self.hist = History::default();
        self.telem = Telemetry::default();
    }

    /// One stats line at `now` (seconds); false when it does not parse.
    pub fn on_stats(&mut self, now: f64, line: &str) -> bool {
        match Snapshot::parse(line) {
            Some(s) => {
                self.hist.push(now, s);
                true
            }
            None => false,
        }
    }

    /// Every stats line waiting in `rx`; true when one parsed.
    pub fn drain_stats(&mut self, now: f64, rx: &Receiver<String>) -> bool {
        let mut fresh = false;
        while let Ok(l) = rx.try_recv() {
            fresh |= self.on_stats(now, &l);
        }
        fresh
    }

    /// Everything waiting on the other channels. Returns one line per
    /// settings row whose value or state changed, for a front end that
    /// logs them (a set that "hangs" shows whether the drone answered).
    pub fn drain(&mut self, now: f64, f: Feeds) -> Vec<String> {
        while let Ok(p) = f.mav.try_recv() {
            self.telem.on_packet(&p, now);
        }
        let mut changes = Vec::new();
        while let Ok(l) = f.params.try_recv() {
            if let Some(t) = Table::parse(&l) {
                for (i, r) in t.rows.iter().enumerate() {
                    let old = self.table.as_ref().and_then(|o| o.rows.get(i));
                    if old.is_none_or(|o| o.state != r.state || o.value != r.value) {
                        changes.push(format!(
                            "param {} = {:?}: {} {}",
                            r.id,
                            r.value.as_deref().unwrap_or("-"),
                            r.state,
                            r.status
                        ));
                    }
                }
                self.table = Some(t);
            }
        }
        while let Ok(r) = f.set_res.try_recv() {
            self.edits.last = Some(r);
        }
        changes
    }

    /// The decoder's running frame count at `now` (seconds), once per OSD
    /// tick: stamps the picture's age for the stall marker and returns the
    /// decoded fps, recomputed once a second.
    pub fn on_frames(&mut self, now: f64, frames: u64) -> f64 {
        self.age.note(now, frames);
        self.stall = self.age.stall(now);
        let dt = self.fps.2.elapsed().as_secs_f64();
        if dt >= 1.0 {
            self.fps = (
                frames.saturating_sub(self.fps.1) as f64 / dt,
                frames,
                Instant::now(),
            );
        }
        self.fps.0
    }

    /// How stale the picture is as of the last `on_frames`.
    pub fn stall(&self) -> Stall {
        self.stall
    }

    /// The flight overlay: HUD, stall marker, status line, stats chips and
    /// the key banner. True when the operator moved a chip.
    pub fn overlay(&mut self, ui: &mut egui::Ui, now: f64, o: OverlayIn) -> bool {
        let r = o.hud;
        let hud_band = crate::hud::hud(
            ui,
            r,
            o.video.center(),
            &self.telem,
            &self.hist,
            now,
            &self.prefs,
        );
        // On the video itself, whatever the HUD prefs: it cannot be hidden.
        crate::hud::stall(ui, o.video, self.stall, self.prefs.scale);
        let mismatch = self.hist.last().is_some_and(|s| s.key_mismatch);
        let mut moved = false;
        if self.prefs.stats || mismatch || o.status.is_some() {
            let base = if o.chips_in_hud { r } else { o.video };
            let mut top = base.shrink(8.0).translate(egui::vec2(0.0, o.top_offset));
            // Below the HUD's top line, whatever its size: a TELEMETRY LOST
            // under the arm state, or a headset's larger text, would otherwise
            // sit under the first chip row or the banner.
            top.min.y = top.min.y.max(hud_band + 6.0);
            ui.allocate_new_ui(egui::UiBuilder::new().max_rect(top), |ui| {
                if let Some(s) = o.status {
                    ui.label(s);
                }
                if self.prefs.stats {
                    moved = crate::chips(
                        ui,
                        &mut self.prefs.chip_pos,
                        &self.hist,
                        o.fps,
                        o.kbps,
                        o.dec_errors,
                    );
                }
                crate::key_banner(ui, &self.hist);
            });
        }
        moved
    }

    pub fn save_prefs(&self) {
        self.prefs.save(&self.prefs_path);
    }

    /// The detailed link statistics on or off, saved at once.
    pub fn toggle_stats(&mut self) {
        self.prefs.stats = !self.prefs.stats;
        self.save_prefs();
    }

    /// The settings panel for this frame. `link_info` and `log_info` are
    /// the front end's own lines; `channels` the link's set (empty: no
    /// channel choice).
    pub fn show_panel(
        &mut self,
        ctx: &egui::Context,
        link_info: String,
        log_info: String,
        channels: &[u8],
    ) -> PanelOut {
        let (panel, i) = self.panel_parts(link_info, log_info, channels);
        crate::panel::docked(ctx, panel, i)
    }

    /// The settings on a surface of their own ([`crate::panel::surface`]):
    /// the headset's menu quad.
    pub fn show_surface(
        &mut self,
        ctx: &egui::Context,
        link_info: String,
        log_info: String,
        channels: &[u8],
    ) -> PanelOut {
        let (panel, i) = self.panel_parts(link_info, log_info, channels);
        crate::panel::surface(ctx, panel, i)
    }

    /// The panel and what it reads this frame, for a container.
    pub fn panel_parts<'a>(
        &'a mut self,
        link_info: String,
        log_info: String,
        channels: &'a [u8],
    ) -> (&'a mut Panel, PanelIn<'a>) {
        (
            &mut self.panel,
            PanelIn {
                table: self.table.as_ref(),
                edits: &mut self.edits,
                prefs: &mut self.prefs,
                link_info,
                log_info,
                cal: self.hist.last().and_then(|s| s.cal.as_ref()),
                scan: self.hist.last().and_then(|s| s.chscan.as_ref()),
                cards: self.hist.last().map_or(0, |s| s.cards.len()),
                channels,
                warning: self.hist.last().and_then(crate::key_warning),
                armed: self.telem.last_s.map(|_| self.telem.armed),
            },
        )
    }

    /// The panel's command to the core (when there is a link) and its
    /// prefs saved. `relink` is the front end's to act on.
    pub fn settle(&mut self, out: &mut PanelOut, cmd_tx: Option<&Sender<Cmd>>) {
        if let (Some(c), Some(tx)) = (out.cmd.take(), cmd_tx) {
            let _ = tx.send(c);
        }
        if out.prefs_changed {
            self.save_prefs();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    fn tmp(name: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("pal-console-{}-{name}", std::process::id()));
        let _ = std::fs::remove_file(&p);
        p
    }

    fn table(value: &str, state: &str) -> String {
        serde_json::json!({
            "linked": true, "list": "complete",
            "rows": [{"id": "RADIO_LDPC", "value": value, "state": state, "status": ""}]
        })
        .to_string()
    }

    #[test]
    fn drain_reports_only_rows_that_changed() {
        let mut c = Console::new(tmp("drain"), Panel::default());
        let (mav_tx, mav) = mpsc::channel();
        let (params_tx, params) = mpsc::channel();
        let (res_tx, set_res) = mpsc::channel();
        let feeds = || Feeds {
            mav: &mav,
            params: &params,
            set_res: &set_res,
        };
        params_tx.send(table("1", "idle")).unwrap();
        assert_eq!(
            c.drain(0.0, feeds()),
            vec!["param RADIO_LDPC = \"1\": idle ".to_string()]
        );
        params_tx.send(table("1", "idle")).unwrap();
        assert!(c.drain(0.1, feeds()).is_empty(), "same value and state");
        params_tx.send(table("0", "accepted")).unwrap();
        res_tx.send((0, Ok(()))).unwrap();
        mav_tx.send(vec![0u8; 3]).unwrap();
        assert_eq!(c.drain(0.2, feeds()).len(), 1);
        assert_eq!(c.edits.last, Some((0, Ok(()))));
        assert_eq!(
            c.table.as_ref().unwrap().rows[0].value.as_deref(),
            Some("0")
        );
    }

    #[test]
    fn stats_lines_feed_the_history_and_a_session_clears_it() {
        let mut c = Console::new(tmp("stats"), Panel::default());
        assert!(!c.on_stats(0.0, "not json"));
        let (tx, rx) = mpsc::channel();
        tx.send("{}".to_string()).unwrap();
        let parsed = c.drain_stats(0.0, &rx);
        assert_eq!(parsed, c.hist.last().is_some());
        c.new_session();
        assert!(c.hist.last().is_none());
    }

    #[test]
    fn settle_forwards_the_command_and_saves_changed_prefs() {
        let path = tmp("settle");
        let mut c = Console::new(path.clone(), Panel::default());
        let (tx, rx) = mpsc::channel();
        c.prefs.link_channels = vec![144];
        let mut out = PanelOut {
            cmd: Some(Cmd::Refresh),
            prefs_changed: true,
            relink: true,
            close: false,
        };
        c.settle(&mut out, Some(&tx));
        assert_eq!(rx.try_recv(), Ok(Cmd::Refresh));
        assert!(out.relink, "relink stays the front end's");
        assert_eq!(HudPrefs::load(&path).link_channels, vec![144]);
        // No link: the command is dropped, prefs still saved.
        let mut out = PanelOut {
            cmd: Some(Cmd::Refresh),
            ..Default::default()
        };
        c.settle(&mut out, None);
        assert!(rx.try_recv().is_err());
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn the_picture_age_survives_a_new_session() {
        let mut c = Console::new(tmp("stall"), Panel::default());
        c.on_frames(0.0, 0);
        assert_eq!(c.stall(), Stall::None, "no video yet");
        c.on_frames(1.0, 10);
        c.new_session();
        // The new link restarts the count; the old picture is still up.
        c.on_frames(1.6, 0);
        assert!(matches!(c.stall(), Stall::NoVideo(_)));
        c.on_frames(1.7, 1);
        assert_eq!(c.stall(), Stall::None);
    }

    #[test]
    fn toggle_stats_saves() {
        let path = tmp("toggle");
        let mut c = Console::new(path.clone(), Panel::default());
        let was = c.prefs.stats;
        c.toggle_stats();
        assert_eq!(HudPrefs::load(&path).stats, !was);
        let _ = std::fs::remove_file(path);
    }
}
