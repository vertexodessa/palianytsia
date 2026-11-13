//! The automatic drone update as the UI stats line reports it (`update`, from
//! `runtime::drone_update::spawn_auto`): the banner over the video while it
//! runs or after it failed, and the System tab's line.

use serde_json::Value;

use crate::json::u64_or;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct UpdateView {
    /// waiting | current | armed | no_lane | busy | error | updating |
    /// updated | failed
    pub state: String,
    pub step: String,
    /// Bytes of the current upload the drone has, of `total`.
    pub done: u64,
    pub total: u64,
    pub secs: Option<f64>,
    pub error: String,
    /// The first 12 hex digits of this app's maburd and the drone's.
    pub app: String,
    pub drone: String,
}

impl UpdateView {
    pub fn parse(v: &Value) -> Option<Self> {
        let s = |k: &str| v.get(k).and_then(Value::as_str).unwrap_or("").to_string();
        Some(Self {
            state: v.get("state")?.as_str()?.to_string(),
            step: s("step"),
            done: u64_or(v, "done", 0),
            total: u64_or(v, "total", 0),
            secs: v.get("secs").and_then(Value::as_f64),
            error: s("error"),
            app: s("app"),
            drone: s("drone"),
        })
    }

    /// The banner over the video and whether it is an error: while the
    /// update runs (the drone's video is off on purpose), once it is done,
    /// and after it failed.
    pub fn banner(&self) -> Option<(String, bool)> {
        match self.state.as_str() {
            "updating" => {
                let pct = (100 * self.done.min(self.total))
                    .checked_div(self.total)
                    .map_or(String::new(), |p| format!(" {p}%"));
                Some((
                    format!(
                        "UPDATING DRONE: {}{pct}. Keep it powered, the video is back in about a minute",
                        self.step
                    ),
                    false,
                ))
            }
            "updated" => Some((
                format!(
                    "DRONE UPDATED to this app's maburd in {:.0} s",
                    self.secs.unwrap_or(0.0)
                ),
                false,
            )),
            "failed" => Some((format!("DRONE UPDATE FAILED: {}", self.error), true)),
            _ => None,
        }
    }

    /// The System tab's line.
    pub fn line(&self) -> String {
        if let Some((b, _)) = self.banner() {
            return b;
        }
        match self.state.as_str() {
            "waiting" => format!(
                "waiting for the drone; this app carries maburd {}",
                self.app
            ),
            "current" => format!("the drone runs this app's maburd {}", self.app),
            "armed" => "not checked while the drone is armed".into(),
            "no_lane" => "the drone does not answer on the maintenance lane: a maburd without \
                          it is updated from a PC (drone/install.py)"
                .into(),
            "busy" => "the drone is in the middle of a maburd swap".into(),
            "error" => format!("check failed: {}", self.error),
            s => s.to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn view(j: &str) -> UpdateView {
        UpdateView::parse(&serde_json::from_str(j).unwrap()).unwrap()
    }

    #[test]
    fn the_banner_shows_only_a_running_finished_or_failed_update() {
        let up = view(r#"{"state":"updating","step":"uploading maburd","done":512,"total":1024}"#);
        let (t, bad) = up.banner().unwrap();
        assert!(t.starts_with("UPDATING DRONE: uploading maburd 50%"), "{t}");
        assert!(!bad);
        let (t, bad) = view(r#"{"state":"failed","error":"no steady video"}"#)
            .banner()
            .unwrap();
        assert_eq!(
            (t.as_str(), bad),
            ("DRONE UPDATE FAILED: no steady video", true)
        );
        assert!(view(r#"{"state":"updated","secs":50.9}"#)
            .banner()
            .unwrap()
            .0
            .contains("51 s"));
        for s in ["waiting", "current", "armed", "no_lane", "busy", "error"] {
            let v = view(&format!(r#"{{"state":"{s}","app":"abc"}}"#));
            assert_eq!(v.banner(), None, "{s}");
            assert!(!v.line().is_empty());
        }
        assert!(UpdateView::parse(&serde_json::json!({})).is_none());
    }
}
