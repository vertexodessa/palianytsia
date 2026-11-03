//! What the flight HUD shows, kept per ground station (desktop: a file in
//! ~/.config/palianytsia, phone: the app's files dir). Unknown or missing
//! keys fall back to the defaults, so an old file never blocks start.
use serde_json::{json, Value};

#[derive(Clone, Debug, PartialEq)]
pub struct HudPrefs {
    pub dot: bool,
    pub battery: bool,
    pub timer: bool,
    /// ALT, SPD, HOME, SAT.
    pub nav: bool,
    /// The compact RSSI/MCS readout.
    pub link: bool,
    pub messages: bool,
    /// The detailed link statistics over the HUD.
    pub stats: bool,
    /// Text size factor.
    pub scale: f32,
    /// Video link width in MHz (20 or 40): every ladder rung's `bw`
    /// (`pal_core::config::with_link_bw`), applied when the link starts.
    pub link_bw: u8,
    /// The channels the link may sit on (`pal_core::config::with_channels`),
    /// empty = every channel of the set; one = that channel only. Applied
    /// when the link starts and, changed in the panel, on the live link.
    pub link_channels: Vec<u8>,
    /// Quest video layer filter (XR_FB_composition_layer_settings): the
    /// 1920 px video lands on fewer display pixels, so supersampling may
    /// beat sharpening; judged by eye. Other front ends ignore it.
    pub video_filter: VideoFilter,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum VideoFilter {
    #[default]
    Off,
    Sharpen,
    Supersample,
}

impl VideoFilter {
    fn key(self) -> &'static str {
        match self {
            Self::Off => "off",
            Self::Sharpen => "sharpen",
            Self::Supersample => "supersample",
        }
    }

    fn parse(s: &str) -> Self {
        match s {
            "sharpen" => Self::Sharpen,
            "supersample" => Self::Supersample,
            _ => Self::Off,
        }
    }
}

impl Default for HudPrefs {
    fn default() -> Self {
        Self {
            dot: true,
            battery: true,
            timer: true,
            nav: true,
            link: true,
            messages: true,
            stats: false,
            scale: 1.0,
            link_bw: 20,
            link_channels: Vec::new(),
            video_filter: VideoFilter::Off,
        }
    }
}

impl HudPrefs {
    pub fn to_json(&self) -> String {
        json!({
            "dot": self.dot, "battery": self.battery, "timer": self.timer,
            "nav": self.nav, "link": self.link, "messages": self.messages,
            "stats": self.stats, "scale": self.scale, "link_bw": self.link_bw,
            "link_channels": self.link_channels,
            "video_filter": self.video_filter.key(),
        })
        .to_string()
    }

    pub fn from_json(s: &str) -> Self {
        let d = Self::default();
        let Ok(v) = serde_json::from_str::<Value>(s) else {
            return d;
        };
        let b = |k: &str, def: bool| v.get(k).and_then(Value::as_bool).unwrap_or(def);
        Self {
            dot: b("dot", d.dot),
            battery: b("battery", d.battery),
            timer: b("timer", d.timer),
            nav: b("nav", d.nav),
            link: b("link", d.link),
            messages: b("messages", d.messages),
            stats: b("stats", d.stats),
            scale: v
                .get("scale")
                .and_then(Value::as_f64)
                .map_or(d.scale, |x| (x as f32).clamp(0.6, 2.0)),
            link_bw: match v.get("link_bw").and_then(Value::as_u64) {
                Some(40) => 40,
                _ => 20,
            },
            link_channels: match v.get("link_channels").and_then(Value::as_array) {
                Some(a) => a
                    .iter()
                    .filter_map(|c| c.as_u64().and_then(|c| u8::try_from(c).ok()))
                    .collect(),
                // The single fixed channel of earlier versions (0 = auto).
                None => v
                    .get("link_channel")
                    .and_then(Value::as_u64)
                    .and_then(|c| u8::try_from(c).ok())
                    .filter(|&c| c != 0)
                    .into_iter()
                    .collect(),
            },
            video_filter: VideoFilter::parse(
                v.get("video_filter").and_then(Value::as_str).unwrap_or(""),
            ),
        }
    }

    pub fn load(path: &std::path::Path) -> Self {
        std::fs::read_to_string(path).map_or_else(|_| Self::default(), |s| Self::from_json(&s))
    }

    pub fn save(&self, path: &std::path::Path) {
        if let Some(d) = path.parent() {
            let _ = std::fs::create_dir_all(d);
        }
        let _ = std::fs::write(path, self.to_json());
    }
}

#[cfg(test)]
mod tests {
    use super::HudPrefs;

    #[test]
    fn roundtrip_and_tolerant_parse() {
        let p = HudPrefs {
            nav: false,
            stats: true,
            scale: 1.4,
            link_bw: 40,
            link_channels: vec![144, 40],
            video_filter: super::VideoFilter::Supersample,
            ..Default::default()
        };
        assert_eq!(HudPrefs::from_json(&p.to_json()), p);
        assert_eq!(
            HudPrefs::from_json(r#"{"link_channel":64}"#).link_channels,
            vec![64]
        );
        assert!(HudPrefs::from_json(r#"{"link_channel":0}"#)
            .link_channels
            .is_empty());
        assert_eq!(HudPrefs::from_json("garbage"), HudPrefs::default());
        let q = HudPrefs::from_json(r#"{"dot":false,"scale":9,"unknown":1}"#);
        assert!(!q.dot && q.battery && q.scale == 2.0 && q.link_bw == 20);
    }
}
