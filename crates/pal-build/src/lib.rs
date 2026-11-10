//! Which build this is. A flight log, a panic record and `--version` name
//! it, so a log directory can be matched to a release, a commit and the
//! mabur pin that decided its FEC and channel rules.
//!
//! Its own crate (not `runtime`) because `flightlog` writes it and
//! `runtime` depends on `flightlog`; and so a commit rebuilds only the
//! crates that print it, not pal-core or pal-ref.

/// release/VERSION, the one source of the product version (both APKs'
/// versionName read it too).
pub const VERSION: &str = env!("PAL_VERSION");
/// The superproject's HEAD commit, or "unknown".
pub const GIT_SHA: &str = env!("PAL_GIT_SHA");
/// "1" with uncommitted changes to tracked files, "0", or "unknown".
pub const GIT_DIRTY: &str = env!("PAL_GIT_DIRTY");
/// The commits third_party/mabur and third_party/devourer are pinned to.
pub const MABUR_PIN: &str = env!("PAL_MABUR_PIN");
pub const DEVOURER_PIN: &str = env!("PAL_DEVOURER_PIN");
/// "0.11.0 9c5838f1a2b3" (12 hex digits, "-dirty" appended when dirty).
pub const ID: &str = env!("PAL_BUILD_ID");

/// The `--version` text: the id, then the commit and the pins in full.
pub fn version_text(app: &str) -> String {
    format!(
        "{app} {ID}\ncommit {GIT_SHA} dirty={GIT_DIRTY}\nmabur {MABUR_PIN}\ndevourer {DEVOURER_PIN}"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_starts_with_the_version() {
        assert!(ID.starts_with(VERSION), "{ID}");
        assert!(!VERSION.is_empty());
        assert!(["0", "1", "unknown"].contains(&GIT_DIRTY));
        assert!(version_text("pal").starts_with("pal "));
    }
}
