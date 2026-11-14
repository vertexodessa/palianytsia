//! The drone's logs and state into the flight log directory, over the
//! maintenance lane: a drone with no cable is looked at from the app the pilot
//! carries. The UI asks with `Cmd::DroneFetch`; the state goes to the UI
//! stats line as `fetch`.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::drone_update::exec;
use crate::Shared;

/// The tail of a log that keeps a minute of lane time at most.
const TAIL: u32 = 256 * 1024;

/// (file name, the command whose output it holds).
fn items() -> [(&'static str, String); 5] {
    [
        (
            "mabur.log",
            format!("tail -c {TAIL} /tmp/mabur.log 2>/dev/null; true"),
        ),
        (
            "mabur.log.prev",
            format!("tail -c {TAIL} /tmp/mabur.log.prev 2>/dev/null; true"),
        ),
        (
            "swap.log",
            "cat /tmp/mabur-up/swap.log 2>/dev/null; true".into(),
        ),
        ("mabur.toml", "cat /etc/mabur.toml".into()),
        (
            "state.txt",
            "uptime; echo; df -k /overlay; echo; free; echo; \
             ls -l /usr/bin/maburd /etc/mabur-lkg /etc/mabur.unproven /etc/init.d 2>&1; echo; \
             echo \"arm=$(cat /tmp/mabur.arm 2>/dev/null) up=$(cat /tmp/mabur-up/state 2>/dev/null)\"; \
             echo; dmesg | tail -n 100"
                .into(),
        ),
    ]
}

/// A command's exit status and output.
type Output = (i16, Vec<u8>);

/// Runs each item through `run` and writes its output under `dir`; `step`
/// hears each file's name first. Returns the bytes written. Stops at the
/// first command that fails: the lane is gone or the drone refused.
pub fn fetch_into(
    dir: &Path,
    run: &mut dyn FnMut(&str) -> Result<Output, String>,
    step: &mut dyn FnMut(&str),
) -> Result<u64, String> {
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut bytes = 0;
    for (name, cmd) in items() {
        step(name);
        let (st, out) = run(&cmd).map_err(|e| format!("{name}: {e}"))?;
        if st != 0 {
            return Err(format!("{name}: `{cmd}` exited {st}"));
        }
        let p = dir.join(name);
        std::fs::write(&p, &out).map_err(|e| format!("{}: {e}", p.display()))?;
        bytes += out.len() as u64;
    }
    Ok(bytes)
}

static FETCHING: AtomicBool = AtomicBool::new(false);

/// Fetches into `log_dir/drone-<unix time>` on a thread of its own; the state
/// goes to `sh.fetch`. One fetch at a time, none while a drone update runs
/// (the drone takes one lane session at a time).
pub fn spawn_fetch(sh: Arc<Shared>, log_dir: Option<PathBuf>) {
    let set = |v: serde_json::Value| *sh.fetch.lock().unwrap_or_else(|e| e.into_inner()) = Some(v);
    let Some(log_dir) = log_dir else {
        set(
            serde_json::json!({ "state": "failed", "error": "no flight log directory to put it in" }),
        );
        return;
    };
    let updating = sh
        .update
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .and_then(|v| v.get("state").and_then(|s| s.as_str()).map(str::to_string))
        .is_some_and(|s| s == "updating");
    if updating {
        set(serde_json::json!({ "state": "failed", "error": "a drone update is running" }));
        return;
    }
    if FETCHING.swap(true, Ordering::AcqRel) {
        return;
    }
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let dir = log_dir.join(format!("drone-{secs}"));
    let r = std::thread::Builder::new()
        .name("drone-fetch".into())
        .spawn(move || {
            let set = |v: serde_json::Value| {
                *sh.fetch.lock().unwrap_or_else(|e| e.into_inner()) = Some(v)
            };
            let r = fetch_into(
                &dir,
                &mut |cmd| {
                    exec(
                        &sh.maint,
                        cmd,
                        &[],
                        Duration::from_secs(3),
                        Duration::from_secs(60),
                        |_| {},
                    )
                    .map_err(String::from)
                },
                &mut |name| set(serde_json::json!({ "state": "fetching", "step": name })),
            );
            let v = match &r {
                Ok(b) => serde_json::json!({ "state": "done", "dir": dir.display().to_string(), "bytes": b }),
                Err(e) => serde_json::json!({ "state": "failed", "error": e }),
            };
            sh.event("drone_fetch", v.clone());
            set(v);
            FETCHING.store(false, Ordering::Release);
        });
    if r.is_err() {
        FETCHING.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_item_lands_in_its_file_and_a_failure_stops_the_rest() {
        let d = std::env::temp_dir().join(format!("pal-fetch-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        let mut steps = Vec::new();
        let n = fetch_into(&d, &mut |cmd| Ok((0, cmd.as_bytes().to_vec())), &mut |s| {
            steps.push(s.to_string())
        })
        .unwrap();
        let names: Vec<_> = items().iter().map(|(n, _)| n.to_string()).collect();
        assert_eq!(steps, names);
        let log = std::fs::read_to_string(d.join("mabur.log")).unwrap();
        assert_eq!(log, items()[0].1);
        assert_eq!(n, items().iter().map(|(_, c)| c.len() as u64).sum::<u64>());

        let mut calls = 0;
        let e = fetch_into(
            &d.join("b"),
            &mut |_| {
                calls += 1;
                if calls == 2 {
                    Err("the drone is armed".into())
                } else {
                    Ok((0, vec![]))
                }
            },
            &mut |_| {},
        )
        .unwrap_err();
        assert_eq!(e, "mabur.log.prev: the drone is armed");
        assert_eq!(calls, 2);
        let e = fetch_into(&d.join("c"), &mut |_| Ok((1, vec![])), &mut |_| {}).unwrap_err();
        assert!(e.starts_with("mabur.log: "), "{e}");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// Each command runs as busybox sh would take it: quoting holds.
    #[test]
    fn each_command_parses_as_sh() {
        for (n, c) in items() {
            let st = std::process::Command::new("sh")
                .arg("-n")
                .arg("-c")
                .arg(&c)
                .status()
                .unwrap();
            assert!(st.success(), "{n}: {c}");
        }
    }
}
