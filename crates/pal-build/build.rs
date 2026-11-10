//! Exports the build's identity as compile-time environment variables:
//! `PAL_VERSION` (release/VERSION), `PAL_GIT_SHA`, `PAL_GIT_DIRTY`,
//! `PAL_MABUR_PIN`, `PAL_DEVOURER_PIN` and `PAL_BUILD_ID`.
//!
//! Every git question can fail (a source tarball, no git installed, a
//! foreign repository around the tree): the answer is then "unknown" and the
//! build goes on.
//!
//! Reruns only when HEAD, the branch ref it names (or the directory a not
//! yet loose ref will appear in), packed-refs or release/VERSION change, so
//! an ordinary edit-compile cycle does not rebuild this crate's dependents.
//! The dirty flag is therefore the tree's state at the last rerun.
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const UNKNOWN: &str = "unknown";

fn git(root: &Path, args: &[&str]) -> Option<String> {
    let out = Command::new("git")
        .arg("-C")
        .arg(root)
        .args(args)
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    Some(String::from_utf8(out.stdout).ok()?.trim().to_string())
}

/// `git rev-parse --git-path`: in a worktree HEAD lives in the worktree's
/// own git dir and refs in the common one; this resolves both.
fn git_path(root: &Path, name: &str) -> Option<PathBuf> {
    let p = PathBuf::from(git(root, &["rev-parse", "--git-path", name])?);
    Some(if p.is_absolute() { p } else { root.join(p) })
}

/// A missing path makes cargo rerun the script on every build.
fn watch(p: &Path) {
    if p.exists() {
        println!("cargo:rerun-if-changed={}", p.display());
    }
}

/// The commit a gitlink in HEAD pins (`160000 commit <sha>\t<path>`); read
/// from the superproject's tree, so it holds whatever the submodule
/// directory has checked out.
fn pin(root: &Path, path: &str) -> Option<String> {
    let line = git(root, &["ls-tree", "HEAD", path])?;
    let mut f = line.split_whitespace();
    (f.next()? == "160000" && f.next()? == "commit")
        .then(|| f.next().map(str::to_string))
        .flatten()
}

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("cargo sets it"));
    let root = manifest
        .join("../..")
        .canonicalize()
        .unwrap_or_else(|_| manifest.join("../.."));

    let version_file = root.join("release/VERSION");
    println!("cargo:rerun-if-changed={}", version_file.display());
    let version = fs::read_to_string(&version_file)
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| UNKNOWN.into());

    // Only this repository's own checkout answers: a tree unpacked inside
    // some other repository must not report that repository's commit.
    let ours = git(&root, &["rev-parse", "--show-toplevel"])
        .and_then(|t| Path::new(&t).canonicalize().ok())
        .is_some_and(|t| t == root);
    let (mut sha, mut dirty, mut mabur, mut devourer) = (
        UNKNOWN.to_string(),
        UNKNOWN,
        UNKNOWN.to_string(),
        UNKNOWN.to_string(),
    );
    if ours {
        if let Some(h) = git_path(&root, "HEAD") {
            watch(&h);
        }
        if let Some(r) = git(&root, &["symbolic-ref", "-q", "HEAD"]) {
            if let Some(p) = git_path(&root, &r) {
                match p.exists() {
                    true => watch(&p),
                    // A packed ref: the next commit writes it loose here.
                    false => p.parent().into_iter().for_each(watch),
                }
            }
        }
        if let Some(p) = git_path(&root, "packed-refs") {
            watch(&p);
        }
        if let Some(s) = git(&root, &["rev-parse", "HEAD"]) {
            sha = s;
        }
        // Submodules count when their checkout differs from the pin or has
        // edits (pal-ref compiles mabur's sources); their untracked files
        // do not.
        if let Some(s) = git(
            &root,
            &[
                "--no-optional-locks",
                "status",
                "--porcelain",
                "--untracked-files=no",
                "--ignore-submodules=untracked",
            ],
        ) {
            dirty = if s.is_empty() { "0" } else { "1" };
        }
        mabur = pin(&root, "third_party/mabur").unwrap_or_else(|| UNKNOWN.into());
        devourer = pin(&root, "third_party/devourer").unwrap_or_else(|| UNKNOWN.into());
    }

    let short = sha.get(..12).unwrap_or(&sha);
    let id = match dirty {
        "1" => format!("{version} {short}-dirty"),
        _ => format!("{version} {short}"),
    };
    println!("cargo:rustc-env=PAL_VERSION={version}");
    println!("cargo:rustc-env=PAL_GIT_SHA={sha}");
    println!("cargo:rustc-env=PAL_GIT_DIRTY={dirty}");
    println!("cargo:rustc-env=PAL_MABUR_PIN={mabur}");
    println!("cargo:rustc-env=PAL_DEVOURER_PIN={devourer}");
    println!("cargo:rustc-env=PAL_BUILD_ID={id}");
}
