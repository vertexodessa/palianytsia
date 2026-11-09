//! Field access for the stats, parameter and prefs JSON. A missing field or
//! one of the wrong type reads as the default the caller names.
use serde_json::Value;

pub fn str_or(v: &Value, k: &str, def: &str) -> String {
    v.get(k).and_then(Value::as_str).unwrap_or(def).to_string()
}

pub fn bool_or(v: &Value, k: &str, def: bool) -> bool {
    v.get(k).and_then(Value::as_bool).unwrap_or(def)
}

pub fn u64_or(v: &Value, k: &str, def: u64) -> u64 {
    v.get(k).and_then(Value::as_u64).unwrap_or(def)
}

pub fn opt_f64(v: &Value, k: &str) -> Option<f64> {
    v.get(k).and_then(Value::as_f64)
}

pub fn opt_i64(v: &Value, k: &str) -> Option<i64> {
    v.get(k).and_then(Value::as_i64)
}

/// `f` over the array at `k`; empty when it is missing or not an array.
pub fn list<T>(v: &Value, k: &str, f: impl FnMut(&Value) -> T) -> Vec<T> {
    v.get(k)
        .and_then(Value::as_array)
        .map(|a| a.iter().map(f).collect())
        .unwrap_or_default()
}
