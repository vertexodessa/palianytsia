//! A tiny JSON writer whose output matches nlohmann::json's `dump()` byte for
//! byte for the values the stats readout uses: object keys sorted (std::map),
//! no whitespace, doubles in nlohmann's shortest form ("1.0", "0.0001",
//! "1e-05", "1.5e+16"). The parity gate compares the stats line of both cores
//! as bytes, so serde_json's float spelling ("1e-5") would not do.
use std::collections::BTreeMap;
use std::fmt::Write;

#[derive(Clone, Debug)]
pub enum J {
    Null,
    Bool(bool),
    I(i64),
    U(u64),
    F(f64),
    S(String),
    A(Vec<J>),
    O(BTreeMap<&'static str, J>),
}

impl J {
    pub fn obj() -> Self {
        J::O(BTreeMap::new())
    }
    /// Insert into an object; a no-op on anything else.
    pub fn set(&mut self, k: &'static str, v: impl Into<J>) {
        if let J::O(m) = self {
            m.insert(k, v.into());
        }
    }
    pub fn arr() -> Self {
        J::A(Vec::new())
    }
    /// Append to an array; a no-op on anything else.
    pub fn push(&mut self, v: impl Into<J>) {
        if let J::A(a) = self {
            a.push(v.into());
        }
    }
    pub fn dump(&self) -> String {
        let mut s = String::new();
        self.write(&mut s);
        s
    }
    fn write(&self, out: &mut String) {
        match self {
            J::Null => out.push_str("null"),
            J::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
            J::I(v) => {
                let _ = write!(out, "{v}");
            }
            J::U(v) => {
                let _ = write!(out, "{v}");
            }
            J::F(v) => write_double(out, *v),
            J::S(s) => write_str(out, s),
            J::A(a) => {
                out.push('[');
                for (i, v) in a.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    v.write(out);
                }
                out.push(']');
            }
            J::O(m) => {
                out.push('{');
                for (i, (k, v)) in m.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    write_str(out, k);
                    out.push(':');
                    v.write(out);
                }
                out.push('}');
            }
        }
    }
}

impl From<bool> for J {
    fn from(v: bool) -> Self {
        J::Bool(v)
    }
}
impl From<i64> for J {
    fn from(v: i64) -> Self {
        J::I(v)
    }
}
impl From<i32> for J {
    fn from(v: i32) -> Self {
        J::I(i64::from(v))
    }
}
impl From<u64> for J {
    fn from(v: u64) -> Self {
        J::U(v)
    }
}
impl From<u32> for J {
    fn from(v: u32) -> Self {
        J::U(u64::from(v))
    }
}
impl From<u8> for J {
    fn from(v: u8) -> Self {
        J::U(u64::from(v))
    }
}
impl From<f64> for J {
    fn from(v: f64) -> Self {
        // nlohmann dumps a non-finite double as null.
        if v.is_finite() {
            J::F(v)
        } else {
            J::Null
        }
    }
}
impl From<&str> for J {
    fn from(v: &str) -> Self {
        J::S(v.to_string())
    }
}
impl From<String> for J {
    fn from(v: String) -> Self {
        J::S(v)
    }
}
impl<T: Into<J>> From<Option<T>> for J {
    fn from(v: Option<T>) -> Self {
        v.map_or(J::Null, Into::into)
    }
}

fn write_str(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{08}' => out.push_str("\\b"),
            '\u{0c}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

/// nlohmann `dtoa_impl::format_buffer` with min_exp -4, max_exp 15 over the
/// shortest round-trip digits.
fn write_double(out: &mut String, v: f64) {
    if v == 0.0 {
        out.push_str(if v.is_sign_negative() { "-0.0" } else { "0.0" });
        return;
    }
    if v < 0.0 {
        out.push('-');
    }
    // "d.ddde-x": shortest digits and the exponent of the first one.
    let e = format!("{:e}", v.abs());
    let (mant, exp) = e.split_once('e').expect("LowerExp has an exponent");
    let digits: String = mant.chars().filter(|c| *c != '.').collect();
    let k = digits.len() as i32;
    let n = exp.parse::<i32>().expect("exponent") + 1;
    if k <= n && n <= 15 {
        out.push_str(&digits);
        out.extend(std::iter::repeat_n('0', (n - k) as usize));
        out.push_str(".0");
    } else if 0 < n && n <= 15 {
        out.push_str(&digits[..n as usize]);
        out.push('.');
        out.push_str(&digits[n as usize..]);
    } else if -4 < n && n <= 0 {
        out.push_str("0.");
        out.extend(std::iter::repeat_n('0', (-n) as usize));
        out.push_str(&digits);
    } else {
        out.push_str(&digits[..1]);
        if k > 1 {
            out.push('.');
            out.push_str(&digits[1..]);
        }
        let x = n - 1;
        let _ = write!(out, "e{}{:02}", if x < 0 { '-' } else { '+' }, x.abs());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(v: f64) -> String {
        J::from(v).dump()
    }

    #[test]
    fn doubles_as_nlohmann() {
        assert_eq!(d(1.0), "1.0");
        assert_eq!(d(0.0), "0.0");
        assert_eq!(d(-2.5), "-2.5");
        assert_eq!(d(0.03389830508474576), "0.03389830508474576");
        assert_eq!(d(0.0001), "0.0001");
        assert_eq!(d(0.00001), "1e-05");
        assert_eq!(d(1.5e16), "1.5e+16");
        assert_eq!(d(123456789012345.0), "123456789012345.0");
        assert_eq!(d(1e15), "1e+15");
        assert_eq!(d(4.685160659309358), "4.685160659309358");
        assert_eq!(d(f64::NAN), "null");
    }

    #[test]
    fn object_keys_sorted() {
        let mut o = J::obj();
        o.set("b", 1u64);
        o.set("a", Some(-1i64));
        o.set("c", None::<u64>);
        o.set("s", "x\"y");
        assert_eq!(o.dump(), r#"{"a":-1,"b":1,"c":null,"s":"x\"y"}"#);
    }
}
