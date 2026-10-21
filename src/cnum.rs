//! The C library's number handling, reproduced where longmynd's output
//! depends on it.
//!
//! A drop-in replacement has to agree with the C on inputs nobody would
//! type on purpose as well as the obvious ones: `437000abc` is 437000 to
//! strtol, `-5` as a uint32 prints as `-5` through `%i`, and a BER computed
//! as 0/0 comes out of the C as 0 rather than as Rust's saturating cast
//! would have it. Each function here names the C behaviour it copies.

/// glibc `isspace` in the C locale.
fn is_c_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// `strtol(s, NULL, 10)` on LP64: optional leading whitespace and sign, the
/// longest run of decimal digits, 0 when there are none, and LONG_MAX /
/// LONG_MIN when the value does not fit (glibc's ERANGE result).
pub fn strtol(s: &[u8]) -> i64 {
    let mut i = 0;
    while i < s.len() && is_c_space(s[i]) {
        i += 1;
    }
    let mut negative = false;
    if i < s.len() && (s[i] == b'+' || s[i] == b'-') {
        negative = s[i] == b'-';
        i += 1;
    }
    let mut value: i64 = 0;
    let mut overflow = false;
    while i < s.len() && s[i].is_ascii_digit() {
        let d = (s[i] - b'0') as i64;
        // accumulate negatively so LONG_MIN is reachable, as glibc does
        match value.checked_mul(10).and_then(|v| v.checked_sub(d)) {
            Some(v) => value = v,
            None => overflow = true,
        }
        i += 1;
    }
    if overflow {
        return if negative { i64::MIN } else { i64::MAX };
    }
    if negative {
        value
    } else {
        value.checked_neg().unwrap_or(i64::MAX)
    }
}

/// A C99 hexadecimal float body after "0x" ("1.8p3"), or None when there
/// is no hex digit. Exact to 13 hex digits, with any further non-zero digit
/// kept as a sticky bit, which is enough for a correctly rounded f32.
fn hex_float(s: &[u8]) -> Option<f64> {
    let mut i = 0;
    let mut mant: u64 = 0;
    let mut digits = 0;
    let mut exp: i64 = 0;
    let mut seen = false;
    let mut sticky = false;
    let mut point = false;
    while i < s.len() {
        let c = s[i];
        let d = match c {
            b'0'..=b'9' => (c - b'0') as u64,
            b'a'..=b'f' => (c - b'a' + 10) as u64,
            b'A'..=b'F' => (c - b'A' + 10) as u64,
            b'.' if !point => {
                point = true;
                i += 1;
                continue;
            }
            _ => break,
        };
        seen = true;
        if mant == 0 && d == 0 {
            if point {
                exp -= 4;
            }
        } else if digits < 13 {
            mant = mant << 4 | d;
            digits += 1;
            if point {
                exp -= 4;
            }
        } else {
            sticky |= d != 0;
            if !point {
                exp += 4;
            }
        }
        i += 1;
    }
    if !seen {
        return None;
    }
    if sticky {
        mant = mant << 1 | 1;
        exp -= 1;
    }
    if i < s.len() && (s[i] | 0x20) == b'p' {
        let mut j = i + 1;
        let neg = j < s.len() && s[j] == b'-';
        if j < s.len() && (s[j] == b'-' || s[j] == b'+') {
            j += 1;
        }
        let start = j;
        let mut e: i64 = 0;
        while j < s.len() && s[j].is_ascii_digit() {
            e = (e * 10 + (s[j] - b'0') as i64).min(100_000);
            j += 1;
        }
        if j > start {
            exp += if neg { -e } else { e };
        }
    }
    let mut v = mant as f64;
    // scale by 2^exp in steps that stay exact
    let mut e = exp;
    while e > 0 {
        let k = e.min(1000);
        v *= 2f64.powi(k as i32);
        e -= k;
    }
    while e < 0 {
        let k = (-e).min(1000);
        v /= 2f64.powi(k as i32);
        e += k;
    }
    Some(v)
}

/// `strtof(s, NULL)`: the longest prefix that is a decimal or hexadecimal
/// float, `inf`, `infinity` or `nan` (any case), 0.0 when there is none.
/// Rust's parser is correctly rounded, as glibc's is, so the same text
/// gives the same float.
pub fn strtof(s: &[u8]) -> f32 {
    let mut i = 0;
    while i < s.len() && is_c_space(s[i]) {
        i += 1;
    }
    let start = i;
    if i < s.len() && (s[i] == b'+' || s[i] == b'-') {
        i += 1;
    }
    let lower: Vec<u8> = s[i..].iter().map(|b| b.to_ascii_lowercase()).collect();
    let sign = if start < s.len() && s[start] == b'-' { -1.0 } else { 1.0 };
    if lower.starts_with(b"inf") {
        return sign * f32::INFINITY;
    }
    if lower.starts_with(b"nan") {
        return f32::NAN.copysign(sign);
    }
    if lower.starts_with(b"0x") {
        if let Some(v) = hex_float(&s[i + 2..]) {
            return (sign as f64 * v) as f32;
        }
    }
    let mut digits = 0;
    while i < s.len() && s[i].is_ascii_digit() {
        i += 1;
        digits += 1;
    }
    if i < s.len() && s[i] == b'.' {
        i += 1;
        while i < s.len() && s[i].is_ascii_digit() {
            i += 1;
            digits += 1;
        }
    }
    if digits == 0 {
        return 0.0;
    }
    let mantissa_end = i;
    if i < s.len() && (s[i] == b'e' || s[i] == b'E') {
        let mut j = i + 1;
        if j < s.len() && (s[j] == b'+' || s[j] == b'-') {
            j += 1;
        }
        let exp_start = j;
        while j < s.len() && s[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_start {
            i = j;
        }
    }
    let text = std::str::from_utf8(&s[start..i]).unwrap_or("0");
    // A trailing '.' ("1.") is valid C but not valid Rust; it adds nothing.
    let text = text.strip_suffix('.').unwrap_or(text);
    text.parse::<f32>().unwrap_or_else(|_| {
        std::str::from_utf8(&s[start..mantissa_end])
            .ok()
            .and_then(|t| t.strip_suffix('.').unwrap_or(t).parse().ok())
            .unwrap_or(0.0)
    })
}

/// `(uint32_t)d` as gcc compiles it for x86-64: `cvttsd2si` into a 64-bit
/// register, keeping the low 32 bits. Out-of-range and NaN give the
/// "integer indefinite" 0x8000000000000000, whose low half is 0 - so a BER
/// of 0/0 or n/0 reads 0, where Rust's saturating `as` would give 0 and
/// u32::MAX. The reference longmynd is the x86-64 build OpenHamShack ships.
pub fn f64_to_u32_x86(d: f64) -> u32 {
    if d.is_nan() || d >= 9.223_372_036_854_775_808e18 || d < -9.223_372_036_854_775_808e18 {
        0
    } else {
        (d as i64) as u32
    }
}

/// `(int32_t)d` on x86-64: `cvttsd2si` into a 32-bit register; out of range
/// and NaN give 0x80000000.
pub fn f64_to_i32_x86(d: f64) -> i32 {
    if d.is_nan() || d >= 2_147_483_648.0 || d < -2_147_483_648.0 {
        i32::MIN
    } else {
        d as i32
    }
}

/// A float converted to `uint32_t` on x86-64 (`cvttss2si` into 64 bits).
pub fn f32_to_u32_x86(f: f32) -> u32 {
    f64_to_u32_x86(f as f64)
}

/// strncpy into a `char[size]` that started zeroed, then read back as a C
/// string: at most size-1 bytes, and only up to the first NUL.
pub fn c_strncpy(src: &[u8], size: usize) -> Vec<u8> {
    let s = &src[..src.len().min(size - 1)];
    match s.iter().position(|&b| b == 0) {
        Some(n) => s[..n].to_vec(),
        None => s.to_vec(),
    }
}

/// glibc's `strerror(errno)` text, which the C prints in its fifo errors.
/// std formats an OS error as "<strerror text> (os error N)" using the same
/// libc call, so the text is taken from there rather than kept in a table
/// that could disagree with the platform.
pub fn strerror(err: &std::io::Error) -> String {
    let s = match err.raw_os_error() {
        Some(code) => std::io::Error::from_raw_os_error(code).to_string(),
        None => err.to_string(),
    };
    match s.rfind(" (os error ") {
        Some(n) => s[..n].to_string(),
        None => s,
    }
}

/// `inet_addr(cp)`: the numbers-and-dots forms the C accepts ("a.b.c.d",
/// "a.b.c", "a.b", "a", each part decimal, octal with a leading 0 or hex
/// with 0x), or INADDR_NONE (255.255.255.255) when it is not one. Rust's
/// Ipv4Addr parser takes only the first form.
pub fn inet_addr(cp: &[u8]) -> std::net::Ipv4Addr {
    fn parse(cp: &[u8]) -> Option<u32> {
        let mut parts: Vec<u64> = Vec::new();
        let mut i = 0;
        loop {
            if i >= cp.len() || !cp[i].is_ascii_digit() {
                return None;
            }
            let (base, mut j) = if cp[i] == b'0' && i + 1 < cp.len() && (cp[i + 1] | 0x20) == b'x' {
                (16, i + 2)
            } else if cp[i] == b'0' {
                (8, i + 1)
            } else {
                (10, i)
            };
            let mut v: u64 = 0;
            let digits_start = j;
            while j < cp.len() {
                let c = cp[j];
                let d = match c {
                    b'0'..=b'9' if base != 8 || c < b'8' => (c - b'0') as u64,
                    b'a'..=b'f' if base == 16 => (c - b'a' + 10) as u64,
                    b'A'..=b'F' if base == 16 => (c - b'A' + 10) as u64,
                    _ => break,
                };
                v = v.checked_mul(base)?.checked_add(d)?;
                if v > 0xffff_ffff {
                    return None;
                }
                j += 1;
            }
            if base == 16 && j == digits_start {
                return None;
            }
            parts.push(v);
            if j < cp.len() && cp[j] == b'.' {
                if parts.len() == 4 {
                    return None;
                }
                i = j + 1;
                continue;
            }
            // glibc's inet_aton stops at whitespace and ignores the rest.
            if j < cp.len() && !is_c_space(cp[j]) {
                return None;
            }
            break;
        }
        let max_last = [0xffff_ffffu64, 0xff_ffff, 0xffff, 0xff];
        let n = parts.len();
        if parts[..n - 1].iter().any(|&p| p > 0xff) || parts[n - 1] > max_last[n - 1] {
            return None;
        }
        let mut addr = parts[n - 1] as u32;
        for (k, p) in parts[..n - 1].iter().enumerate() {
            addr |= (*p as u32) << (24 - 8 * k);
        }
        Some(addr)
    }
    std::net::Ipv4Addr::from(parse(cp).unwrap_or(0xffff_ffff))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strtol_matches_glibc() {
        assert_eq!(strtol(b"437000"), 437000);
        assert_eq!(strtol(b"  437000abc"), 437000);
        assert_eq!(strtol(b"-5"), -5);
        assert_eq!(strtol(b"+12"), 12);
        assert_eq!(strtol(b"abc"), 0);
        assert_eq!(strtol(b""), 0);
        assert_eq!(strtol(b"99999999999999999999"), i64::MAX);
        assert_eq!(strtol(b"-99999999999999999999"), i64::MIN);
        assert_eq!(strtol(b"-9223372036854775808"), i64::MIN);
        assert_eq!((strtol(b"-5") as u32) as i32, -5);
    }

    #[test]
    fn strtof_matches_glibc() {
        assert_eq!(strtof(b"1.5"), 1.5);
        assert_eq!(strtof(b"0.6"), 0.6f32);
        assert_eq!(strtof(b"2"), 2.0);
        assert_eq!(strtof(b"1."), 1.0);
        assert_eq!(strtof(b".5"), 0.5);
        assert_eq!(strtof(b"1.5xyz"), 1.5);
        assert_eq!(strtof(b"1e2"), 100.0);
        assert_eq!(strtof(b"1e"), 1.0);
        assert_eq!(strtof(b"-3"), -3.0);
        assert_eq!(strtof(b"abc"), 0.0);
        assert!(strtof(b"inf").is_infinite());
        assert!(strtof(b"nan").is_nan());
    }

    #[test]
    fn x86_conversions() {
        assert_eq!(f64_to_u32_x86(f64::NAN), 0);
        assert_eq!(f64_to_u32_x86(f64::INFINITY), 0);
        assert_eq!(f64_to_u32_x86(1234.9), 1234);
        assert_eq!(f64_to_u32_x86(4294967296.0), 0);
        assert_eq!(f64_to_u32_x86(-1.0), u32::MAX);
        assert_eq!(f64_to_i32_x86(-1234.9), -1234);
        assert_eq!(f64_to_i32_x86(3e9), i32::MIN);
    }

    #[test]
    fn inet_addr_forms() {
        use std::net::Ipv4Addr;
        assert_eq!(inet_addr(b"198.51.100.1"), Ipv4Addr::new(192, 168, 1, 1));
        assert_eq!(inet_addr(b"127.1"), Ipv4Addr::new(127, 0, 0, 1));
        assert_eq!(inet_addr(b"0x7f.1"), Ipv4Addr::new(127, 0, 0, 1));
        assert_eq!(inet_addr(b"010.0.0.1"), Ipv4Addr::new(8, 0, 0, 1));
        assert_eq!(inet_addr(b"1.2.3.4.5"), Ipv4Addr::BROADCAST);
        assert_eq!(inet_addr(b"bogus"), Ipv4Addr::BROADCAST);
        assert_eq!(inet_addr(b"256.1.1.1"), Ipv4Addr::BROADCAST);
    }

    #[test]
    fn strncpy_truncates_like_a_c_array() {
        assert_eq!(c_strncpy(b"203.0.113.200:1", 16), b"203.0.113.200".to_vec());
        assert_eq!(c_strncpy(b"abc", 16), b"abc".to_vec());
    }
}
