//! The equivalence harness: a simulated MiniTiouner shared by the C
//! reference and the Rust port, the scenarios both are run through, and
//! the Rust-side driver. See README.md, "Proof of equivalence".

pub mod capture;
pub mod d2xxsim;
pub mod rsdrive;
pub mod scenario;
pub mod sim;

/// Inputs for the C-library conversion check (tests/equivalence.rs,
/// cnum_matches_the_c): command-line text through strtol and strtof.
pub const CNUM_TEXT: &[&str] = &[
    "437000", "  437000abc", "-5", "+12", "abc", "", "99999999999999999999", "-99999999999999999999", "1.5", "0.6",
    "2", "1.", ".5", "1.5xyz", "1e2", "1e", "-3", "100.00001", "0.1", "1e-50", "3.4028236e38", "infinity", "-inf",
    "0x10", "0x1.8p1", "-0X.4P-2", "0x1.000001p0", "0x", "007",
];

/// Inputs for the double-to-integer conversions (BER, symbol rate,
/// carrier offset): ordinary, out of range, infinite, NaN.
pub const CNUM_DOUBLES: &[f64] = &[
    0.0, 1234.9, -1234.9, 4294967295.0, 4294967296.0, 3e9, -3e9, 1e19, -1e19, 1e300, f64::INFINITY,
    f64::NEG_INFINITY, f64::NAN, -1.0, 2147483647.5, -2147483648.0, -2147483649.0,
];
