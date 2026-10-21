//! The bugs longmynd-rs fixes rather than copies, each shown on both sides:
//! the same fault is put to the C and to the Rust, and the test asserts
//! the C has the bug and the Rust does not. These are all on error paths;
//! with no fault, tests/equivalence.rs holds the two byte-identical.

use std::path::{Path, PathBuf};
use std::process::Command;

struct Side {
    stdout: String,
    trace: String,
    exit: String,
}

fn run(bin: &str, scn: &str, dir: &Path) -> Side {
    let out = Command::new(bin).arg(scn).arg(dir).output().unwrap();
    Side {
        stdout: String::from_utf8_lossy(&out.stdout).into_owned(),
        trace: std::fs::read_to_string(dir.join("trace.txt")).unwrap(),
        exit: std::fs::read_to_string(dir.join("exit.txt")).unwrap().trim().to_string(),
    }
}

fn both(scn: &str) -> (Side, Side) {
    let base = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("fixes").join(scn);
    (
        run(env!("CARGO_BIN_EXE_c_reference"), scn, &base.join("c")),
        run(env!("CARGO_BIN_EXE_rs_reference"), scn, &base.join("rs")),
    )
}

/// The transfers after the injected short reads, up to `n` of them.
fn after_fault(trace: &str, n: usize) -> Vec<&str> {
    let lines: Vec<&str> = trace.lines().filter(|l| l.contains(" bulk ")).collect();
    let last = lines.iter().rposition(|l| l.contains("fault: short read")).expect("fault was injected");
    lines[last + 1..].iter().take(n).copied().collect()
}

#[test]
fn a_short_usb_read_does_not_leave_i2c_reading_stale_bytes() {
    let (c, r) = both("fault_short_usb_read");

    // The C: no USB read ever again - every ACK and register value from
    // then on comes out of the old buffer - and the init fails much later,
    // at the demodulator PLL, having been written blind.
    let c_next = after_fault(&c.trace, 40);
    assert!(c_next.iter().all(|l| !l.contains("bulk 81 in")), "C read the USB again:\n{}", c_next.join("\n"));
    assert!(c.stdout.contains("ERROR: STV0910 pll lock timeout"));
    assert_eq!(c.exit, "42");

    // The Rust: the I2C retry reads afresh, and the init completes.
    let r_next = after_fault(&r.trace, 2);
    assert!(r_next[1].contains("bulk 81 in"), "Rust did not read the USB again:\n{}", r_next.join("\n"));
    assert!(r.stdout.contains("Flow: STV0910 start scan"));
    assert!(!r.stdout.contains("pll lock timeout"));
    assert_eq!(r.exit, "43", "ran to the end of the scenario");
}

#[test]
fn a_nim_failing_the_readback_check_is_an_error() {
    let (c, r) = both("fault_demod_readback");

    // The C sees the mismatch, says so, then overwrites the error and runs
    // on with the NIM it has just found broken.
    assert!(c.stdout.contains("ERROR: nim_init didn't r/w to the modulator"));
    assert!(c.stdout.contains("Flow: STV0910 start scan"));
    assert_eq!(c.exit, "43");

    // The Rust stops there, as the check intends.
    assert!(r.stdout.contains("ERROR: nim_init didn't r/w to the modulator"));
    assert!(r.stdout.contains("ERROR: nim_init\n"));
    assert!(r.stdout.contains("ERROR: failed to init a device - is the NIM powered on?"));
    assert!(!r.stdout.contains("Flow: STV0910 start scan"));
    assert_eq!(r.exit, "42");
}
