//! The D2XX backend (longmynd::d2xx) against the C: the equivalence
//! scenarios run through longmynd-rs's D2XX backend over a simulated D2XX
//! library (equiv/src/d2xxsim.rs, the same simulated MiniTiouner behind
//! it), and the C on the libusb path as always.
//!
//! The call layer cannot be compared - D2XX calls are not libusb calls -
//! so the traces are compared above it: every MPSSE byte written to the
//! FTDI, every decoded I2C transaction, every GPIO write and every sleep,
//! each with its virtual time. Stdout, the status output, the TS output,
//! UDP datagrams and the exit code are compared whole. Equal virtual times
//! mean the two paths also waited the same, including the C's timeouts.

use std::path::{Path, PathBuf};
use std::process::Command;

use longmynd_equiv::scenario;

/// Scenarios that do not apply to D2XX, and why. Everything else in
/// scenario::all() runs here.
const NOT_FOR_D2XX: &[(&str, &str)] = &[
    ("usb_bus_device", "-u bus/device is libusb's numbering; D2XX numbers its tuners 1/1, 1/2, ..."),
    ("usb_wrong_device", "-u, as above"),
    ("usb_no_such_device", "-u, as above"),
    ("picotuner_serial_ts", "a PicoTuner is not an FTDI device: D2XX never lists it (auto falls back to libusb)"),
    ("picotuner_parallel_ts", "as above"),
    ("usb_claim_busy", "another program's FTDI device is listed by D2XX without its details, so it is not found (102) rather than found busy (103): d2xx_busy_tuner_is_not_found"),
];

struct Run {
    dir: PathBuf,
    stdout: String,
}

fn run(bin: &str, scn: &str, dir: &Path, extra: &[&str]) -> Run {
    let out = Command::new(bin).arg(scn).arg(dir).args(extra).output().unwrap();
    assert!(out.status.code().is_some(), "{} died by a signal", scn);
    let stdout = String::from_utf8_lossy(&out.stdout).replace(dir.to_str().unwrap(), "{dir}");
    Run { dir: dir.to_path_buf(), stdout }
}

/// The trace above the call layer: MPSSE bytes out, I2C, GPIO, sleeps.
fn above_call_layer(trace: &str) -> Vec<String> {
    let mut v = Vec::new();
    for l in trace.lines() {
        let (t, rest) = l.trim_start().split_once(' ').unwrap();
        if rest.starts_with("i2c ") || rest.starts_with("gpio ") || rest.starts_with("sleep ") {
            v.push(format!("{} {}", t, rest));
        } else if let Some(x) = rest.split_once(" bulk 02 out ").map(|(_, x)| x).or_else(|| rest.split_once(" FT_Write ").map(|(_, x)| x)) {
            // "[n] hex -> 0 n": the bytes written
            let bytes = x.split_once("] ").unwrap().1.split(" -> ").next().unwrap();
            v.push(format!("{} mpsse-out {}", t, bytes));
        }
    }
    v
}

fn masked(scn: &scenario::Scenario, s: &str) -> String {
    scn.mask(s)
}

fn files(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .filter(|f| f != "trace.txt")
        .collect();
    v.sort();
    v
}

fn check(name: &str) {
    let scn = scenario::find(name).unwrap();
    let base = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("d2xx").join(name);
    let c = run(env!("CARGO_BIN_EXE_c_reference"), name, &base.join("c"), &[]);
    let d = run(env!("CARGO_BIN_EXE_rs_reference"), name, &base.join("d2xx"), &["d2xx"]);

    assert_eq!(masked(&scn, &c.stdout), masked(&scn, &d.stdout), "{}: stdout differs", name);
    assert_eq!(files(&c.dir), files(&d.dir), "{}: different output files", name);
    for f in files(&c.dir) {
        let (a, b) = (std::fs::read(c.dir.join(&f)).unwrap(), std::fs::read(d.dir.join(&f)).unwrap());
        assert!(a == b, "{}: {} differs between the C and D2XX", name, f);
    }
    let read = |r: &Run| masked(&scn, &std::fs::read_to_string(r.dir.join("trace.txt")).unwrap());
    let (ct, dt) = (above_call_layer(&read(&c)), above_call_layer(&read(&d)));
    if let Some(n) = ct.iter().zip(dt.iter()).position(|(a, b)| a != b) {
        panic!("{}: trace differs at {}:\n  C:    {}\n  D2XX: {}", name, n, ct[n], dt[n]);
    }
    assert_eq!(ct.len(), dt.len(), "{}: one trace is a prefix of the other", name);
}

#[test]
fn every_scenario_runs_through_d2xx_or_says_why_not() {
    for s in scenario::all() {
        if NOT_FOR_D2XX.iter().any(|(n, _)| *n == s.name) {
            continue;
        }
        check(s.name);
    }
}

#[test]
fn d2xx_busy_tuner_is_not_found() {
    // MiniTioune (or another longmynd) has the tuner open: D2XX lists it
    // as "opened" with no ID or name, so it cannot be told from any other
    // FTDI device and is not opened - exit 102, with a line saying why.
    let name = "usb_claim_busy";
    let base = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("d2xx").join(name);
    let d = run(env!("CARGO_BIN_EXE_rs_reference"), name, &base.join("d2xx"), &["d2xx"]);
    assert!(d.stdout.contains("D2XX: 2 FTDI interface(s) open in another program (MiniTioune?)"));
    assert!(d.stdout.contains("ERROR: Unable to open device with VID and PID"));
    assert_eq!(std::fs::read_to_string(d.dir.join("exit.txt")).unwrap().trim(), "102");
}
