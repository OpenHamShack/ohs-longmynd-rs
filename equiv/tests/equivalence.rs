//! The proof: every scenario run through the C longmynd and through
//! longmynd-rs on the same simulated MiniTiouner must produce byte-identical
//! results - the USB/I2C trace (every libusb call, MPSSE byte, decoded I2C
//! transaction and sleep, with virtual timestamps), stdout, the status
//! output, the TS output, UDP datagrams and the exit status. Each is then
//! checked against golden/<scenario>/, which was written from the C, so a
//! change to the harness or the simulator that moved both sides together
//! is caught too.
//!
//! LONGMYND_RS_BLESS=1 rewrites the goldens from the C (review the diff).

use std::path::{Path, PathBuf};
use std::process::Command;

use sha2::{Digest, Sha256};

use longmynd_equiv::scenario;

struct Outcome {
    dir: PathBuf,
    stdout: String,
}

fn run(bin: &str, scn: &str, dir: &Path) -> Outcome {
    let out = Command::new(bin).arg(scn).arg(dir).output().expect("run reference binary");
    assert!(out.status.code().is_some(), "{} {} died by a signal: {:?}", bin, scn, out.status);
    let stdout = String::from_utf8_lossy(&out.stdout).replace(dir.to_str().unwrap(), "{dir}");
    // beside the run directory, for reading after a failure
    std::fs::write(dir.with_extension("stdout"), &stdout).unwrap();
    Outcome { dir: dir.to_path_buf(), stdout }
}

fn files(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    v.sort();
    v
}

fn first_difference(a: &[u8], b: &[u8]) -> String {
    let (sa, sb) = (String::from_utf8_lossy(a), String::from_utf8_lossy(b));
    for (n, (la, lb)) in sa.lines().zip(sb.lines()).enumerate() {
        if la != lb {
            return format!("line {}:\n  C:    {}\n  Rust: {}", n + 1, la, lb);
        }
    }
    format!("one is a prefix of the other ({} vs {} lines)", sa.lines().count(), sb.lines().count())
}

fn sha(data: &[u8]) -> String {
    Sha256::digest(data).iter().map(|b| format!("{:02x}", b)).collect()
}

/// The parts of the trace kept in the golden as text: everything but the
/// raw bulk/control lines (whose bytes the sha256 covers).
fn i2c_log(trace: &[u8]) -> String {
    String::from_utf8_lossy(trace)
        .lines()
        .filter(|l| !l.contains(" bulk ") && !l.contains(" control "))
        .map(|l| format!("{}\n", l))
        .collect()
}

fn golden_dir(scn: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("golden").join(scn)
}

/// What a run leaves that the golden holds: (name, content). Masked, so a
/// golden never holds a value the C read from past the end of a table
/// (which moves with the link layout).
fn golden_view(scn: &scenario::Scenario, o: &Outcome) -> Vec<(String, Vec<u8>)> {
    let mut v = vec![("stdout.txt".to_string(), masked(scn, o.stdout.as_bytes()))];
    let mut manifest = String::new();
    for f in files(&o.dir) {
        let data = masked(scn, &std::fs::read(o.dir.join(&f)).unwrap());
        manifest.push_str(&format!("{}  {}  {}\n", sha(&data), data.len(), f));
        match f.as_str() {
            "trace.txt" => v.push(("i2c.log".into(), i2c_log(&data).into_bytes())),
            "status" | "exit.txt" => v.push((f.clone(), data)),
            n if n.starts_with("udp_") && n.contains("47311") => v.push((f.clone(), data)),
            _ => {}
        }
    }
    v.push(("manifest.txt".into(), manifest.into_bytes()));
    v
}

/// The scenario's masks, for the few runs where the C's behaviour is
/// undefined (see scenario.rs); a no-op everywhere else.
fn masked(scn: &scenario::Scenario, data: &[u8]) -> Vec<u8> {
    if scn.masks.is_empty() {
        return data.to_vec();
    }
    scn.mask(&String::from_utf8_lossy(data)).into_bytes()
}

fn check(name: &str) {
    let scn = scenario::find(name).unwrap();
    let base = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("equiv").join(name);
    let c = run(env!("CARGO_BIN_EXE_c_reference"), name, &base.join("c"));
    let r = run(env!("CARGO_BIN_EXE_rs_reference"), name, &base.join("rs"));

    // C and Rust, byte for byte
    let (cs, rs) = (masked(&scn, c.stdout.as_bytes()), masked(&scn, r.stdout.as_bytes()));
    assert!(cs == rs, "{}: stdout differs\n{}", name, first_difference(&cs, &rs));
    assert_eq!(files(&c.dir), files(&r.dir), "{}: different output files", name);
    for f in files(&c.dir) {
        let (a, b) = (std::fs::read(c.dir.join(&f)).unwrap(), std::fs::read(r.dir.join(&f)).unwrap());
        let (a, b) = (masked(&scn, &a), masked(&scn, &b));
        assert!(a == b, "{}: {} differs between C and Rust ({} and {} bytes)\n{}", name, f, a.len(), b.len(), first_difference(&a, &b));
    }

    // and both as the golden from the C says
    let g = golden_dir(name);
    let view = golden_view(&scn, &c);
    if std::env::var("LONGMYND_RS_BLESS").is_ok() {
        let _ = std::fs::remove_dir_all(&g);
        std::fs::create_dir_all(&g).unwrap();
        for (f, data) in &view {
            std::fs::write(g.join(f), data).unwrap();
        }
        return;
    }
    assert!(g.exists(), "{}: no golden (run with LONGMYND_RS_BLESS=1 and review)", name);
    let mut want = files(&g);
    want.sort();
    let mut have: Vec<String> = view.iter().map(|(f, _)| f.clone()).collect();
    have.sort();
    assert_eq!(want, have, "{}: golden holds different files", name);
    for (f, data) in &view {
        let gold = std::fs::read(g.join(f)).unwrap();
        assert!(&gold == data, "{}: {} differs from the golden\n{}", name, f, first_difference(&gold, data));
    }
}

macro_rules! scenarios {
    ($($name:ident),* $(,)?) => {
        $( #[test] fn $name() { check(stringify!($name)); } )*

        #[test]
        fn every_scenario_has_a_test() {
            let mut listed = vec![$(stringify!($name)),*];
            listed.sort();
            let mut all: Vec<&str> = scenario::all().iter().map(|s| s.name).collect();
            all.sort();
            assert_eq!(listed, all);
        }
    };
}

scenarios!(
    startup_437_333,
    lock_s2_1255_1500,
    lock_s_then_lose,
    scan_rates_and_freqs,
    lnb_h_swapped_halfscan,
    lnb_v_timeout_off,
    picotuner_serial_ts,
    picotuner_parallel_ts,
    usb_bus_device,
    tuner_lock_retries,
    no_lna_old_nim,
    low_band_146,
    top_band_2400,
    lna_agc_stepping,
    status_udp,
    ts_fifo_and_names,
    ts_udp,
    args_missing,
    args_freq_low,
    args_lists_need_timeout,
    args_bad_polarisation,
    args_fifo_and_ip,
    args_short_timeout,
    args_unknown_option,
    status_fifo_missing,
    no_tuner_plugged_in,
    usb_wrong_device,
    usb_no_such_device,
    usb_claim_busy,
);

/// cnum.rs against the reference build's own C library and compiler: the
/// same text through strtol/strtof, the same doubles through the casts.
#[test]
fn cnum_matches_the_c() {
    use longmynd::cnum::*;
    let out = Command::new(env!("CARGO_BIN_EXE_c_reference")).arg("--cnum").output().unwrap();
    let c = String::from_utf8(out.stdout).unwrap();
    let mut rust = String::new();
    for t in longmynd_equiv::CNUM_TEXT {
        rust.push_str(&format!("strtol {:?} {}\n", t, strtol(t.as_bytes())));
        rust.push_str(&format!("strtof {:?} {:08x}\n", t, strtof(t.as_bytes()).to_bits()));
    }
    for d in longmynd_equiv::CNUM_DOUBLES {
        rust.push_str(&format!("d2u32 {:e} {}\n", d, f64_to_u32_x86(*d)));
        rust.push_str(&format!("d2i32 {:e} {}\n", d, f64_to_i32_x86(*d)));
    }
    let diffs: Vec<String> = c
        .lines()
        .zip(rust.lines())
        .filter(|(a, b)| a != b)
        .map(|(a, b)| format!("C: {}\nR: {}", a, b))
        .collect();
    assert!(diffs.is_empty() && c.lines().count() == rust.lines().count(), "{}", diffs.join("\n"));
}
