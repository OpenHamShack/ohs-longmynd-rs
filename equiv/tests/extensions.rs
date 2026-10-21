//! Each opt-in option (config::Extensions) does what it says and nothing
//! else. They change the trace on purpose, so they are checked against
//! the C's default run of the same thing rather than for equality.

use std::path::{Path, PathBuf};
use std::process::Command;

use longmynd::regs::stvvglna_regs::*;

struct Run {
    stdout: String,
    trace: String,
}

fn run(bin: &str, scn: &str, dir: &Path) -> Run {
    let out = Command::new(bin).arg(scn).arg(dir).output().unwrap();
    Run {
        stdout: String::from_utf8_lossy(&out.stdout).into_owned(),
        trace: std::fs::read_to_string(dir.join("trace.txt")).unwrap(),
    }
}

fn dir_of(run: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("extensions").join(run)
}

fn rust(scn: &str) -> Run {
    let dir = dir_of(scn);
    run(env!("CARGO_BIN_EXE_rs_reference"), scn, &dir)
}

/// The C on a scenario - for an extension scenario, the same command line
/// with longmynd-rs's options taken out (c_reference does that).
fn c(scn: &str) -> Run {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("extensions").join(format!("{}-c", scn));
    run(env!("CARGO_BIN_EXE_c_reference"), scn, &dir)
}

/// The decoded I2C transactions, without their timestamps.
fn i2c(trace: &str) -> Vec<&str> {
    trace.lines().filter_map(|l| l.find(" i2c ").map(|n| &l[n + 1..])).collect()
}

/// Everything up to the start of the scan, and how long it took.
fn until_scan(trace: &str) -> (Vec<String>, usize, f64) {
    let lines: Vec<&str> = trace.lines().collect();
    let end = lines.iter().position(|l| l.ends_with("i2c d2 f2 16 15")).expect("scan started");
    let transfers = lines[..end].iter().filter(|l| l.contains(" bulk ") || l.contains(" control ")).count();
    let ms: f64 = lines[end].split_whitespace().next().unwrap().parse().unwrap();
    let head = lines[..=end].join("\n");
    (i2c(&head).into_iter().map(String::from).collect(), transfers, ms)
}

#[test]
fn fast_i2c_writes_the_same_registers_with_far_fewer_transfers() {
    let base = c("startup_437_333");
    let fast = rust("ext_fast_i2c");
    let (c_i2c, c_transfers, c_ms) = until_scan(&base.trace);
    let (f_i2c, f_transfers, f_ms) = until_scan(&fast.trace);
    // the same I2C transactions, byte for byte, in the same order
    assert_eq!(c_i2c, f_i2c);
    assert!(f_transfers * 4 < c_transfers, "{} vs {} transfers", f_transfers, c_transfers);
    assert!(f_ms * 2.0 < c_ms, "{} vs {} ms", f_ms, c_ms);
    println!("start-up to scan: C {} USB transfers, {:.0} ms; --fast-i2c {} transfers, {:.0} ms", c_transfers, c_ms, f_transfers, f_ms);
    assert!(fast.stdout.contains("longmynd-rs extensions: --fast-i2c"));
}

#[test]
fn hf_filter_fix_picks_the_widest_filter_above_2265mhz() {
    let base = c("top_band_2400");
    let fixed = rust("ext_hf_filter_fix");
    // CTRL8 = TCAL/CALTIME (0x40) | CFHF: the C wraps to 0xff, the fix 0
    assert!(i2c(&base.trace).contains(&"i2c c0 07 ff"));
    assert!(i2c(&fixed.trace).contains(&"i2c c0 07 40"));
    assert!(!i2c(&fixed.trace).contains(&"i2c c0 07 ff"));
}

#[test]
fn lna_options_set_the_bottom_lna_and_read_it() {
    let r = rust("ext_lna_options");
    let t = i2c(&r.trace);
    // with -w the BOTTOM LNA (0xc8) is the one turned on
    let reg0 = (STVVGLNA_REG0_AGC_TUPD_FAST << STVVGLNA_REG0_AGC_TUPD_SHIFT)
        | (STVVGLNA_REG0_AGC_TLOCK_FAST << STVVGLNA_REG0_AGC_TLOCK_SHIFT);
    let reg2 = (STVVGLNA_REG2_PATH_ACTIVE << STVVGLNA_REG2_PATH2OFF_SHIFT)
        | (STVVGLNA_REG2_RFAGC_PREF_N23DBM << STVVGLNA_REG2_RFAGC_PREF_SHIFT)
        | (STVVGLNA_REG2_PATH_ACTIVE << STVVGLNA_REG2_PATH1OFF_SHIFT)
        | (STVVGLNA_REG2_RFAGC_MODE_MAXIMAL_GAIN_INTERNAL << STVVGLNA_REG2_RFAGC_MODE_SHIFT);
    assert!(t.contains(&format!("i2c c8 00 {:02x}", reg0).as_str()), "REG0");
    assert!(t.contains(&format!("i2c c8 02 {:02x}", reg2).as_str()), "REG2");
    // and the gain is read from it, not from TOP
    let after_scan: Vec<&&str> = t.iter().skip_while(|l| **l != "i2c d2 f2 16 15").collect();
    assert!(after_scan.iter().any(|l| l.starts_with("i2c c8 01 20")));
    assert!(!after_scan.iter().any(|l| l.starts_with("i2c ce ")));
}

#[test]
fn no_lna_poll_leaves_the_lnas_alone_after_init() {
    let r = rust("ext_no_lna_poll");
    let t = i2c(&r.trace);
    let after_scan: Vec<&&str> = t.iter().skip_while(|l| **l != "i2c d2 f2 16 15").collect();
    assert!(after_scan.len() > 100, "the receiver ran");
    assert!(!after_scan.iter().any(|l| l.starts_with("i2c ce ") || l.starts_with("i2c c8 ")));
    // and with no LNA traffic the repeater is never opened in the loop
    assert!(!after_scan.iter().any(|l| l.starts_with("i2c d2 f1 2a b8")));
}

#[test]
fn ts_latency_goes_to_interface_b() {
    let r = rust("ext_ts_latency");
    assert!(r.trace.lines().any(|l| l.ends_with("h1 control 40 09 0002 0002 -> 0")), "latency 2ms, wIndex 2");
}

/// The value of the last GPIO high-byte write (AC0-AC7) in a trace.
fn last_gpio(trace: &str) -> u8 {
    let l = trace.lines().filter(|l| l.contains(" gpio high value=")).last().expect("a GPIO write");
    let v = l.split("value=").nth(1).unwrap();
    u8::from_str_radix(&v[..2], 16).unwrap()
}

#[test]
fn lnb_off_at_exit_turns_the_supply_off_when_stopped() {
    const LNB_ENABLE: u8 = 1 << 4;
    // The C (same command line without the option): the supply is left on
    // after it has stopped - this is the behaviour being fixed.
    let base = c("ext_lnb_off_at_exit");
    assert!(last_gpio(&base.trace) & LNB_ENABLE != 0, "the C leaves the LNB supply on");
    // With the option the last thing done to the GPIOs is enable off.
    let r = rust("ext_lnb_off_at_exit");
    assert!(r.stdout.contains("Flow: LNB supply off at exit"));
    assert_eq!(last_gpio(&r.trace) & LNB_ENABLE, 0, "LNB supply still on at exit");
    // and nothing else about the run changed: the same I2C as the C's
    let n = i2c(&base.trace).len().min(i2c(&r.trace).len());
    assert_eq!(i2c(&base.trace)[..n], i2c(&r.trace)[..n]);
}

#[test]
fn lnb_off_at_exit_does_nothing_without_p() {
    // no -p: the supply was never on, so the exit is the C's exactly
    let r = rust("ext_lnb_off_no_p");
    assert!(!r.stdout.contains("LNB supply off at exit"));
    assert_eq!(r.trace, c("ext_lnb_off_no_p").trace);
}

#[test]
fn strict_errors_keeps_a_failed_reinit_error() {
    let fail = "ERROR: failed to init a device - is the NIM powered on?";
    // The C: the re-init fails (tuner never locks), it says so, then the
    // next scan-state read overwrites the error and it runs on to the end
    // of the scenario (43, stopped as by SIGTERM) still reporting DVB-S.
    let base = c("ext_strict_errors");
    assert!(base.stdout.contains(fail));
    assert_eq!(std::fs::read_to_string(dir_of("ext_strict_errors-c").join("exit.txt")).unwrap().trim(), "43");
    let after = &base.stdout[base.stdout.find(fail).unwrap()..];
    assert!(!after.contains("Flow: STV0910 start scan"), "no new scan was started");
    // With --strict-errors the error stands: longmynd stops with 42.
    let r = rust("ext_strict_errors");
    assert!(r.stdout.contains(fail));
    assert_eq!(std::fs::read_to_string(dir_of("ext_strict_errors").join("exit.txt")).unwrap().trim(), "42");
}

/// Per scan step (each "start scan"): the USB transfers of the (re-)init
/// that led to it, and the value each tuner PLL and demodulator symbol-rate/carrier
/// register was left at (a full init writes some twice: table default,
/// then the computed value).
fn scan_steps(trace: &str) -> Vec<(usize, std::collections::BTreeMap<String, String>)> {
    let mut steps = Vec::new();
    let mut transfers = 0;
    let mut counting = true;
    let mut tuning = std::collections::BTreeMap::new();
    for l in trace.lines() {
        if counting && (l.contains(" bulk ") || l.contains(" control ")) {
            transfers += 1;
        }
        if let Some(n) = l.find(" i2c ") {
            let t = &l[n + 1..];
            // a re-tune starts at nim_init's first write, or a fast
            // re-tune's demodulator stop; the reports between steps are
            // not counted
            if t == "i2c d2 f5 36 aa" || t == "i2c d2 f2 16 1c" {
                counting = true;
            }
            // tuner CTRL3-CTRL8 (N, F, ICP, P, CFHF); demod CFRUP/CFRLOW
            // (0xf242-0xf247) and SFRINIT (0xf25e/f)
            let tuner = ["i2c c0 02 ", "i2c c0 03 ", "i2c c0 04 ", "i2c c0 05 ", "i2c c0 06 ", "i2c c0 07 "];
            let demod = ["i2c d2 f2 42 ", "i2c d2 f2 43 ", "i2c d2 f2 46 ", "i2c d2 f2 47 ", "i2c d2 f2 5e ", "i2c d2 f2 5f "];
            if let Some(p) = tuner.iter().chain(demod.iter()).find(|p| t.starts_with(**p)) {
                tuning.insert(p.to_string(), t[p.len()..].to_string());
            }
            if t == "i2c d2 f2 16 15" {
                steps.push((transfers, std::mem::take(&mut tuning)));
                transfers = 0;
                counting = false;
            }
        }
    }
    steps
}

#[test]
fn fast_retune_tunes_the_same_and_skips_the_reload() {
    let base = c("ext_fast_retune");
    let fast = rust("ext_fast_retune");
    let (cs, fs) = (scan_steps(&base.trace), scan_steps(&fast.trace));
    assert!(cs.len() >= 4 && fs.len() > cs.len(), "{} C steps, {} fast", cs.len(), fs.len());
    // The first step is a full init on both. After it, every step leaves
    // the tuner and demodulator exactly as the C's full init does (same
    // PLL words, same SFRINIT, same carrier range) ...
    for (i, (c_step, f_step)) in cs.iter().zip(fs.iter()).enumerate() {
        assert_eq!(c_step.1, f_step.1, "step {} tunes differently", i);
    }
    // ... for a fraction of the USB traffic.
    for (i, (c_step, f_step)) in cs.iter().zip(fs.iter()).enumerate().skip(1) {
        assert!(f_step.0 * 20 < c_step.0, "step {}: {} transfers vs the C's {}", i, f_step.0, c_step.0);
    }
    println!("re-tune: C {} USB transfers, --fast-retune {}", cs[1].0, fs[1].0);
    // The demodulator's ID is read once (the one full init), not per step.
    let mid_reads = |t: &str| i2c(t).iter().filter(|l| l.starts_with("i2c d2 f1 00 | d3 ")).count();
    assert_eq!(mid_reads(&fast.trace), 1);
    assert_eq!(mid_reads(&base.trace), cs.len());
    // Same scan order as the C's config cycle.
    let cycles = |s: &str| s.lines().filter(|l| l.contains("Config cycle")).map(String::from).collect::<Vec<_>>();
    let (cc, fc) = (cycles(&base.stdout), cycles(&fast.stdout));
    assert_eq!(cc[..], fc[..cc.len()]);
}

#[test]
fn fast_retune_falls_back_to_a_full_init_when_the_tuner_will_not_lock() {
    let r = rust("ext_fast_retune_fallback");
    let fast = r.stdout.find("Flow: Fast re-tune to 437000 KHz, 250 KSymbols/s").expect("fast re-tune tried");
    let after = &r.stdout[fast..];
    assert!(after.contains("ERROR: tuner wait on lock timed out"));
    let fb = after.find("Flow: Fast re-tune failed, full re-init").expect("fell back");
    // the full init, then the scan, as the C does after every config change
    assert!(after[fb..].contains("Flow: NIM init"));
    assert!(after[fb..].contains("Flow: STV0910 start scan"));
    assert!(!after[fb..].contains("ERROR: failed to init a device"));
}

/// The values of status line $n, in order.
fn status_values(dir: &Path, n: u32) -> Vec<String> {
    let prefix = format!("${},", n);
    std::fs::read_to_string(dir.join("status"))
        .unwrap()
        .lines()
        .filter_map(|l| l.strip_prefix(&prefix).map(String::from))
        .collect()
}

#[test]
fn ts_parse_fix_reports_the_true_null_share_and_changes_nothing_else() {
    let base = c("ext_ts_parse_fix");
    let fixed = rust("ext_ts_parse_fix");
    let (cd, fd) = (dir_of("ext_ts_parse_fix-c"), dir_of("ext_ts_parse_fix"));
    // The TS written out is the C's, byte for byte: only the parse changed.
    assert_eq!(std::fs::read(cd.join("ts")).unwrap(), std::fs::read(fd.join("ts")).unwrap());
    // Each service in the stream is PAT, PMT, SDT, 40 video packets and 14
    // nulls: 14/57 = 24 %. The fixed parse reports that (25 where a chunk
    // starts on a null); the C, missing the last packet of every chunk and
    // counting 0x47 bytes inside the video payloads and FTDI status bytes
    // as packets, reports 12-17. The last chunks are short reads of 7
    // packets with one null, 14 %.
    let pct = |d: &Path| -> Vec<u32> { status_values(d, 15).iter().map(|v| v.parse().unwrap()).collect() };
    let (cp, fp) = (pct(&cd), pct(&fd));
    let live = |v: &[u32]| -> Vec<u32> { v.iter().copied().filter(|&p| p != 100 && p != 14).collect() };
    assert!(live(&fp).iter().all(|&p| p == 24 || p == 25), "fixed: {:?}", fp);
    assert!(live(&cp).iter().all(|&p| p < 20), "C: {:?}", cp);
    // the same names, in the same order
    assert_eq!(status_values(&cd, 13), status_values(&fd, 13));
    assert_eq!(status_values(&cd, 14), status_values(&fd, 14));
    assert!(fixed.stdout.contains("longmynd-rs extensions: --ts-parse-fix"));
    assert!(!base.stdout.contains("longmynd-rs extensions"));
}

#[test]
fn extra_status_adds_four_lines_after_the_cs_and_changes_none() {
    let base = c("ext_extra_status");
    rust("ext_extra_status");
    let (cd, rd) = (dir_of("ext_extra_status-c"), dir_of("ext_extra_status"));
    let blocks = |d: &Path| -> Vec<Vec<String>> {
        let text = std::fs::read_to_string(d.join("status")).unwrap();
        let mut v: Vec<Vec<String>> = Vec::new();
        for l in text.lines() {
            if l.starts_with("$1,") {
                v.push(Vec::new());
            }
            v.last_mut().unwrap().push(l.split(',').next().unwrap().to_string());
        }
        v
    };
    // every block is the C's sequence of numbers with $28-$31 appended
    let cb = blocks(&cd);
    for b in blocks(&rd) {
        assert_eq!(&b[b.len() - 4..], ["$28", "$29", "$30", "$31"]);
        assert!(cb.iter().any(|c| c[..] == b[..b.len() - 4]), "a block the C never writes: {:?}", b);
    }
    assert!(!base.stdout.contains("--extra-status"));
    // carrier offset in Hz: the simulator's -73000 hunting and 1830
    // locked, to the register's resolution (135MHz / 2^24 = 8Hz); $6 only
    // shows them as whole kHz (-72 and +1)
    let hz: Vec<i32> = status_values(&rd, 28).iter().map(|v| v.parse().unwrap()).collect();
    assert!((hz[0] + 73000).abs() <= 8, "{:?}", hz);
    assert!((hz.last().unwrap() - 1830).abs() <= 8, "{:?}", hz);
    // roll-off: the simulator's TMGOBS reads 0, i.e. 0.35
    assert!(status_values(&rd, 29).iter().all(|v| v == "35"));
    // TS rate: 0 until a second of TS has been seen, then steady and > 0
    let rate: Vec<u64> = status_values(&rd, 30).iter().map(|v| v.parse().unwrap()).collect();
    assert_eq!(rate[0], 0);
    assert!(*rate.last().unwrap() > 0, "{:?}", rate);
    assert!(status_values(&rd, 31).iter().all(|v| v == "0"));
}

#[test]
fn warn_low_sr_warns_where_the_c_is_silent() {
    let w = "WARNING: SR (33) is below 66 KSymbols/s";
    assert!(!c("ext_warn_low_sr").stdout.contains(w), "the C says nothing");
    let r = rust("ext_warn_low_sr");
    assert!(r.stdout.contains(w));
    assert!(!r.stdout.contains("SR (125) is below"));
    // and carries on exactly as the C: here, to "no tuner" (exit 102)
    assert!(r.stdout.contains("ERROR: Unable to open device with VID and PID"));
}

#[test]
fn ts_timer_after_init_does_not_fire_during_an_init() {
    let timeout = "Flow: No-data timeout, re-init config.";
    let scan = "Flow: STV0910 start scan";
    // The C: the tuner retries take longer than the 5s timeout, so the
    // timer fires before the first scan has even started, queueing a
    // second full re-init behind the first.
    let base = c("ext_ts_timer_after_init");
    let first_scan = base.stdout.find(scan).unwrap();
    assert!(base.stdout[..first_scan].contains(timeout), "the C fires during the init");
    // With the option it cannot fire until 5s after the init has finished.
    let r = rust("ext_ts_timer_after_init");
    let first_scan = r.stdout.find(scan).unwrap();
    assert!(!r.stdout[..first_scan].contains(timeout));
    let inits = |s: &str| s.matches("Flow: NIM init").count();
    assert!(inits(&r.stdout) < inits(&base.stdout), "{} vs {}", inits(&r.stdout), inits(&base.stdout));
}

fn exit_of(run: &str) -> String {
    std::fs::read_to_string(dir_of(run).join("exit.txt")).unwrap().trim().to_string()
}

#[test]
fn reconnect_waits_for_the_tuner_and_starts_again() {
    // The C: the unplug is fatal, exit 42.
    let base = c("ext_reconnect");
    assert_eq!(exit_of("ext_reconnect-c"), "42");
    assert!(!base.stdout.contains("Tuner lost"));
    // With --reconnect: wait, reopen, full init, scanning again; it runs
    // to the end of the scenario (43, as stopped by SIGTERM).
    let r = rust("ext_reconnect");
    assert_eq!(exit_of("ext_reconnect"), "43");
    let lost = r.stdout.find("Flow: Tuner lost, waiting for it to come back (--reconnect)").expect("lost");
    let back = r.stdout[lost..].find("Flow: Tuner is back, reconnecting").expect("back") + lost;
    let after = &r.stdout[back..];
    for step in ["Flow: FTDI/Pico init", "Flow: NIM init", "Flow: STV0910 init", "Flow: STV0910 start scan"] {
        assert!(after.contains(step), "no {} after reconnecting", step);
    }
    // it looked for the tuner once a second while it was away, quietly
    let t = &r.trace;
    let gone = t.find("fault: tuner unplugged").unwrap();
    let returned = t.find("fault: tuner plugged back in").unwrap();
    let polls = t[gone..returned].matches("open_device_with_vid_pid 0403:6010 -> NULL").count();
    assert!((2..=4).contains(&polls), "{} polls while unplugged", polls);
    // and reports resumed: the last status block is hunting
    let states = status_values(&dir_of("ext_reconnect"), 1);
    assert_eq!(states.last().map(String::as_str), Some("1"));
}

#[test]
fn the_decided_defaults_work_together() {
    // what a plain command line does once config::DEFAULTS_AFTER_HW_TEST
    // is set: each of the four as in its own test, all at once
    let base = c("ext_decided_defaults");
    let r = rust("ext_decided_defaults");
    assert!(r.stdout.contains("longmynd-rs extensions: --hf-filter-fix --lna-follow-input --fast-i2c --lnb-off-at-exit"));
    let (c_i2c, c_transfers, _) = until_scan(&base.trace);
    let (r_i2c, r_transfers, _) = until_scan(&r.trace);
    assert!(r_transfers * 4 < c_transfers, "--fast-i2c: {} vs {}", r_transfers, c_transfers);
    // the same I2C bar CTRL8 (the C's narrowest filter, 0xff; the widest,
    // 0x40) and CTRL6 (the charge pump the C reads from past its table)
    let ctrl8 = |v: &[String]| -> Vec<String> {
        v.iter().filter(|l| !l.starts_with("i2c c0 07 ") && !l.starts_with("i2c c0 05 ")).cloned().collect()
    };
    assert_eq!(ctrl8(&c_i2c), ctrl8(&r_i2c));
    assert!(c_i2c.iter().any(|l| l == "i2c c0 07 ff") && r_i2c.iter().any(|l| l == "i2c c0 07 40"));
    // $2 from the BOTTOM LNA (0xc8), which -w uses
    let after_scan: Vec<&str> = i2c(&r.trace).into_iter().skip_while(|l| *l != "i2c d2 f2 16 15").collect();
    assert!(after_scan.iter().any(|l| l.starts_with("i2c c8 01 20")));
    assert!(!after_scan.iter().any(|l| l.starts_with("i2c ce ")));
    // and the LNB supply off at the end
    assert_eq!(last_gpio(&r.trace) & (1 << 4), 0);
    assert_ne!(last_gpio(&base.trace) & (1 << 4), 0);
}

#[test]
fn second_receiver_runs_the_bottom_demodulator_and_tuner_2() {
    let base = c("ext_second_receiver");
    let r = rust("ext_second_receiver");
    let (t, ct) = (i2c(&r.trace), i2c(&base.trace));
    // tuner 2 programmed (CTRL12, its N), its low-pass calibration waited
    // on through STAT2 (0x12; the C would poll STAT1); the C never does
    assert!(t.iter().any(|l| l.starts_with("i2c c0 0c ")));
    assert!(!ct.iter().any(|l| l.starts_with("i2c c0 0c ")));
    // (a register read is the pointer written alone, then a read)
    assert!(t.contains(&"i2c c0 12"));
    assert!(!ct.contains(&"i2c c0 12"));
    // the bottom demodulator's symbol rate the right way round: 1500kS/s
    // is 0x02d8, high byte to P1_SFRINIT1 (0xF45E) as for the top one
    assert!(t.contains(&"i2c d2 f4 5e 02") && t.contains(&"i2c d2 f4 5f d8"));
    // both demodulators started
    assert!(t.contains(&"i2c d2 f2 16 15") && t.contains(&"i2c d2 f4 16 15"));
    // each status from its own demodulator: main DVB-S2 at 333kS/s, the
    // second DVB-S at 1500
    let (main, second) = (dir_of("ext_second_receiver"), dir_of("ext_second_receiver"));
    assert_eq!(status_values(&main, 1).last().map(String::as_str), Some("4"));
    assert_eq!(status_values(&second.join("../ext_second_receiver"), 1).len(), status_values(&main, 1).len());
    let s2 = |n| -> Vec<String> {
        let prefix = format!("${},", n);
        std::fs::read_to_string(second.join("status2")).unwrap().lines().filter_map(|l| l.strip_prefix(&prefix).map(String::from)).collect()
    };
    assert_eq!(s2(1).last().map(String::as_str), Some("3"));
    let sr: u32 = s2(9).last().unwrap().parse().unwrap();
    assert!((1_499_000..=1_501_000).contains(&sr), "{}", sr);
    assert_eq!(s2(6).last().unwrap().parse::<u32>().unwrap() / 1000, 741, "$6 is the second's own frequency");
}

#[test]
fn second_receiver_needs_a_picotuner() {
    let r = rust("ext_second_on_minitiouner");
    assert!(r.stdout.contains("ERROR: --second needs a PicoTuner"));
    assert_eq!(exit_of("ext_second_on_minitiouner"), "2");
}

#[test]
fn second_receiver_ts_goes_to_its_own_output() {
    let base = c("ext_second_ts");
    rust("ext_second_ts");
    let (cd, rd) = (dir_of("ext_second_ts-c"), dir_of("ext_second_ts"));
    // the main TS is the C's, byte for byte, though read in turn with the
    // second endpoint
    assert_eq!(std::fs::read(cd.join("ts")).unwrap(), std::fs::read(rd.join("ts")).unwrap());
    assert!(!base.stdout.contains("--second"));
    // the second's TS: 30 video packets and its tables, status bytes gone
    let ts2 = std::fs::read(rd.join("ts2")).unwrap();
    assert_eq!(ts2.len(), 33 * 188);
    assert!(ts2.chunks(188).all(|p| p[0] == 0x47));
    // each status names its own stream
    assert_eq!(status_values(&rd, 13).last().map(String::as_str), Some("GB3HV"));
    let st2 = std::fs::read_to_string(rd.join("status2")).unwrap();
    assert!(st2.contains("$13,BEACON\n") && st2.contains("$14,QO-100\n"));
    assert!(!st2.contains("GB3HV"));
}
