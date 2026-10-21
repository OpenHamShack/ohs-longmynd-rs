//! main.c's configuration: the command line (process_command_line) and the
//! frequency / symbol rate cycling the no-TS timeout drives
//! (config_reinit).
//!
//! The parser is the C's, quirk for quirk, because scripts and the OpenHamShack
//! daemon already build command lines for the C and a drop-in has to read
//! them identically: options are only looked for while at least two
//! arguments remain after them, an unrecognised option swallows the
//! argument after it, numbers are strtol/strtof prefixes, and strings are
//! cut to the C's array sizes.

use crate::cnum::{c_strncpy, strtof, strtol};
use crate::errors::*;

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    pub port_swap: bool,
    pub halfscan_ratio: f32,
    pub freq_index: u8,
    pub sr_index: u8,
    /// kHz; 0 ends the list
    pub freq_requested: [u32; 4],
    /// kS/s; 0 ends the list
    pub sr_requested: [u32; 4],
    pub beep_enabled: bool,

    pub device_usb_bus: u8,
    pub device_usb_addr: u8,

    pub ts_use_ip: bool,
    pub ts_fifo_path: Vec<u8>,
    pub ts_ip_addr: Vec<u8>,
    pub ts_ip_port: i32,

    pub status_use_ip: bool,
    pub status_fifo_path: Vec<u8>,
    pub status_ip_addr: Vec<u8>,
    pub status_ip_port: i32,

    pub polarisation_supply: bool,
    /// false: 13V, true: 18V
    pub polarisation_horizontal: bool,

    /// ms, or -1 for never
    pub ts_timeout: i32,

    /// A new configuration for the receiver to apply.
    pub new: bool,

    /// Opt-in behaviour beyond the C's (see docs/improvements.md). All off
    /// by default; the equivalence proof covers the default.
    pub ext: Extensions,
}

/// Behaviour that is not the C's, each behind its own `--` option and off
/// by default (docs/improvements.md has the case for each). With none of
/// them given, longmynd-rs is byte-identical to the C - the equivalence
/// tests hold it to that.
#[derive(Clone, Debug, PartialEq)]
pub struct Extensions {
    /// --hf-filter-fix: above 2265MHz pick the widest STV6120 HF filter,
    /// where the C's search finds no entry and wraps to the narrowest.
    pub hf_filter_fix: bool,
    /// --lna-follow-input: report ($2) the LNA of the input in use; the C
    /// always reads the TOP one, even with -w.
    pub lna_follow_input: bool,
    /// --no-lna-poll: do not ask the LNA for its gain every 100ms (the I2C
    /// traffic, and the repeater opening and closing, stop; $2 stays at 0).
    pub lna_poll: bool,
    /// --lna-mode track|request|min|max: STVVGLNA RFAGC_MODE (C: track).
    pub lna_mode: Option<u8>,
    /// --lna-pref -25..-18: the LNA AGC's target level, dBm (C: -20).
    pub lna_pref: Option<u8>,
    /// --lna-tlock fast|slow: the LNA AGC's lock time constant (C: slow).
    pub lna_tlock_fast: Option<bool>,
    /// --fast-i2c: load the STV0910's 935-register table in batches, one
    /// USB round trip per batch instead of four per register.
    pub fast_i2c: bool,
    /// --ts-latency MS: set the TS interface's (B) latency timer; the C
    /// sends its latency to interface A twice and leaves B at the default.
    pub ts_latency_ms: Option<u8>,
    /// --lnb-off-at-exit: when longmynd stops (signal or error) after -p
    /// turned the RT5047A LNB supply on, turn it off again. The C leaves
    /// the GPIO as it was, so the feed keeps 13/18V until the next start.
    pub lnb_off_at_exit: bool,
    /// --strict-errors: an error left by a failed re-init (or a failed
    /// report) is kept in every receiver state, so longmynd stops (exit
    /// 42). The C guards only the HUNTING state's scan-state read; in the
    /// others the read overwrites the error and it runs on half-initialised.
    pub strict_errors: bool,
    /// --fast-retune: on a new frequency/symbol rate with the NIM already
    /// up, re-tune the demodulator and tuner only, instead of nim_init's
    /// whole reload (935 demodulator registers, PLL, tuner, LNAs).
    pub fast_retune: bool,
    /// --ts-parse-fix: parse the TS for names, PIDs and the null share
    /// without the C's faults (FTDI status bytes parsed as TS, 0x47 inside
    /// payloads taken as packet starts, the last packet of each chunk
    /// skipped, SDT services found at the wrong offsets).
    pub ts_parse_fix: bool,
    /// --extra-status: four more status lines on numbers the C does not
    /// use ($28-$31): carrier offset in Hz, roll-off, TS rate, TS pieces
    /// dropped. No existing line changes.
    pub extra_status: bool,
    /// --warn-low-sr: say so when a symbol rate below 66kS/s is asked for.
    /// The C accepts down to 33; a MiniTiouner's STV0910 has not been seen
    /// to lock below about 66 (OpenHamShack DATV's research).
    pub warn_low_sr: bool,
    /// --ts-timer-after-init: the no-TS timeout does not run while the
    /// NIM is being (re-)initialised, and starts from the end of the init.
    /// The C's timer runs through the init (about 2s, much longer with
    /// tuner retries), so a second re-init can be queued during the first.
    pub ts_timer_after_init: bool,
    /// --reconnect: when a worker thread fails (a USB unplug, typically),
    /// close the tuner, wait for it to come back (checked once a second)
    /// and start again with the current configuration, instead of exiting.
    pub reconnect: bool,
    /// --usb-backend auto|libusb|d2xx: how to reach the tuner (src/d2xx.rs).
    /// Not behaviour towards the NIM: what goes to it is the same either way.
    pub usb_backend: UsbBackendChoice,
    /// --second FREQ SR: a PicoTuner's second receiver (the bottom
    /// demodulator and tuner 2, TS on endpoint 0x84). The C never starts it.
    pub second: Option<Second>,
}

/// Where an output goes, as -t/-s or -i/-I give it for the main receiver.
#[derive(Clone, Debug, PartialEq)]
pub enum Out {
    Fifo(Vec<u8>),
    Udp(Vec<u8>, i32),
}

/// The second receiver (--second and its outputs).
#[derive(Clone, Debug, PartialEq)]
pub struct Second {
    /// kHz
    pub freq: u32,
    /// kS/s
    pub sr: u32,
    pub ts: Out,
    pub status: Out,
}

/// --usb-backend
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UsbBackendChoice {
    /// Windows: FTDI's D2XX when ftd2xx.dll loads and sees the tuner, else
    /// libusb. Elsewhere: libusb.
    Auto,
    Libusb,
    D2xx,
}

/// Brian's decision of 06/10/2026: once the MiniTiouner hardware test
/// confirms them, `--fast-i2c`, `--hf-filter-fix`, `--lna-follow-input`
/// and `--lnb-off-at-exit` become the defaults, and `--c-compat` gives the
/// C's behaviour back. Setting this to true is that whole change (HANDOVER,
/// "Apply after hardware test"); the equivalence harness then runs its
/// Rust side with `--c-compat`, so the proof against the C still holds.
pub const DEFAULTS_AFTER_HW_TEST: bool = false;

impl Default for Extensions {
    fn default() -> Self {
        Extensions::defaults(DEFAULTS_AFTER_HW_TEST)
    }
}

impl Extensions {
    /// The defaults: the C's behaviour, and with `after_hw_test` the four
    /// options Brian decided on turned on.
    pub fn defaults(after_hw_test: bool) -> Self {
        let mut e = Extensions::c_behaviour();
        if after_hw_test {
            e.set_decided_defaults();
        }
        e
    }

    /// The four decided defaults (see [`DEFAULTS_AFTER_HW_TEST`]).
    fn set_decided_defaults(&mut self) {
        self.fast_i2c = true;
        self.hf_filter_fix = true;
        self.lna_follow_input = true;
        self.lnb_off_at_exit = true;
    }

    /// Exactly what the C does: every option off.
    pub fn c_behaviour() -> Self {
        Extensions {
            hf_filter_fix: false,
            lna_follow_input: false,
            lna_poll: true,
            lna_mode: None,
            lna_pref: None,
            lna_tlock_fast: None,
            fast_i2c: false,
            ts_latency_ms: None,
            lnb_off_at_exit: false,
            strict_errors: false,
            fast_retune: false,
            ts_parse_fix: false,
            extra_status: false,
            warn_low_sr: false,
            ts_timer_after_init: false,
            reconnect: false,
            usb_backend: UsbBackendChoice::Auto,
            second: None,
        }
    }

    /// Whether anything differs from the C (and so the configuration
    /// summary gets a "longmynd-rs extensions" line).
    pub fn any(&self) -> bool {
        *self != Extensions::c_behaviour()
    }

    fn describe(&self) -> String {
        let mut v: Vec<String> = Vec::new();
        if self.hf_filter_fix {
            v.push("--hf-filter-fix".into());
        }
        if self.lna_follow_input {
            v.push("--lna-follow-input".into());
        }
        if !self.lna_poll {
            v.push("--no-lna-poll".into());
        }
        if let Some(m) = self.lna_mode {
            v.push(format!("--lna-mode {}", ["track", "request", "min", "max"][m as usize]));
        }
        if let Some(p) = self.lna_pref {
            v.push(format!("--lna-pref {}", p as i32 - 25));
        }
        if let Some(f) = self.lna_tlock_fast {
            v.push(format!("--lna-tlock {}", if f { "fast" } else { "slow" }));
        }
        if self.fast_i2c {
            v.push("--fast-i2c".into());
        }
        if let Some(ms) = self.ts_latency_ms {
            v.push(format!("--ts-latency {}", ms));
        }
        if self.lnb_off_at_exit {
            v.push("--lnb-off-at-exit".into());
        }
        if self.strict_errors {
            v.push("--strict-errors".into());
        }
        if self.fast_retune {
            v.push("--fast-retune".into());
        }
        if self.ts_parse_fix {
            v.push("--ts-parse-fix".into());
        }
        if self.extra_status {
            v.push("--extra-status".into());
        }
        if self.warn_low_sr {
            v.push("--warn-low-sr".into());
        }
        if self.ts_timer_after_init {
            v.push("--ts-timer-after-init".into());
        }
        if self.reconnect {
            v.push("--reconnect".into());
        }
        if let Some(s) = &self.second {
            let out = |o: &Out| match o {
                Out::Fifo(p) => format!("fifo {}", String::from_utf8_lossy(p)),
                Out::Udp(ip, port) => format!("udp {}:{}", String::from_utf8_lossy(ip), port),
            };
            v.push(format!("--second {} {} (TS {}, status {})", s.freq, s.sr, out(&s.ts), out(&s.status)));
        }
        match self.usb_backend {
            UsbBackendChoice::Auto => {}
            UsbBackendChoice::Libusb => v.push("--usb-backend libusb".into()),
            UsbBackendChoice::D2xx => v.push("--usb-backend d2xx".into()),
        }
        v.join(" ")
    }
}

/// Take longmynd-rs's own `--` options out of the command line, leaving
/// the C's for process_command_line. Only these exact names are taken: any
/// other `--x` stays where it is and the C-compatible parser treats it as
/// the C does (an unknown option, swallowing the argument after it).
pub fn split_extensions(args: &[Vec<u8>]) -> Result<(Vec<Vec<u8>>, Extensions), String> {
    // Options only ever turn things on, from the C's behaviour; the
    // decided defaults are added at the end unless --c-compat was given,
    // so "--c-compat --fast-i2c" is the C plus --fast-i2c whatever order.
    let mut ext = Extensions::c_behaviour();
    let mut c_compat = false;
    let mut second: Option<(u32, u32)> = None;
    let mut second_ts: Option<Out> = None;
    let mut second_status: Option<Out> = None;
    let mut rest = Vec::with_capacity(args.len());
    let mut i = 0;
    while i < args.len() {
        let a = String::from_utf8_lossy(&args[i]).into_owned();
        let value = |i: usize| -> Result<String, String> {
            args.get(i + 1).map(|v| String::from_utf8_lossy(v).into_owned()).ok_or(format!("{} needs a value", a))
        };
        let value2 = |i: usize| -> Result<(String, String), String> {
            match (args.get(i + 1), args.get(i + 2)) {
                (Some(x), Some(y)) => Ok((String::from_utf8_lossy(x).into_owned(), String::from_utf8_lossy(y).into_owned())),
                _ => Err(format!("{} needs two values", a)),
            }
        };
        let port = |p: &str| -> Result<i32, String> { p.parse::<u16>().map(|v| v as i32).map_err(|_| format!("{}: bad port {}", a, p)) };
        let values = match a.as_str() {
            "--second" => {
                let (f, s) = value2(i)?;
                let (f, s) = match (f.parse::<u32>(), s.parse::<u32>()) {
                    (Ok(f), Ok(s)) if (144000..=2450000).contains(&f) && (33..=27500).contains(&s) => (f, s),
                    _ => return Err(format!("--second {} {}: expected FREQ (144000-2450000 kHz) SR (33-27500 kS/s)", f, s)),
                };
                second = Some((f, s));
                2
            }
            "--second-ts-fifo" | "--second-status-fifo" => {
                let o = Out::Fifo(value(i)?.into_bytes());
                if a == "--second-ts-fifo" { second_ts = Some(o) } else { second_status = Some(o) }
                1
            }
            "--second-ts-ip" | "--second-status-ip" => {
                let (ip, p) = value2(i)?;
                let o = Out::Udp(ip.into_bytes(), port(&p)?);
                if a == "--second-ts-ip" { second_ts = Some(o) } else { second_status = Some(o) }
                2
            }
            "--hf-filter-fix" => {
                ext.hf_filter_fix = true;
                0
            }
            "--lna-follow-input" => {
                ext.lna_follow_input = true;
                0
            }
            "--no-lna-poll" => {
                ext.lna_poll = false;
                0
            }
            "--fast-i2c" => {
                ext.fast_i2c = true;
                0
            }
            "--c-compat" => {
                c_compat = true;
                0
            }
            "--lnb-off-at-exit" => {
                ext.lnb_off_at_exit = true;
                0
            }
            "--strict-errors" => {
                ext.strict_errors = true;
                0
            }
            "--fast-retune" => {
                ext.fast_retune = true;
                0
            }
            "--ts-parse-fix" => {
                ext.ts_parse_fix = true;
                0
            }
            "--extra-status" => {
                ext.extra_status = true;
                0
            }
            "--warn-low-sr" => {
                ext.warn_low_sr = true;
                0
            }
            "--ts-timer-after-init" => {
                ext.ts_timer_after_init = true;
                0
            }
            "--reconnect" => {
                ext.reconnect = true;
                0
            }
            "--lna-mode" => {
                let v = value(i)?;
                ext.lna_mode = Some(match v.as_str() {
                    "track" => 0,
                    "request" => 1,
                    "min" => 2,
                    "max" => 3,
                    _ => return Err(format!("--lna-mode {}: expected track, request, min or max", v)),
                });
                1
            }
            "--lna-pref" => {
                let v = value(i)?;
                match v.parse::<i32>() {
                    Ok(d) if (-25..=-18).contains(&d) => ext.lna_pref = Some((d + 25) as u8),
                    _ => return Err(format!("--lna-pref {}: expected -25 to -18 (dBm)", v)),
                }
                1
            }
            "--lna-tlock" => {
                let v = value(i)?;
                ext.lna_tlock_fast = Some(match v.as_str() {
                    "fast" => true,
                    "slow" => false,
                    _ => return Err(format!("--lna-tlock {}: expected fast or slow", v)),
                });
                1
            }
            "--usb-backend" => {
                let v = value(i)?;
                ext.usb_backend = match v.as_str() {
                    "auto" => UsbBackendChoice::Auto,
                    "libusb" => UsbBackendChoice::Libusb,
                    "d2xx" => UsbBackendChoice::D2xx,
                    _ => return Err(format!("--usb-backend {}: expected auto, libusb or d2xx", v)),
                };
                1
            }
            "--ts-latency" => {
                let v = value(i)?;
                match v.parse::<u8>() {
                    Ok(ms) if ms >= 1 => ext.ts_latency_ms = Some(ms),
                    _ => return Err(format!("--ts-latency {}: expected 1 to 255 (ms)", v)),
                }
                1
            }
            _ => {
                rest.push(args[i].clone());
                i += 1;
                continue;
            }
        };
        i += 1 + values;
    }
    match second {
        Some((freq, sr)) => {
            ext.second = Some(Second {
                freq,
                sr,
                ts: second_ts.unwrap_or(Out::Fifo(b"longmynd_second_ts".to_vec())),
                status: second_status.unwrap_or(Out::Fifo(b"longmynd_second_status".to_vec())),
            })
        }
        None if second_ts.is_some() || second_status.is_some() => {
            return Err("--second-ts-*/--second-status-* need --second FREQ SR".into());
        }
        None => {}
    }
    if DEFAULTS_AFTER_HW_TEST && !c_compat {
        ext.set_decided_defaults();
    }
    Ok((rest, ext))
}

impl Default for Config {
    fn default() -> Self {
        Config {
            port_swap: false,
            halfscan_ratio: 0.0,
            freq_index: 0,
            sr_index: 0,
            freq_requested: [0; 4],
            sr_requested: [0; 4],
            beep_enabled: false,
            device_usb_bus: 0,
            device_usb_addr: 0,
            ts_use_ip: false,
            ts_fifo_path: Vec::new(),
            ts_ip_addr: Vec::new(),
            ts_ip_port: 0,
            status_use_ip: false,
            status_fifo_path: Vec::new(),
            status_ip_addr: Vec::new(),
            status_ip_port: 0,
            polarisation_supply: false,
            polarisation_horizontal: false,
            ts_timeout: 0,
            new: false,
            ext: Extensions::default(),
        }
    }
}

fn arg_byte(arg: &[u8], i: usize) -> u8 {
    arg.get(i).copied().unwrap_or(0)
}

/// Parse a comma list of up to four numbers as the C does. `check_each`:
/// the frequency list rejects any zero entry, the symbol rate list only
/// checks the first (so "1500,x" is a one-entry list).
fn parse_list(arg: &[u8], out: &mut [u32; 4], check_each: bool, msg: &str) -> u8 {
    let mut err = ERROR_NONE;
    let mut rest: &[u8] = arg;
    let mut i = 0;
    while i < 4 && err == ERROR_NONE {
        let comma = rest.iter().position(|&b| b == b',');
        let piece = match comma {
            Some(n) => &rest[..n],
            None => rest,
        };
        out[i] = strtol(piece) as u32;
        let bad = if check_each { out[i] == 0 } else { out[0] == 0 };
        if bad {
            err = ERROR_ARGS_INPUT;
            println!("{}", msg);
        }
        match comma {
            None => {
                for slot in out.iter_mut().skip(i + 1) {
                    *slot = 0;
                }
                i = 4;
                continue;
            }
            Some(n) => rest = &rest[n + 1..],
        }
        i += 1;
    }
    err
}

/// process_command_line(): `args` is the whole argv, program name first.
pub fn process_command_line(args: &[Vec<u8>], config: &mut Config) -> u8 {
    let (args, ext) = match split_extensions(args) {
        Ok(r) => r,
        Err(msg) => {
            println!("ERROR: {}", msg);
            println!("Please refer to the longmynd manual page via:");
            println!("    man -l longmynd.1");
            config.new = true;
            return ERROR_ARGS_INPUT;
        }
    };
    let args = &args[..];
    config.ext = ext;
    let mut err = ERROR_NONE;
    let argc = args.len() as isize;
    let mut main_usb_set = false;
    let mut ts_ip_set = false;
    let mut ts_fifo_set = false;
    let mut status_ip_set = false;
    let mut status_fifo_set = false;

    config.port_swap = false;
    config.halfscan_ratio = 1.5;
    config.beep_enabled = false;
    config.device_usb_addr = 0;
    config.device_usb_bus = 0;
    config.ts_use_ip = false;
    config.ts_fifo_path = b"longmynd_main_ts".to_vec();
    config.status_use_ip = false;
    config.status_fifo_path = b"longmynd_main_status".to_vec();
    config.polarisation_supply = false;
    // the C's polarisation_str is an uninitialised char[8]; only read after -p
    let mut polarisation_str: Vec<u8> = Vec::new();
    config.ts_timeout = 5 * 1000;

    let arg = |i: isize| -> &[u8] { args.get(i as usize).map(|a| a.as_slice()).unwrap_or(b"") };

    let mut param: isize = 1;
    while param < argc - 2 {
        if arg_byte(arg(param), 0) == b'-' {
            let opt = arg_byte(arg(param), 1);
            param += 1;
            match opt {
                b'u' => {
                    config.device_usb_bus = strtol(arg(param)) as u8;
                    param += 1;
                    config.device_usb_addr = strtol(arg(param)) as u8;
                    main_usb_set = true;
                }
                b'i' => {
                    config.ts_ip_addr = c_strncpy(arg(param), 16);
                    param += 1;
                    config.ts_ip_port = strtol(arg(param)) as u16 as i32;
                    config.ts_use_ip = true;
                    ts_ip_set = true;
                }
                b't' => {
                    config.ts_fifo_path = c_strncpy(arg(param), 128);
                    ts_fifo_set = true;
                }
                b'I' => {
                    config.status_ip_addr = c_strncpy(arg(param), 16);
                    param += 1;
                    config.status_ip_port = strtol(arg(param)) as u16 as i32;
                    config.status_use_ip = true;
                    status_ip_set = true;
                }
                b's' => {
                    config.status_fifo_path = c_strncpy(arg(param), 128);
                    status_fifo_set = true;
                }
                b'p' => {
                    polarisation_str = c_strncpy(arg(param), 8);
                    config.polarisation_supply = true;
                }
                b'w' => {
                    config.port_swap = true;
                    param -= 1;
                }
                b'S' => {
                    config.halfscan_ratio = strtof(arg(param));
                }
                b'b' => {
                    config.beep_enabled = true;
                    param -= 1;
                }
                b'r' => {
                    // long to int: the low 32 bits
                    config.ts_timeout = strtol(arg(param)) as i32;
                }
                _ => {}
            }
        }
        param += 1;
    }

    if argc - param < 2 {
        err = ERROR_ARGS_INPUT;
        println!("ERROR: Main Frequency and Main Symbol Rate not found.");
    }

    if err == ERROR_NONE && (config.halfscan_ratio < 0.0 || config.halfscan_ratio > 100.0) {
        err = ERROR_ARGS_INPUT;
        println!("ERROR: Scan width not valid.");
    }

    if err == ERROR_NONE {
        err = parse_list(arg(param), &mut config.freq_requested, true, "ERROR: Main Frequency not in a valid format.");
        param += 1;
    }

    if err == ERROR_NONE {
        err = parse_list(arg(param), &mut config.sr_requested, false, "ERROR: Main Symbol Rate not in a valid format.");
    }

    if err == ERROR_NONE && config.polarisation_supply {
        if polarisation_str.eq_ignore_ascii_case(b"h") {
            config.polarisation_horizontal = true;
        } else if polarisation_str.eq_ignore_ascii_case(b"v") {
            config.polarisation_horizontal = false;
        } else {
            config.polarisation_supply = false;
            err = ERROR_ARGS_INPUT;
            println!("ERROR: Polarisation voltage supply parameter not recognised");
        }
    }

    let f = config.freq_requested;
    if err == ERROR_NONE {
        if f[0] > 2450000 {
            err = ERROR_ARGS_INPUT;
            println!("ERROR: Freq ({}) must be <= 2450 MHz", f[0] as i32);
        } else if f[0] < 144000 {
            err = ERROR_ARGS_INPUT;
            println!("ERROR: Freq ({}) must be >= 144 MHz", f[0] as i32);
        } else if f[1] != 0 {
            if config.ts_timeout == -1 {
                err = ERROR_ARGS_INPUT;
                println!("ERROR: TS Timeout must be enabled when multiple frequencies are specified.");
            }
            for &fi in f.iter().skip(1).take_while(|&&x| x != 0) {
                if fi > 2450000 {
                    err = ERROR_ARGS_INPUT;
                    println!("ERROR: Freq ({}) must be <= 2450 MHz", fi as i32);
                } else if fi < 144000 {
                    err = ERROR_ARGS_INPUT;
                    println!("ERROR: Freq ({}) must be >= 144 MHz", fi as i32);
                }
            }
        }
    }
    let s = config.sr_requested;
    if err == ERROR_NONE {
        if s[0] > 27500 {
            err = ERROR_ARGS_INPUT;
            println!("ERROR: SR ({}) must be <= 27 Msymbols/s", s[0] as i32);
        } else if s[0] < 33 {
            err = ERROR_ARGS_INPUT;
            println!("ERROR: SR ({}) must be >= 33 Ksymbols/s", s[0] as i32);
        } else if s[1] != 0 {
            if config.ts_timeout == -1 {
                err = ERROR_ARGS_INPUT;
                println!("ERROR: TS Timeout must be enabled when multiple symbolrates are specified.");
            }
            for &si in s.iter().skip(1).take_while(|&&x| x != 0) {
                if si > 27500 {
                    err = ERROR_ARGS_INPUT;
                    println!("ERROR: SR ({}) must be <= 27 Msymbols/s", si as i32);
                } else if si < 33 {
                    err = ERROR_ARGS_INPUT;
                    println!("ERROR: SR ({}) must be >= 33 Ksymbols/s", si as i32);
                }
            }
        }
    }
    if err == ERROR_NONE {
        let same_ip = config.ts_ip_addr == config.status_ip_addr;
        if ts_ip_set && ts_fifo_set {
            err = ERROR_ARGS_INPUT;
            println!("ERROR: Cannot set TS FIFO and TS IP address");
        } else if status_ip_set && status_fifo_set {
            err = ERROR_ARGS_INPUT;
            println!("ERROR: Cannot set Status FIFO and Status IP address");
        } else if config.ts_use_ip && config.status_use_ip && config.ts_ip_port == config.status_ip_port && same_ip {
            err = ERROR_ARGS_INPUT;
            println!("ERROR: Cannot set Status IP & Port identical to TS IP & Port");
        } else if config.ts_timeout != -1 && config.ts_timeout <= 500 {
            err = ERROR_ARGS_INPUT;
            println!("ERROR: TS Timeout if enabled must be >500ms.");
        } else {
            let lossy = |v: &[u8]| String::from_utf8_lossy(v).into_owned();
            println!("      Status: Main Frequency={} KHz", f[0] as i32);
            for &fi in f.iter().skip(1).take_while(|&&x| x != 0) {
                println!("              Alternative Frequency={} KHz", fi as i32);
            }
            println!("              Main Symbol Rate={} KSymbols/s", s[0] as i32);
            for &si in s.iter().skip(1).take_while(|&&x| x != 0) {
                println!("              Alternative Symbol Rate={} KSymbols/s", si as i32);
            }
            if !main_usb_set {
                println!("              Using First Minitiouner detected on USB");
            } else {
                println!("              USB bus/device={},{}", config.device_usb_bus, config.device_usb_addr);
            }
            if !config.ts_use_ip {
                println!("              Main TS output to FIFO={}", lossy(&config.ts_fifo_path));
            } else {
                println!("              Main TS output to IP={}:{}", lossy(&config.ts_ip_addr), config.ts_ip_port);
            }
            if !config.status_use_ip {
                println!("              Main Status output to FIFO={}", lossy(&config.status_fifo_path));
            } else {
                println!("              Main Status output to IP={}:{}", lossy(&config.status_ip_addr), config.status_ip_port);
            }
            if config.port_swap {
                println!("              NIM inputs are swapped (Main now refers to BOTTOM F-Type");
            } else {
                println!("              Main refers to TOP F-Type");
            }
            if config.beep_enabled {
                println!("              MER Beep enabled");
            }
            if config.polarisation_supply {
                println!(
                    "              Polarisation Voltage Supply enabled: {}",
                    if config.polarisation_horizontal { "H, 18V" } else { "V, 13V" }
                );
            }
            if config.ts_timeout != -1 {
                println!("              TS Timeout Period ={} milliseconds", config.ts_timeout);
            } else {
                println!("              TS Timeout Disabled.");
            }
            if config.ext.any() {
                println!("              longmynd-rs extensions: {}", config.ext.describe());
            }
            if config.ext.warn_low_sr {
                for line in low_sr_warnings(&config.sr_requested) {
                    println!("{}", line);
                }
            }
        }
    }

    if err != ERROR_NONE {
        println!("Please refer to the longmynd manual page via:");
        println!("    man -l longmynd.1");
    }

    config.new = true;
    err
}

/// The lowest symbol rate a MiniTiouner has been seen to lock, kS/s.
pub const LOW_SR_KS: u32 = 66;

/// --warn-low-sr: a warning for each requested symbol rate below
/// [`LOW_SR_KS`]. Still accepted, as the C accepts it - scripts pass 33.
pub fn low_sr_warnings(sr_requested: &[u32; 4]) -> Vec<String> {
    sr_requested
        .iter()
        .take_while(|&&s| s != 0)
        .filter(|&&s| s < LOW_SR_KS)
        .map(|&s| {
            format!(
                "WARNING: SR ({}) is below {} KSymbols/s: the STV0910 is not known to lock that low (RB-TV at 25/33 KS/s needs software decoding)",
                s as i32, LOW_SR_KS
            )
        })
        .collect()
}

/// config_reinit(): flag a re-tune; with `increment_frsr`, step to the next
/// symbol rate for this frequency, and after the last one to the next
/// frequency. Returns the line the C prints after unlocking.
pub fn config_reinit(config: &mut Config, increment_frsr: bool) -> Option<String> {
    if increment_frsr {
        loop {
            config.sr_index = (config.sr_index + 1) & 0x3;
            if config.sr_index == 0 {
                loop {
                    config.freq_index = (config.freq_index + 1) & 0x3;
                    if config.freq_requested[config.freq_index as usize] != 0 {
                        break;
                    }
                }
            }
            if config.sr_requested[config.sr_index as usize] != 0 {
                break;
            }
        }
    }
    config.new = true;
    if increment_frsr {
        Some(format!(
            "Flow: Config cycle: Frequency [{}] = {} KHz, Symbol Rate [{}] = {} KSymbols/s",
            config.freq_index,
            config.freq_requested[config.freq_index as usize] as i32,
            config.sr_index,
            config.sr_requested[config.sr_index as usize] as i32
        ))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(line: &str) -> (u8, Config) {
        let args: Vec<Vec<u8>> = line.split(' ').map(|s| s.as_bytes().to_vec()).collect();
        let mut c = Config::default();
        let e = process_command_line(&args, &mut c);
        (e, c)
    }

    #[test]
    fn plain_frequency_and_rate() {
        let (e, c) = parse("longmynd 741500 1500");
        assert_eq!(e, ERROR_NONE);
        assert_eq!(c.freq_requested, [741500, 0, 0, 0]);
        assert_eq!(c.sr_requested, [1500, 0, 0, 0]);
        assert_eq!(c.ts_fifo_path, b"longmynd_main_ts");
        assert_eq!(c.ts_timeout, 5000);
        assert_eq!(c.halfscan_ratio, 1.5);
        assert!(c.new);
    }

    #[test]
    fn options_and_lists() {
        let (e, c) = parse("longmynd -u 1 4 -i 198.51.100.1 87 -w -S 0.6 -r 6000 -p h 145000,146000 35,66,125");
        assert_eq!(e, ERROR_NONE);
        assert_eq!((c.device_usb_bus, c.device_usb_addr), (1, 4));
        assert!(c.ts_use_ip && c.port_swap && c.polarisation_supply && c.polarisation_horizontal);
        assert_eq!(c.ts_ip_addr, b"198.51.100.1");
        assert_eq!(c.ts_ip_port, 87);
        assert_eq!(c.halfscan_ratio, 0.6f32);
        assert_eq!(c.freq_requested, [145000, 146000, 0, 0]);
        assert_eq!(c.sr_requested, [35, 66, 125, 0]);
    }

    #[test]
    fn unknown_option_swallows_its_neighbour() {
        // "-x" is not an option, and like any option it consumes the next
        // argument, so 9999 is never seen as a frequency.
        let (e, c) = parse("longmynd -x 9999 437000 333");
        assert_eq!(e, ERROR_NONE);
        assert_eq!(c.freq_requested[0], 437000);
    }

    #[test]
    fn symbol_rate_list_only_checks_the_first() {
        let (e, c) = parse("longmynd 437000 333,abc,500");
        assert_eq!(e, ERROR_NONE);
        assert_eq!(c.sr_requested, [333, 0, 500, 0]);
    }

    #[test]
    fn errors() {
        assert_eq!(parse("longmynd 437000").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd 100000 333").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd 437000 20").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd -r 500 437000 333").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd -r -1 437000,438000 333").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd -p x 437000 333").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd -i 1.2.3.4 5 -t x 437000 333").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd -i 1.2.3.4 5 -I 1.2.3.4 5 437000 333").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd -S 101 437000 333").0, ERROR_ARGS_INPUT);
    }

    #[test]
    fn extensions_are_taken_out_and_others_left_to_the_c_parser() {
        let (e, c) = parse("longmynd --fast-i2c --lna-mode max -w --lna-pref -23 437000 333");
        assert_eq!(e, ERROR_NONE);
        assert!(c.ext.fast_i2c && c.port_swap);
        assert_eq!((c.ext.lna_mode, c.ext.lna_pref), (Some(3), Some(2)));
        // not ours: the C's parser swallows the argument after it
        let (e, c) = parse("longmynd --bogus 9999 437000 333");
        assert_eq!(e, ERROR_NONE);
        assert_eq!(c.ext, Extensions::default());
        assert_eq!(parse("longmynd --lna-mode loud 437000 333").0, ERROR_ARGS_INPUT);
        assert_eq!(parse("longmynd --ts-latency 0 437000 333").0, ERROR_ARGS_INPUT);
    }

    #[test]
    fn low_rates_are_warned_about_only_with_the_option() {
        assert_eq!(low_sr_warnings(&[33, 125, 35, 0]).len(), 2);
        assert!(low_sr_warnings(&[33, 0, 0, 0])[0].starts_with("WARNING: SR (33) is below 66"));
        assert!(low_sr_warnings(&[66, 1500, 0, 0]).is_empty());
        let (e, c) = parse("longmynd --warn-low-sr 437000 33");
        assert_eq!(e, ERROR_NONE, "still accepted");
        assert!(c.ext.warn_low_sr);
    }

    #[test]
    fn defaults_are_the_c_until_the_hardware_test_and_c_compat_is_the_c_after() {
        // today
        assert_eq!(Extensions::default(), Extensions::defaults(DEFAULTS_AFTER_HW_TEST));
        assert_eq!(Extensions::defaults(false), Extensions::c_behaviour());
        assert!(!Extensions::defaults(false).any());
        // the prepared flip: exactly the four decided options
        let after = Extensions::defaults(true);
        let mut want = Extensions::c_behaviour();
        want.fast_i2c = true;
        want.hf_filter_fix = true;
        want.lna_follow_input = true;
        want.lnb_off_at_exit = true;
        assert_eq!(after, want);
        assert_eq!(after.describe(), "--hf-filter-fix --lna-follow-input --fast-i2c --lnb-off-at-exit");
        // --c-compat is the C whatever the defaults, and taken out of the
        // command line like the other options
        let (rest, ext) = split_extensions(&[b"longmynd".to_vec(), b"--c-compat".to_vec(), b"437000".to_vec(), b"333".to_vec()]).unwrap();
        assert_eq!(ext, Extensions::c_behaviour());
        assert_eq!(rest.len(), 3);
        let (_, ext) = split_extensions(&[b"--fast-i2c".to_vec(), b"--c-compat".to_vec()]).unwrap();
        assert!(ext.fast_i2c && !ext.hf_filter_fix && !ext.lnb_off_at_exit, "an option given with --c-compat stays on");
    }

    #[test]
    fn reinit_cycles_rates_then_frequencies() {
        let (_, mut c) = parse("longmynd 145000,146000 35,66,125");
        let mut seen = vec![];
        for _ in 0..7 {
            config_reinit(&mut c, true);
            seen.push((c.freq_requested[c.freq_index as usize], c.sr_requested[c.sr_index as usize]));
        }
        assert_eq!(
            seen,
            vec![(145000, 66), (145000, 125), (146000, 35), (146000, 66), (146000, 125), (145000, 35), (145000, 66)]
        );
    }
}
