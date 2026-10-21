//! The scenarios the C and the Rust are both put through.
//!
//! Each is a command line, the USB devices present, and how the NIM
//! behaves over (virtual) time: when the demodulator hunts, finds a header,
//! locks to DVB-S or DVB-S2 and what it then reads; what the LNA's AGC
//! says; whether the tuner's PLL locks; what TS arrives. "{dir}" in an
//! argument is the run's output directory. Times are milliseconds after
//! the first NIM init has finished (sim::STARTUP_MS).

use crate::sim::{DemodPhase, DevSpec, LnaPhase, FTDI_STATUS};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// The real main(), for runs that end before any thread would start
    /// (argument errors, no device, a missing status FIFO).
    Main,
    /// main() up to the threads, then the receiver thread driven on the
    /// virtual clock with main()'s loop body run at each of its sleeps.
    Receiver,
    /// main() up to the threads, then the TS thread fed canned TS, with
    /// the parse thread's work done between reads.
    Ts,
}

#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: &'static str,
    pub about: &'static str,
    pub mode: Mode,
    pub args: Vec<String>,
    pub devices: Vec<DevSpec>,
    pub duration_ms: u64,
    pub lna_present: bool,
    /// how many VCO calibrations (tuner set_freq calls) fail to lock first
    pub tuner_lock_failures: u32,
    /// Fault: from this VCO calibration on (counted from 1), the tuner
    /// never locks again - a tuner that dies part way through a run.
    pub tuner_dead_from_cal: Option<u32>,
    /// Fault: these VCO calibrations (counted from 1) fail to lock.
    pub tuner_fail_cals: Vec<u32>,
    /// Fault: the tuner is unplugged from the first (ms) to the second:
    /// every handle open at the time fails from then on with
    /// LIBUSB_ERROR_NO_DEVICE, and the tuner cannot be opened until it is
    /// back - then as a freshly powered device.
    pub unplugged: Option<(u64, u64)>,
    pub demod: Vec<(u64, DemodPhase)>,
    /// the bottom demodulator's phases (--second); empty: it hunts
    pub demod2: Vec<(u64, DemodPhase)>,
    pub lna: Vec<(u64, LnaPhase)>,
    /// intervals (ms) during which TS is arriving, for the no-TS timer
    pub ts_flow: Vec<(u64, u64)>,
    /// what successive reads of the TS endpoint return (FTDI framed)
    pub ts_chunks: Vec<Vec<u8>>,
    /// the same for a PicoTuner's second TS endpoint (0x84, --second)
    pub ts2_chunks: Vec<Vec<u8>>,
    /// UDP ports on 127.0.0.1 the run listens on and records
    pub udp_ports: Vec<u16>,
    /// files created (empty) in the run directory first
    pub files: Vec<&'static str>,
    /// What to hide from both sides' stdout, status and trace before they
    /// are compared, for the places the C's behaviour is undefined (it
    /// reads past the end of an array) and the Rust deliberately differs.
    /// Empty for every scenario on a defined path. See [`Mask`].
    pub masks: Vec<Mask>,
    /// Faults: these reads of the I2C endpoint (0x81, counted from 1)
    /// return only the FTDI's two status bytes, as a stalled or
    /// disconnecting device does.
    pub short_reads: Vec<u32>,
    /// Fault: the demodulator does not hold what is written to 0xF536 (the
    /// register nim_init() writes and reads back to prove the NIM answers).
    pub demod_readback_corrupt: bool,
}

/// A value hidden from the comparison (Scenario::masks).
///
/// What the C reads past the end of a table depends on what the linker put
/// there, which moves whenever anything else linked into the harness does;
/// so a mask names where the value is, never the value itself.
#[derive(Clone, Copy, Debug)]
pub enum Mask {
    /// `prefix` followed by a run of hex digits / 'x' (a number as the C
    /// prints it): the number is replaced by `<past the table>`.
    Printed(&'static str),
    /// The value written to this tuner register (I2C 0xC0): in the decoded
    /// I2C log ("i2c c0 RR VV") and in the raw MPSSE bytes (the byte sent
    /// after the tuner address and this register number).
    TunerWrite(u8),
}

impl Mask {
    fn apply(&self, text: &str) -> String {
        match *self {
            Mask::Printed(prefix) => {
                let mut out = String::with_capacity(text.len());
                let mut rest = text;
                while let Some(n) = rest.find(prefix) {
                    out.push_str(&rest[..n + prefix.len()]);
                    rest = &rest[n + prefix.len()..];
                    let len = rest.find(|c: char| !(c.is_ascii_hexdigit() || c == 'x')).unwrap_or(rest.len());
                    if len > 0 {
                        out.push_str("<past the table>");
                        rest = &rest[len..];
                    }
                }
                out.push_str(rest);
                out
            }
            Mask::TunerWrite(reg) => {
                // raw: the byte clocked out ("11 00 00 VV") in the first
                // write after the tuner's address and then `reg`
                let decoded = format!(" i2c c0 {:02x} ", reg);
                let reg_byte = format!("11 00 00 {:02x} ", reg);
                // 0: idle, 1: tuner addressed, 2: the register is `reg`
                let mut state = 0;
                let mut out = String::with_capacity(text.len());
                for line in text.split_inclusive('\n') {
                    let mut line = line.to_string();
                    if let Some(n) = line.find(&decoded) {
                        line.replace_range(n + decoded.len().., "<masked>\n");
                    }
                    let is_out = line.contains(" bulk 02 out ") || line.contains(" FT_Write ");
                    if is_out {
                        if let Some(n) = line.find("11 00 00 ") {
                            let byte = &line[n + 9..n + 11].to_string();
                            match state {
                                2 => {
                                    line.replace_range(n + 9..n + 11, "<>");
                                    state = 0;
                                }
                                1 => state = if line.contains(&reg_byte) { 2 } else { 0 },
                                _ => state = if byte == "c0" { 1 } else { 0 },
                            }
                        }
                    }
                    out.push_str(&line);
                }
                out
            }
        }
    }
}

impl Scenario {
    /// `text` with this scenario's masks applied (unchanged when none).
    pub fn mask(&self, text: &str) -> String {
        self.masks.iter().fold(text.to_string(), |t, m| m.apply(&t))
    }

    fn new(name: &'static str, about: &'static str, mode: Mode, args: &str) -> Self {
        Scenario {
            name,
            about,
            mode,
            args: args.split_whitespace().map(String::from).collect(),
            devices: vec![DevSpec::other(1, 1, 0x1d6b, 0x0002), DevSpec::ftdi(1, 5)],
            duration_ms: 2000,
            lna_present: true,
            tuner_lock_failures: 0,
            tuner_dead_from_cal: None,
            tuner_fail_cals: vec![],
            unplugged: None,
            demod: vec![],
            demod2: vec![],
            lna: vec![],
            ts_flow: vec![],
            ts_chunks: vec![],
            ts2_chunks: vec![],
            udp_ports: vec![],
            files: vec!["status", "ts"],
            masks: vec![],
            short_reads: vec![],
            demod_readback_corrupt: false,
        }
    }

    pub fn demod2_at(&self, t: u64) -> DemodPhase {
        self.demod2.iter().rev().find(|(s, _)| *s <= t).map(|(_, p)| p.clone()).unwrap_or_else(DemodPhase::hunting)
    }

    pub fn demod_at(&self, t: u64) -> DemodPhase {
        self.demod.iter().rev().find(|(s, _)| *s <= t).map(|(_, p)| p.clone()).unwrap_or_else(DemodPhase::hunting)
    }

    pub fn lna_at(&self, t: u64) -> LnaPhase {
        self.lna
            .iter()
            .rev()
            .find(|(s, _)| *s <= t)
            .map(|(_, p)| *p)
            .unwrap_or(LnaPhase { swlnagain: 2, vgo: 0x0c })
    }

    /// The command line with {dir} filled in.
    pub fn argv(&self, dir: &str) -> Vec<String> {
        self.args.iter().map(|a| a.replace("{dir}", dir)).collect()
    }
}

fn found_header() -> DemodPhase {
    DemodPhase { header_mode: 1, carrier_hz: 2_150, sr: 1_498_210, power: 0x31, agc2: 0x1240, ..DemodPhase::hunting() }
}

/// DVB-S2 QPSK 2/3, pilots on, normal frames, MER 8.7dB.
fn locked_s2(sr: u32, mer: i32) -> DemodPhase {
    DemodPhase {
        header_mode: 2,
        carrier_hz: 1_830,
        sr,
        mer: Some(mer),
        modcod_reg: (5 << 2) | 0x01,
        curpun: 0x12,
        verror: 0,
        ber_count: 0x0012_3456_789a,
        ber_errors: 0x000123,
        errorflag: true,
        bch_errors: 0,
        ldpc_errors: 0x0102,
        agc1: 0x2a10,
        agc2: 0x1a00,
        power: 0x44,
    }
}

/// DVB-S, Viterbi 2/3.
fn locked_s(sr: u32, curpun: u8) -> DemodPhase {
    DemodPhase {
        header_mode: 3,
        carrier_hz: -4_475,
        sr,
        mer: Some(64),
        modcod_reg: 0x2a,
        curpun,
        verror: 0x14,
        ber_count: 0,
        ber_errors: 0,
        errorflag: true,
        bch_errors: 3,
        ldpc_errors: 0,
        agc1: 0x2c00,
        agc2: 0x1800,
        power: 0x40,
    }
}

// ---- TS -------------------------------------------------------------------------

fn crc32_mpeg2(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xffff_ffff;
    for &b in data {
        crc ^= (b as u32) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 { (crc << 1) ^ 0x04c1_1db7 } else { crc << 1 };
        }
    }
    crc
}

/// A packet carrying one PSI section (pointer field 0), CRC appended.
fn psi_packet(pid: u16, cc: u8, section: &[u8]) -> Vec<u8> {
    let mut s = section.to_vec();
    let len = s.len() - 3 + 4;
    s[1] = (s[1] & 0xf0) | ((len >> 8) as u8 & 0x0f);
    s[2] = len as u8;
    let crc = crc32_mpeg2(&s);
    s.extend_from_slice(&crc.to_be_bytes());
    let mut p = vec![0x47, 0x40 | (pid >> 8) as u8 & 0x1f, pid as u8, 0x10 | (cc & 0x0f), 0x00];
    p.extend_from_slice(&s);
    p.resize(188, 0xff);
    p
}

fn pat(cc: u8) -> Vec<u8> {
    psi_packet(0, cc, &[0x00, 0xb0, 0, 0x00, 0x01, 0xc1, 0x00, 0x00, 0x00, 0x01, 0xe1, 0x00])
}

fn pmt(cc: u8, streams: &[(u8, u16)]) -> Vec<u8> {
    let mut s = vec![0x02, 0xb0, 0, 0x00, 0x01, 0xc1, 0x00, 0x00, 0xe2, 0x56, 0xf0, 0x00];
    for &(t, pid) in streams {
        s.extend_from_slice(&[t, 0xe0 | (pid >> 8) as u8, pid as u8, 0xf0, 0x00]);
    }
    psi_packet(0x1000, cc, &s)
}

fn sdt(cc: u8, provider: &[u8], name: &[u8]) -> Vec<u8> {
    let mut d = vec![0x48, 0, 0x01, provider.len() as u8];
    d.extend_from_slice(provider);
    d.push(name.len() as u8);
    d.extend_from_slice(name);
    d[1] = (d.len() - 2) as u8;
    let mut s = vec![0x42, 0xf0, 0, 0x00, 0x01, 0xc1, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x01, 0xfc, 0x80, d.len() as u8];
    s.extend_from_slice(&d);
    psi_packet(0x11, cc, &s)
}

fn null_packet() -> Vec<u8> {
    let mut p = vec![0x47, 0x1f, 0xff, 0x10];
    p.resize(188, 0xff);
    p
}

/// A video packet whose payload is deterministic noise (it may contain
/// 0x47 bytes, which the C's parser trips over - that is part of the test).
fn video_packet(cc: u8, seed: u32) -> Vec<u8> {
    let mut p = vec![0x47, 0x01, 0x00, 0x10 | (cc & 0x0f)];
    let mut x = seed.wrapping_mul(2654435761).wrapping_add(1);
    while p.len() < 188 {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        p.push(x as u8);
    }
    p
}

/// TS bytes as successive FTDI reads: every 512-byte USB packet starts
/// with the two status bytes, and a read returns up to `per_read` packets.
fn ftdi_reads(ts: &[u8], per_read: usize) -> Vec<Vec<u8>> {
    let mut packets: Vec<Vec<u8>> = ts
        .chunks(510)
        .map(|c| {
            let mut p = FTDI_STATUS.to_vec();
            p.extend_from_slice(c);
            p
        })
        .collect();
    let mut reads = Vec::new();
    while !packets.is_empty() {
        let n = per_read.min(packets.len());
        reads.push(packets.drain(..n).flatten().collect());
    }
    reads
}

fn ts_stream(services: &[(&[u8], &[u8])], streams: &[(u8, u16)], nulls_every: usize, n_video: usize) -> Vec<u8> {
    let mut ts = Vec::new();
    let mut cc = 0u8;
    for (i, (provider, name)) in services.iter().enumerate() {
        ts.extend(pat(cc));
        ts.extend(pmt(cc, streams));
        ts.extend(sdt(cc, provider, name));
        for v in 0..n_video {
            ts.extend(video_packet(cc.wrapping_add(v as u8), (i * 1000 + v) as u32));
            if nulls_every > 0 && v % nulls_every == 0 {
                ts.extend(null_packet());
            }
        }
        cc = cc.wrapping_add(1);
    }
    ts
}

/// ts_fifo_and_names's TS: a stale stream to flush, then six services with
/// names in several character sets, then short reads ending mid-packet.
fn names_chunks() -> Vec<Vec<u8>> {
    let stale = ts_stream(&[(b"OLD", b"STALE")], &[(0x1b, 0x100)], 0, 30);
    let mut chunks = ftdi_reads(&stale, 20);
    chunks.push(FTDI_STATUS.to_vec()); // the flush ends at an empty read
    let names: [(&[u8], &[u8]); 6] = [
        (b"BATC", b"GB3HV"),
        (b"QO-100", &[0xc2, b'e', b't', b'e', b'!']),         // ISO 6937 acute e
        (&[0x05, b'D', b'A', b'T', b'V'], &[0x05, 0xdd, b'z']), // ISO-8859-9 selector
        (&[0x15, 0xc3, 0xa9], &[0x10, 0x00, 0x02, 0xb1, b'X']), // UTF-8; A.4 ISO-8859-2
        (&[0x11, 0, 0, 0, b'U'], &[0x12, 0x41]),                // UCS-4; EUC-KR (passed raw)
        (b"", b"G4ABC portable"),
    ];
    let live = ts_stream(&names, &[(0x1b, 0x100), (0x0f, 0x101), (0x06, 0x102)], 3, 40);
    chunks.extend(ftdi_reads(&live, 20));
    chunks.extend(ftdi_reads(&live[..188 * 7 + 100], 3)); // short reads, a partial packet
    chunks
}

// ---- the scenarios ------------------------------------------------------------

pub fn all() -> Vec<Scenario> {
    let mut v = Vec::new();

    // -- receiver --------------------------------------------------------------
    let mut s = Scenario::new(
        "startup_437_333",
        "Start up at 437MHz 333kS/s and hunt: the whole NIM init, then reports while nothing is found.",
        Mode::Receiver,
        "longmynd -s {dir}/status -t {dir}/ts 437000 333",
    );
    s.duration_ms = 1500;
    v.push(s);

    let mut s = Scenario::new(
        "lock_s2_1255_1500",
        "1255MHz 1.5MS/s: hunt, find a header, lock to DVB-S2 with TS flowing, MER moving.",
        Mode::Receiver,
        "longmynd -s {dir}/status 1255000 1500",
    );
    s.duration_ms = 3000;
    s.demod = vec![
        (500, found_header()),
        (700, locked_s2(1_499_870, 87)),
        (1500, locked_s2(1_499_902, 91)),
        (2200, DemodPhase { mer: None, ..locked_s2(1_499_902, 0) }),
        (2400, locked_s2(1_499_902, -12)),
    ];
    s.ts_flow = vec![(800, 3000)];
    v.push(s);

    let mut s = Scenario::new(
        "lock_s_then_lose",
        "437MHz 333kS/s DVB-S lock, then the signal goes: back to hunting, the no-TS timeout fires and re-inits.",
        Mode::Receiver,
        "longmynd -r 1000 -s {dir}/status 437000 333",
    );
    s.duration_ms = 4500;
    s.demod = vec![
        (300, found_header()),
        (450, locked_s(333_120, 0x12)),
        (1200, locked_s(333_090, 0x15)),
        (2000, DemodPhase::hunting()),
    ];
    s.ts_flow = vec![(500, 2000)];
    v.push(s);

    let mut s = Scenario::new(
        "scan_rates_and_freqs",
        "Nothing found: -r 600 cycles four symbol rates on two frequencies, re-initialising the NIM each time.",
        Mode::Receiver,
        "longmynd -r 600 -s {dir}/status 437000,1255000 333,250,125,66",
    );
    s.duration_ms = 9000;
    v.push(s);

    let mut s = Scenario::new(
        "lnb_h_swapped_halfscan",
        "LNB supply on at 18V (-p h), bottom input (-w), half-scan 0.6 (-S), QO-100 IF 741.5MHz 1.5MS/s.",
        Mode::Receiver,
        "longmynd -w -p h -S 0.6 -s {dir}/status 741500 1500",
    );
    s.duration_ms = 1200;
    v.push(s);

    let mut s = Scenario::new(
        "lnb_v_timeout_off",
        "LNB supply on at 13V (-p v), the no-TS timeout disabled (-r -1), 10491.5 LNB IF at 2MS/s.",
        Mode::Receiver,
        "longmynd -p V -r -1 -s {dir}/status 741525 2000",
    );
    s.duration_ms = 1200;
    v.push(s);

    let mut s = Scenario::new(
        "picotuner_serial_ts",
        "A PicoTuner (2E8A:BA2C, bcdDevice 0x0102): opened after the FTDI VID/PID is not found, STV0910 set to serial TS.",
        Mode::Receiver,
        "longmynd -s {dir}/status 1249000 333",
    );
    s.devices = vec![DevSpec::other(1, 1, 0x1d6b, 0x0002), DevSpec::pico(1, 9, 0x0102)];
    s.duration_ms = 1200;
    s.demod = vec![(400, locked_s2(333_301, 55))];
    v.push(s);

    let mut s = Scenario::new(
        "picotuner_parallel_ts",
        "A PicoTuner reporting bcdDevice 0x0101 (parallel TS, like the FTDI): no serial-TS writes.",
        Mode::Receiver,
        "longmynd -s {dir}/status 1249000 333",
    );
    s.devices = vec![DevSpec::pico(2, 3, 0x0101)];
    s.duration_ms = 600;
    v.push(s);

    let mut s = Scenario::new(
        "usb_bus_device",
        "-u 3 7 picks the tuner by bus/device from a list of three.",
        Mode::Receiver,
        "longmynd -u 3 7 -s {dir}/status 437000 333",
    );
    s.devices = vec![
        DevSpec::other(1, 1, 0x1d6b, 0x0002),
        DevSpec::ftdi(2, 4),
        DevSpec { bus: 3, addr: 7, ..DevSpec::ftdi(3, 7) },
    ];
    s.duration_ms = 600;
    v.push(s);

    let mut s = Scenario::new(
        "tuner_lock_retries",
        "The tuner PLL fails to lock three times: power down, wait, retry, then lock.",
        Mode::Receiver,
        "longmynd -s {dir}/status 2200000 4000",
    );
    s.tuner_lock_failures = 3;
    // each retry is a whole NIM init (about 2.2s here), so the 5s no-TS
    // timer fires during them - which the C does too
    s.duration_ms = 9000;
    v.push(s);

    let mut s = Scenario::new(
        "no_lna_old_nim",
        "An older NIM without LNAs: every LNA access is NAKed and retried 1000 times before it gives up.",
        Mode::Receiver,
        "longmynd -s {dir}/status 437000 333",
    );
    s.lna_present = false;
    s.duration_ms = 3000;
    v.push(s);

    // The C's two table lookups that run off the end of their tables:
    // below about 284.7MHz the HF filter cut-off search reads past
    // stv6120_cfhf[31], and above 2380MHz the charge-pump search past
    // stv6120_icp_lookup[6]. What the C then writes depends on what the
    // compiler put next in memory; the Rust uses the last entry. Everything
    // else in these runs must still match.
    let mut s = Scenario::new(
        "low_band_146",
        "146.5MHz 125kS/s: the tuner's lowest divider; the C's cut-off lookup reads past its table here.",
        Mode::Receiver,
        "longmynd -s {dir}/status 146500 125",
    );
    s.duration_ms = 600;
    s.masks = vec![Mask::Printed(", cfhf="), Mask::TunerWrite(0x07)];
    v.push(s);

    let mut s = Scenario::new(
        "top_band_2400",
        "2400MHz 1MS/s: VCO above the charge-pump table, which the C reads past.",
        Mode::Receiver,
        "longmynd -s {dir}/status 2400000 1000",
    );
    s.duration_ms = 600;
    // and above 2265MHz the cut-off search finds nothing, so the C prints
    // stv6120_cfhf[255] - past the table again (it writes the same CTRL8
    // as the Rust, 0xff)
    s.masks = vec![Mask::Printed(", cfhf="), Mask::Printed("icp="), Mask::TunerWrite(0x05)];
    v.push(s);

    let mut s = Scenario::new(
        "lna_agc_stepping",
        "Locked DVB-S2 while the LNA's AGC steps between gain curves: what $2 reports.",
        Mode::Receiver,
        "longmynd -s {dir}/status 741500 1500",
    );
    s.duration_ms = 2500;
    s.demod = vec![(300, locked_s2(1_500_010, 120))];
    s.lna = vec![
        (0, LnaPhase { swlnagain: 3, vgo: 0x1f }),
        (800, LnaPhase { swlnagain: 2, vgo: 0x03 }),
        (1500, LnaPhase { swlnagain: 3, vgo: 0x1e }),
    ];
    s.ts_flow = vec![(400, 2500)];
    v.push(s);

    let mut s = Scenario::new(
        "status_udp",
        "Status over UDP (-I) and TS over UDP (-i): the datagrams, line for line.",
        Mode::Receiver,
        "longmynd -I 127.0.0.1 47311 -i 127.0.0.1 47312 1255000 1500",
    );
    s.duration_ms = 1500;
    s.demod = vec![(600, locked_s2(1_499_999, 77))];
    s.ts_flow = vec![(700, 1500)];
    s.udp_ports = vec![47311];
    s.files = vec![];
    v.push(s);

    // -- TS ---------------------------------------------------------------------
    let mut s = Scenario::new(
        "ts_fifo_and_names",
        "TS to the FIFO (FTDI status bytes skipped every 510), stale TS flushed first, service names in several character sets.",
        Mode::Ts,
        "longmynd -s {dir}/status -t {dir}/ts 437000 333",
    );
    s.ts_chunks = names_chunks();
    v.push(s);

    let mut s = Scenario::new(
        "ts_udp",
        "TS over UDP (-i): 510-byte datagrams, one per FTDI packet.",
        Mode::Ts,
        "longmynd -i 127.0.0.1 47321 -s {dir}/status 437000 333",
    );
    s.ts_chunks = std::iter::once(FTDI_STATUS.to_vec())
        .chain(ftdi_reads(&ts_stream(&[(b"BATC", b"GB3HV")], &[(0x1b, 0x100)], 2, 60), 20))
        .collect();
    s.udp_ports = vec![47321];
    v.push(s);

    // -- main() to an early end ---------------------------------------------------
    for (name, about, args) in [
        ("args_missing", "Only one positional argument.", "longmynd 437000"),
        ("args_freq_low", "A frequency below 144MHz.", "longmynd 100000 333"),
        ("args_lists_need_timeout", "Frequency and rate lists with the timeout disabled.", "longmynd -r -1 437000,438000 333"),
        ("args_bad_polarisation", "-p with something other than h or v.", "longmynd -p x 437000 333"),
        ("args_fifo_and_ip", "Both a TS FIFO and a TS IP address.", "longmynd -t a -i 127.0.0.1 1234 437000 333"),
        ("args_short_timeout", "-r of 500ms or less.", "longmynd -r 400 437000 333"),
        ("args_unknown_option", "An unknown option swallows its neighbour, then a bad scan width.", "longmynd -x 5 -S 200 437000 333"),
    ] {
        let mut s = Scenario::new(name, about, Mode::Main, args);
        s.files = vec![];
        v.push(s);
    }

    let mut s = Scenario::new(
        "status_fifo_missing",
        "The status FIFO does not exist: an error before the tuner is touched.",
        Mode::Main,
        "longmynd -s {dir}/nonexistent 437000 333",
    );
    s.files = vec![];
    v.push(s);

    let mut s = Scenario::new(
        "no_tuner_plugged_in",
        "Neither VID/PID is present.",
        Mode::Main,
        "longmynd -s {dir}/status 437000 333",
    );
    s.devices = vec![DevSpec::other(1, 1, 0x1d6b, 0x0002)];
    v.push(s);

    let s = Scenario::new(
        "usb_wrong_device",
        "-u names a device that is not a MiniTiouner.",
        Mode::Main,
        "longmynd -u 1 1 -s {dir}/status 437000 333",
    );
    v.push(s);

    let s = Scenario::new(
        "usb_no_such_device",
        "-u names a bus/device that does not exist.",
        Mode::Main,
        "longmynd -u 4 2 -s {dir}/status 437000 333",
    );
    v.push(s);

    let mut s = Scenario::new(
        "usb_claim_busy",
        "The interface is busy (another longmynd has it).",
        Mode::Main,
        "longmynd -s {dir}/status 437000 333",
    );
    s.devices = vec![DevSpec { claim_ok: false, ..DevSpec::ftdi(1, 5) }];
    v.push(s);

    v
}

/// Scenarios for behaviour longmynd-rs deliberately does not share with
/// the C: faults where the C misbehaves and the Rust has been fixed
/// (tests/fixes.rs runs both and shows the difference).
pub fn faults() -> Vec<Scenario> {
    let mut v = Vec::new();

    let mut s = Scenario::new(
        "fault_short_usb_read",
        "Two reads of the I2C endpoint come back with only the FTDI status bytes (a stalled or unplugging device).",
        Mode::Receiver,
        "longmynd -s {dir}/status 437000 333",
    );
    // during the STV0910 table load: an ACK read, both retries short
    s.short_reads = vec![2001, 2002];
    s.duration_ms = 300;
    v.push(s);

    let mut s = Scenario::new(
        "fault_demod_readback",
        "The demodulator does not hold a written value (nim_init's alternating-bits check fails).",
        Mode::Receiver,
        "longmynd -s {dir}/status 437000 333",
    );
    s.demod_readback_corrupt = true;
    s.duration_ms = 300;
    v.push(s);

    v
}

/// Runs with longmynd-rs's opt-in options (config::Extensions), which
/// change what goes to the NIM on purpose; tests/extensions.rs checks each
/// does what it says and nothing else.
pub fn extensions() -> Vec<Scenario> {
    let mut v = Vec::new();
    for (name, about, args, duration) in [
        ("ext_fast_i2c", "--fast-i2c: the STV0910 table in batches.", "longmynd --fast-i2c -s {dir}/status 437000 333", 300),
        ("ext_hf_filter_fix", "--hf-filter-fix at 2400MHz.", "longmynd --hf-filter-fix -s {dir}/status 2400000 1000", 300),
        (
            "ext_lna_options",
            "-w with --lna-follow-input, --lna-mode max, --lna-pref -23, --lna-tlock fast.",
            "longmynd -w --lna-follow-input --lna-mode max --lna-pref -23 --lna-tlock fast -s {dir}/status 437000 333",
            300,
        ),
        ("ext_no_lna_poll", "--no-lna-poll.", "longmynd --no-lna-poll -s {dir}/status 437000 333", 300),
        ("ext_ts_latency", "--ts-latency 2.", "longmynd --ts-latency 2 -s {dir}/status 437000 333", 100),
        (
            "ext_lnb_off_at_exit",
            "--lnb-off-at-exit with the LNB supply on at 18V: off again when stopped.",
            "longmynd --lnb-off-at-exit -w -p h -S 0.6 -s {dir}/status 741500 1500",
            300,
        ),
        ("ext_lnb_off_no_p", "--lnb-off-at-exit without -p.", "longmynd --lnb-off-at-exit -s {dir}/status 437000 333", 300),
    ] {
        let mut s = Scenario::new(name, about, Mode::Receiver, args);
        s.duration_ms = duration;
        v.push(s);
    }

    let mut s = Scenario::new(
        "ext_fast_retune",
        "--fast-retune: the scan_rates_and_freqs scan, re-tuning in place after the first init.",
        Mode::Receiver,
        "longmynd --fast-retune -r 600 -s {dir}/status 437000,1255000 333,250,125,66",
    );
    s.duration_ms = 9000;
    v.push(s);

    let mut s = Scenario::new(
        "ext_ts_parse_fix",
        "--ts-parse-fix on ts_fifo_and_names's stream.",
        Mode::Ts,
        "longmynd --ts-parse-fix -s {dir}/status -t {dir}/ts 437000 333",
    );
    s.ts_chunks = names_chunks();
    v.push(s);

    let mut s = Scenario::new(
        "ext_extra_status",
        "--extra-status on lock_s2_1255_1500: $28-$31 after the C's lines.",
        Mode::Receiver,
        "longmynd --extra-status -s {dir}/status 1255000 1500",
    );
    s.duration_ms = 3000;
    s.demod = vec![(500, found_header()), (700, locked_s2(1_499_870, 87))];
    s.ts_flow = vec![(800, 3000)];
    v.push(s);

    let mut s = Scenario::new(
        "ext_warn_low_sr",
        "--warn-low-sr with 33 and 125kS/s (no tuner, so it ends after the parse).",
        Mode::Main,
        "longmynd --warn-low-sr -s {dir}/status 437000 33,125",
    );
    s.devices = vec![DevSpec::other(1, 1, 0x1d6b, 0x0002)];
    v.push(s);

    let mut s = Scenario::new(
        "ext_ts_timer_after_init",
        "--ts-timer-after-init on tuner_lock_retries: the no-TS timer waits for the init.",
        Mode::Receiver,
        "longmynd --ts-timer-after-init -s {dir}/status 2200000 4000",
    );
    s.tuner_lock_failures = 3;
    s.duration_ms = 9000;
    v.push(s);

    let mut s = Scenario::new(
        "ext_reconnect",
        "--reconnect: the tuner is unplugged while hunting and plugged back in 3.7s later.",
        Mode::Receiver,
        "longmynd --reconnect -s {dir}/status 437000 333",
    );
    s.unplugged = Some((300, 4000));
    s.duration_ms = 9000;
    v.push(s);

    let mut s = Scenario::new(
        "ext_decided_defaults",
        "The four decided defaults together (what a plain command line will do after the hardware test), 13cm, -w, -p h.",
        Mode::Receiver,
        "longmynd --fast-i2c --hf-filter-fix --lna-follow-input --lnb-off-at-exit -w -p h -s {dir}/status 2400000 1000",
    );
    s.duration_ms = 600;
    v.push(s);

    let mut s = Scenario::new(
        "ext_second_receiver",
        "--second on a PicoTuner: the bottom demodulator and tuner 2 at 741.5MHz 1500kS/s beside the main receiver.",
        Mode::Receiver,
        "longmynd --second 741500 1500 --second-status-fifo {dir}/status2 -s {dir}/status 1249000 333",
    );
    s.devices = vec![DevSpec::other(1, 1, 0x1d6b, 0x0002), DevSpec::pico(1, 9, 0x0102)];
    s.files = vec!["status", "status2"];
    s.demod = vec![(300, locked_s2(333_301, 55))];
    s.demod2 = vec![(450, locked_s(1_500_120, 0x12))];
    s.duration_ms = 900;
    v.push(s);

    let mut s = Scenario::new(
        "ext_second_on_minitiouner",
        "--second on a MiniTiouner, which has one TS: refused once the tuner is open.",
        Mode::Receiver,
        "longmynd --second 741500 1500 --second-status-fifo {dir}/status2 -s {dir}/status 1249000 333",
    );
    s.files = vec!["status", "status2"];
    s.duration_ms = 300;
    v.push(s);

    let mut s = Scenario::new(
        "ext_second_ts",
        "--second's TS: each endpoint's stream to its own output, each parsed into its own status.",
        Mode::Ts,
        "longmynd --second 741500 1500 --second-ts-fifo {dir}/ts2 --second-status-fifo {dir}/status2 -s {dir}/status -t {dir}/ts 1249000 333",
    );
    s.devices = vec![DevSpec::pico(1, 9, 0x0102)];
    s.files = vec!["status", "status2", "ts", "ts2"];
    s.ts_chunks = std::iter::once(FTDI_STATUS.to_vec())
        .chain(ftdi_reads(&ts_stream(&[(b"BATC", b"GB3HV")], &[(0x1b, 0x100)], 2, 40), 20))
        .collect();
    s.ts2_chunks = std::iter::once(FTDI_STATUS.to_vec())
        .chain(ftdi_reads(&ts_stream(&[(b"QO-100", b"BEACON")], &[(0x1b, 0x200)], 0, 30), 20))
        .collect();
    v.push(s);

    let mut s = Scenario::new(
        "ext_fast_retune_fallback",
        "--fast-retune when the tuner fails to lock on a re-tune: a full init, as the C would do.",
        Mode::Receiver,
        "longmynd --fast-retune -r 600 -s {dir}/status 437000 333,250",
    );
    s.tuner_fail_cals = vec![2];
    s.duration_ms = 1500;
    v.push(s);

    // Locked to DVB-S, then the TS stops, the no-TS timer re-inits, and the
    // tuner never locks again: all 11 attempts fail. The C overwrites that
    // error with the next scan-state read and runs on reporting the old
    // lock (item 11); --strict-errors stops.
    let mut s = Scenario::new(
        "ext_strict_errors",
        "--strict-errors: a re-init that fails while locked is an error, not overwritten.",
        Mode::Receiver,
        "longmynd --strict-errors -r 1000 -s {dir}/status 437000 333",
    );
    s.demod = vec![(200, locked_s(333_120, 0x12))];
    s.ts_flow = vec![(250, 600)];
    s.tuner_dead_from_cal = Some(2);
    s.duration_ms = 32_000;
    v.push(s);
    v
}

pub fn find(name: &str) -> Option<Scenario> {
    all().into_iter().chain(faults()).chain(extensions()).find(|s| s.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tuner_write_mask_hides_the_value_in_both_forms() {
        let trace = "\
   1.000 h0 bulk 02 out [37] 80 03 13 80 00 13 11 00 00 c0 80 00 11 27 00 87 -> 0 37
   1.100 h0 bulk 81 in [4096] -> 0 3: 32 60 00
   1.200 h0 bulk 02 out [13] 80 00 13 11 00 00 05 80 00 11 27 00 87 -> 0 13
   1.300 h0 bulk 02 out [13] 80 00 13 11 00 00 48 80 00 11 27 00 87 -> 0 13
   1.400 i2c c0 05 48
   1.500 h0 bulk 02 out [13] 80 00 13 11 00 00 48 80 00 11 27 00 87 -> 0 13
";
        let m = Mask::TunerWrite(0x05).apply(trace);
        let lines: Vec<&str> = m.lines().collect();
        assert!(lines[3].contains("11 00 00 <> 80"), "{}", lines[3]);
        assert_eq!(lines[4], "   1.400 i2c c0 05 <masked>");
        assert!(lines[5].contains("11 00 00 48 80"), "only the byte after the register is hidden");
        let p = Mask::Printed("icp=").apply("x icp=0x64, f=0x0");
        assert_eq!(p, "x icp=<past the table>, f=0x0");
    }
}
