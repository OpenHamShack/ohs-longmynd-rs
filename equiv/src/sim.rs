//! A simulated MiniTiouner (or PicoTuner) at the libusb boundary, shared by
//! the C reference and the Rust port, and the trace it keeps.
//!
//! One process runs one scenario against one global [`Sim`]. The C build
//! reaches it through a fake libusb (src/bin/c_reference.rs); the Rust port
//! through its UsbBackend trait (src/rsdrive.rs). Both therefore talk to
//! the same model, and the model writes down every libusb call each makes,
//! every MPSSE byte, every decoded I2C transaction and every sleep, against
//! a virtual clock that only sleeps and USB transfers move. Equal traces
//! mean the two programs did the same thing to the hardware at the same
//! (virtual) times.
//!
//! The model is deliberately the protocol, not the electronics: MPSSE
//! commands are parsed, START/STOP are seen as SDA edges with SCL high,
//! bytes clocked out are I2C bytes, bits clocked in are ACKs. Behind the
//! bus sit the STV0910 (0xD2), the STV6120 (0xC0) and two STVVGLNAs (0xCE,
//! 0xC8), the last three only answering while the demodulator's repeater
//! (0xF12A bit 7) is open, as on the real NIM.

use std::collections::{HashMap, VecDeque};
use std::fmt::Write as _;
use std::sync::Mutex;

use crate::scenario::Scenario;

/// Virtual time starts here (1000 s), so the C's CLOCK_MONOTONIC and the
/// Rust's monotonic_ms() are both comfortably non-zero.
pub const T0_US: u64 = 1_000_000_000;

/// See [`Sim::scenario_ms`].
pub const STARTUP_MS: u64 = 2300;

// What a USB call costs in virtual time. Only consistency matters (C and
// Rust pay the same), but realistic orders of magnitude keep the timeouts
// in the C - 50ms tuner calibration, 100ms receiver loop - meaningful.
const COST_CONTROL_US: u64 = 100;
const COST_BULK_OUT_US: u64 = 125;
const COST_BULK_IN_US: u64 = 250;
const COST_TS_IN_US: u64 = 1000;

/// The FTDI's two modem-status bytes at the head of every 512-byte packet.
pub const FTDI_STATUS: [u8; 2] = [0x32, 0x60];

pub const LIBUSB_ERROR_ACCESS: i32 = -3;
pub const LIBUSB_ERROR_NO_DEVICE: i32 = -4;
pub const LIBUSB_ERROR_BUSY: i32 = -6;

pub static SIM: Mutex<Option<Sim>> = Mutex::new(None);

/// Run `f` on the global simulator.
pub fn with<R>(f: impl FnOnce(&mut Sim) -> R) -> R {
    let mut g = SIM.lock().unwrap();
    f(g.as_mut().expect("simulator not started"))
}

#[derive(Clone, Debug)]
pub struct DevSpec {
    pub bus: u8,
    pub addr: u8,
    pub vid: u16,
    pub pid: u16,
    pub bcd: u16,
    /// This entry is the MiniTiouner/PicoTuner the model answers for.
    pub tuner: bool,
    pub open_ok: bool,
    pub claim_ok: bool,
    /// libusb_kernel_driver_active(): 1 for ftdi_sio on Linux.
    pub kernel_driver: i32,
}

impl DevSpec {
    pub fn ftdi(bus: u8, addr: u8) -> Self {
        DevSpec { bus, addr, vid: 0x0403, pid: 0x6010, bcd: 0x0700, tuner: true, open_ok: true, claim_ok: true, kernel_driver: 1 }
    }
    pub fn pico(bus: u8, addr: u8, bcd: u16) -> Self {
        DevSpec { bus, addr, vid: 0x2e8a, pid: 0xba2c, bcd, tuner: true, open_ok: true, claim_ok: true, kernel_driver: 0 }
    }
    pub fn other(bus: u8, addr: u8, vid: u16, pid: u16) -> Self {
        DevSpec { bus, addr, vid, pid, bcd: 0x0100, tuner: false, open_ok: true, claim_ok: true, kernel_driver: 0 }
    }
}

// ---- the chips ---------------------------------------------------------------

/// What the demodulator reports, set by the scenario per phase.
#[derive(Clone, Debug)]
pub struct DemodPhase {
    /// HEADER_MODE: 0 hunting, 1 found header, 2 DVB-S2, 3 DVB-S
    pub header_mode: u8,
    /// carrier offset, Hz
    pub carrier_hz: i32,
    /// symbol rate the demodulator has found, S/s
    pub sr: u32,
    /// MER in 0.1dB, None when the noise RAM has no valid value
    pub mer: Option<i32>,
    /// DMDMODCOD as the register holds it
    pub modcod_reg: u8,
    /// Viterbi puncture register value (0x0d = 1/2 ...)
    pub curpun: u8,
    pub verror: u8,
    pub ber_count: u64,
    pub ber_errors: u32,
    pub errorflag: bool,
    pub bch_errors: u8,
    pub ldpc_errors: u16,
    /// AGC1 / AGC2 / power centre values; reads wobble around them
    pub agc1: u16,
    pub agc2: u16,
    pub power: u8,
}

impl DemodPhase {
    pub fn hunting() -> Self {
        DemodPhase {
            header_mode: 0,
            carrier_hz: -73_000,
            sr: 0,
            mer: None,
            modcod_reg: 0,
            curpun: 0x0d,
            verror: 0xff,
            ber_count: 0,
            ber_errors: 0,
            errorflag: false,
            bch_errors: 0,
            ldpc_errors: 0,
            agc1: 0x2f40,
            agc2: 0x0b00,
            power: 0x18,
        }
    }
}

/// The LNA's AGC readings for a phase (the C reports gain<<5 | vgo).
#[derive(Clone, Copy, Debug)]
pub struct LnaPhase {
    pub swlnagain: u8,
    pub vgo: u8,
}

pub struct NimModel {
    stv0910: HashMap<u16, u8>,
    stv6120: [u8; 0x20],
    lna: [[u8; 4]; 2],
    lna_getagc_reads: [u32; 2],
    /// reads of the tuner status since the last calibration start
    tuner_cal_reads: u32,
    /// VCO calibrations started (one per set_freq)
    pub tuner_vco_cals: u32,
    /// read counters per demodulator register, for the wobble
    reads: HashMap<u16, u32>,
}

impl NimModel {
    fn new() -> Self {
        NimModel {
            stv0910: HashMap::new(),
            stv6120: [0; 0x20],
            lna: [[0x20, 0, 0, 0]; 2],
            lna_getagc_reads: [0; 2],
            tuner_cal_reads: 0,
            tuner_vco_cals: 0,
            reads: HashMap::new(),
        }
    }

    fn repeater_open(&self) -> bool {
        self.stv0910.get(&0xf12a).copied().unwrap_or(0) & 0x80 != 0
    }
}

/// A deterministic wobble: the n-th read of a register returns a value
/// that moves, so the status output is not a constant (and an off-by-one
/// read anywhere shows up as different numbers).
fn wobble(reg: u16, n: u32, span: u32) -> u32 {
    let mut x = (reg as u32).wrapping_mul(2654435761) ^ n.wrapping_mul(40503);
    x ^= x >> 13;
    x = x.wrapping_mul(0x5bd1e995);
    x ^= x >> 15;
    x % span.max(1)
}

// ---- I2C on MPSSE ---------------------------------------------------------------

#[derive(Default)]
struct I2cState {
    scl: bool,
    sda: bool,
    in_transaction: bool,
    /// bytes so far in this segment (since START or repeated START)
    seg: Vec<u8>,
    /// address of the current segment, if any
    addr: Option<u8>,
    acked_addr: bool,
    /// ACK (true) / NAK for the last byte clocked out
    last_ack: bool,
    /// for logging: the transaction so far
    text: String,
}

struct Mpsse {
    pending: Vec<u8>,
    rx: VecDeque<u8>,
    i2c: I2cState,
    /// register pointer per 7-bit-address device
    pointer: HashMap<u8, u16>,
}

pub struct Handle {
    pub dev: usize,
    pub claimed: Option<u8>,
}

pub struct Sim {
    pub now_us: u64,
    pub trace: String,
    pub scn: Scenario,
    pub nim: NimModel,
    mpsse: Mpsse,
    pub handles: Vec<Handle>,
    pub ts_chunks: VecDeque<Vec<u8>>,
    pub ts2_chunks: VecDeque<Vec<u8>>,
    /// device lists handed out by get_device_list
    pub contexts: u32,
    /// reads of the I2C endpoint so far (for Scenario::short_reads)
    i2c_reads: u32,
    /// Scenario::unplugged: handles below this index belong to a tuner that
    /// has been unplugged, and stay dead.
    dead_below: usize,
    /// the tuner is unplugged now (as of the last USB call)
    gone: bool,
    /// D2XX (equiv/src/d2xxsim.rs): open handles, as (device, interface)
    pub d2xx_handles: Vec<(usize, u8)>,
    /// D2XX: the rest of the TS chunk the last FT_Read on B started
    d2xx_ts_rest: VecDeque<u8>,
    /// D2XX: per interface, an FT_GetQueueStatus has been answered since
    /// the last FT_Read - the next read takes bytes already in the driver
    d2xx_continuation: [bool; 2],
}

fn hex(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 3);
    for (i, b) in data.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        let _ = write!(s, "{:02x}", b);
    }
    s
}

/// FNV-1a, for logging bulk TS payloads without dumping them.
pub fn fnv(data: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for &b in data {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

impl Sim {
    pub fn start(scn: Scenario) {
        let ts_chunks = scn.ts_chunks.iter().cloned().collect();
        let ts2_chunks = scn.ts2_chunks.iter().cloned().collect();
        *SIM.lock().unwrap() = Some(Sim {
            now_us: T0_US,
            trace: String::new(),
            scn,
            nim: NimModel::new(),
            mpsse: Mpsse { pending: Vec::new(), rx: VecDeque::new(), i2c: I2cState::default(), pointer: HashMap::new() },
            handles: Vec::new(),
            ts_chunks,
            ts2_chunks,
            contexts: 0,
            i2c_reads: 0,
            dead_below: 0,
            gone: false,
            d2xx_handles: Vec::new(),
            d2xx_ts_rest: VecDeque::new(),
            d2xx_continuation: [false; 2],
        });
    }

    pub fn take_trace() -> String {
        with(|s| std::mem::take(&mut s.trace))
    }

    /// Milliseconds since the scenario started.
    pub fn elapsed_ms(&self) -> u64 {
        (self.now_us - T0_US) / 1000
    }

    /// Scenario time: milliseconds since the first NIM initialisation will
    /// have finished. A full init is about 2.2s of USB traffic (as on a
    /// real MiniTiouner, where longmynd takes a second or two to start);
    /// scenarios describe what happens after it.
    pub fn scenario_ms(&self) -> u64 {
        self.elapsed_ms().saturating_sub(STARTUP_MS)
    }

    pub fn log(&mut self, line: &str) {
        let t = self.now_us - T0_US;
        let _ = writeln!(self.trace, "{:6}.{:03} {}", t / 1000, t % 1000, line);
    }

    // ---- time -----------------------------------------------------------------

    pub fn usleep(&mut self, us: u64) {
        self.log(&format!("sleep {}", us));
        self.now_us += us;
    }

    pub fn monotonic_us(&self) -> u64 {
        self.now_us
    }

    /// Whether the scenario's run time is up (the harness then stops the
    /// program as SIGTERM would).
    pub fn terminate_due(&self) -> bool {
        match self.scn.mode {
            // (the second TS, if any, is read in turn with the first and
            // has fewer chunks in every scenario; the C never reads it)
            crate::scenario::Mode::Ts => self.ts_chunks.is_empty(),
            _ => self.scenario_ms() >= self.scn.duration_ms,
        }
    }

    /// TS bytes "arriving" now, for the receiver's no-data timer.
    pub fn ts_flow(&self) -> u32 {
        let t = self.scenario_ms();
        if self.scn.ts_flow.iter().any(|&(a, b)| t >= a && t < b) {
            188 * 7
        } else {
            0
        }
    }

    // ---- libusb -----------------------------------------------------------------

    pub fn libusb_init(&mut self) -> i32 {
        self.contexts += 1;
        self.log("libusb_init -> 0");
        0
    }

    pub fn set_option_log_info(&mut self) {
        self.log("libusb_set_option log_level=info");
    }

    fn new_handle(&mut self, dev: usize) -> usize {
        self.handles.push(Handle { dev, claimed: None });
        self.handles.len() - 1
    }

    /// Scenario::unplugged, as of now: on the way out every open handle
    /// dies; on the way back the device is a fresh one (MPSSE engine and
    /// NIM registers at power-on).
    fn plug_state(&mut self) -> bool {
        let gone = self.scn.unplugged.is_some_and(|(a, b)| (a..b).contains(&self.scenario_ms()));
        if gone && !self.gone {
            self.dead_below = self.handles.len();
            self.log("fault: tuner unplugged");
        } else if !gone && self.gone {
            self.mpsse = Mpsse { pending: Vec::new(), rx: VecDeque::new(), i2c: I2cState::default(), pointer: HashMap::new() };
            self.nim = NimModel::new();
            self.log("fault: tuner plugged back in");
        }
        self.gone = gone;
        gone
    }

    fn dead(&mut self, h: usize) -> bool {
        self.plug_state();
        h < self.dead_below
    }

    pub fn open_vid_pid(&mut self, vid: u16, pid: u16) -> Option<usize> {
        let gone = self.plug_state();
        let found = self.scn.devices.iter().position(|d| d.vid == vid && d.pid == pid && !(gone && d.tuner));
        let r = match found {
            Some(i) if self.scn.devices[i].open_ok => Some(self.new_handle(i)),
            _ => None,
        };
        match r {
            Some(h) => self.log(&format!("open_device_with_vid_pid {:04x}:{:04x} -> h{}", vid, pid, h)),
            None => self.log(&format!("open_device_with_vid_pid {:04x}:{:04x} -> NULL", vid, pid)),
        }
        r
    }

    pub fn device_list(&mut self) -> usize {
        // (-u while unplugged is not modelled: the list is the scenario's)
        let n = self.scn.devices.len();
        self.log(&format!("get_device_list -> {}", n));
        n
    }

    pub fn dev(&self, i: usize) -> &DevSpec {
        &self.scn.devices[i]
    }

    pub fn open_dev(&mut self, i: usize) -> Result<usize, i32> {
        let d = &self.scn.devices[i];
        let (bus, addr) = (d.bus, d.addr);
        if d.open_ok {
            let h = self.new_handle(i);
            self.log(&format!("open {},{} -> h{}", bus, addr, h));
            Ok(h)
        } else {
            self.log(&format!("open {},{} -> {}", bus, addr, LIBUSB_ERROR_ACCESS));
            Err(LIBUSB_ERROR_ACCESS)
        }
    }

    pub fn kernel_driver_active(&mut self, h: usize, iface: u8) -> i32 {
        let r = self.scn.devices[self.handles[h].dev].kernel_driver;
        self.log(&format!("h{} kernel_driver_active {} -> {}", h, iface, r));
        r
    }

    pub fn detach_kernel_driver(&mut self, h: usize, iface: u8) -> i32 {
        self.log(&format!("h{} detach_kernel_driver {} -> 0", h, iface));
        0
    }

    pub fn claim_interface(&mut self, h: usize, iface: u8) -> i32 {
        let ok = self.scn.devices[self.handles[h].dev].claim_ok;
        let r = if ok { 0 } else { LIBUSB_ERROR_BUSY };
        if ok {
            self.handles[h].claimed = Some(iface);
        }
        self.log(&format!("h{} claim_interface {} -> {}", h, iface, r));
        r
    }

    pub fn handle_descriptor(&mut self, h: usize) -> (u16, u16, u16) {
        let d = &self.scn.devices[self.handles[h].dev];
        let r = (d.vid, d.pid, d.bcd);
        self.log(&format!("h{} get_device_descriptor -> {:04x}:{:04x} bcd {:04x}", h, r.0, r.1, r.2));
        r
    }

    pub fn control(&mut self, h: usize, rt: u8, req: u8, val: u16, idx: u16) -> i32 {
        self.now_us += COST_CONTROL_US;
        if self.dead(h) {
            self.log(&format!("h{} control {:02x} {:02x} {:04x} {:04x} -> {}", h, rt, req, val, idx, LIBUSB_ERROR_NO_DEVICE));
            return LIBUSB_ERROR_NO_DEVICE;
        }
        self.log(&format!("h{} control {:02x} {:02x} {:04x} {:04x} -> 0", h, rt, req, val, idx));
        // SIO_RESET / PURGE_RX: the chip drops what it had for the host
        if req == 0 && val == 1 {
            self.mpsse.rx.clear();
        }
        0
    }

    /// libusb_bulk_transfer(): (result, transferred).
    pub fn bulk(&mut self, h: usize, ep: u8, data: &mut [u8]) -> (i32, usize) {
        if self.dead(h) {
            self.now_us += if ep & 0x80 != 0 { COST_BULK_IN_US } else { COST_BULK_OUT_US };
            self.log(&format!("h{} bulk {:02x} [{}] -> {} 0", h, ep, data.len(), LIBUSB_ERROR_NO_DEVICE));
            return (LIBUSB_ERROR_NO_DEVICE, 0);
        }
        match ep {
            0x02 => {
                self.now_us += COST_BULK_OUT_US;
                self.log(&format!("h{} bulk 02 out [{}] {} -> 0 {}", h, data.len(), hex(data), data.len()));
                self.mpsse.pending.extend_from_slice(data);
                self.run_mpsse();
                (0, data.len())
            }
            0x81 => {
                self.now_us += COST_BULK_IN_US;
                self.i2c_reads += 1;
                if self.scn.short_reads.contains(&self.i2c_reads) {
                    // the reply is lost, as when the device stalls or goes
                    self.mpsse.rx.clear();
                    data[..2].copy_from_slice(&FTDI_STATUS);
                    self.log(&format!("h{} bulk 81 in [{}] -> 0 2: 32 60 (fault: short read)", h, data.len()));
                    return (0, 2);
                }
                let mut out = Vec::new();
                loop {
                    out.extend_from_slice(&FTDI_STATUS);
                    let room = 510.min(data.len() - out.len());
                    for _ in 0..room {
                        match self.mpsse.rx.pop_front() {
                            Some(b) => out.push(b),
                            None => break,
                        }
                    }
                    if self.mpsse.rx.is_empty() || out.len() + 2 >= data.len() {
                        break;
                    }
                }
                data[..out.len()].copy_from_slice(&out);
                self.log(&format!("h{} bulk 81 in [{}] -> 0 {}: {}", h, data.len(), out.len(), hex(&out)));
                (0, out.len())
            }
            0x83 => {
                self.now_us += COST_TS_IN_US;
                let chunk = self.ts_chunks.pop_front().unwrap_or_else(|| FTDI_STATUS.to_vec());
                let n = chunk.len().min(data.len());
                data[..n].copy_from_slice(&chunk[..n]);
                self.log(&format!("h{} bulk 83 in [{}] -> 0 {} fnv {:016x}", h, data.len(), n, fnv(&chunk[..n])));
                (0, n)
            }
            0x84 if self.scn.devices[self.handles[h].dev].vid == 0x2e8a => {
                // a PicoTuner's second TS (TS1, the bottom demodulator)
                self.now_us += COST_TS_IN_US;
                let chunk = self.ts2_chunks.pop_front().unwrap_or_else(|| FTDI_STATUS.to_vec());
                let n = chunk.len().min(data.len());
                data[..n].copy_from_slice(&chunk[..n]);
                self.log(&format!("h{} bulk 84 in [{}] -> 0 {} fnv {:016x}", h, data.len(), n, fnv(&chunk[..n])));
                (0, n)
            }
            _ => {
                self.log(&format!("h{} bulk {:02x} [{}] -> -5", h, ep, data.len()));
                (-5, 0)
            }
        }
    }

    // ---- D2XX (equiv/src/d2xxsim.rs) -----------------------------------------------
    //
    // The same device seen through FTDI's driver: each FT2232H is two list
    // entries (interfaces A and B); reads return payload only, no status
    // bytes. Costs in virtual time are the libusb path's for the same
    // work - a write, an IN transfer, a control request - so a run through
    // D2XX keeps the C's timing: FT_Read starting a transfer costs a bulk
    // IN, a read of what an FT_GetQueueStatus has just reported costs
    // nothing (the bytes are already in the driver), opening and setting
    // up a handle nothing (as claiming an interface costs nothing).

    /// FT_CreateDeviceInfoList + FT_GetDeviceInfoDetail: the FTDI devices,
    /// two entries each. One another program has open (claim_ok false)
    /// shows only as "opened", without its details, as D2XX reports it.
    pub fn d2xx_list(&mut self) -> Vec<longmynd::d2xx::FtDeviceInfo> {
        use longmynd::d2xx::*;
        let mut v = Vec::new();
        for (i, d) in self.scn.devices.iter().enumerate() {
            if d.vid != 0x0403 {
                continue;
            }
            for (k, letter) in ["A", "B"].iter().enumerate() {
                let loc = 0x1000 + (i as u32) * 0x10 + k as u32 + 1;
                v.push(if d.claim_ok {
                    FtDeviceInfo {
                        flags: if self.d2xx_handles.contains(&(i, k as u8)) { FT_FLAGS_OPENED } else { 0 },
                        ft_type: FT_DEVICE_2232H,
                        id: ((d.vid as u32) << 16) | d.pid as u32,
                        loc_id: loc,
                        serial: format!("LM{:02}{:02}{}", d.bus, d.addr, letter),
                        description: format!("USB <-> NIM tuner {}", letter),
                    }
                } else {
                    FtDeviceInfo { flags: FT_FLAGS_OPENED, ..Default::default() }
                });
            }
        }
        self.log(&format!("d2xx FT_CreateDeviceInfoList -> 0 {}", v.len()));
        v
    }

    /// FT_OpenEx by location: Ok(handle).
    pub fn d2xx_open(&mut self, loc: u32) -> Result<usize, u32> {
        let i = ((loc - 0x1000) / 0x10) as usize;
        let k = ((loc & 0xf) - 1) as u8;
        let ok = self.scn.devices.get(i).is_some_and(|d| d.claim_ok && d.vid == 0x0403) && k < 2;
        if !ok {
            self.log(&format!("d2xx FT_OpenEx loc {:04x} -> 3", loc));
            return Err(longmynd::d2xx::FT_DEVICE_NOT_OPENED);
        }
        self.d2xx_handles.push((i, k));
        let h = self.d2xx_handles.len() - 1;
        self.log(&format!("d2xx FT_OpenEx loc {:04x} -> 0 d{}", loc, h));
        Ok(h)
    }

    fn d2xx_iface(&self, h: usize) -> u8 {
        self.d2xx_handles[h].1
    }

    /// FT_SetTimeouts, FT_SetUSBParameters, FT_Close: logged only.
    pub fn d2xx_setup(&mut self, h: usize, what: &str) -> u32 {
        self.log(&format!("d2xx d{} {} -> 0", h, what));
        0
    }

    /// FT_ResetDevice / FT_Purge / FT_SetLatencyTimer / FT_SetBitMode: a
    /// control request's cost, and a purge of A's RX drops its replies as
    /// SIO_RESET_PURGE_RX does.
    pub fn d2xx_control(&mut self, h: usize, what: &str, purge_rx: bool) -> u32 {
        self.now_us += COST_CONTROL_US;
        self.log(&format!("d2xx d{} {} -> 0", h, what));
        if purge_rx && self.d2xx_iface(h) == 0 {
            self.mpsse.rx.clear();
        }
        0
    }

    /// FT_Write: interface A takes MPSSE commands.
    pub fn d2xx_write(&mut self, h: usize, data: &[u8]) -> (u32, usize) {
        self.now_us += COST_BULK_OUT_US;
        self.log(&format!("d2xx d{} FT_Write [{}] {} -> 0 {}", h, data.len(), hex(data), data.len()));
        if self.d2xx_iface(h) == 0 {
            self.mpsse.pending.extend_from_slice(data);
            self.run_mpsse();
        }
        (0, data.len())
    }

    /// FT_GetQueueStatus: bytes waiting (A: MPSSE replies; B: the rest of
    /// the TS chunk being read).
    pub fn d2xx_queue(&mut self, h: usize) -> u32 {
        let k = self.d2xx_iface(h) as usize;
        let n = if k == 0 { self.mpsse.rx.len() } else { self.d2xx_ts_rest.len() };
        // (with nothing waiting no read follows, and the next is a new one)
        self.d2xx_continuation[k] = n > 0;
        self.log(&format!("d2xx d{} FT_GetQueueStatus -> 0 {}", h, n));
        n as u32
    }

    /// Whether an FT_Read on this handle now would start a TS transfer (the
    /// harness's TS hook runs before each, as before each libusb TS read).
    pub fn d2xx_starts_ts_read(&self, h: usize) -> bool {
        self.d2xx_iface(h) == 1 && !self.d2xx_continuation[1]
    }

    /// FT_Read of up to `buf.len()` bytes. On B a read that starts a
    /// transfer takes the next TS chunk (FTDI-framed in the scenario) and
    /// returns its payload without the status bytes; an empty chunk is a
    /// read that times out with nothing.
    pub fn d2xx_read(&mut self, h: usize, buf: &mut [u8]) -> (u32, usize) {
        let k = self.d2xx_iface(h) as usize;
        let cont = std::mem::take(&mut self.d2xx_continuation[k]);
        let n;
        if k == 0 {
            if !cont {
                self.now_us += COST_BULK_IN_US;
            }
            let mut i = 0;
            while i < buf.len() {
                match self.mpsse.rx.pop_front() {
                    Some(b) => buf[i] = b,
                    None => break,
                }
                i += 1;
            }
            n = i;
        } else {
            if !cont {
                self.now_us += COST_TS_IN_US;
                let chunk = self.ts_chunks.pop_front().unwrap_or_else(|| FTDI_STATUS.to_vec());
                self.d2xx_ts_rest = chunk.chunks(512).flat_map(|p| p[2.min(p.len())..].to_vec()).collect();
            }
            let mut i = 0;
            while i < buf.len() {
                match self.d2xx_ts_rest.pop_front() {
                    Some(b) => buf[i] = b,
                    None => break,
                }
                i += 1;
            }
            n = i;
        }
        self.log(&format!("d2xx d{} FT_Read [{}] -> 0 {}: {}", h, buf.len(), n, if k == 0 { hex(&buf[..n]) } else { format!("fnv {:016x}", fnv(&buf[..n])) }));
        (0, n)
    }

    // ---- MPSSE ---------------------------------------------------------------------

    fn run_mpsse(&mut self) {
        loop {
            let p = &self.mpsse.pending;
            if p.is_empty() {
                return;
            }
            let need = match p[0] {
                0x80 | 0x82 | 0x86 => 3,
                0x11 | 0x10 | 0x19 | 0x18 => {
                    if p.len() < 3 {
                        return;
                    }
                    3 + (u16::from_le_bytes([p[1], p[2]]) as usize + 1)
                }
                0x20 | 0x24 | 0x25 | 0x28 | 0x2c => 3,
                0x22 | 0x26 | 0x27 | 0x2a | 0x2e => 2,
                _ => 1,
            };
            if p.len() < need {
                return;
            }
            let cmd: Vec<u8> = self.mpsse.pending.drain(..need).collect();
            self.mpsse_cmd(&cmd);
        }
    }

    fn mpsse_cmd(&mut self, c: &[u8]) {
        match c[0] {
            0x80 => self.pins(c[1], c[2]),
            0x82 => self.log(&format!("gpio high value={:02x} dir={:02x}", c[1], c[2])),
            0x86 | 0x8a | 0x8b | 0x8c | 0x8d | 0x97 | 0x96 | 0x84 | 0x85 | 0x87 => {}
            0x11 | 0x10 | 0x19 | 0x18 => {
                for &b in &c[3..] {
                    self.i2c_byte_out(b);
                }
            }
            0x20 | 0x24 | 0x25 | 0x28 | 0x2c => {
                let n = u16::from_le_bytes([c[1], c[2]]) as usize + 1;
                for _ in 0..n {
                    let b = self.i2c_byte_in();
                    self.mpsse.rx.push_back(b);
                }
            }
            0x22 | 0x26 | 0x27 | 0x2a | 0x2e => {
                // bits in: the ACK after a byte out is the one bit wanted
                let ack = self.mpsse.i2c.last_ack;
                self.mpsse.rx.push_back(if ack { 0x00 } else { 0x01 });
            }
            other => {
                // an unknown opcode: the FT2232 answers 0xFA and echoes it
                self.mpsse.rx.push_back(0xFA);
                self.mpsse.rx.push_back(other);
            }
        }
    }

    fn pins(&mut self, val: u8, dir: u8) {
        let scl = val & 0x01 != 0;
        // SDA driven only when its direction bit is set; otherwise pulled up
        let sda = if dir & 0x02 != 0 { val & 0x02 != 0 } else { true };
        let st = &self.mpsse.i2c;
        let (was_scl, was_sda) = (st.scl, st.sda);
        if scl && was_scl && was_sda && !sda {
            self.i2c_start();
        } else if scl && was_scl && !was_sda && sda {
            self.i2c_stop();
        }
        self.mpsse.i2c.scl = scl;
        self.mpsse.i2c.sda = sda;
    }

    fn i2c_start(&mut self) {
        let st = &mut self.mpsse.i2c;
        if st.in_transaction {
            st.text.push_str(" |");
        } else {
            st.in_transaction = true;
            st.text = String::from("i2c");
        }
        st.seg.clear();
        st.addr = None;
        st.acked_addr = false;
    }

    fn i2c_stop(&mut self) {
        if self.mpsse.i2c.in_transaction {
            let text = std::mem::take(&mut self.mpsse.i2c.text);
            self.log(&text);
        }
        let st = &mut self.mpsse.i2c;
        st.in_transaction = false;
        st.seg.clear();
        st.addr = None;
    }

    fn present(&self, addr7: u8) -> bool {
        match addr7 << 1 {
            0xd2 => true,
            0xc0 => self.nim.repeater_open(),
            0xce => self.nim.repeater_open() && self.scn.lna_present,
            0xc8 => self.nim.repeater_open() && self.scn.lna_present,
            _ => false,
        }
    }

    fn i2c_byte_out(&mut self, b: u8) {
        if !self.mpsse.i2c.in_transaction {
            // clocked with no START: nobody is listening
            self.mpsse.i2c.last_ack = false;
            return;
        }
        let ack;
        if self.mpsse.i2c.addr.is_none() {
            let a7 = b >> 1;
            ack = self.present(a7);
            let st = &mut self.mpsse.i2c;
            st.addr = Some(b);
            st.acked_addr = ack;
        } else {
            let addr = self.mpsse.i2c.addr.unwrap();
            ack = self.mpsse.i2c.acked_addr && addr & 1 == 0;
            if ack {
                let idx = self.mpsse.i2c.seg.len(); // bytes after the address so far
                self.device_write(addr >> 1, idx, b);
            }
            self.mpsse.i2c.seg.push(b);
        }
        let st = &mut self.mpsse.i2c;
        st.last_ack = ack;
        let _ = write!(st.text, " {:02x}{}", b, if ack { "" } else { "!" });
    }

    fn i2c_byte_in(&mut self) -> u8 {
        let st = &self.mpsse.i2c;
        let v = match st.addr {
            Some(a) if st.in_transaction && st.acked_addr && a & 1 == 1 => self.device_read(a >> 1),
            _ => 0xff,
        };
        let _ = write!(self.mpsse.i2c.text, " <{:02x}", v);
        v
    }

    // ---- register devices --------------------------------------------------------

    /// A byte written after the address: the first one or two set the
    /// register pointer, the rest are data, auto-incrementing.
    fn device_write(&mut self, a7: u8, idx: usize, b: u8) {
        let wide = a7 << 1 == 0xd2;
        let ptr_len = if wide { 2 } else { 1 };
        let p = self.mpsse.pointer.entry(a7).or_insert(0);
        if idx < ptr_len {
            *p = if wide && idx == 0 { (b as u16) << 8 } else if wide { *p | b as u16 } else { b as u16 };
            return;
        }
        let reg = *p;
        *p = p.wrapping_add(1);
        match a7 << 1 {
            0xd2 => {
                self.nim.stv0910.insert(reg, b);
            }
            0xc0 => {
                let r = (reg & 0x1f) as usize;
                self.nim.stv6120[r] = b;
                if r == 0x08 || r == 0x12 {
                    if b & 0x02 != 0 || b & 0x04 != 0 {
                        self.nim.tuner_cal_reads = 0;
                    }
                    if b & 0x04 != 0 {
                        self.nim.tuner_vco_cals += 1;
                    }
                }
            }
            0xce | 0xc8 => {
                let l = if a7 << 1 == 0xce { 0 } else { 1 };
                let r = (reg & 3) as usize;
                self.nim.lna[l][r] = b;
                if r == 1 && b & 0x20 != 0 {
                    self.nim.lna_getagc_reads[l] = 0;
                }
            }
            _ => {}
        }
    }

    fn device_read(&mut self, a7: u8) -> u8 {
        let p = self.mpsse.pointer.entry(a7).or_insert(0);
        let reg = *p;
        *p = p.wrapping_add(1);
        match a7 << 1 {
            0xd2 => self.stv0910_read(reg),
            0xc0 => self.stv6120_read((reg & 0x1f) as u8),
            0xce => self.lna_read(0, (reg & 3) as u8),
            0xc8 => self.lna_read(1, (reg & 3) as u8),
            _ => 0xff,
        }
    }

    fn reads_of(&self, reg: u16) -> u32 {
        self.nim.reads.get(&reg).copied().unwrap_or(0)
    }

    fn stv6120_read(&mut self, r: u8) -> u8 {
        let v = self.nim.stv6120[r as usize];
        if r != 0x08 && r != 0x12 {
            return v;
        }
        // calibration bits clear after the first status read; LOCK is set
        // unless the scenario has this VCO calibration fail to lock
        self.nim.tuner_cal_reads += 1;
        let mut v = v & !0x01;
        if self.nim.tuner_cal_reads > 1 {
            v &= !0x06;
        }
        let dead = self.scn.tuner_dead_from_cal.is_some_and(|k| self.nim.tuner_vco_cals >= k)
            || self.scn.tuner_fail_cals.contains(&self.nim.tuner_vco_cals);
        if self.nim.tuner_vco_cals > self.scn.tuner_lock_failures && !dead {
            v |= 0x01;
        }
        v
    }

    fn lna_read(&mut self, l: usize, r: u8) -> u8 {
        let v = self.nim.lna[l][r as usize];
        let phase = self.scn.lna_at(self.scenario_ms());
        match r {
            0 => (v & 0x0f) | 0x20,
            1 => {
                self.nim.lna_getagc_reads[l] += 1;
                let busy = if self.nim.lna_getagc_reads[l] <= 1 && v & 0x20 != 0 { 0x20 } else { 0 };
                (v & 0xc0) | busy | (phase.vgo & 0x1f)
            }
            3 => (v & 0xfc) | (phase.swlnagain & 0x03),
            _ => v,
        }
    }

    fn stv0910_read(&mut self, reg: u16) -> u8 {
        let n = {
            let c = self.nim.reads.entry(reg).or_insert(0);
            *c += 1;
            *c
        };
        let written = self.nim.stv0910.get(&reg).copied();
        // --second scenarios give the bottom demodulator (P1, 0xF4xx-0xF5xx,
        // each register 0x200 above its P2 twin) its own phases; without
        // them P1 answers what was written, as before
        let p1 = !self.scn.demod2.is_empty() && (0xf400..0xf600).contains(&reg) && reg != 0xf536;
        let (ph, reg) = if p1 {
            (self.scn.demod2_at(self.scenario_ms()), reg - 0x200)
        } else {
            (self.scn.demod_at(self.scenario_ms()), reg)
        };
        let cfr = ((ph.carrier_hz as i64 * (1 << 24)) / 135_000_000) as i32 as u32;
        let sfr = ((ph.sr as u64 * (1u64 << 32)) / 135_000_000) as u32;
        match reg {
            0xf536 if self.scn.demod_readback_corrupt => written.unwrap_or(0) ^ 0xff,
            0xf100 => 0x51,
            0xf101 => 0x20,
            0xf1b8 => 0x01, // PLLSTAT: locked
            0xf21b => (written.unwrap_or(0) & 0x9f) | (ph.header_mode << 5), // DMDSTATE
            0xf33a => (written.unwrap_or(0) & 0xe0) | ph.curpun,             // VSTATUSVIT
            0xf33b => ph.verror,
            0xf202 | 0xf203 => {
                // ISYMB/QSYMB: a locked signal clusters, hunting is noise
                let w = wobble(reg, n, if ph.header_mode >= 2 { 24 } else { 256 }) as u8;
                if ph.header_mode >= 2 {
                    (if n % 2 == 0 { 0x40u8 } else { 0xc0 }).wrapping_add(w).wrapping_sub(12)
                } else {
                    w
                }
            }
            0xf20a | 0xf20b => ph.power.wrapping_add(wobble(reg, n, 5) as u8), // POWERI/Q
            // 16-bit AGC values are read low byte first; the high byte
            // takes the wobble of the low byte's read so the pair agrees
            0xf20f => ((ph.agc1 as u32 + wobble(reg, n, 64)) & 0xff) as u8, // AGCIQIN0
            0xf20e => ((ph.agc1 as u32 + wobble(0xf20f, self.reads_of(0xf20f), 64)) >> 8) as u8, // AGCIQIN1
            0xf237 => ((ph.agc2 as u32 + wobble(reg, n, 32)) & 0xff) as u8, // AGC2I0
            0xf236 => ((ph.agc2 as u32 + wobble(0xf237, self.reads_of(0xf237), 32)) >> 8) as u8, // AGC2I1
            0xf211 => ph.modcod_reg,
            0xf24c => (cfr >> 16) as u8,
            0xf24d => (cfr >> 8) as u8,
            0xf24e => cfr as u8,
            0xf264 => (sfr >> 24) as u8,
            0xf265 => (sfr >> 16) as u8,
            0xf266 => (sfr >> 8) as u8,
            0xf267 => sfr as u8,
            0xf30e => match ph.mer {
                Some(m) if m < 0 => 0x04 | 0x02 | (((m + 512) >> 8) & 1) as u8,
                Some(m) => 0x04 | ((m >> 8) & 1) as u8,
                None => written.unwrap_or(0) & 0xf8,
            },
            0xf30f => match ph.mer {
                Some(m) if m < 0 => (m + 512) as u8,
                Some(m) => m as u8,
                None => 0,
            },
            0xf3a8 => (ph.ber_count >> 32) as u8,
            0xf3a9 => (ph.ber_count >> 24) as u8,
            0xf3aa => (ph.ber_count >> 16) as u8,
            0xf3ab => (ph.ber_count >> 8) as u8,
            0xf3ac => ph.ber_count as u8,
            0xf3ad => (ph.ber_errors >> 16) as u8,
            0xf3ae => (ph.ber_errors >> 8) as u8,
            0xf3af => ph.ber_errors as u8,
            0xfa96 => (ph.ldpc_errors >> 8) as u8,
            0xfa97 => ph.ldpc_errors as u8,
            0xfa98 => (if ph.errorflag { 0x10 } else { 0 }) | (ph.bch_errors & 0x0f),
            _ => written.unwrap_or(0),
        }
    }
}
