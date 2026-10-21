//! main.h's status and the state the four threads share, and main.c's
//! status_all_write(): the `$n,value` lines.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::Mutex;

use crate::config::Config;
use crate::errors::*;
use crate::output::StatusSink;

// main-loop states
pub const STATE_INIT: u8 = 0;
pub const STATE_DEMOD_HUNTING: u8 = 1;
pub const STATE_DEMOD_FOUND_HEADER: u8 = 2;
pub const STATE_DEMOD_S: u8 = 3;
pub const STATE_DEMOD_S2: u8 = 4;

// status report numbers
pub const STATUS_STATE: u8 = 1;
pub const STATUS_LNA_GAIN: u8 = 2;
pub const STATUS_PUNCTURE_RATE: u8 = 3;
pub const STATUS_POWER_I: u8 = 4;
pub const STATUS_POWER_Q: u8 = 5;
pub const STATUS_CARRIER_FREQUENCY: u8 = 6;
pub const STATUS_CONSTELLATION_I: u8 = 7;
pub const STATUS_CONSTELLATION_Q: u8 = 8;
pub const STATUS_SYMBOL_RATE: u8 = 9;
pub const STATUS_VITERBI_ERROR_RATE: u8 = 10;
pub const STATUS_BER: u8 = 11;
pub const STATUS_MER: u8 = 12;
pub const STATUS_SERVICE_NAME: u8 = 13;
pub const STATUS_SERVICE_PROVIDER_NAME: u8 = 14;
pub const STATUS_TS_NULL_PERCENTAGE: u8 = 15;
pub const STATUS_ES_PID: u8 = 16;
pub const STATUS_ES_TYPE: u8 = 17;
pub const STATUS_MODCOD: u8 = 18;
pub const STATUS_SHORT_FRAME: u8 = 19;
pub const STATUS_PILOTS: u8 = 20;
pub const STATUS_ERRORS_LDPC_COUNT: u8 = 21;
pub const STATUS_ERRORS_BCH_COUNT: u8 = 22;
pub const STATUS_ERRORS_BCH_UNCORRECTED: u8 = 23;
pub const STATUS_LNB_SUPPLY: u8 = 24;
pub const STATUS_LNB_POLARISATION_H: u8 = 25;
pub const STATUS_AGC1_GAIN: u8 = 26;
pub const STATUS_AGC2_GAIN: u8 = 27;

// --extra-status only: numbers the C (BATC master, 1-27) does not use
/// carrier offset from the requested frequency, Hz, signed ($6 is kHz,
/// truncated)
pub const STATUS_CARRIER_OFFSET_HZ: u8 = 28;
/// roll-off in hundredths (35, 25, 20; 0 = not known)
pub const STATUS_ROLLOFF: u8 = 29;
/// TS payload arriving from the tuner, bytes per second
pub const STATUS_TS_RATE: u8 = 30;
/// 510-byte TS pieces dropped because the TS FIFO was full, since start
pub const STATUS_TS_DROPPED: u8 = 31;

pub const NUM_CONSTELLATIONS: usize = 16;
pub const NUM_ELEMENT_STREAMS: usize = 16;
/// The C's char service_name[255]: at most 254 bytes and a NUL.
pub const SERVICE_NAME_MAX: usize = 254;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Status {
    pub state: u8,
    pub demod_state: u8,
    pub lna_ok: bool,
    pub lna_gain: u16,
    pub agc1_gain: u16,
    pub agc2_gain: u16,
    pub power_i: u8,
    pub power_q: u8,
    pub frequency_requested: u32,
    /// Hz
    pub frequency_offset: i32,
    pub polarisation_supply: bool,
    pub polarisation_horizontal: bool,
    pub symbolrate_requested: u32,
    pub symbolrate: u32,
    pub viterbi_error_rate: u32,
    pub bit_error_rate: u32,
    pub modulation_error_rate: i32,
    pub errors_bch_uncorrected: bool,
    pub errors_bch_count: u32,
    pub errors_ldpc_count: u32,
    pub constellation: [[u8; 2]; NUM_CONSTELLATIONS],
    pub puncture_rate: u8,
    pub service_name: Vec<u8>,
    pub service_provider_name: Vec<u8>,
    pub ts_null_percentage: u8,
    pub ts_elementary_streams: [[u16; 2]; NUM_ELEMENT_STREAMS],
    pub modcod: u32,
    pub short_frame: bool,
    pub pilots: bool,
    pub last_ts_or_reinit_monotonic: u64,
    pub last_updated_monotonic: u64,
    // --extra-status
    /// hundredths; 0 not known
    pub rolloff: u8,
    /// bytes/s
    pub ts_rate: u32,
    pub ts_dropped: u32,
}

/// Everything the threads share: main.c's longmynd_config and
/// longmynd_status, the flags the C reads without a lock (here atomics),
/// and each thread's error, which main() watches.
pub struct Shared {
    pub config: Mutex<Config>,
    pub status: Mutex<Status>,
    /// --second: the second receiver's status, as `status` is the first's.
    pub status2: Mutex<Status>,
    /// config->ts_reset: set by the receiver on a new configuration, cleared
    /// by the TS thread once it has flushed the stale TS.
    pub ts_reset: AtomicBool,
    /// status->ts_packet_count_nolock: TS bytes since the last reset.
    pub ts_packet_count_nolock: AtomicU32,
    /// TS payload bytes received (FTDI status bytes excluded), never reset:
    /// for --extra-status's rate. Counting changes nothing the C does.
    pub ts_payload_bytes: AtomicU64,
    /// 510-byte TS pieces the FIFO writer dropped (item 15).
    pub ts_dropped: AtomicU32,
    /// The receiver is (re-)initialising the NIM (--ts-timer-after-init).
    pub init_in_progress: AtomicBool,
    /// main()'s err, which every thread polls.
    pub main_err: AtomicU8,
    pub thread_err_ts: AtomicU8,
    pub thread_err_ts_parse: AtomicU8,
    pub thread_err_i2c: AtomicU8,
    pub thread_err_beep: AtomicU8,
}

impl Shared {
    pub fn new(config: Config) -> Self {
        Shared {
            config: Mutex::new(config),
            status: Mutex::new(Status::default()),
            status2: Mutex::new(Status::default()),
            ts_reset: AtomicBool::new(false),
            ts_packet_count_nolock: AtomicU32::new(0),
            ts_payload_bytes: AtomicU64::new(0),
            ts_dropped: AtomicU32::new(0),
            init_in_progress: AtomicBool::new(false),
            main_err: AtomicU8::new(ERROR_NONE),
            thread_err_ts: AtomicU8::new(ERROR_NONE),
            thread_err_ts_parse: AtomicU8::new(ERROR_NONE),
            thread_err_i2c: AtomicU8::new(ERROR_NONE),
            thread_err_beep: AtomicU8::new(ERROR_NONE),
        }
    }

    pub fn main_err(&self) -> u8 {
        self.main_err.load(Ordering::SeqCst)
    }

    pub fn any_thread_err(&self) -> bool {
        [&self.thread_err_ts, &self.thread_err_ts_parse, &self.thread_err_beep, &self.thread_err_i2c]
            .iter()
            .any(|e| e.load(Ordering::SeqCst) != ERROR_NONE)
    }
}

/// A thread's `*err`, kept in its atomic so main() sees every change as it
/// happens (main reads the C's thread_err directly).
pub struct ErrCell<'a>(pub &'a AtomicU8);

impl ErrCell<'_> {
    pub fn get(&self) -> u8 {
        self.0.load(Ordering::SeqCst)
    }
    pub fn set(&self, v: u8) {
        self.0.store(v, Ordering::SeqCst)
    }
    pub fn ok(&self) -> bool {
        self.get() == ERROR_NONE
    }
}

/// status_all_write(): every status line, in the C's order, stopping at the
/// first error or as soon as the output stops being ready. `extra` is
/// --extra-status: four more lines at the end, after everything the C
/// writes, so a reader that stops at $20 sees no change.
pub fn status_all_write(status: &Status, sink: &mut dyn StatusSink, ready: &mut bool, extra: bool) -> u8 {
    let mut err = ERROR_NONE;
    macro_rules! w {
        ($msg:expr, $val:expr) => {
            if err == ERROR_NONE && *ready {
                err = sink.write($msg, $val, ready);
            }
        };
    }
    w!(STATUS_STATE, status.state as u32);
    if status.lna_ok {
        w!(STATUS_LNA_GAIN, status.lna_gain as u32);
    }
    w!(STATUS_AGC1_GAIN, status.agc1_gain as u32);
    w!(STATUS_AGC2_GAIN, status.agc2_gain as u32);
    w!(STATUS_POWER_I, status.power_i as u32);
    w!(STATUS_POWER_Q, status.power_q as u32);
    for c in status.constellation.iter() {
        w!(STATUS_CONSTELLATION_I, c[0] as u32);
        w!(STATUS_CONSTELLATION_Q, c[1] as u32);
    }
    w!(STATUS_PUNCTURE_RATE, status.puncture_rate as u32);
    // the offset is Hz, the requested frequency kHz: the C adds offset/1000
    // (int division) in uint32_t
    w!(
        STATUS_CARRIER_FREQUENCY,
        status.frequency_requested.wrapping_add((status.frequency_offset / 1000) as u32)
    );
    w!(STATUS_LNB_SUPPLY, status.polarisation_supply as u32);
    w!(STATUS_LNB_POLARISATION_H, status.polarisation_horizontal as u32);
    w!(STATUS_SYMBOL_RATE, status.symbolrate);
    w!(STATUS_VITERBI_ERROR_RATE, status.viterbi_error_rate);
    w!(STATUS_BER, status.bit_error_rate);
    w!(STATUS_MER, status.modulation_error_rate as u32);
    w!(STATUS_ERRORS_BCH_UNCORRECTED, status.errors_bch_uncorrected as u32);
    w!(STATUS_ERRORS_BCH_COUNT, status.errors_bch_count);
    w!(STATUS_ERRORS_LDPC_COUNT, status.errors_ldpc_count);
    if err == ERROR_NONE && *ready {
        err = sink.write_str(STATUS_SERVICE_NAME, &status.service_name, ready);
    }
    if err == ERROR_NONE && *ready {
        err = sink.write_str(STATUS_SERVICE_PROVIDER_NAME, &status.service_provider_name, ready);
    }
    w!(STATUS_TS_NULL_PERCENTAGE, status.ts_null_percentage as u32);
    for es in status.ts_elementary_streams.iter() {
        if es[0] > 0 {
            w!(STATUS_ES_PID, es[0] as u32);
            w!(STATUS_ES_TYPE, es[1] as u32);
        }
    }
    w!(STATUS_MODCOD, status.modcod);
    w!(STATUS_SHORT_FRAME, status.short_frame as u32);
    w!(STATUS_PILOTS, status.pilots as u32);
    if extra {
        w!(STATUS_CARRIER_OFFSET_HZ, status.frequency_offset as u32);
        w!(STATUS_ROLLOFF, status.rolloff as u32);
        w!(STATUS_TS_RATE, status.ts_rate);
        w!(STATUS_TS_DROPPED, status.ts_dropped);
    }
    err
}
