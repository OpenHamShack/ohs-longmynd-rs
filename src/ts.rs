//! ts.c: the TS thread (read the TS endpoint, write the stream out, hand a
//! copy to the parser when it is free) and the parse thread (turn what it
//! is handed into service name, provider, PIDs and null share).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::Duration;

use crate::charset::{a3, a4, to_utf8, Charset};
use crate::errors::*;
use crate::ftdi_usb::TsUsb;
use crate::libts::{ts_parse, ts_parse_fixed, TsCallbacks};
use crate::output::TsSink;
use crate::status::{ErrCell, Shared, Status, NUM_ELEMENT_STREAMS, SERVICE_NAME_MAX};

/// 20 FTDI USB packets of 512 bytes.
pub const TS_FRAME_SIZE: usize = 20 * 512;

/// longmynd_ts_parse_buffer: one chunk of TS on its way from the TS thread
/// to the parse thread. The TS thread only hands a chunk over when the
/// parser is waiting and the lock is free; otherwise the parser simply
/// misses that chunk.
pub struct TsParseHandoff {
    pub inner: Mutex<HandoffInner>,
    pub signal: Condvar,
    pub waiting: AtomicBool,
}

pub struct HandoffInner {
    pub buffer: Vec<u8>,
    pub length: u32,
}

impl TsParseHandoff {
    pub fn new() -> Self {
        TsParseHandoff {
            inner: Mutex::new(HandoffInner { buffer: vec![0; TS_FRAME_SIZE], length: 0 }),
            signal: Condvar::new(),
            waiting: AtomicBool::new(false),
        }
    }

    /// The TS thread's side: copy `data` across if the parser is waiting
    /// and the lock can be had without waiting (pthread_mutex_trylock).
    pub fn offer(&self, data: &[u8]) {
        if !self.waiting.load(Ordering::SeqCst) {
            return;
        }
        if let Ok(mut inner) = self.inner.try_lock() {
            inner.buffer[..data.len()].copy_from_slice(data);
            inner.length = data.len() as u32;
            self.signal.notify_one();
            self.waiting.store(false, Ordering::SeqCst);
        }
    }
}

impl Default for TsParseHandoff {
    fn default() -> Self {
        Self::new()
    }
}

/// Where the TS goes, chosen from the configuration: the C's ts_write
/// pointer plus the fifo path for re-opening.
pub enum TsOut {
    Udp { ip: Vec<u8>, port: i32 },
    Fifo { path: Vec<u8> },
}

/// loop_ts()
pub fn loop_ts(shared: &Shared, usb: &mut TsUsb, out: TsOut, handoff: &TsParseHandoff) {
    let err = ErrCell(&shared.thread_err_ts);
    err.set(ERROR_NONE);
    let mut buffer = vec![0u8; TS_FRAME_SIZE];
    let mut len: u16 = 0;
    let mut fifo_ready = false;
    let use_ip = matches!(out, TsOut::Udp { .. });

    let mut sink: Option<Box<dyn TsSink>> = match &out {
        TsOut::Udp { ip, port } => match crate::output::Udp::init(ip, *port) {
            Ok(u) => Some(Box::new(crate::output::UdpTs(u))),
            Err(e) => {
                err.set(e);
                None
            }
        },
        TsOut::Fifo { path } => match crate::output::Fifo::init(path, &mut fifo_ready) {
            Ok(f) => Some(Box::new(crate::output::FifoTs(f, 0))),
            Err(e) => {
                err.set(e);
                None
            }
        },
    };

    while err.ok() && shared.main_err() == ERROR_NONE {
        // a new configuration: drain what the old one left in the FTDI
        if shared.ts_reset.load(Ordering::SeqCst) {
            loop {
                if err.ok() {
                    err.set(usb.read(&mut buffer, &mut len, TS_FRAME_SIZE));
                }
                if !(err.ok() && len > 2) {
                    break;
                }
            }
            {
                let mut st = shared.status.lock().unwrap();
                st.service_name.clear();
                st.service_provider_name.clear();
                st.ts_null_percentage = 100;
                shared.ts_packet_count_nolock.store(0, Ordering::SeqCst);
                for es in st.ts_elementary_streams.iter_mut() {
                    es[0] = 0;
                }
            }
            shared.ts_reset.store(false, Ordering::SeqCst);
        }

        err.set(usb.read(&mut buffer, &mut len, TS_FRAME_SIZE));
        // the first two bytes of the read are the FTDI's status, not TS
        if err.ok() && len > 2 {
            let data = &buffer[2..len as usize];
            let s = sink.as_mut().expect("sink exists while err is none");
            if use_ip || fifo_ready {
                err.set(s.write(data, &mut fifo_ready));
            } else {
                err.set(s.reopen(&mut fifo_ready));
            }
            handoff.offer(data);
            shared.ts_packet_count_nolock.fetch_add(len as u32 - 2, Ordering::SeqCst);
            // payload only: every 512-byte USB packet starts with 2 status bytes
            let payload = len as u64 - 2 * (len as u64).div_ceil(512);
            shared.ts_payload_bytes.fetch_add(payload, Ordering::SeqCst);
            shared.ts_dropped.store(s.dropped(), Ordering::SeqCst);
        }
    }
}

/// --second's TS thread: the C's loop_ts for the main TS on 0x83 (sink,
/// parse handoff and counters exactly as loop_ts), and in the same loop
/// the second receiver's TS from 0x84 to its own output, parsed here
/// rather than by a second parse thread (a PicoTuner is USB full speed,
/// so this is at most ~1 MB/s). The two endpoints are read in turn; the
/// PicoTuner answers each within 16 ms, with a status-only packet when it
/// has no TS, so neither starves the other.
pub fn loop_ts_dual(shared: &Shared, usb: &mut TsUsb, out: TsOut, out2: TsOut, handoff: &TsParseHandoff) {
    let err = ErrCell(&shared.thread_err_ts);
    err.set(ERROR_NONE);
    let mut buffer = vec![0u8; TS_FRAME_SIZE];
    let mut len: u16 = 0;
    let fixed = shared.config.lock().unwrap().ext.ts_parse_fix;
    let mut ready = [false, false];
    let use_ip = [matches!(out, TsOut::Udp { .. }), matches!(out2, TsOut::Udp { .. })];
    let mut sinks: Vec<Box<dyn TsSink>> = Vec::new();
    for (i, o) in [out, out2].into_iter().enumerate() {
        match open_ts_sink(&o, &mut ready[i]) {
            Ok(s) => sinks.push(s),
            Err(e) => {
                err.set(e);
                return;
            }
        }
    }

    while err.ok() && shared.main_err() == ERROR_NONE {
        if shared.ts_reset.load(Ordering::SeqCst) {
            // both demodulators were re-initialised: drain both
            for ep in [0x83, 0x84] {
                loop {
                    if err.ok() {
                        err.set(usb.read_ep(ep, &mut buffer, &mut len, TS_FRAME_SIZE));
                    }
                    if !(err.ok() && len > 2) {
                        break;
                    }
                }
            }
            for st in [&shared.status, &shared.status2] {
                let mut st = st.lock().unwrap();
                st.service_name.clear();
                st.service_provider_name.clear();
                st.ts_null_percentage = 100;
                for es in st.ts_elementary_streams.iter_mut() {
                    es[0] = 0;
                }
            }
            shared.ts_packet_count_nolock.store(0, Ordering::SeqCst);
            shared.ts_reset.store(false, Ordering::SeqCst);
        }

        for (i, ep) in [(0usize, 0x83u8), (1, 0x84)] {
            if !err.ok() {
                break;
            }
            err.set(usb.read_ep(ep, &mut buffer, &mut len, TS_FRAME_SIZE));
            if err.ok() && len > 2 {
                let data = &buffer[2..len as usize];
                let r = &mut ready[i];
                err.set(if use_ip[i] || *r { sinks[i].write(data, r) } else { sinks[i].reopen(r) });
                if i == 0 {
                    handoff.offer(data);
                    shared.ts_packet_count_nolock.fetch_add(len as u32 - 2, Ordering::SeqCst);
                    let payload = len as u64 - 2 * (len as u64).div_ceil(512);
                    shared.ts_payload_bytes.fetch_add(payload, Ordering::SeqCst);
                    shared.ts_dropped.store(sinks[0].dropped(), Ordering::SeqCst);
                } else {
                    parse_chunk_into(&shared.status2, data, data.len() as u32, fixed);
                }
            }
        }
    }
}

/// A TS output, opened as loop_ts opens its own.
fn open_ts_sink(out: &TsOut, ready: &mut bool) -> Result<Box<dyn TsSink>, u8> {
    match out {
        TsOut::Udp { ip, port } => crate::output::Udp::init(ip, *port).map(|u| Box::new(crate::output::UdpTs(u)) as Box<dyn TsSink>),
        TsOut::Fifo { path } => crate::output::Fifo::init(path, ready).map(|f| Box::new(crate::output::FifoTs(f, 0)) as Box<dyn TsSink>),
    }
}

impl TsOut {
    /// --second's output, as a TsOut.
    pub fn from_out(o: &crate::config::Out) -> TsOut {
        match o {
            crate::config::Out::Fifo(p) => TsOut::Fifo { path: p.clone() },
            crate::config::Out::Udp(ip, port) => TsOut::Udp { ip: ip.clone(), port: *port },
        }
    }
}

/// The parse thread's callbacks (ts_callback_*), writing into a status
/// (the main receiver's, or with --second the second's).
pub struct StatusCallbacks<'a> {
    pub status: &'a Mutex<Status>,
}

/// Decode one name as ts_callback_sdt_service() does: pick the character
/// set from the selector bytes, skip them (shortening `len`, which the
/// parser then uses), convert, and keep the raw bytes if conversion fails.
pub fn decode_name(buf: &[u8], mut off: usize, len: &mut u32) -> Vec<u8> {
    let get = |i: usize| buf.get(i).copied().unwrap_or(0);
    let raw = |off: usize, len: u32| -> Vec<u8> {
        let n = (len as usize).min(SERVICE_NAME_MAX + 1);
        (0..n).map(|i| get(off + i)).collect()
    };
    if *len > 0 {
        let first = get(off);
        let (charset, skip): (Charset, u32) = if first < 0x20 {
            if let Some(cs) = a3(first) {
                (cs, 1)
            } else if first == 0x10 && get(off + 1) == 0x00 && get(off + 2) < 0x10 {
                (a4(get(off + 2)).unwrap_or(Charset::Iso6937), 3)
            } else {
                (Charset::Iso6937, 1)
            }
        } else {
            (Charset::Iso6937, 0)
        };
        off += skip as usize;
        *len = len.wrapping_sub(skip);
        // a length that wrapped (a 1-byte name that is only a selector)
        // would have the C copy gigabytes; nothing past the buffer counts
        let avail = buf.len().saturating_sub(off).min(*len as usize);
        let input: Vec<u8> = (0..avail).map(|i| get(off + i)).collect();
        if input.len() == *len as usize {
            if let Some(out) = to_utf8(charset, &input) {
                return out;
            }
        }
    }
    raw(off, *len)
}

/// memcpy into the C's char[255] and terminate: names longer than 254
/// bytes overflow the C's array; here they are cut to fit.
fn store_name(dst: &mut Vec<u8>, name: Vec<u8>) {
    let mut name = name;
    name.truncate(SERVICE_NAME_MAX);
    *dst = name;
}

impl TsCallbacks for StatusCallbacks<'_> {
    fn sdt_service(&mut self, buf: &[u8], provider_off: usize, provider_len: &mut u32, name_off: usize, name_len: &mut u32) {
        let name = decode_name(buf, name_off, name_len);
        let provider = decode_name(buf, provider_off, provider_len);
        let mut st = self.status.lock().unwrap();
        store_name(&mut st.service_name, name);
        store_name(&mut st.service_provider_name, provider);
    }

    fn pmt_pids(&mut self, index: u32, es_pid: u32, es_type: u32) {
        // The C writes ts_elementary_streams[index] for any index, past the
        // 16-entry array into the rest of the status for a PMT with more
        // streams; those are dropped here.
        if (index as usize) < NUM_ELEMENT_STREAMS {
            let mut st = self.status.lock().unwrap();
            st.ts_elementary_streams[index as usize] = [es_pid as u16, es_type as u16];
        }
    }

    fn ts_stats(&mut self, total_count: u32, null_percentage: u32) {
        if total_count > 0 {
            self.status.lock().unwrap().ts_null_percentage = null_percentage as u8;
        }
    }
}

/// One parse of a handed-over chunk (the body of loop_ts_parse's loop);
/// `fixed` is --ts-parse-fix.
pub fn parse_chunk(shared: &Shared, buffer: &[u8], length: u32, fixed: bool) {
    parse_chunk_into(&shared.status, buffer, length, fixed)
}

fn parse_chunk_into(status: &Mutex<Status>, buffer: &[u8], length: u32, fixed: bool) {
    let mut cb = StatusCallbacks { status };
    if fixed {
        ts_parse_fixed(buffer, length, &mut cb);
    } else {
        ts_parse(buffer, length, &mut cb);
    }
}

/// loop_ts_parse()
pub fn loop_ts_parse(shared: &Shared, handoff: &TsParseHandoff) {
    let err = ErrCell(&shared.thread_err_ts_parse);
    err.set(ERROR_NONE);
    let fixed = shared.config.lock().unwrap().ext.ts_parse_fix;
    let mut guard = handoff.inner.lock().unwrap();
    while err.ok() && shared.main_err() == ERROR_NONE {
        handoff.waiting.store(true, Ordering::SeqCst);
        while handoff.waiting.load(Ordering::SeqCst) && shared.main_err() == ERROR_NONE {
            let (g, _) = handoff.signal.wait_timeout(guard, Duration::from_millis(100)).unwrap();
            guard = g;
        }
        let length = guard.length;
        parse_chunk(shared, &guard.buffer, length, fixed);
    }
}

/// Used by tests: a status with what the parse of `ts` reports.
pub fn parse_into_status(ts: &[u8]) -> Status {
    let shared = Shared::new(crate::config::Config::default());
    let mut buf = vec![0u8; TS_FRAME_SIZE.max(ts.len())];
    buf[..ts.len()].copy_from_slice(ts);
    parse_chunk(&shared, &buf, ts.len() as u32, false);
    let st = shared.status.lock().unwrap().clone();
    st
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selector_bytes_pick_the_charset_and_shorten_the_length() {
        let buf = [0x05u8, b'A', 0xdd]; // ISO-8859-9: 0xdd is I with dot
        let mut len = 3;
        assert_eq!(decode_name(&buf, 0, &mut len), "Aİ".as_bytes());
        assert_eq!(len, 2);
        let buf = [0x10u8, 0x00, 0x01, 0xe9]; // A.4: ISO-8859-1
        let mut len = 4;
        assert_eq!(decode_name(&buf, 0, &mut len), "é".as_bytes());
        assert_eq!(len, 1);
        let buf = [0x15u8, 0xff]; // UTF-8 selector, invalid UTF-8: raw
        let mut len = 2;
        assert_eq!(decode_name(&buf, 0, &mut len), vec![0xff]);
    }

    #[test]
    fn oversized_pmt_is_cut_at_sixteen_streams() {
        let shared = Shared::new(crate::config::Config::default());
        let mut cb = StatusCallbacks { status: &shared.status };
        for i in 0..20 {
            cb.pmt_pids(i, 0x100 + i, 0x1b);
        }
        let st = shared.status.lock().unwrap();
        assert_eq!(st.ts_elementary_streams[15], [0x10f, 0x1b]);
        assert_eq!(st.modcod, 0, "nothing past the array was touched");
    }
}
