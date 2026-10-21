//! fifo.c and udp.c: where the status lines and the transport stream go.
//!
//! Linux keeps the C's named FIFOs exactly: the consumer creates the FIFO,
//! longmynd opens it O_WRONLY|O_NONBLOCK, a missing path is fatal at start,
//! "nobody reading yet" (ENXIO) is retried at the next write, a reader
//! going away (EPIPE) closes it until then. A regular file works too, as it
//! does with the C.
//!
//! Windows has no named FIFOs in the filesystem. There a FIFO path names a
//! named pipe, `\\.\pipe\<name>`, created by the consumer (the pipe server)
//! and opened by longmynd as a client - the same roles as on Linux, so a
//! consumer ports by swapping mkfifo+open for CreateNamedPipe+
//! ConnectNamedPipe. A path that does not already start with `\\` is taken
//! as the pipe name, so the defaults become `\\.\pipe\longmynd_main_ts` and
//! `\\.\pipe\longmynd_main_status`. "No pipe server yet" is the ENXIO case:
//! not ready, retried. UDP (-i/-I) works the same on both and needs no
//! consumer changes at all.

use std::fs::File;
use std::io::{ErrorKind, Write};
use std::net::{SocketAddrV4, UdpSocket};

use crate::cnum::{inet_addr, strerror};
use crate::errors::*;

/// Where the status lines go: fifo_status_write / udp_status_write.
pub trait StatusSink: Send {
    /// `$<message>,<data as %i>\n` (the C passes the uint32_t to %i, so it
    /// prints signed).
    fn write(&mut self, message: u8, data: u32, ready: &mut bool) -> u8;
    /// `$<message>,<string>\n`
    fn write_str(&mut self, message: u8, data: &[u8], ready: &mut bool) -> u8;
    /// fifo_status_init() again after the reader went away; UDP never asks.
    fn reopen(&mut self, ready: &mut bool) -> u8;
}

/// Where the TS goes: fifo_ts_write / udp_ts_write.
pub trait TsSink: Send {
    fn write(&mut self, buffer: &[u8], ready: &mut bool) -> u8;
    fn reopen(&mut self, ready: &mut bool) -> u8;
    /// 510-byte pieces dropped so far because the reader was not keeping up.
    fn dropped(&self) -> u32 {
        0
    }
}

fn status_line(message: u8, data: u32) -> Vec<u8> {
    format!("${},{}\n", message, data as i32).into_bytes()
}

fn status_string_line(message: u8, data: &[u8]) -> Vec<u8> {
    // %s stops at the first NUL
    let s = match data.iter().position(|&b| b == 0) {
        Some(n) => &data[..n],
        None => data,
    };
    let mut v = format!("${},", message).into_bytes();
    v.extend_from_slice(s);
    v.push(b'\n');
    v
}

// ---- FIFOs ------------------------------------------------------------------

/// A FIFO longmynd writes to: the path, and the open file once a reader
/// has appeared.
pub struct Fifo {
    path: Vec<u8>,
    file: Option<File>,
}

#[cfg(unix)]
fn os_path(path: &[u8]) -> std::path::PathBuf {
    use std::os::unix::ffi::OsStrExt;
    std::path::PathBuf::from(std::ffi::OsStr::from_bytes(path))
}

#[cfg(windows)]
fn os_path(path: &[u8]) -> std::path::PathBuf {
    let s = String::from_utf8_lossy(path).into_owned();
    if s.starts_with("\\\\") {
        std::path::PathBuf::from(s)
    } else {
        std::path::PathBuf::from(format!("\\\\.\\pipe\\{}", s))
    }
}

enum OpenResult {
    Opened(File),
    NoReader,
    Failed(std::io::Error),
}

#[cfg(unix)]
fn open_fifo(path: &std::path::Path) -> OpenResult {
    use std::os::unix::fs::OpenOptionsExt;
    // "First check the file exists": a missing FIFO is an error, unlike one
    // with nobody reading it yet.
    if let Err(e) = std::fs::metadata(path) {
        return OpenResult::Failed(e);
    }
    match std::fs::OpenOptions::new().write(true).custom_flags(libc::O_NONBLOCK).open(path) {
        Ok(f) => OpenResult::Opened(f),
        Err(e) if e.raw_os_error() == Some(libc::ENXIO) => OpenResult::NoReader,
        Err(e) => OpenResult::Failed(e),
    }
}

#[cfg(windows)]
fn open_fifo(path: &std::path::Path) -> OpenResult {
    // ERROR_FILE_NOT_FOUND: no server has created the pipe yet;
    // ERROR_PIPE_BUSY (231): it exists but every instance is connected.
    // Both are "nobody to write to yet".
    match std::fs::OpenOptions::new().write(true).open(path) {
        Ok(f) => OpenResult::Opened(f),
        Err(e) if e.kind() == ErrorKind::NotFound || e.raw_os_error() == Some(231) => OpenResult::NoReader,
        Err(e) => OpenResult::Failed(e),
    }
}

impl Fifo {
    /// fifo_init(): Ok with `ready` false when nobody is reading yet.
    pub fn init(path: &[u8], ready: &mut bool) -> Result<Fifo, u8> {
        let mut f = Fifo { path: path.to_vec(), file: None };
        let err = f.open(ready);
        if err == ERROR_NONE {
            Ok(f)
        } else {
            Err(err)
        }
    }

    fn open(&mut self, ready: &mut bool) -> u8 {
        let mut err = ERROR_NONE;
        *ready = false;
        let shown = String::from_utf8_lossy(&self.path).into_owned();
        match open_fifo(&os_path(&self.path)) {
            OpenResult::Opened(f) => {
                println!("      Status: opened fifo ok");
                self.file = Some(f);
                *ready = true;
            }
            OpenResult::NoReader => self.file = None,
            OpenResult::Failed(e) => {
                println!("ERROR: Failed to open fifo {} (error: {})", shown, strerror(&e));
                self.file = None;
                err = ERROR_OPEN_TS_FIFO;
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: fifo init");
        }
        err
    }

    fn raw_write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        match self.file.as_mut() {
            Some(f) => f.write(buf),
            None => Err(std::io::Error::from(ErrorKind::BrokenPipe)),
        }
    }

    /// The C's error handling after a short write, by errno.
    fn classify(e: &std::io::Error) -> WriteFailure {
        if e.kind() == ErrorKind::BrokenPipe {
            WriteFailure::Pipe
        } else if e.kind() == ErrorKind::WouldBlock {
            WriteFailure::Again
        } else {
            WriteFailure::Other
        }
    }

    fn write_status_line(&mut self, line: &[u8], ready: &mut bool, what: &str) -> u8 {
        let mut err = ERROR_NONE;
        match self.write_complete(line) {
            Ok(()) => {}
            Err(e) => match Self::classify(&e) {
                WriteFailure::Pipe => {
                    println!("WARNING: broken status fifo");
                    *ready = false;
                }
                WriteFailure::Again => {} // dropped, as the C does
                WriteFailure::Other => {
                    println!("ERROR: status fifo {}write (error: {})", what, strerror(&e));
                    err = ERROR_TS_FIFO_WRITE;
                }
            },
        }
        err
    }

    /// One write(2), as the C makes. A short write that is not an error
    /// (which the C would misread through a stale errno) has its remainder
    /// written too.
    fn write_complete(&mut self, mut buf: &[u8]) -> std::io::Result<()> {
        let mut first = true;
        while !buf.is_empty() {
            match self.raw_write(buf) {
                Ok(0) => return Err(std::io::Error::from(ErrorKind::WriteZero)),
                Ok(n) => buf = &buf[n..],
                Err(e) if !first && e.kind() == ErrorKind::WouldBlock => continue,
                Err(e) => return Err(e),
            }
            first = false;
        }
        Ok(())
    }
}

enum WriteFailure {
    Pipe,
    Again,
    Other,
}

pub struct FifoStatus(pub Fifo);

impl StatusSink for FifoStatus {
    fn write(&mut self, message: u8, data: u32, ready: &mut bool) -> u8 {
        let err = self.0.write_status_line(&status_line(message, data), ready, "");
        if err != ERROR_NONE {
            println!("ERROR: fifo status write");
        }
        err
    }
    fn write_str(&mut self, message: u8, data: &[u8], ready: &mut bool) -> u8 {
        let err = self.0.write_status_line(&status_string_line(message, data), ready, "string ");
        if err != ERROR_NONE {
            println!("ERROR: fifo status string write");
        }
        err
    }
    fn reopen(&mut self, ready: &mut bool) -> u8 {
        self.0.open(ready)
    }
}

/// The TS FIFO, and how many pieces it has dropped (the C drops them
/// without a word; --extra-status reports the count as $31).
pub struct FifoTs(pub Fifo, pub u32);

impl TsSink for FifoTs {
    /// fifo_ts_write(): the buffer is a run of FTDI USB packets with each
    /// packet's 2 status bytes still in it every 512 bytes (the first pair
    /// already stripped), so it goes out as 510-byte writes skipping 2.
    /// As in the C, a write that would block drops that 510-byte piece and
    /// carries on with the next.
    fn write(&mut self, buffer: &[u8], ready: &mut bool) -> u8 {
        let mut err = ERROR_NONE;
        let len = buffer.len() as i32;
        let mut remaining_len: i32 = len;
        while remaining_len > 0 {
            let start = (len - remaining_len) as usize;
            let write_size: usize;
            let ret;
            if remaining_len > 510 {
                write_size = 510;
                ret = self.0.write_complete(&buffer[start..start + write_size]);
                remaining_len -= 512;
            } else {
                write_size = remaining_len as usize;
                ret = self.0.write_complete(&buffer[start..start + write_size]);
                remaining_len -= write_size as i32;
            }
            if let Err(e) = ret {
                match Fifo::classify(&e) {
                    WriteFailure::Pipe => {
                        println!("WARNING: broken ts fifo");
                        *ready = false;
                    }
                    WriteFailure::Again => {
                        self.1 = self.1.wrapping_add(1);
                        continue;
                    }
                    WriteFailure::Other => {
                        println!("ERROR: ts fifo write (error: {})", strerror(&e));
                        err = ERROR_TS_FIFO_WRITE;
                    }
                }
                break;
            }
        }
        if err == ERROR_NONE && *ready && remaining_len != 0 {
            println!("ERROR: ts fifo write incorrect number of bytes");
            err = ERROR_TS_FIFO_WRITE;
        }
        if err != ERROR_NONE {
            println!("ERROR: fifo ts write");
        }
        err
    }
    fn reopen(&mut self, ready: &mut bool) -> u8 {
        self.0.open(ready)
    }
    fn dropped(&self) -> u32 {
        self.1
    }
}

// ---- UDP ----------------------------------------------------------------------

/// A UDP destination: an unbound IPv4 socket and the address from
/// inet_addr(), as the C has.
pub struct Udp {
    socket: UdpSocket,
    dest: SocketAddrV4,
}

impl Udp {
    /// udp_init()
    pub fn init(udp_ip: &[u8], udp_port: i32) -> Result<Udp, u8> {
        println!("Flow: UDP Init");
        match UdpSocket::bind("0.0.0.0:0") {
            Ok(socket) => Ok(Udp { socket, dest: SocketAddrV4::new(inet_addr(udp_ip), udp_port as u16) }),
            Err(_) => {
                println!("ERROR: socket creation failed");
                println!("ERROR: UDP init");
                Err(ERROR_UDP_SOCKET_OPEN)
            }
        }
    }

    fn send(&self, data: &[u8]) {
        // sendto()'s result is ignored by the C
        let _ = self.socket.send_to(data, self.dest);
    }
}

pub struct UdpStatus(pub Udp);

impl StatusSink for UdpStatus {
    fn write(&mut self, message: u8, data: u32, _ready: &mut bool) -> u8 {
        self.0.send(&status_line(message, data));
        ERROR_NONE
    }
    fn write_str(&mut self, message: u8, data: &[u8], _ready: &mut bool) -> u8 {
        self.0.send(&status_string_line(message, data));
        ERROR_NONE
    }
    fn reopen(&mut self, _ready: &mut bool) -> u8 {
        ERROR_NONE
    }
}

pub struct UdpTs(pub Udp);

impl TsSink for UdpTs {
    /// udp_ts_write(): the same 510-of-every-512 framing as the FIFO, one
    /// datagram per piece.
    fn write(&mut self, buffer: &[u8], _ready: &mut bool) -> u8 {
        let mut err = ERROR_NONE;
        let len = buffer.len() as i32;
        let mut remaining_len: i32 = len;
        while remaining_len > 0 {
            let start = (len - remaining_len) as usize;
            if remaining_len > 510 {
                self.0.send(&buffer[start..start + 510]);
                remaining_len -= 512;
            } else {
                self.0.send(&buffer[start..start + remaining_len as usize]);
                remaining_len = 0;
            }
        }
        if remaining_len != 0 {
            println!("ERROR: UDP socket write incorrect number of bytes");
            err = ERROR_UDP_WRITE;
        }
        if err != ERROR_NONE {
            println!("ERROR: UDP socket ts write");
        }
        err
    }
    fn reopen(&mut self, _ready: &mut bool) -> u8 {
        ERROR_NONE
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;

    fn tmpdir(tag: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("longmynd-rs-{}-{}", tag, std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn mkfifo(path: &std::path::Path) {
        let ok = std::process::Command::new("mkfifo").arg(path).status().unwrap().success();
        assert!(ok);
    }

    #[test]
    fn status_lines_print_data_signed() {
        assert_eq!(status_line(12, (-35i32) as u32), b"$12,-35\n");
        assert_eq!(status_line(6, 437123), b"$6,437123\n");
        assert_eq!(status_string_line(13, b"CALL\0junk"), b"$13,CALL\n");
    }

    #[test]
    fn missing_fifo_is_fatal() {
        let d = tmpdir("missing");
        let mut ready = true;
        let r = Fifo::init(d.join("nope").as_os_str().to_str().unwrap().as_bytes(), &mut ready);
        assert_eq!(r.err(), Some(ERROR_OPEN_TS_FIFO));
        assert!(!ready);
    }

    #[test]
    fn fifo_without_reader_is_not_ready_then_opens() {
        let d = tmpdir("noreader");
        let p = d.join("status");
        mkfifo(&p);
        let mut ready = true;
        let mut f = FifoStatus(Fifo::init(p.to_str().unwrap().as_bytes(), &mut ready).unwrap());
        assert!(!ready, "nobody is reading, so not ready");
        // a reader appears
        let reader = std::fs::OpenOptions::new()
            .read(true)
            .custom_flags_nonblock()
            .open(&p)
            .unwrap();
        assert_eq!(f.reopen(&mut ready), ERROR_NONE);
        assert!(ready);
        assert_eq!(f.write(1, 3, &mut ready), ERROR_NONE);
        let mut got = vec![0u8; 64];
        let n = std::io::Read::read(&mut &reader, &mut got).unwrap();
        assert_eq!(&got[..n], b"$1,3\n");
        // the reader goes away: EPIPE, not ready, no error
        drop(reader);
        assert_eq!(f.write(1, 4, &mut ready), ERROR_NONE);
        assert!(!ready);
    }

    trait NonBlock {
        fn custom_flags_nonblock(&mut self) -> &mut Self;
    }
    impl NonBlock for std::fs::OpenOptions {
        fn custom_flags_nonblock(&mut self) -> &mut Self {
            use std::os::unix::fs::OpenOptionsExt;
            self.custom_flags(libc::O_NONBLOCK)
        }
    }

    #[test]
    fn ts_fifo_skips_the_ftdi_status_bytes() {
        let d = tmpdir("tsskip");
        let p = d.join("ts");
        std::fs::write(&p, b"").unwrap();
        let mut ready = false;
        let mut f = FifoTs(Fifo::init(p.to_str().unwrap().as_bytes(), &mut ready).unwrap(), 0);
        assert!(ready, "a regular file opens at once, as with the C");
        // 510 data, 2 status, 100 data
        let mut buf = vec![0xAAu8; 510];
        buf.extend_from_slice(&[0x31, 0x60]);
        buf.extend(std::iter::repeat(0xBB).take(100));
        assert_eq!(f.write(&buf, &mut ready), ERROR_NONE);
        let out = std::fs::read(&p).unwrap();
        assert_eq!(out.len(), 610);
        assert!(out[..510].iter().all(|&b| b == 0xAA));
        assert!(out[510..].iter().all(|&b| b == 0xBB));
    }

    #[test]
    fn a_full_ts_fifo_drops_pieces_and_counts_them() {
        let d = tmpdir("tsdrop");
        let p = d.join("ts");
        mkfifo(&p);
        // a reader that never reads: the pipe fills (64 KiB on Linux)
        let _reader = std::fs::OpenOptions::new().read(true).custom_flags_nonblock().open(&p).unwrap();
        let mut ready = false;
        let mut f = FifoTs(Fifo::init(p.to_str().unwrap().as_bytes(), &mut ready).unwrap(), 0);
        assert!(ready);
        let chunk = vec![0x47u8; 20 * 512 - 2];
        for _ in 0..20 {
            // the C's behaviour, kept: no error, the piece is just lost
            assert_eq!(f.write(&chunk, &mut ready), ERROR_NONE);
        }
        // 20 x 20 pieces offered, 64 KiB fits: 128 pieces of 510 written
        let written = 65536 / 510;
        assert!(f.dropped() >= 400 - written as u32 - 1 && f.dropped() < 400, "{}", f.dropped());
    }
}
