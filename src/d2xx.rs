//! A second USB backend: FTDI's own driver through its D2XX library
//! (`ftd2xx.dll` on Windows), behind the same libusb-shaped traits as
//! [`crate::usb::RusbBackend`].
//!
//! Why: on Windows a MiniTiouner is bound to FTDI's driver, which libusb
//! cannot use; the libusb route needs the driver swapped to WinUSB, and
//! that takes the tuner away from F6DZP's MiniTioune. Through D2XX the
//! tuner works as Windows installed it, and the two programs can take
//! turns with it.
//!
//! The rest of longmynd still speaks libusb: it opens the device twice (one
//! handle per interface), sends FTDI vendor control requests, and reads
//! bulk data with the FTDI's two status bytes at the head of every 512-byte
//! USB packet. This module maps each of those onto D2XX, keeping what
//! reaches the NIM identical:
//!
//! - **Devices.** D2XX lists an FT2232H as two devices, interface A and B
//!   (descriptions "... A" / "... B"). A handle here is the chip; claiming
//!   interface 0 or 1 opens A or B with `FT_OpenEx` (by location, else
//!   serial, else description). Both of longmynd's handles to one tuner
//!   share the chip, so a request addressed to an interface reaches it
//!   whichever handle sent it - as on the USB.
//! - **Control requests** go to the interface their wIndex names (1 = A,
//!   2 = B), exactly as on the wire. The C sends all of them with wIndex 1,
//!   so the TS handle's purge/latency/bit-mode requests go to A again, and
//!   B is left as the EEPROM set it; that is reproduced, not corrected.
//!   SIO_RESET 0/1/2 is `FT_ResetDevice` / `FT_Purge(RX)` / `FT_Purge(TX)`,
//!   SET_LATENCY_TIMER is `FT_SetLatencyTimer`, SET_BITMODE is
//!   `FT_SetBitMode(value & 0xff, value >> 8)`.
//! - **Bulk OUT** (0x02, the MPSSE commands) is `FT_Write` on A.
//! - **Bulk IN** (0x81 replies on A, 0x83 TS on B): D2XX returns payload
//!   only - it strips the status bytes - and blocks until it has as many
//!   bytes as asked for. longmynd expects libusb's shape: whatever has
//!   arrived, framed, or after the latency timer a packet of status bytes
//!   alone. So: `FT_Read` of one byte with the read timeout set to the
//!   FTDI latency timer (16 ms) - nothing in that time is the empty
//!   answer - then `FT_GetQueueStatus` and an `FT_Read` of what is already
//!   there, up to what the libusb buffer would have held (510 per 512);
//!   then the status pair is put back in front of every 510 bytes, so the
//!   I2C layer and the TS writer see byte for byte what they see from
//!   libusb. The status bytes put back are an idle FT2232H's (0x32 0x60);
//!   longmynd never looks at them.
//!
//! What cannot be byte-compared with the C: the call layer itself (D2XX
//! calls are not libusb calls), and anything D2XX does inside the driver.
//! The equivalence harness runs scenarios through this module over a
//! D2XX-shaped simulator and compares everything above it - MPSSE bytes,
//! I2C transactions, sleeps, all with their virtual times, stdout, status,
//! TS, exit code (equiv/tests/d2xx.rs).
//!
//! `ftd2xx.dll` is loaded at run time ([`dll`]), so the program starts
//! without it; `--usb-backend auto` (the default) uses D2XX on Windows when
//! the DLL loads and finds a tuner, and libusb otherwise.

use std::sync::{Arc, Mutex, Weak};

use crate::usb::*;

/// FT_STATUS
pub type FtStatus = u32;
pub const FT_OK: FtStatus = 0;
pub const FT_INVALID_HANDLE: FtStatus = 1;
pub const FT_DEVICE_NOT_FOUND: FtStatus = 2;
pub const FT_DEVICE_NOT_OPENED: FtStatus = 3;
pub const FT_IO_ERROR: FtStatus = 4;
pub const FT_INSUFFICIENT_RESOURCES: FtStatus = 5;
pub const FT_INVALID_PARAMETER: FtStatus = 6;
pub const FT_NOT_SUPPORTED: FtStatus = 17;

pub const FT_OPEN_BY_SERIAL_NUMBER: u32 = 1;
pub const FT_OPEN_BY_DESCRIPTION: u32 = 2;
pub const FT_OPEN_BY_LOCATION: u32 = 4;

pub const FT_PURGE_RX: u32 = 1;
pub const FT_PURGE_TX: u32 = 2;

/// FT_DEVICE_LIST_INFO_NODE.Flags: open (by this or another process)
pub const FT_FLAGS_OPENED: u32 = 1;

/// FT_DEVICE values that matter here (FT_DEVICE_2232H is a MiniTiouner).
pub const FT_DEVICE_2232C: u32 = 4;
pub const FT_DEVICE_2232H: u32 = 6;
pub const FT_DEVICE_4232H: u32 = 7;
pub const FT_DEVICE_232H: u32 = 8;

/// What a D2XX handle is to us: an opaque number.
pub type FtHandle = usize;

/// The status pair put back in front of every 510 bytes: what an idle
/// FT2232H sends (the simulator's too). longmynd skips them unread.
pub const STATUS_BYTES: [u8; 2] = [0x32, 0x60];

/// How long an IN request waits for a first byte before answering "status
/// bytes only", as the FTDI does when its latency timer (16 ms, which
/// longmynd sets) expires with nothing to send.
pub const EMPTY_READ_MS: u32 = 16;

/// FT_SetUSBParameters for the TS interface: the largest IN transfer
/// FTDI allows, so the driver keeps up at the highest symbol rates.
pub const TS_USB_TRANSFER_SIZE: u32 = 65536;

/// One entry of FT_GetDeviceInfoDetail.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FtDeviceInfo {
    pub flags: u32,
    pub ft_type: u32,
    /// (VID << 16) | PID
    pub id: u32,
    pub loc_id: u32,
    pub serial: String,
    pub description: String,
}

/// How to name an interface to FT_OpenEx.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OpenBy {
    Location(u32),
    Serial(String),
    Description(String),
}

/// The D2XX calls this backend makes, in D2XX's own terms. The real one is
/// [`dll::DllApi`]; the equivalence harness implements it over its
/// simulator.
pub trait D2xxApi: Send + Sync {
    fn create_device_info_list(&self) -> Result<u32, FtStatus>;
    fn get_device_info_detail(&self, index: u32) -> Result<FtDeviceInfo, FtStatus>;
    fn open_ex(&self, by: &OpenBy) -> Result<FtHandle, FtStatus>;
    fn close(&self, h: FtHandle) -> FtStatus;
    fn reset_device(&self, h: FtHandle) -> FtStatus;
    fn purge(&self, h: FtHandle, mask: u32) -> FtStatus;
    fn set_latency_timer(&self, h: FtHandle, ms: u8) -> FtStatus;
    fn set_bit_mode(&self, h: FtHandle, mask: u8, mode: u8) -> FtStatus;
    fn set_usb_parameters(&self, h: FtHandle, in_size: u32, out_size: u32) -> FtStatus;
    fn set_timeouts(&self, h: FtHandle, read_ms: u32, write_ms: u32) -> FtStatus;
    fn get_queue_status(&self, h: FtHandle) -> Result<u32, FtStatus>;
    /// FT_Read: (status, bytes read).
    fn read(&self, h: FtHandle, buf: &mut [u8]) -> (FtStatus, usize);
    /// FT_Write: (status, bytes written).
    fn write(&self, h: FtHandle, buf: &[u8]) -> (FtStatus, usize);
}

/// An FT_STATUS as the libusb error longmynd would have seen in its place.
pub fn libusb_code(st: FtStatus) -> i32 {
    match st {
        FT_OK => LIBUSB_SUCCESS,
        FT_INVALID_HANDLE | FT_DEVICE_NOT_FOUND => LIBUSB_ERROR_NO_DEVICE,
        FT_DEVICE_NOT_OPENED => LIBUSB_ERROR_BUSY,
        FT_IO_ERROR => LIBUSB_ERROR_IO,
        FT_INSUFFICIENT_RESOURCES => LIBUSB_ERROR_NO_MEM,
        FT_INVALID_PARAMETER => LIBUSB_ERROR_INVALID_PARAM,
        FT_NOT_SUPPORTED => LIBUSB_ERROR_NOT_SUPPORTED,
        _ => LIBUSB_ERROR_OTHER,
    }
}

/// bcdDevice for an FT_DEVICE type (the FTDI's defaults; D2XX does not
/// report the descriptor itself). longmynd only looks at it on a
/// PicoTuner, which D2XX never lists.
fn bcd_for(ft_type: u32) -> u16 {
    match ft_type {
        FT_DEVICE_2232C => 0x0500,
        FT_DEVICE_2232H => 0x0700,
        FT_DEVICE_4232H => 0x0800,
        FT_DEVICE_232H => 0x0900,
        _ => 0x0600,
    }
}

/// One FTDI chip seen through D2XX: its two interfaces' list entries and,
/// once claimed, their handles. Closed when the last longmynd handle to it
/// goes.
pub struct Chip {
    api: Arc<dyn D2xxApi>,
    entries: [FtDeviceInfo; 2],
    handles: Mutex<[Option<FtHandle>; 2]>,
}

impl Drop for Chip {
    fn drop(&mut self) {
        let hs = self.handles.lock().unwrap_or_else(|e| e.into_inner());
        for h in hs.iter().flatten() {
            self.api.close(*h);
        }
    }
}

impl Chip {
    fn open_by(e: &FtDeviceInfo) -> OpenBy {
        if e.loc_id != 0 {
            OpenBy::Location(e.loc_id)
        } else if !e.serial.is_empty() {
            OpenBy::Serial(e.serial.clone())
        } else {
            OpenBy::Description(e.description.clone())
        }
    }

    fn handle(&self, iface: usize) -> Option<FtHandle> {
        self.handles.lock().unwrap()[iface]
    }
}

/// The D2XX backend.
#[derive(Clone)]
pub struct D2xxBackend {
    api: Arc<dyn D2xxApi>,
    /// Chips this process has open: the second open of a tuner (for its
    /// TS interface) must find the chip the first one claimed, which D2XX
    /// now lists as "opened" with no details.
    chips: Arc<Mutex<Vec<Weak<Chip>>>>,
}

impl D2xxBackend {
    pub fn new(api: Arc<dyn D2xxApi>) -> Self {
        D2xxBackend { api, chips: Arc::new(Mutex::new(Vec::new())) }
    }

    /// The system's D2XX library, loaded now.
    pub fn load() -> Result<Self, String> {
        dll::DllApi::load().map(|a| D2xxBackend::new(Arc::new(a)))
    }

    fn list(&self) -> Result<Vec<FtDeviceInfo>, FtStatus> {
        let n = self.api.create_device_info_list()?;
        (0..n).map(|i| self.api.get_device_info_detail(i)).collect()
    }

    /// The chips in D2XX's list, in its order: an interface-A entry and
    /// the B entry after it. Ours (already open here) are recognised by
    /// location / serial; ones another program has open are left out, and
    /// counted.
    fn chips(&self) -> Result<(Vec<Arc<Chip>>, usize), FtStatus> {
        let list = self.list()?;
        let mut mine = self.chips.lock().unwrap();
        mine.retain(|w| w.strong_count() > 0);
        let live: Vec<Arc<Chip>> = mine.iter().filter_map(Weak::upgrade).collect();
        let mut out = Vec::new();
        let mut busy = 0;
        let mut i = 0;
        while i < list.len() {
            let e = &list[i];
            let ours = live.iter().find(|c| {
                let a = &c.entries[0];
                (a.loc_id != 0 && a.loc_id == e.loc_id) || (!a.serial.is_empty() && a.serial == e.serial)
            });
            if let Some(c) = ours {
                out.push(c.clone());
                i += 2;
                continue;
            }
            if e.flags & FT_FLAGS_OPENED != 0 {
                busy += 1;
                i += 1;
                continue;
            }
            let is_a = e.description.ends_with(" A") || e.serial.ends_with('A');
            if is_a {
                if let Some(b) = list.get(i + 1) {
                    if b.id == e.id && b.flags & FT_FLAGS_OPENED == 0 {
                        let chip = Arc::new(Chip {
                            api: self.api.clone(),
                            entries: [e.clone(), b.clone()],
                            handles: Mutex::new([None, None]),
                        });
                        mine.push(Arc::downgrade(&chip));
                        out.push(chip);
                        i += 2;
                        continue;
                    }
                }
            }
            i += 1;
        }
        Ok((out, busy))
    }
}

impl UsbBackend for D2xxBackend {
    fn init(&self) -> Result<Box<dyn UsbContext>, i32> {
        Ok(Box::new(D2xxCtx(self.clone())))
    }
}

struct D2xxCtx(D2xxBackend);

impl UsbContext for D2xxCtx {
    /// No library logging to set: D2XX has none.
    fn set_log_level_info(&mut self) {}

    fn open_device_with_vid_pid(&mut self, vid: u16, pid: u16) -> Option<Box<dyn UsbHandle>> {
        let want = ((vid as u32) << 16) | pid as u32;
        let (chips, busy) = self.0.chips().ok()?;
        match chips.into_iter().find(|c| c.entries[0].id == want) {
            Some(chip) => Some(Box::new(D2xxHandle { chip })),
            None => {
                if busy > 0 && vid == crate::ftdi::FTDI_VID {
                    // the one thing worth saying that libusb could not: a
                    // tuner is there, but another program has it
                    println!("      Status: D2XX: {} FTDI interface(s) open in another program (MiniTioune?)", busy);
                }
                None
            }
        }
    }

    /// `-u BUS DEVICE` has no meaning to D2XX: its tuners are numbered here
    /// as bus 1, devices 1, 2, ... in D2XX's list order.
    fn get_device_list(&mut self) -> Result<Vec<Box<dyn UsbDevice>>, i32> {
        let (chips, _) = self.0.chips().map_err(libusb_code)?;
        Ok(chips
            .into_iter()
            .enumerate()
            .map(|(i, chip)| Box::new(D2xxDevice { chip, addr: (i + 1) as u8 }) as Box<dyn UsbDevice>)
            .collect())
    }
}

fn descriptor(chip: &Chip) -> DeviceDescriptor {
    let e = &chip.entries[0];
    DeviceDescriptor { id_vendor: (e.id >> 16) as u16, id_product: e.id as u16, bcd_device: bcd_for(e.ft_type) }
}

struct D2xxDevice {
    chip: Arc<Chip>,
    addr: u8,
}

impl UsbDevice for D2xxDevice {
    fn bus_number(&self) -> u8 {
        1
    }
    fn device_address(&self) -> u8 {
        self.addr
    }
    fn device_descriptor(&self) -> Result<DeviceDescriptor, i32> {
        Ok(descriptor(&self.chip))
    }
    fn open(&self) -> Result<Box<dyn UsbHandle>, i32> {
        Ok(Box::new(D2xxHandle { chip: self.chip.clone() }))
    }
}

struct D2xxHandle {
    chip: Arc<Chip>,
}

/// Payload a libusb read of `len` bytes could carry: 510 of every 512.
fn payload_room(len: usize) -> usize {
    (len / 512) * 510 + (len % 512).saturating_sub(2)
}

impl D2xxHandle {
    fn iface_for_endpoint(ep: u8) -> usize {
        match ep {
            0x02 | 0x81 => 0,
            _ => 1,
        }
    }

    /// A libusb-shaped IN transfer on one interface (module comment).
    fn read_framed(&self, h: FtHandle, data: &mut [u8]) -> (i32, usize) {
        let api = &self.chip.api;
        let room = payload_room(data.len());
        let mut payload = vec![0u8; room.max(1)];
        let (st, n) = api.read(h, &mut payload[..1]);
        if st != FT_OK {
            return (libusb_code(st), 0);
        }
        let mut got = n;
        if got == 1 && room > 1 {
            match api.get_queue_status(h) {
                Ok(q) if q > 0 => {
                    let take = (q as usize).min(room - 1);
                    let (st, n) = api.read(h, &mut payload[1..1 + take]);
                    if st != FT_OK {
                        return (libusb_code(st), 0);
                    }
                    got += n;
                }
                Ok(_) => {}
                Err(st) => return (libusb_code(st), 0),
            }
        }
        // the status pair in front of every 510 bytes, as the USB had it
        let mut out = 0;
        let mut pieces = payload[..got].chunks(510).peekable();
        if pieces.peek().is_none() {
            data[..2].copy_from_slice(&STATUS_BYTES);
            return (LIBUSB_SUCCESS, 2);
        }
        for piece in pieces {
            data[out..out + 2].copy_from_slice(&STATUS_BYTES);
            data[out + 2..out + 2 + piece.len()].copy_from_slice(piece);
            out += 2 + piece.len();
        }
        (LIBUSB_SUCCESS, out)
    }
}

impl UsbHandle for D2xxHandle {
    /// D2XX has no kernel driver to detach: the driver is FTDI's, and it
    /// is the one being used.
    fn kernel_driver_active(&mut self, _interface: u8) -> i32 {
        0
    }
    fn detach_kernel_driver(&mut self, _interface: u8) -> i32 {
        LIBUSB_SUCCESS
    }

    /// Open the interface's D2XX device. A second claim of an interface
    /// this process already has open is a success, as libusb's is.
    fn claim_interface(&mut self, interface: u8) -> i32 {
        let i = interface as usize;
        if i > 1 {
            return LIBUSB_ERROR_NOT_FOUND;
        }
        if self.chip.handle(i).is_some() {
            return LIBUSB_SUCCESS;
        }
        let api = &self.chip.api;
        let h = match api.open_ex(&Chip::open_by(&self.chip.entries[i])) {
            Ok(h) => h,
            Err(st) => return libusb_code(st),
        };
        // reads wait EMPTY_READ_MS for a first byte (read_framed); writes
        // as long as the C ever waits for one
        let mut st = api.set_timeouts(h, EMPTY_READ_MS, crate::ftdi_usb::USB_TIMEOUT);
        if st == FT_OK && i == 1 {
            st = api.set_usb_parameters(h, TS_USB_TRANSFER_SIZE, 0);
        }
        if st != FT_OK {
            api.close(h);
            return libusb_code(st);
        }
        self.chip.handles.lock().unwrap()[i] = Some(h);
        LIBUSB_SUCCESS
    }

    fn device_descriptor(&mut self) -> Result<DeviceDescriptor, i32> {
        Ok(descriptor(&self.chip))
    }

    fn bulk_transfer(&mut self, endpoint: u8, data: &mut [u8], _timeout_ms: u32) -> (i32, usize) {
        let Some(h) = self.chip.handle(Self::iface_for_endpoint(endpoint)) else {
            return (LIBUSB_ERROR_IO, 0);
        };
        if endpoint & 0x80 == 0 {
            let (st, n) = self.chip.api.write(h, data);
            (libusb_code(st), n)
        } else {
            self.read_framed(h, data)
        }
    }

    fn control_transfer(&mut self, request_type: u8, request: u8, value: u16, index: u16, _timeout_ms: u32) -> i32 {
        // FTDI vendor requests to the device, wIndex naming the interface
        if request_type != 0x40 || !(1..=2).contains(&index) {
            return LIBUSB_ERROR_NOT_SUPPORTED;
        }
        let Some(h) = self.chip.handle(index as usize - 1) else {
            return LIBUSB_ERROR_IO;
        };
        let api = &self.chip.api;
        let st = match (request, value) {
            (0x00, 0) => api.reset_device(h),
            (0x00, 1) => api.purge(h, FT_PURGE_RX),
            (0x00, 2) => api.purge(h, FT_PURGE_TX),
            (0x09, v) => api.set_latency_timer(h, v as u8),
            (0x0b, v) => api.set_bit_mode(h, (v & 0xff) as u8, (v >> 8) as u8),
            _ => return LIBUSB_ERROR_NOT_SUPPORTED,
        };
        libusb_code(st)
    }
}

/// The D2XX library itself, loaded at run time.
///
/// This is longmynd-rs's only `unsafe`: it calls a C library through
/// function pointers. Every call passes
/// pointers to buffers it owns, with their true lengths, as the D2XX
/// Programmer's Guide specifies, and the library is kept loaded for as
/// long as the pointers exist.
pub mod dll {
    use super::*;
    use std::ffi::c_void;

    type Handle = *mut c_void;
    type CreateList = unsafe extern "system" fn(*mut u32) -> FtStatus;
    type InfoDetail = unsafe extern "system" fn(u32, *mut u32, *mut u32, *mut u32, *mut u32, *mut u8, *mut u8, *mut Handle) -> FtStatus;
    type OpenEx = unsafe extern "system" fn(*mut c_void, u32, *mut Handle) -> FtStatus;
    type One = unsafe extern "system" fn(Handle) -> FtStatus;
    type U32 = unsafe extern "system" fn(Handle, u32) -> FtStatus;
    type U8 = unsafe extern "system" fn(Handle, u8) -> FtStatus;
    type U8U8 = unsafe extern "system" fn(Handle, u8, u8) -> FtStatus;
    type U32U32 = unsafe extern "system" fn(Handle, u32, u32) -> FtStatus;
    type Queue = unsafe extern "system" fn(Handle, *mut u32) -> FtStatus;
    type Io = unsafe extern "system" fn(Handle, *mut c_void, u32, *mut u32) -> FtStatus;

    pub struct DllApi {
        create_list: CreateList,
        info_detail: InfoDetail,
        open_ex: OpenEx,
        close: One,
        reset: One,
        purge: U32,
        latency: U8,
        bit_mode: U8U8,
        usb_params: U32U32,
        timeouts: U32U32,
        queue: Queue,
        read: Io,
        write: Io,
        // last, so the function pointers above never outlive it
        _lib: libloading::Library,
    }

    // FT_HANDLEs are plain tokens the library accepts from any thread
    // (FTDI's D2XX is thread-safe per handle, and longmynd uses each
    // interface from one thread at a time).
    unsafe impl Send for DllApi {}
    unsafe impl Sync for DllApi {}

    #[cfg(windows)]
    const NAMES: &[&str] = &["ftd2xx.dll", "ftd2xx64.dll"];
    #[cfg(not(windows))]
    const NAMES: &[&str] = &["libftd2xx.so", "libftd2xx.so.1"];

    #[cfg(windows)]
    fn open_lib(name: &str) -> Result<libloading::Library, libloading::Error> {
        // the exe's own folder, then System32 - never the current
        // directory, where anyone could have left an ftd2xx.dll
        const LOAD_LIBRARY_SEARCH_APPLICATION_DIR: u32 = 0x0000_0200;
        const LOAD_LIBRARY_SEARCH_SYSTEM32: u32 = 0x0000_0800;
        // SAFETY: loading runs the DLL's initialisation; ftd2xx.dll is
        // FTDI's driver library, found only in these two trusted places
        unsafe {
            libloading::os::windows::Library::load_with_flags(
                name,
                LOAD_LIBRARY_SEARCH_APPLICATION_DIR | LOAD_LIBRARY_SEARCH_SYSTEM32,
            )
            .map(libloading::Library::from)
        }
    }

    #[cfg(not(windows))]
    fn open_lib(name: &str) -> Result<libloading::Library, libloading::Error> {
        // SAFETY: as above; FTDI's libftd2xx from the library path
        unsafe { libloading::Library::new(name) }
    }

    impl DllApi {
        pub fn load() -> Result<DllApi, String> {
            let mut tried = Vec::new();
            for name in NAMES {
                match open_lib(name) {
                    Ok(lib) => return Self::bind(lib).map_err(|e| format!("{}: {}", name, e)),
                    Err(e) => tried.push(format!("{}: {}", name, e)),
                }
            }
            Err(tried.join("; "))
        }

        fn bind(lib: libloading::Library) -> Result<DllApi, libloading::Error> {
            // SAFETY: each symbol is looked up with the signature ftd2xx.h
            // declares for it (WINAPI = "system")
            unsafe {
                Ok(DllApi {
                    create_list: *lib.get::<CreateList>(b"FT_CreateDeviceInfoList\0")?,
                    info_detail: *lib.get::<InfoDetail>(b"FT_GetDeviceInfoDetail\0")?,
                    open_ex: *lib.get::<OpenEx>(b"FT_OpenEx\0")?,
                    close: *lib.get::<One>(b"FT_Close\0")?,
                    reset: *lib.get::<One>(b"FT_ResetDevice\0")?,
                    purge: *lib.get::<U32>(b"FT_Purge\0")?,
                    latency: *lib.get::<U8>(b"FT_SetLatencyTimer\0")?,
                    bit_mode: *lib.get::<U8U8>(b"FT_SetBitMode\0")?,
                    usb_params: *lib.get::<U32U32>(b"FT_SetUSBParameters\0")?,
                    timeouts: *lib.get::<U32U32>(b"FT_SetTimeouts\0")?,
                    queue: *lib.get::<Queue>(b"FT_GetQueueStatus\0")?,
                    read: *lib.get::<Io>(b"FT_Read\0")?,
                    write: *lib.get::<Io>(b"FT_Write\0")?,
                    _lib: lib,
                })
            }
        }
    }

    fn h(x: FtHandle) -> Handle {
        x as Handle
    }

    /// A NUL-terminated C string from a fixed buffer.
    fn cstr(b: &[u8]) -> String {
        let n = b.iter().position(|&c| c == 0).unwrap_or(b.len());
        String::from_utf8_lossy(&b[..n]).into_owned()
    }

    impl D2xxApi for DllApi {
        fn create_device_info_list(&self) -> Result<u32, FtStatus> {
            let mut n = 0u32;
            // SAFETY: one out-parameter we own
            let st = unsafe { (self.create_list)(&mut n) };
            if st == FT_OK {
                Ok(n)
            } else {
                Err(st)
            }
        }
        fn get_device_info_detail(&self, index: u32) -> Result<FtDeviceInfo, FtStatus> {
            let (mut flags, mut ty, mut id, mut loc) = (0u32, 0u32, 0u32, 0u32);
            // ftd2xx.h: serial number 16 bytes, description 64
            let mut serial = [0u8; 16];
            let mut desc = [0u8; 64];
            let mut handle: Handle = std::ptr::null_mut();
            // SAFETY: out-parameters and buffers of the sizes the API states
            let st = unsafe {
                (self.info_detail)(index, &mut flags, &mut ty, &mut id, &mut loc, serial.as_mut_ptr(), desc.as_mut_ptr(), &mut handle)
            };
            if st != FT_OK {
                return Err(st);
            }
            Ok(FtDeviceInfo { flags, ft_type: ty, id, loc_id: loc, serial: cstr(&serial), description: cstr(&desc) })
        }
        fn open_ex(&self, by: &OpenBy) -> Result<FtHandle, FtStatus> {
            let mut handle: Handle = std::ptr::null_mut();
            let st = match by {
                // SAFETY: by location the first argument is the location
                // itself, passed in the pointer's place (D2XX guide)
                OpenBy::Location(loc) => unsafe { (self.open_ex)(*loc as usize as *mut c_void, FT_OPEN_BY_LOCATION, &mut handle) },
                OpenBy::Serial(s) | OpenBy::Description(s) => {
                    let flag = if matches!(by, OpenBy::Serial(_)) { FT_OPEN_BY_SERIAL_NUMBER } else { FT_OPEN_BY_DESCRIPTION };
                    let mut c = s.as_bytes().to_vec();
                    c.push(0);
                    // SAFETY: a NUL-terminated string alive for the call
                    unsafe { (self.open_ex)(c.as_mut_ptr() as *mut c_void, flag, &mut handle) }
                }
            };
            if st == FT_OK {
                Ok(handle as FtHandle)
            } else {
                Err(st)
            }
        }
        fn close(&self, x: FtHandle) -> FtStatus {
            // SAFETY: a handle FT_OpenEx gave us, closed once (Chip::drop)
            unsafe { (self.close)(h(x)) }
        }
        fn reset_device(&self, x: FtHandle) -> FtStatus {
            // SAFETY: an open handle
            unsafe { (self.reset)(h(x)) }
        }
        fn purge(&self, x: FtHandle, mask: u32) -> FtStatus {
            // SAFETY: an open handle
            unsafe { (self.purge)(h(x), mask) }
        }
        fn set_latency_timer(&self, x: FtHandle, ms: u8) -> FtStatus {
            // SAFETY: an open handle
            unsafe { (self.latency)(h(x), ms) }
        }
        fn set_bit_mode(&self, x: FtHandle, mask: u8, mode: u8) -> FtStatus {
            // SAFETY: an open handle
            unsafe { (self.bit_mode)(h(x), mask, mode) }
        }
        fn set_usb_parameters(&self, x: FtHandle, in_size: u32, out_size: u32) -> FtStatus {
            // SAFETY: an open handle
            unsafe { (self.usb_params)(h(x), in_size, out_size) }
        }
        fn set_timeouts(&self, x: FtHandle, read_ms: u32, write_ms: u32) -> FtStatus {
            // SAFETY: an open handle
            unsafe { (self.timeouts)(h(x), read_ms, write_ms) }
        }
        fn get_queue_status(&self, x: FtHandle) -> Result<u32, FtStatus> {
            let mut n = 0u32;
            // SAFETY: an open handle and an out-parameter we own
            let st = unsafe { (self.queue)(h(x), &mut n) };
            if st == FT_OK {
                Ok(n)
            } else {
                Err(st)
            }
        }
        fn read(&self, x: FtHandle, buf: &mut [u8]) -> (FtStatus, usize) {
            let mut n = 0u32;
            // SAFETY: a buffer we own, with its true length
            let st = unsafe { (self.read)(h(x), buf.as_mut_ptr() as *mut c_void, buf.len() as u32, &mut n) };
            (st, n as usize)
        }
        fn write(&self, x: FtHandle, buf: &[u8]) -> (FtStatus, usize) {
            let mut n = 0u32;
            // SAFETY: as read; FT_Write does not write to the buffer
            let st = unsafe { (self.write)(h(x), buf.as_ptr() as *mut c_void, buf.len() as u32, &mut n) };
            (st, n as usize)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn payload_room_is_510_of_every_512() {
        assert_eq!(payload_room(4096), 8 * 510);
        assert_eq!(payload_room(10240), 20 * 510);
        assert_eq!(payload_room(600), 510 + 86);
    }

    #[test]
    fn status_codes_map_to_what_libusb_would_say() {
        assert_eq!(libusb_code(FT_OK), 0);
        assert_eq!(libusb_code(FT_DEVICE_NOT_FOUND), LIBUSB_ERROR_NO_DEVICE);
        assert_eq!(libusb_code(FT_DEVICE_NOT_OPENED), LIBUSB_ERROR_BUSY);
        assert_eq!(libusb_code(FT_IO_ERROR), LIBUSB_ERROR_IO);
        assert_eq!(libusb_code(99), LIBUSB_ERROR_OTHER);
    }
}
