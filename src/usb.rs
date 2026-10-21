//! The USB boundary: the libusb calls longmynd makes, one trait method per
//! call, in libusb's own terms (negative error codes, transferred counts).
//!
//! The port talks to the MiniTiouner only through these traits. The real
//! implementation ([`RusbBackend`]) is a thin layer over rusb, i.e. libusb;
//! the equivalence harness implements them over a simulated FT2232 and NIM
//! and records every call. Keeping the traits at libusb's granularity -
//! rather than "write an I2C register" - is what makes the recorded traces
//! comparable with the C byte for byte: the same calls, the same buffers,
//! in the same order.

pub const LIBUSB_SUCCESS: i32 = 0;
pub const LIBUSB_ERROR_IO: i32 = -1;
pub const LIBUSB_ERROR_INVALID_PARAM: i32 = -2;
pub const LIBUSB_ERROR_ACCESS: i32 = -3;
pub const LIBUSB_ERROR_NO_DEVICE: i32 = -4;
pub const LIBUSB_ERROR_NOT_FOUND: i32 = -5;
pub const LIBUSB_ERROR_BUSY: i32 = -6;
pub const LIBUSB_ERROR_TIMEOUT: i32 = -7;
pub const LIBUSB_ERROR_OVERFLOW: i32 = -8;
pub const LIBUSB_ERROR_PIPE: i32 = -9;
pub const LIBUSB_ERROR_INTERRUPTED: i32 = -10;
pub const LIBUSB_ERROR_NO_MEM: i32 = -11;
pub const LIBUSB_ERROR_NOT_SUPPORTED: i32 = -12;
pub const LIBUSB_ERROR_OTHER: i32 = -99;

/// `libusb_error_name()`, which the C prints in its TS read error.
pub fn error_name(code: i32) -> &'static str {
    match code {
        LIBUSB_SUCCESS => "LIBUSB_SUCCESS / LIBUSB_TRANSFER_COMPLETED",
        LIBUSB_ERROR_IO => "LIBUSB_ERROR_IO",
        LIBUSB_ERROR_INVALID_PARAM => "LIBUSB_ERROR_INVALID_PARAM",
        LIBUSB_ERROR_ACCESS => "LIBUSB_ERROR_ACCESS",
        LIBUSB_ERROR_NO_DEVICE => "LIBUSB_ERROR_NO_DEVICE",
        LIBUSB_ERROR_NOT_FOUND => "LIBUSB_ERROR_NOT_FOUND",
        LIBUSB_ERROR_BUSY => "LIBUSB_ERROR_BUSY",
        LIBUSB_ERROR_TIMEOUT => "LIBUSB_ERROR_TIMEOUT",
        LIBUSB_ERROR_OVERFLOW => "LIBUSB_ERROR_OVERFLOW",
        LIBUSB_ERROR_PIPE => "LIBUSB_ERROR_PIPE",
        LIBUSB_ERROR_INTERRUPTED => "LIBUSB_ERROR_INTERRUPTED",
        LIBUSB_ERROR_NO_MEM => "LIBUSB_ERROR_NO_MEM",
        LIBUSB_ERROR_NOT_SUPPORTED => "LIBUSB_ERROR_NOT_SUPPORTED",
        LIBUSB_ERROR_OTHER => "LIBUSB_ERROR_OTHER",
        _ => "**UNKNOWN**",
    }
}

/// The fields of a device descriptor longmynd looks at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceDescriptor {
    pub id_vendor: u16,
    pub id_product: u16,
    /// bcdDevice, as the raw BCD word (the PicoTuner reports 0x0101
    /// parallel TS, 0x0102 serial).
    pub bcd_device: u16,
}

/// `libusb_init()`.
pub trait UsbBackend: Send + Sync {
    fn init(&self) -> Result<Box<dyn UsbContext>, i32>;
}

/// A libusb context.
pub trait UsbContext: Send {
    /// `libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO)`.
    fn set_log_level_info(&mut self);
    /// `libusb_open_device_with_vid_pid()`; None where the C gets NULL.
    fn open_device_with_vid_pid(&mut self, vid: u16, pid: u16) -> Option<Box<dyn UsbHandle>>;
    /// `libusb_get_device_list()`; Err is the C's negative count.
    fn get_device_list(&mut self) -> Result<Vec<Box<dyn UsbDevice>>, i32>;
}

/// An entry of the device list.
pub trait UsbDevice {
    fn bus_number(&self) -> u8;
    fn device_address(&self) -> u8;
    fn device_descriptor(&self) -> Result<DeviceDescriptor, i32>;
    /// `libusb_open()`.
    fn open(&self) -> Result<Box<dyn UsbHandle>, i32>;
}

/// An open device.
pub trait UsbHandle: Send {
    /// `libusb_kernel_driver_active()`: 1, 0 or a negative error, because
    /// the C tests the raw int for truth and so calls detach on an error
    /// too (Windows answers LIBUSB_ERROR_NOT_SUPPORTED).
    fn kernel_driver_active(&mut self, interface: u8) -> i32;
    fn detach_kernel_driver(&mut self, interface: u8) -> i32;
    fn claim_interface(&mut self, interface: u8) -> i32;
    /// `libusb_get_device_descriptor(libusb_get_device(handle), ...)`.
    fn device_descriptor(&mut self) -> Result<DeviceDescriptor, i32>;
    /// `libusb_bulk_transfer()`: (result, transferred). The direction is
    /// the endpoint's bit 7; for IN the data lands at the start of `data`.
    fn bulk_transfer(&mut self, endpoint: u8, data: &mut [u8], timeout_ms: u32) -> (i32, usize);
    /// `libusb_control_transfer()` with no data stage (all longmynd sends).
    fn control_transfer(&mut self, request_type: u8, request: u8, value: u16, index: u16, timeout_ms: u32) -> i32;
}

/// The real thing: libusb through rusb.
pub struct RusbBackend;

mod real {
    use super::*;
    use rusb::UsbContext as _;
    use std::time::Duration;

    pub(super) fn code(e: rusb::Error) -> i32 {
        match e {
            rusb::Error::Io => LIBUSB_ERROR_IO,
            rusb::Error::InvalidParam => LIBUSB_ERROR_INVALID_PARAM,
            rusb::Error::Access => LIBUSB_ERROR_ACCESS,
            rusb::Error::NoDevice => LIBUSB_ERROR_NO_DEVICE,
            rusb::Error::NotFound => LIBUSB_ERROR_NOT_FOUND,
            rusb::Error::Busy => LIBUSB_ERROR_BUSY,
            rusb::Error::Timeout => LIBUSB_ERROR_TIMEOUT,
            rusb::Error::Overflow => LIBUSB_ERROR_OVERFLOW,
            rusb::Error::Pipe => LIBUSB_ERROR_PIPE,
            rusb::Error::Interrupted => LIBUSB_ERROR_INTERRUPTED,
            rusb::Error::NoMem => LIBUSB_ERROR_NO_MEM,
            rusb::Error::NotSupported => LIBUSB_ERROR_NOT_SUPPORTED,
            _ => LIBUSB_ERROR_OTHER,
        }
    }

    fn descriptor(d: &rusb::DeviceDescriptor) -> DeviceDescriptor {
        // rusb decodes bcdDevice into a Version; put the BCD word back.
        let v = d.device_version();
        let (major, minor, sub) = (v.major() as u16, v.minor() as u16, v.sub_minor() as u16);
        DeviceDescriptor {
            id_vendor: d.vendor_id(),
            id_product: d.product_id(),
            bcd_device: ((major / 10) << 12) | ((major % 10) << 8) | (minor << 4) | sub,
        }
    }

    pub(super) struct Ctx(pub rusb::Context);
    struct Dev(rusb::Device<rusb::Context>);
    struct Handle(rusb::DeviceHandle<rusb::Context>);

    impl UsbContext for Ctx {
        fn set_log_level_info(&mut self) {
            // The C asks libusb for INFO logging. On Linux that is all but
            // silent; Windows' libusb backend writes a line to stderr for
            // every root hub and composite device it enumerates, several
            // per open, which buries longmynd's own messages - so Windows
            // gets WARNING. Warnings and errors still come through.
            #[cfg(not(windows))]
            self.0.set_log_level(rusb::LogLevel::Info);
            #[cfg(windows)]
            self.0.set_log_level(rusb::LogLevel::Warning);
        }
        fn open_device_with_vid_pid(&mut self, vid: u16, pid: u16) -> Option<Box<dyn UsbHandle>> {
            self.0
                .open_device_with_vid_pid(vid, pid)
                .map(|h| Box::new(Handle(h)) as Box<dyn UsbHandle>)
        }
        fn get_device_list(&mut self) -> Result<Vec<Box<dyn UsbDevice>>, i32> {
            let list = self.0.devices().map_err(code)?;
            Ok(list.iter().map(|d| Box::new(Dev(d)) as Box<dyn UsbDevice>).collect())
        }
    }

    impl UsbDevice for Dev {
        fn bus_number(&self) -> u8 {
            self.0.bus_number()
        }
        fn device_address(&self) -> u8 {
            self.0.address()
        }
        fn device_descriptor(&self) -> Result<DeviceDescriptor, i32> {
            self.0.device_descriptor().map(|d| descriptor(&d)).map_err(code)
        }
        fn open(&self) -> Result<Box<dyn UsbHandle>, i32> {
            self.0.open().map(|h| Box::new(Handle(h)) as Box<dyn UsbHandle>).map_err(code)
        }
    }

    impl UsbHandle for Handle {
        fn kernel_driver_active(&mut self, interface: u8) -> i32 {
            match self.0.kernel_driver_active(interface) {
                Ok(true) => 1,
                Ok(false) => 0,
                Err(e) => code(e),
            }
        }
        fn detach_kernel_driver(&mut self, interface: u8) -> i32 {
            self.0.detach_kernel_driver(interface).map(|_| 0).unwrap_or_else(code)
        }
        fn claim_interface(&mut self, interface: u8) -> i32 {
            self.0.claim_interface(interface).map(|_| 0).unwrap_or_else(code)
        }
        fn device_descriptor(&mut self) -> Result<DeviceDescriptor, i32> {
            self.0.device().device_descriptor().map(|d| descriptor(&d)).map_err(code)
        }
        fn bulk_transfer(&mut self, endpoint: u8, data: &mut [u8], timeout_ms: u32) -> (i32, usize) {
            // rusb reports a timeout as an error and drops the partial
            // count libusb would have given; longmynd treats any error on
            // these endpoints as fatal or as a failed I2C step, so the
            // count is only ever printed.
            let timeout = Duration::from_millis(timeout_ms as u64);
            let r = if endpoint & 0x80 != 0 {
                self.0.read_bulk(endpoint, data, timeout)
            } else {
                self.0.write_bulk(endpoint, data, timeout)
            };
            match r {
                Ok(n) => (0, n),
                Err(e) => (code(e), 0),
            }
        }
        fn control_transfer(&mut self, request_type: u8, request: u8, value: u16, index: u16, timeout_ms: u32) -> i32 {
            match self.0.write_control(request_type, request, value, index, &[], Duration::from_millis(timeout_ms as u64)) {
                Ok(n) => n as i32,
                Err(e) => code(e),
            }
        }
    }
}

/// `--usb-backend auto` on Windows: D2XX first, libusb if D2XX does not
/// find the tuner. A MiniTiouner on FTDI's driver (as Windows installs it)
/// is found through D2XX; one already switched to WinUSB, or a PicoTuner
/// (which D2XX never lists), through libusb. `-u` is libusb's.
pub struct AutoBackend {
    /// D2XX
    pub first: std::sync::Arc<dyn UsbBackend>,
    /// libusb
    pub second: std::sync::Arc<dyn UsbBackend>,
}

struct AutoCtx {
    d2xx: Option<Box<dyn UsbContext>>,
    libusb: Option<Box<dyn UsbContext>>,
}

impl UsbBackend for AutoBackend {
    fn init(&self) -> Result<Box<dyn UsbContext>, i32> {
        let d2xx = self.first.init().ok();
        let libusb = self.second.init();
        if d2xx.is_none() {
            return libusb;
        }
        Ok(Box::new(AutoCtx { d2xx, libusb: libusb.ok() }))
    }
}

impl UsbContext for AutoCtx {
    fn set_log_level_info(&mut self) {
        for c in [&mut self.d2xx, &mut self.libusb].into_iter().flatten() {
            c.set_log_level_info();
        }
    }
    fn open_device_with_vid_pid(&mut self, vid: u16, pid: u16) -> Option<Box<dyn UsbHandle>> {
        if let Some(h) = self.d2xx.as_mut().and_then(|c| c.open_device_with_vid_pid(vid, pid)) {
            return Some(h);
        }
        self.libusb.as_mut().and_then(|c| c.open_device_with_vid_pid(vid, pid))
    }
    fn get_device_list(&mut self) -> Result<Vec<Box<dyn UsbDevice>>, i32> {
        match self.libusb.as_mut() {
            Some(c) => c.get_device_list(),
            None => Err(LIBUSB_ERROR_NOT_SUPPORTED),
        }
    }
}

/// The backend `--usb-backend` asks for, and a line to print if it could
/// not be had as asked (D2XX's library missing). `d2xx` is ftd2xx.dll
/// loaded, or why not.
pub fn choose_backend(
    choice: crate::config::UsbBackendChoice,
    d2xx: impl FnOnce() -> Result<crate::d2xx::D2xxBackend, String>,
) -> (std::sync::Arc<dyn UsbBackend>, Option<String>) {
    use crate::config::UsbBackendChoice::*;
    use std::sync::Arc;
    match choice {
        Libusb => (Arc::new(RusbBackend), None),
        Auto if !cfg!(windows) => (Arc::new(RusbBackend), None),
        Auto => match d2xx() {
            Ok(d) => (Arc::new(AutoBackend { first: Arc::new(d), second: Arc::new(RusbBackend) }), None),
            // no FTDI driver library: libusb (WinUSB) is all there is
            Err(_) => (Arc::new(RusbBackend), None),
        },
        D2xx => match d2xx() {
            Ok(d) => (Arc::new(d), None),
            Err(e) => (Arc::new(NoBackend), Some(format!("ERROR: --usb-backend d2xx: cannot load the D2XX library ({})", e))),
        },
    }
}

/// A backend whose library is missing: libusb_init() fails, and longmynd
/// says so and exits as the C does when libusb will not start (101).
pub struct NoBackend;

impl UsbBackend for NoBackend {
    fn init(&self) -> Result<Box<dyn UsbContext>, i32> {
        Err(LIBUSB_ERROR_NOT_SUPPORTED)
    }
}

impl UsbBackend for RusbBackend {
    fn init(&self) -> Result<Box<dyn UsbContext>, i32> {
        rusb::Context::new()
            .map(|c| Box::new(real::Ctx(c)) as Box<dyn UsbContext>)
            .map_err(real::code)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// A backend that has the given VID/PIDs and records who opened what.
    struct Fake(&'static str, Vec<(u16, u16)>, Arc<Mutex<Vec<String>>>);
    struct FakeCtx(&'static str, Vec<(u16, u16)>, Arc<Mutex<Vec<String>>>);
    struct FakeHandle;

    impl UsbBackend for Fake {
        fn init(&self) -> Result<Box<dyn UsbContext>, i32> {
            Ok(Box::new(FakeCtx(self.0, self.1.clone(), self.2.clone())))
        }
    }
    impl UsbContext for FakeCtx {
        fn set_log_level_info(&mut self) {}
        fn open_device_with_vid_pid(&mut self, vid: u16, pid: u16) -> Option<Box<dyn UsbHandle>> {
            self.2.lock().unwrap().push(format!("{} {:04x}:{:04x}", self.0, vid, pid));
            self.1.contains(&(vid, pid)).then(|| Box::new(FakeHandle) as Box<dyn UsbHandle>)
        }
        fn get_device_list(&mut self) -> Result<Vec<Box<dyn UsbDevice>>, i32> {
            Ok(vec![])
        }
    }
    impl UsbHandle for FakeHandle {
        fn kernel_driver_active(&mut self, _: u8) -> i32 {
            0
        }
        fn detach_kernel_driver(&mut self, _: u8) -> i32 {
            0
        }
        fn claim_interface(&mut self, _: u8) -> i32 {
            0
        }
        fn device_descriptor(&mut self) -> Result<DeviceDescriptor, i32> {
            Err(LIBUSB_ERROR_NOT_SUPPORTED)
        }
        fn bulk_transfer(&mut self, _: u8, _: &mut [u8], _: u32) -> (i32, usize) {
            (0, 0)
        }
        fn control_transfer(&mut self, _: u8, _: u8, _: u16, _: u16, _: u32) -> i32 {
            0
        }
    }

    #[test]
    fn auto_tries_d2xx_first_then_libusb() {
        let log = Arc::new(Mutex::new(vec![]));
        // a MiniTiouner on FTDI's driver: found through D2XX, libusb not asked
        let auto = AutoBackend {
            first: Arc::new(Fake("d2xx", vec![(0x0403, 0x6010)], log.clone())),
            second: Arc::new(Fake("libusb", vec![], log.clone())),
        };
        assert!(auto.init().unwrap().open_device_with_vid_pid(0x0403, 0x6010).is_some());
        assert_eq!(*log.lock().unwrap(), vec!["d2xx 0403:6010"]);
        // a PicoTuner, or a MiniTiouner on WinUSB: D2XX does not see it
        log.lock().unwrap().clear();
        let auto = AutoBackend {
            first: Arc::new(Fake("d2xx", vec![], log.clone())),
            second: Arc::new(Fake("libusb", vec![(0x2e8a, 0xba2c)], log.clone())),
        };
        assert!(auto.init().unwrap().open_device_with_vid_pid(0x2e8a, 0xba2c).is_some());
        assert_eq!(*log.lock().unwrap(), vec!["d2xx 2e8a:ba2c", "libusb 2e8a:ba2c"]);
    }

    #[test]
    fn an_explicit_d2xx_without_its_library_says_so() {
        let (b, problem) = choose_backend(crate::config::UsbBackendChoice::D2xx, || Err("ftd2xx.dll: not found".into()));
        assert!(b.init().is_err(), "libusb_init fails, so longmynd exits 101 as the C would");
        assert_eq!(problem.unwrap(), "ERROR: --usb-backend d2xx: cannot load the D2XX library (ftd2xx.dll: not found)");
        let (_, problem) = choose_backend(crate::config::UsbBackendChoice::Auto, || Err("missing".into()));
        assert!(problem.is_none(), "auto without the library is libusb, silently");
    }
}
