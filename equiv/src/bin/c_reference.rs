//! Runs one scenario through the reference C longmynd on the simulator.
//!   c_reference <scenario> <outdir>
//!
//! The C is linked (equiv/build.rs) against the libusb functions below,
//! which hand every call to the same Sim the Rust port is driven against,
//! and its usleep()/clock_gettime() against lmsim_usleep and
//! lmsim_clock_gettime.
//!
//! This file is the harness's one use of `unsafe`: it is an FFI shim, and
//! every function receives raw pointers from C. Each dereference is of a
//! pointer longmynd's ftdi_usb.c passes exactly as libusb's API specifies
//! (out-parameters it owns, buffers of the length it states), and the
//! opaque handles it is given back are small integers this file made, never
//! dereferenced. None of this is part of longmynd-rs itself, which has no
//! unsafe code.
#![allow(clippy::missing_safety_doc)]

use std::ffi::{c_char, c_int, c_uint, c_void, CStr};
use std::sync::Mutex;

use longmynd_equiv::{capture, scenario, sim};

extern "C" {
    fn lmref_run(mode: c_int, argc: c_int, argv: *mut *mut c_char) -> c_int;
}

type CHook = extern "C" fn();
static SLEEP_HOOK: Mutex<Option<CHook>> = Mutex::new(None);
static TS_HOOK: Mutex<Option<CHook>> = Mutex::new(None);

fn id_to_ptr(id: usize) -> *mut c_void {
    (id + 1) as *mut c_void
}
fn ptr_to_id(p: *const c_void) -> usize {
    (p as usize) - 1
}

#[repr(C)]
pub struct Timespec {
    tv_sec: i64,
    tv_nsec: i64,
}

#[repr(C)]
pub struct DeviceDescriptor {
    b_length: u8,
    b_descriptor_type: u8,
    bcd_usb: u16,
    b_device_class: u8,
    b_device_sub_class: u8,
    b_device_protocol: u8,
    b_max_packet_size0: u8,
    id_vendor: u16,
    id_product: u16,
    bcd_device: u16,
    i_manufacturer: u8,
    i_product: u8,
    i_serial_number: u8,
    b_num_configurations: u8,
}

fn fill(desc: *mut DeviceDescriptor, vid: u16, pid: u16, bcd: u16) {
    // SAFETY: desc is the caller's struct libusb_device_descriptor
    unsafe {
        *desc = DeviceDescriptor {
            b_length: 18,
            b_descriptor_type: 1,
            bcd_usb: 0x0200,
            b_device_class: 0,
            b_device_sub_class: 0,
            b_device_protocol: 0,
            b_max_packet_size0: 64,
            id_vendor: vid,
            id_product: pid,
            bcd_device: bcd,
            i_manufacturer: 1,
            i_product: 2,
            i_serial_number: 3,
            b_num_configurations: 1,
        };
    }
}

// ---- time --------------------------------------------------------------------

#[no_mangle]
pub extern "C" fn lmsim_usleep(us: c_uint) -> c_int {
    sim::with(|s| s.usleep(us as u64));
    let hook = *SLEEP_HOOK.lock().unwrap();
    if let Some(h) = hook {
        h();
    }
    0
}

#[no_mangle]
pub unsafe extern "C" fn lmsim_clock_gettime(_clk: c_int, tp: *mut Timespec) -> c_int {
    let us = sim::with(|s| s.monotonic_us());
    // SAFETY: tp is the caller's struct timespec
    unsafe {
        (*tp).tv_sec = (us / 1_000_000) as i64;
        (*tp).tv_nsec = ((us % 1_000_000) * 1000) as i64;
    }
    0
}

#[no_mangle]
pub extern "C" fn lmsim_set_sleep_hook(h: Option<CHook>) {
    *SLEEP_HOOK.lock().unwrap() = h;
}

#[no_mangle]
pub extern "C" fn lmsim_set_ts_hook(h: Option<CHook>) {
    *TS_HOOK.lock().unwrap() = h;
}

#[no_mangle]
pub extern "C" fn lmsim_terminate_due() -> c_int {
    sim::with(|s| s.terminate_due()) as c_int
}

#[no_mangle]
pub extern "C" fn lmsim_ts_flow() -> u32 {
    sim::with(|s| s.ts_flow())
}

#[no_mangle]
pub extern "C" fn lmsim_set_option(_ctx: *mut c_void, _option: c_int) {
    sim::with(|s| s.set_option_log_info());
}

// ---- libusb ------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn libusb_init(ctx: *mut *mut c_void) -> c_int {
    let r = sim::with(|s| s.libusb_init());
    // SAFETY: ctx is the caller's libusb_context**
    unsafe { *ctx = id_to_ptr(0) };
    r
}

#[no_mangle]
pub extern "C" fn libusb_exit(_ctx: *mut c_void) {}

#[no_mangle]
pub extern "C" fn libusb_close(_h: *mut c_void) {}

#[no_mangle]
pub extern "C" fn libusb_open_device_with_vid_pid(_ctx: *mut c_void, vid: u16, pid: u16) -> *mut c_void {
    match sim::with(|s| s.open_vid_pid(vid, pid)) {
        Some(h) => id_to_ptr(h),
        None => std::ptr::null_mut(),
    }
}

#[no_mangle]
pub unsafe extern "C" fn libusb_get_device_list(_ctx: *mut c_void, list: *mut *mut *mut c_void) -> isize {
    let n = sim::with(|s| s.device_list());
    let mut v: Vec<*mut c_void> = (0..n).map(id_to_ptr).collect();
    v.push(std::ptr::null_mut());
    let b = v.into_boxed_slice();
    // SAFETY: list is the caller's libusb_device***; the array is leaked
    // until libusb_free_device_list hands it back
    unsafe { *list = Box::into_raw(b) as *mut *mut c_void };
    n as isize
}

#[no_mangle]
pub unsafe extern "C" fn libusb_free_device_list(list: *mut *mut c_void, _unref: c_int) {
    if list.is_null() {
        return;
    }
    // SAFETY: list came from libusb_get_device_list: NULL-terminated, boxed
    unsafe {
        let mut n = 0;
        while !(*list.add(n)).is_null() {
            n += 1;
        }
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(list, n + 1)));
    }
}

#[no_mangle]
pub extern "C" fn libusb_get_bus_number(dev: *mut c_void) -> u8 {
    sim::with(|s| s.dev(ptr_to_id(dev)).bus)
}

#[no_mangle]
pub extern "C" fn libusb_get_device_address(dev: *mut c_void) -> u8 {
    sim::with(|s| s.dev(ptr_to_id(dev)).addr)
}

#[no_mangle]
pub extern "C" fn libusb_get_device_descriptor(dev: *mut c_void, desc: *mut DeviceDescriptor) -> c_int {
    let id = ptr_to_id(dev);
    let (v, p, b) = sim::with(|s| {
        if id >= HANDLE_DEVICE {
            s.handle_descriptor(id - HANDLE_DEVICE)
        } else {
            let d = s.dev(id);
            (d.vid, d.pid, d.bcd)
        }
    });
    fill(desc, v, p, b);
    0
}

/// libusb_get_device() on a handle: the handle's id, offset so it is told
/// apart from a device id when it comes back to libusb_get_device_descriptor.
const HANDLE_DEVICE: usize = 0x10000;

#[no_mangle]
pub extern "C" fn libusb_get_device(h: *mut c_void) -> *mut c_void {
    id_to_ptr(HANDLE_DEVICE + ptr_to_id(h))
}

#[no_mangle]
pub unsafe extern "C" fn libusb_open(dev: *mut c_void, h: *mut *mut c_void) -> c_int {
    match sim::with(|s| s.open_dev(ptr_to_id(dev))) {
        Ok(id) => {
            // SAFETY: h is the caller's libusb_device_handle**
            unsafe { *h = id_to_ptr(id) };
            0
        }
        Err(e) => e,
    }
}

#[no_mangle]
pub extern "C" fn libusb_kernel_driver_active(h: *mut c_void, iface: c_int) -> c_int {
    sim::with(|s| s.kernel_driver_active(ptr_to_id(h), iface as u8))
}

#[no_mangle]
pub extern "C" fn libusb_detach_kernel_driver(h: *mut c_void, iface: c_int) -> c_int {
    sim::with(|s| s.detach_kernel_driver(ptr_to_id(h), iface as u8))
}

#[no_mangle]
pub extern "C" fn libusb_claim_interface(h: *mut c_void, iface: c_int) -> c_int {
    sim::with(|s| s.claim_interface(ptr_to_id(h), iface as u8))
}

#[no_mangle]
pub unsafe extern "C" fn libusb_bulk_transfer(
    h: *mut c_void,
    endpoint: u8,
    data: *mut u8,
    length: c_int,
    transferred: *mut c_int,
    _timeout: c_uint,
) -> c_int {
    if endpoint == 0x83 {
        let hook = *TS_HOOK.lock().unwrap();
        if let Some(f) = hook {
            f();
        }
    }
    // SAFETY: data is the caller's buffer of `length` bytes
    let buf = unsafe { std::slice::from_raw_parts_mut(data, length as usize) };
    let (r, n) = sim::with(|s| s.bulk(ptr_to_id(h), endpoint, buf));
    // SAFETY: transferred is the caller's int*
    unsafe { *transferred = n as c_int };
    r
}

#[no_mangle]
pub extern "C" fn libusb_control_transfer(
    h: *mut c_void,
    rt: u8,
    req: u8,
    val: u16,
    idx: u16,
    _data: *mut u8,
    _len: u16,
    _timeout: c_uint,
) -> c_int {
    sim::with(|s| s.control(ptr_to_id(h), rt, req, val, idx))
}

#[no_mangle]
pub extern "C" fn libusb_error_name(code: c_int) -> *const c_char {
    let s: &'static CStr = match code {
        0 => c"LIBUSB_SUCCESS / LIBUSB_TRANSFER_COMPLETED",
        -1 => c"LIBUSB_ERROR_IO",
        -2 => c"LIBUSB_ERROR_INVALID_PARAM",
        -3 => c"LIBUSB_ERROR_ACCESS",
        -4 => c"LIBUSB_ERROR_NO_DEVICE",
        -5 => c"LIBUSB_ERROR_NOT_FOUND",
        -6 => c"LIBUSB_ERROR_BUSY",
        -7 => c"LIBUSB_ERROR_TIMEOUT",
        -8 => c"LIBUSB_ERROR_OVERFLOW",
        -9 => c"LIBUSB_ERROR_PIPE",
        -10 => c"LIBUSB_ERROR_INTERRUPTED",
        -11 => c"LIBUSB_ERROR_NO_MEM",
        -12 => c"LIBUSB_ERROR_NOT_SUPPORTED",
        -99 => c"LIBUSB_ERROR_OTHER",
        _ => c"**UNKNOWN**",
    };
    s.as_ptr()
}

extern "C" {
    fn lmref_double_to_u32(d: f64) -> u32;
    fn lmref_double_to_i32(d: f64) -> i32;
    fn lmref_strtol(s: *const c_char) -> i64;
    fn lmref_strtof(s: *const c_char) -> f32;
}

/// `c_reference --cnum`: the reference compiler's and C library's answers
/// for the conversions longmynd-rs reproduces in cnum.rs, one per line.
fn cnum_vectors() {
    for t in longmynd_equiv::CNUM_TEXT {
        let c = std::ffi::CString::new(*t).unwrap();
        // SAFETY: c is a valid C string for the duration of the calls
        let (l, f) = unsafe { (lmref_strtol(c.as_ptr()), lmref_strtof(c.as_ptr())) };
        println!("strtol {:?} {}", t, l);
        println!("strtof {:?} {:08x}", t, f.to_bits());
    }
    for d in longmynd_equiv::CNUM_DOUBLES {
        // SAFETY: plain value conversions
        let (u, i) = unsafe { (lmref_double_to_u32(*d), lmref_double_to_i32(*d)) };
        println!("d2u32 {:e} {}", d, u);
        println!("d2i32 {:e} {}", d, i);
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.get(1).map(|s| s.as_str()) == Some("--cnum") {
        cnum_vectors();
        return;
    }
    let scn = scenario::find(&a[1]).unwrap_or_else(|| panic!("no scenario {}", a[1]));
    let dir = std::path::PathBuf::from(&a[2]);
    let run = capture::start(&scn, &dir);
    // longmynd-rs's own options are taken out first, so the C runs any
    // scenario as "the same command line without them": an extension
    // scenario then shows the C's behaviour beside the option's
    // (tests/extensions.rs). The equivalence scenarios have none, so this
    // changes nothing for them.
    let argv: Vec<Vec<u8>> = scn.argv(dir.to_str().unwrap()).into_iter().map(String::into_bytes).collect();
    let argv = longmynd::config::split_extensions(&argv).expect("scenario options parse").0;
    // argv as mutable NUL-terminated buffers: process_command_line()
    // writes NULs over the commas in the frequency and rate lists
    let mut bufs: Vec<Vec<u8>> = argv
        .into_iter()
        .map(|mut b| {
            b.push(0);
            b
        })
        .collect();
    let mut argv: Vec<*mut c_char> = bufs.iter_mut().map(|b| b.as_mut_ptr() as *mut c_char).collect();
    argv.push(std::ptr::null_mut());
    let mode = match scn.mode {
        scenario::Mode::Main => 0,
        scenario::Mode::Receiver => 1,
        scenario::Mode::Ts => 2,
    };
    // SAFETY: argv is argc valid C strings and a NULL, alive for the call
    let code = unsafe { lmref_run(mode, (argv.len() - 1) as c_int, argv.as_mut_ptr()) };
    extern "C" {
        fn fflush(f: *mut c_void) -> c_int;
    }
    // SAFETY: fflush(NULL) flushes every C stdio stream
    unsafe { fflush(std::ptr::null_mut()) };
    run.finish(code as u8);
    std::process::exit(code);
}
