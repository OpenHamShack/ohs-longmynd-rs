//! ftdi_usb.c: opening the FT2232H (or PicoTuner) and moving bytes over its
//! two interfaces - interface 0 (endpoints 0x02 out / 0x81 in) carries the
//! MPSSE commands that drive the NIM's I2C bus, interface 1 (0x83 in)
//! carries the transport stream.

use crate::clock::Clock;
use crate::errors::*;
use crate::usb::{error_name, UsbBackend, UsbContext, UsbHandle};

pub const USB_TIMEOUT: u32 = 5000;
pub const USB_FAST_TIMEOUT: u32 = 500;

const FTDI_USB_READ_RETRIES: i32 = 2;

const SIO_RESET_REQUEST: u8 = 0;
const SIO_SET_LATENCY_TIMER_REQUEST: u8 = 0x09;
const SIO_SET_BITMODE_REQUEST: u8 = 0x0B;
const LATENCY_MS: u16 = 16;
const SIO_RESET_SIO: u16 = 0;
const SIO_RESET_PURGE_RX: u16 = 1;
const SIO_RESET_PURGE_TX: u16 = 2;
/// LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT
const FTDI_DEVICE_OUT_REQTYPE: u8 = 0x40;
const FTDI_RX_CHUNK_SIZE: usize = 4096;

const BITMODE_RESET: u16 = 0x00;
const BITMODE_MPSSE: u16 = 0x02;

/// One opened, claimed interface. The context is kept alive beside the
/// handle, as the C keeps a context per interface.
pub struct UsbLink {
    pub handle: Box<dyn UsbHandle>,
    _context: Box<dyn UsbContext>,
}

/// Interface 0: the MPSSE command channel, and the read-ahead buffer of
/// ftdi_usb_i2c_read() (the C's file-level `rx_chunk` and the function's
/// static `rxed` / `posn`, which persist from call to call).
pub struct I2cUsb {
    link: UsbLink,
    rx_chunk: Vec<u8>,
    rxed: i32,
    posn: i32,
}

impl I2cUsb {
    pub fn new(link: UsbLink) -> Self {
        I2cUsb { link, rx_chunk: vec![0; FTDI_RX_CHUNK_SIZE], rxed: 0, posn: 0 }
    }

    /// ftdi_usb_i2c_write()
    pub fn write(&mut self, buffer: &[u8]) -> u8 {
        let mut err = ERROR_NONE;
        let mut data = buffer.to_vec();
        let (res, sent) = self.link.handle.bulk_transfer(0x02, &mut data, USB_TIMEOUT);
        if res < 0 {
            println!("ERROR: USB Cmd Write failure {}", res);
            err = ERROR_FTDI_USB_CMD;
        }
        if err == ERROR_NONE && sent != buffer.len() {
            println!("ERROR: i2c write incorrect num bytes sent={}, len={}", sent, buffer.len());
            err = ERROR_FTDI_I2C_WRITE_LEN;
        }
        err
    }

    /// ftdi_usb_i2c_read(): one byte of reply, from what an earlier USB
    /// read left over if there is any, else from a new read (whose first
    /// two bytes are the FTDI's modem status and are skipped).
    ///
    /// One deliberate difference from the C, on its error path only: after
    /// a failed read the C has overwritten `rxed` but not `posn`, so the two
    /// no longer match and every later call takes "unused" bytes from the
    /// old buffer - stale ACKs and register values, walking towards the end
    /// of the array - and never reads the USB again. A single stalled read
    /// thus has the rest of an init written blind (the equivalence harness
    /// shows the C doing it: tests/fixes.rs). Here a failed read empties the
    /// read-ahead, so the I2C retry that follows reads afresh.
    pub fn read(&mut self) -> Result<u8, u8> {
        let r = self.read_inner();
        if r.is_err() {
            self.posn = 0;
            self.rxed = 0;
        }
        r
    }

    fn read_inner(&mut self) -> Result<u8, u8> {
        if self.posn != self.rxed {
            let v = self.chunk_at(self.posn);
            self.posn += 1;
            return Ok(v);
        }
        let mut err = ERROR_NONE;
        let mut n = 0;
        while n < FTDI_USB_READ_RETRIES {
            let (res, rxed) = self.link.handle.bulk_transfer(0x81, &mut self.rx_chunk, USB_TIMEOUT);
            self.rxed = rxed as i32;
            if res < 0 {
                println!("ERROR: USB Cmd Read failure {}", res);
                err = ERROR_FTDI_USB_CMD;
                break;
            }
            if self.rxed > 2 {
                break;
            }
            n += 1;
        }
        if n == FTDI_USB_READ_RETRIES {
            return Err(ERROR_FTDI_I2C_READ_LEN);
        }
        if err != ERROR_NONE {
            return Err(err);
        }
        self.posn = 2;
        let v = self.chunk_at(self.posn);
        self.posn += 1;
        Ok(v)
    }

    /// rx_chunk[posn], defensively bounded (read() no longer lets posn
    /// run away as the C's can).
    fn chunk_at(&self, posn: i32) -> u8 {
        if (0..FTDI_RX_CHUNK_SIZE as i32).contains(&posn) {
            self.rx_chunk[posn as usize]
        } else {
            0
        }
    }

    pub fn set_mpsse_mode(&mut self, clock: &dyn Clock) -> u8 {
        set_mpsse_mode(&mut *self.link.handle, clock)
    }
}

/// Interface 1: the TS endpoint.
pub struct TsUsb {
    link: UsbLink,
}

impl TsUsb {
    pub fn new(link: UsbLink) -> Self {
        TsUsb { link }
    }

    pub fn set_mpsse_mode(&mut self, clock: &dyn Clock) -> u8 {
        set_mpsse_mode(&mut *self.link.handle, clock)
    }

    /// --ts-latency: the latency timer of interface B, which carries the TS.
    /// The C's ftdi_usb_set_mpsse_mode() addresses every request to wIndex
    /// 1 - interface A - for both handles, so B keeps the chip's default
    /// (16ms): how long a part-filled USB packet of TS waits before it is
    /// sent. At low symbol rates that is most of the TS latency.
    pub fn set_latency_interface_b(&mut self, ms: u8) -> u8 {
        println!("Flow: FTDI TS latency timer {} ms", ms);
        let res = self.link.handle.control_transfer(
            FTDI_DEVICE_OUT_REQTYPE,
            SIO_SET_LATENCY_TIMER_REQUEST,
            ms as u16,
            2,
            USB_TIMEOUT,
        );
        if res < 0 {
            println!("ERROR: USB Set TS Latency failed {}", res);
            return ERROR_MPSSE;
        }
        ERROR_NONE
    }

    /// ftdi_usb_ts_read(): as the C, `len` is left alone when the read
    /// fails, and is a uint16_t.
    pub fn read(&mut self, buffer: &mut [u8], len: &mut u16, frame_size: usize) -> u8 {
        self.read_ep(0x83, buffer, len, frame_size)
    }

    /// read() from either TS endpoint: 0x83 (the C's; on a PicoTuner the
    /// top demodulator's TS2) or 0x84 (a PicoTuner's TS1, the bottom
    /// demodulator's, for --second).
    pub fn read_ep(&mut self, endpoint: u8, buffer: &mut [u8], len: &mut u16, frame_size: usize) -> u8 {
        let mut err = ERROR_NONE;
        let (res, rxed) = self.link.handle.bulk_transfer(endpoint, &mut buffer[..frame_size], USB_FAST_TIMEOUT);
        if res < 0 {
            println!("ERROR: USB TS Data Read {} ({}), received {}", res, error_name(res), rxed);
            err = ERROR_USB_TS_READ;
        } else {
            *len = rxed as u16;
        }
        if err != ERROR_NONE {
            println!("ERROR: FTDI USB ts read");
        }
        err
    }
}

/// ftdi_usb_set_mpsse_mode(): purge both FIFOs, reset, set the latency
/// timer, then bit mode reset and MPSSE. Several of the C's messages here
/// have no newline; so do these.
fn set_mpsse_mode(h: &mut dyn UsbHandle, clock: &dyn Clock) -> u8 {
    let mut err = ERROR_NONE;
    println!("Flow: FTDI set mpsse mode");
    let res = h.control_transfer(FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST, SIO_RESET_PURGE_RX, 1, USB_TIMEOUT);
    if res < 0 {
        print!("ERROR: USB RX Purge failed {}", res);
        err = ERROR_MPSSE;
    }
    let res = h.control_transfer(FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST, SIO_RESET_PURGE_TX, 1, USB_TIMEOUT);
    if res < 0 {
        print!("ERROR: USB TX Purge failed {}", res);
        err = ERROR_MPSSE;
    }
    let res = h.control_transfer(FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST, SIO_RESET_SIO, 1, USB_TIMEOUT);
    if res < 0 {
        print!("ERROR: USB Reset failed {}", res);
        err = ERROR_MPSSE;
    }
    let res = h.control_transfer(FTDI_DEVICE_OUT_REQTYPE, SIO_SET_LATENCY_TIMER_REQUEST, LATENCY_MS, 1, USB_TIMEOUT);
    if res < 0 {
        print!("ERROR: USB Set Latency failed {}", res);
        err = ERROR_MPSSE;
    }
    let res = h.control_transfer(FTDI_DEVICE_OUT_REQTYPE, SIO_SET_BITMODE_REQUEST, BITMODE_RESET << 8, 1, USB_TIMEOUT);
    if res < 0 {
        println!("USB Reset Bitmode failed {}", res);
        err = ERROR_MPSSE;
    }
    let res = h.control_transfer(FTDI_DEVICE_OUT_REQTYPE, SIO_SET_BITMODE_REQUEST, BITMODE_MPSSE << 8, 1, USB_TIMEOUT);
    if res < 0 {
        println!("USB Set MPSSE failed {}", res);
        err = ERROR_MPSSE;
    }
    clock.usleep(1000);
    err
}

/// What ftdi_usb_init() found besides the link: whether the opened device
/// is a PicoTuner sending serial TS (the patch's stv0910_serialTS), when
/// its descriptor could be read.
pub struct Opened {
    pub link: UsbLink,
    pub serial_ts: Option<bool>,
    /// The device is a PicoTuner (when its descriptor could be read).
    pub pico: Option<bool>,
}

/// ftdi_usb_init(): open by VID/PID (the FTDI's, then the PicoTuner's) when
/// no bus/device was given, otherwise find that bus/device and check it is
/// one of the two; then detach any kernel driver from `interface_num` and
/// claim it.
#[allow(clippy::too_many_arguments)]
pub fn ftdi_usb_init(
    backend: &dyn UsbBackend,
    interface_num: u8,
    usb_bus: u8,
    usb_addr: u8,
    vid: u16,
    pid: u16,
    altvid: u16,
    altpid: u16,
) -> Result<Opened, u8> {
    println!("Flow: FTDI USB init");
    let mut ctx = match backend.init() {
        Ok(c) => c,
        Err(_) => {
            println!("ERROR: Unable to initialise LIBUSB");
            return Err(ERROR_FTDI_USB_INIT_LIBUSB);
        }
    };
    ctx.set_log_level_info();

    let mut handle: Box<dyn UsbHandle>;
    if usb_bus == 0 && usb_addr == 0 {
        handle = match ctx.open_device_with_vid_pid(vid, pid) {
            Some(h) => h,
            None => match ctx.open_device_with_vid_pid(altvid, altpid) {
                Some(h) => h,
                None => {
                    println!("ERROR: Unable to open device with VID and PID");
                    println!("       Is the USB cable plugged in?");
                    return Err(ERROR_FTDI_USB_VID_PID);
                }
            },
        };
    } else {
        println!("Flow: Searching for bus/device={},{}", usb_bus, usb_addr);
        let list = match ctx.get_device_list() {
            Ok(l) if !l.is_empty() => l,
            _ => {
                println!("ERROR: failed to get the list of devices");
                return Err(ERROR_FTDI_USB_DEVICE_LIST);
            }
        };
        let found = list
            .iter()
            .find(|d| d.bus_number() == usb_bus && d.device_address() == usb_addr);
        let device = match found {
            Some(d) => d,
            None => {
                println!("ERROR: invalid USB bus/device number");
                return Err(ERROR_FTDI_USB_BAD_DEVICE_NUM);
            }
        };
        let matches = match device.device_descriptor() {
            Ok(d) => (d.id_vendor == vid && d.id_product == pid) || (d.id_vendor == altvid && d.id_product == altpid),
            Err(_) => false,
        };
        if !matches {
            println!("ERROR: This bus/device is not a Minitiouner");
            return Err(ERROR_FTDI_USB_DEVICE_NUM_OPEN);
        }
        handle = match device.open() {
            Ok(h) => {
                println!("      Status: successfully opened USB Device {},{}", usb_bus, usb_addr);
                h
            }
            Err(_) => {
                println!("ERROR: Unable to open Minitiouner");
                return Err(ERROR_FTDI_USB_DEVICE_NUM_OPEN);
            }
        };
    }

    if handle.kernel_driver_active(interface_num) != 0 {
        handle.detach_kernel_driver(interface_num);
    }
    if handle.claim_interface(interface_num) < 0 {
        println!("ERROR: Unable to claim interface");
        return Err(ERROR_FTDI_USB_CLAIM);
    }

    // The PicoTuner says it reads the NIM's TS one bit wide with bcdDevice
    // 0x0102 (0x0101 is parallel, like the FTDI); the demodulator must then
    // be told to send serial TS.
    let desc = handle.device_descriptor().ok();
    let serial_ts = desc.map(|d| d.id_vendor == altvid && d.id_product == altpid && d.bcd_device == 0x0102);
    let pico = desc.map(|d| d.id_vendor == altvid && d.id_product == altpid);

    Ok(Opened { link: UsbLink { handle, _context: ctx }, serial_ts, pico })
}
