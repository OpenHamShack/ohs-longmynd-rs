//! ftdi.c: I2C over the FT2232H's MPSSE engine (bit-banged START/STOP, one
//! byte and its ACK per USB round trip), the NIM reset line and the LNB
//! supply GPIOs.
//!
//! The MPSSE byte sequences are copied from the C exactly - including the
//! four-fold repeats of each START/STOP edge, which slow the edges down -
//! because the equivalence test compares them byte for byte and because
//! they are what a MiniTiouner is known to work with.

use std::sync::Arc;

use crate::clock::Clock;
use crate::errors::*;
use crate::ftdi_usb::{ftdi_usb_init, I2cUsb, TsUsb};
use crate::usb::UsbBackend;

pub const FTDI_VID: u16 = 0x0403;
pub const FTDI_PID: u16 = 0x6010;
/// BATC PicoTuner (g4eml/PicoTuner firmware): a Raspberry Pi Pico that
/// speaks the same MPSSE subset as the FT2232H, under the Raspberry Pi VID.
pub const PICO_VID: u16 = 0x2E8A;
pub const PICO_PID: u16 = 0xBA2C;

const FTDI_NUM_TRIES: i32 = 10;
const FTDI_STOP_START_REPEATS: usize = 4;
const FTDI_RDWR_TIMEOUT: i32 = 100;

// FTDI GPIO pins (high byte, AC0..AC7)
const FTDI_GPIO_PINID_NIM_RESET: u8 = 0;
#[allow(dead_code)]
const FTDI_GPIO_PINID_TS2SYNC: u8 = 1;
const FTDI_GPIO_PINID_LNB_BIAS_ENABLE: u8 = 4;
const FTDI_GPIO_PINID_LNB_BIAS_VSEL: u8 = 7;

pub struct Ftdi {
    usb: I2cUsb,
    /// out_buffer[] / num_bytes_to_send: MPSSE commands queued for the next
    /// USB write.
    out: Vec<u8>,
    /// Default 0x6f = LNB bias off, LNB 13V, NIM not in reset.
    gpio_value: u8,
    /// Default 0xf1: LNB pins and NIM reset are outputs, TS2SYNC an input.
    gpio_direction: u8,
    /// The byte the last successful I2C read returned; see setup_ftdi_io().
    last_read: u8,
    pub clock: Arc<dyn Clock>,
}

impl Ftdi {
    pub fn new(usb: I2cUsb, clock: Arc<dyn Clock>) -> Self {
        Ftdi { usb, out: Vec::with_capacity(256), gpio_value: 0x6f, gpio_direction: 0xf1, last_read: 0, clock }
    }

    fn push(&mut self, bytes: &[u8]) {
        self.out.extend_from_slice(bytes);
    }

    fn flush(&mut self) -> u8 {
        let out = std::mem::take(&mut self.out);
        let err = self.usb.write(&out);
        self.out = out;
        self.out.clear();
        err
    }

    fn read_byte(&mut self) -> Result<u8, u8> {
        let r = self.usb.read();
        if let Ok(v) = r {
            self.last_read = v;
        }
        r
    }

    /// ftdi_setup_ftdi_io(): synchronise with the MPSSE engine (send the
    /// bad command 0xAA and wait for its echo 0xFA 0xAA), then set clocks,
    /// pin directions and the I2C clock divisor.
    fn setup_ftdi_io(&mut self) -> u8 {
        let mut err = ERROR_NONE;
        let clock_divisor: u16 = 0x0095;
        println!("Flow: FTDI setup io");
        // errors are not accumulated while syncing
        self.out.clear();
        self.push(&[0xAA]);
        let _ = self.flush();
        let mut old_in: u8 = 0;
        for _ in 0..FTDI_NUM_TRIES {
            // The C ignores a failed read here and looks at whatever its
            // pointer last pointed at; the last byte read is the same thing.
            let _ = self.read_byte();
            let inb = self.last_read;
            if old_in == 0xFA && inb == 0xAA {
                print!("      Status: MPSSE Synched ");
                println!("{:02X} {:02X}", inb, old_in);
                break;
            }
            old_in = inb;
        }
        self.push(&[0x8A, 0x97, 0x8D]);
        err = self.flush_if_ok(err);
        self.push(&[0x80, 0x13, 0x13, 0x82, 0x6f, 0xf1, 0x86]);
        self.push(&[(clock_divisor & 0xFF) as u8, ((clock_divisor >> 8) & 0xFF) as u8]);
        err = self.flush_if_ok(err);
        self.clock.usleep(30000);
        self.push(&[0x85]);
        err = self.flush_if_ok(err);
        // need to wait a while for it to work
        self.clock.usleep(30000);
        if err != ERROR_NONE {
            println!("ERROR: set mpsse mode");
        }
        err
    }

    /// `if (err==ERROR_NONE) err=ftdi_usb_i2c_write(...); num_bytes_to_send = 0;`
    fn flush_if_ok(&mut self, err: u8) -> u8 {
        if err == ERROR_NONE {
            self.flush()
        } else {
            self.out.clear();
            err
        }
    }

    fn i2c_set_start(&mut self) -> u8 {
        for _ in 0..FTDI_STOP_START_REPEATS {
            self.push(&[0x80, 0x03, 0x13]);
        }
        for _ in 0..FTDI_STOP_START_REPEATS {
            self.push(&[0x80, 0x01, 0x13]);
        }
        ERROR_NONE
    }

    fn i2c_set_stop(&mut self) -> u8 {
        for _ in 0..FTDI_STOP_START_REPEATS {
            self.push(&[0x80, 0x01, 0x13]);
        }
        for _ in 0..FTDI_STOP_START_REPEATS {
            self.push(&[0x80, 0x03, 0x13]);
        }
        self.push(&[0x80, 0x03, 0x10]);
        ERROR_NONE
    }

    fn i2c_send_byte_check_ack(&mut self, b: u8) -> u8 {
        self.push(&[0x80, 0x00, 0x13, 0x11, 0x00, 0x00, b, 0x80, 0x00, 0x11, 0x27, 0x00, 0x87]);
        let mut err = self.flush();
        if err == ERROR_NONE {
            match self.read_byte() {
                Ok(v) => {
                    if v & 0x01 != 0 {
                        err = ERROR_I2C_NO_ACK;
                    }
                }
                Err(e) => err = e,
            }
        }
        err
    }

    fn i2c_read_byte_send_nak(&mut self, b: &mut u8) -> u8 {
        self.push(&[0x80, 0x00, 0x13, 0x80, 0x00, 0x11, 0x25, 0x00, 0x00, 0x87]);
        let mut err = self.flush();
        if err == ERROR_NONE {
            match self.read_byte() {
                Ok(v) => *b = v,
                Err(e) => err = e,
            }
        }
        // On a failed read the C copies from an uninitialised pointer;
        // here the value is simply left as it was.
        err
    }

    fn i2c_output(&mut self) -> u8 {
        self.flush()
    }

    pub fn i2c_read_reg16(&mut self, addr: u8, reg: u16, val: &mut u8) -> u8 {
        let mut err: u8;
        let mut timeout = 0;
        loop {
            err = ERROR_NONE;
            for _ in 0..FTDI_NUM_TRIES {
                err = self.i2c_set_start();
                err |= self.i2c_send_byte_check_ack(addr);
                err |= self.i2c_send_byte_check_ack((reg >> 8) as u8);
                err |= self.i2c_send_byte_check_ack((reg & 0xff) as u8);
                if err == ERROR_NONE {
                    break;
                }
            }
            if err == ERROR_NONE {
                for _ in 0..FTDI_NUM_TRIES {
                    err = self.i2c_set_start();
                    err |= self.i2c_send_byte_check_ack(addr | 0x01);
                    err |= self.i2c_read_byte_send_nak(val);
                    err |= self.i2c_set_stop();
                    err |= self.i2c_output();
                    if err == ERROR_NONE {
                        break;
                    }
                }
            }
            timeout += 1;
            if err == ERROR_NONE || timeout == FTDI_RDWR_TIMEOUT {
                break;
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: i2c read reg16 0x{:02x}, 0x{:04x}", addr, reg);
        }
        err
    }

    pub fn i2c_write_reg16(&mut self, addr: u8, reg: u16, val: u8) -> u8 {
        let mut err: u8;
        let mut timeout = 0;
        loop {
            err = ERROR_NONE;
            for _ in 0..FTDI_NUM_TRIES {
                err = self.i2c_set_start();
                err |= self.i2c_send_byte_check_ack(addr);
                err |= self.i2c_send_byte_check_ack((reg >> 8) as u8);
                err |= self.i2c_send_byte_check_ack((reg & 0xff) as u8);
                err |= self.i2c_send_byte_check_ack(val);
                err |= self.i2c_set_stop();
                err |= self.i2c_output();
                if err == ERROR_NONE {
                    break;
                }
            }
            timeout += 1;
            if err == ERROR_NONE || timeout == FTDI_RDWR_TIMEOUT {
                break;
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: i2c write reg16 0x{:02x}, 0x{:04x}, 0x{:02x}", addr, reg, val);
        }
        err
    }

    pub fn i2c_read_reg8(&mut self, addr: u8, reg: u8, val: &mut u8) -> u8 {
        let mut err: u8;
        let mut timeout = 0;
        loop {
            err = ERROR_NONE;
            for _ in 0..FTDI_NUM_TRIES {
                err = self.i2c_set_start();
                err |= self.i2c_send_byte_check_ack(addr);
                err |= self.i2c_send_byte_check_ack(reg);
                err |= self.i2c_set_stop();
                err |= self.i2c_output();
                if err == ERROR_NONE {
                    break;
                }
            }
            if err == ERROR_NONE {
                for _ in 0..FTDI_NUM_TRIES {
                    err = self.i2c_set_start();
                    err |= self.i2c_send_byte_check_ack(addr | 0x01);
                    err |= self.i2c_read_byte_send_nak(val);
                    err |= self.i2c_set_stop();
                    err |= self.i2c_output();
                    if err == ERROR_NONE {
                        break;
                    }
                }
            }
            timeout += 1;
            if err == ERROR_NONE || timeout == FTDI_RDWR_TIMEOUT {
                break;
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: i2c read reg8 0x{:02x}, 0x{:02x}", addr, reg);
        }
        err
    }

    pub fn i2c_write_reg8(&mut self, addr: u8, reg: u8, val: u8) -> u8 {
        let mut err: u8;
        let mut timeout = 0;
        loop {
            err = ERROR_NONE;
            for _ in 0..FTDI_NUM_TRIES {
                err = self.i2c_set_start();
                err |= self.i2c_send_byte_check_ack(addr);
                err |= self.i2c_send_byte_check_ack(reg);
                err |= self.i2c_send_byte_check_ack(val);
                err |= self.i2c_set_stop();
                err |= self.i2c_output();
                if err == ERROR_NONE {
                    break;
                }
            }
            timeout += 1;
            if err == ERROR_NONE || timeout == FTDI_RDWR_TIMEOUT {
                break;
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: i2c_write reg8 0x{:02x}, 0x{:02x}, 0x{:02x}", addr, reg, val);
        }
        err
    }

    /// --fast-i2c: write many 16-bit-address registers with one USB write
    /// and one read per batch. The MPSSE commands for each register are
    /// exactly those of i2c_write_reg16() - START, four bytes each followed
    /// by an ACK bit read, STOP - but queued back to back with a single
    /// "send immediate" at the end, so the bus sees the same waveform while
    /// the USB round trip (four per register in the C) is paid once per
    /// batch. Every ACK is still read and checked: a batch with any NAK is
    /// written again register by register through the C's path, retries
    /// and all. 32 registers make 128 ACK bytes, well inside one 512-byte
    /// USB packet, so the replies never straddle an FTDI status header.
    pub fn i2c_write_reg16_batch(&mut self, addr: u8, regs: &[(u16, u8)]) -> u8 {
        const BATCH: usize = 32;
        for chunk in regs.chunks(BATCH) {
            self.out.clear();
            for &(reg, val) in chunk {
                self.i2c_set_start();
                for b in [addr, (reg >> 8) as u8, (reg & 0xff) as u8, val] {
                    self.push(&[0x80, 0x00, 0x13, 0x11, 0x00, 0x00, b, 0x80, 0x00, 0x11, 0x27, 0x00]);
                }
                self.i2c_set_stop();
            }
            self.push(&[0x87]);
            let err = self.flush();
            if err != ERROR_NONE {
                return err;
            }
            let mut nak = false;
            for _ in 0..chunk.len() * 4 {
                match self.read_byte() {
                    Ok(v) => nak |= v & 0x01 != 0,
                    Err(e) => return e,
                }
            }
            if nak {
                for &(reg, val) in chunk {
                    let err = self.i2c_write_reg16(addr, reg, val);
                    if err != ERROR_NONE {
                        return err;
                    }
                }
            }
        }
        ERROR_NONE
    }

    /// ftdi_gpio_write(): set AC<pin_id>. Discards any queued MPSSE bytes
    /// first, and ignores a failed write, as the C does.
    fn gpio_write(&mut self, pin_id: u8, pin_value: bool) -> u8 {
        println!("Flow: FTDI GPIO Write: pin {} -> value {}", pin_id, pin_value as i32);
        if pin_value {
            self.gpio_value |= 1 << pin_id;
        } else {
            self.gpio_value &= !(1 << pin_id);
        }
        self.out.clear();
        let (v, d) = (self.gpio_value, self.gpio_direction);
        self.push(&[0x82, v, d]);
        let _ = self.flush();
        ERROR_NONE
    }

    /// ftdi_nim_reset(): pulse the NIM's reset line low for 10ms.
    fn nim_reset(&mut self) -> u8 {
        println!("Flow: FTDI nim reset");
        self.gpio_write(FTDI_GPIO_PINID_NIM_RESET, false);
        self.clock.usleep(10000);
        self.gpio_write(FTDI_GPIO_PINID_NIM_RESET, true);
        self.clock.usleep(10000);
        ERROR_NONE
    }

    /// ftdi_set_polarisation_supply(): the RT5047A LNB supply on an add-on
    /// board - select 13V/18V first, then enable; or disable.
    pub fn set_polarisation_supply(&mut self, supply_enable: bool, supply_horizontal: bool) -> u8 {
        if supply_enable {
            self.gpio_write(FTDI_GPIO_PINID_LNB_BIAS_VSEL, supply_horizontal);
            self.gpio_write(FTDI_GPIO_PINID_LNB_BIAS_ENABLE, true);
        } else {
            self.gpio_write(FTDI_GPIO_PINID_LNB_BIAS_ENABLE, false);
        }
        ERROR_NONE
    }
}

/// What ftdi_init() hands back: the I2C side, the TS side, and whether the
/// device is a PicoTuner in serial-TS mode (stv0910_serialTS).
pub struct FtdiOpened {
    pub ftdi: Ftdi,
    pub ts: TsUsb,
    pub serial_ts: bool,
    /// A PicoTuner, which has a second TS endpoint (--second).
    pub pico: bool,
}

/// ftdi_init(): open and claim both interfaces, put both in MPSSE mode,
/// sync the command channel and reset the NIM.
///
/// `ts_latency_ms` is --ts-latency (None, as the C, leaves it alone).
pub fn ftdi_init(
    backend: &dyn UsbBackend,
    clock: Arc<dyn Clock>,
    usb_bus: u8,
    usb_addr: u8,
    ts_latency_ms: Option<u8>,
) -> Result<FtdiOpened, u8> {
    println!("Flow: FTDI/Pico init");
    let r = (|| {
        let mut serial_ts = false;
        let i2c = ftdi_usb_init(backend, 0, usb_bus, usb_addr, FTDI_VID, FTDI_PID, PICO_VID, PICO_PID)?;
        let pico = i2c.pico.unwrap_or(false);
        if let Some(s) = i2c.serial_ts {
            serial_ts = s;
        }
        let mut i2c = I2cUsb::new(i2c.link);
        let err = i2c.set_mpsse_mode(&*clock);
        if err != ERROR_NONE {
            return Err(err);
        }
        let ts = ftdi_usb_init(backend, 1, usb_bus, usb_addr, FTDI_VID, FTDI_PID, PICO_VID, PICO_PID)?;
        if let Some(s) = ts.serial_ts {
            serial_ts = s;
        }
        let mut ts = TsUsb::new(ts.link);
        let err = ts.set_mpsse_mode(&*clock);
        if err != ERROR_NONE {
            return Err(err);
        }
        if let Some(ms) = ts_latency_ms {
            let err = ts.set_latency_interface_b(ms);
            if err != ERROR_NONE {
                return Err(err);
            }
        }
        let mut ftdi = Ftdi::new(i2c, clock.clone());
        let err = ftdi.setup_ftdi_io();
        if err != ERROR_NONE {
            return Err(err);
        }
        let err = ftdi.nim_reset();
        if err != ERROR_NONE {
            return Err(err);
        }
        Ok(FtdiOpened { ftdi, ts, serial_ts, pico })
    })();
    if r.is_err() {
        println!("ERROR: FTDI/Pico init");
    }
    r
}
