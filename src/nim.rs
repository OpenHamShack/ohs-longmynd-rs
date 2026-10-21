//! nim.c: the Serit NIM as a whole - the STV0910 demodulator on the I2C bus
//! directly, the STV6120 tuner and the two STVVGLNA LNAs behind the
//! demodulator's I2C repeater.
//!
//! In the C every chip driver is a set of functions over file-level
//! globals (shadow registers, the repeater flag, tuner control-register
//! copies). Here they are the fields of one [`Nim`], and the drivers are
//! `impl Nim` blocks in stv0910.rs, stv6120.rs and stvvglna.rs - the same
//! functions, owned by the receiver thread that is the only one to call
//! them.

use crate::errors::*;
use crate::ftdi::Ftdi;
use crate::regs::stv0910_regs::{RSTV0910_MID, RSTV0910_TSTTSRS};
use crate::regs::stvvglna_regs::{STVVGLNA_I2C_ADDR0, STVVGLNA_I2C_ADDR3};

pub const NIM_DEMOD_ADDR: u8 = 0xd2;
pub const NIM_TUNER_ADDR: u8 = 0xc0;
pub const NIM_LNA_0_ADDR: u8 = STVVGLNA_I2C_ADDR3;
pub const NIM_LNA_1_ADDR: u8 = STVVGLNA_I2C_ADDR0;
/// kHz
pub const NIM_TUNER_XTAL: u32 = 30000;
/// Hz
#[allow(dead_code)]
pub const NIM_DEMOD_MCLK: u32 = 135_000_000;
pub const NIM_INPUT_TOP: u8 = 1;
pub const NIM_INPUT_BOTTOM: u8 = 2;

pub(crate) const STV0910_START_ADDR: u16 = RSTV0910_MID;
pub(crate) const STV0910_END_ADDR: u16 = RSTV0910_TSTTSRS;

pub struct Nim {
    pub ftdi: Ftdi,
    /// The demodulator's I2C repeater is open (tuner/LNA traffic). Opened
    /// on demand and closed again before the next demodulator access,
    /// which keeps the tuner's I2C lines quiet the rest of the time.
    repeater_on: bool,
    /// stv0910_shadow_regs: what was last written to each demodulator
    /// register, so a bitfield can be changed without a read.
    pub(crate) stv0910_shadow_regs: Vec<u8>,
    /// stv0910_serialTS (the PicoTuner patch): the device reads TS serially.
    pub stv0910_serial_ts: bool,
    // stv6120.c's globals
    pub(crate) rdiv: u8,
    pub(crate) ctrl7: u8,
    pub(crate) ctrl8: u8,
    pub(crate) ctrl16: u8,
    pub(crate) ctrl17: u8,
    /// longmynd-rs's opt-in behaviour (config::Extensions); default = C.
    pub ext: crate::config::Extensions,
}

impl Nim {
    pub fn new(ftdi: Ftdi, serial_ts: bool, ext: crate::config::Extensions) -> Self {
        Nim {
            ftdi,
            repeater_on: false,
            stv0910_shadow_regs: vec![0; (STV0910_END_ADDR - STV0910_START_ADDR + 1) as usize],
            stv0910_serial_ts: serial_ts,
            rdiv: 0,
            ctrl7: 0,
            ctrl8: 0,
            ctrl16: 0,
            ctrl17: 0,
            ext,
        }
    }

    pub fn nim_read_demod(&mut self, reg: u16, val: &mut u8) -> u8 {
        let mut err = ERROR_NONE;
        // bit 7 of Px_I2CRPT is the repeater; the other bits set I2C speed
        if self.repeater_on {
            self.repeater_on = false;
            err = self.nim_write_demod(0xf12a, 0x38);
        }
        if err == ERROR_NONE {
            err = self.ftdi.i2c_read_reg16(NIM_DEMOD_ADDR, reg, val);
        }
        if err != ERROR_NONE {
            println!("ERROR: demod read 0x{:04x}", reg);
        }
        err
    }

    pub fn nim_write_demod(&mut self, reg: u16, val: u8) -> u8 {
        let mut err = ERROR_NONE;
        if self.repeater_on {
            self.repeater_on = false;
            err = self.nim_write_demod(0xf12a, 0x38);
        }
        if err == ERROR_NONE {
            err = self.ftdi.i2c_write_reg16(NIM_DEMOD_ADDR, reg, val);
        }
        if err != ERROR_NONE {
            println!("ERROR: demod write 0x{:04x}, 0x{:02x}", reg, val);
        }
        err
    }

    fn open_repeater(&mut self) -> u8 {
        let mut err = ERROR_NONE;
        if !self.repeater_on {
            err = self.nim_write_demod(0xf12a, 0xb8);
            self.repeater_on = true;
        }
        err
    }

    /// --fast-i2c's batched demodulator writes (see Ftdi::
    /// i2c_write_reg16_batch); closes the repeater first, as
    /// nim_write_demod() would.
    pub fn nim_write_demod_batch(&mut self, regs: &[(u16, u8)]) -> u8 {
        let mut err = ERROR_NONE;
        if self.repeater_on {
            self.repeater_on = false;
            err = self.nim_write_demod(0xf12a, 0x38);
        }
        if err == ERROR_NONE {
            err = self.ftdi.i2c_write_reg16_batch(NIM_DEMOD_ADDR, regs);
        }
        if err != ERROR_NONE {
            println!("ERROR: demod batch write");
        }
        err
    }

    pub fn nim_read_lna(&mut self, lna_addr: u8, reg: u8, val: &mut u8) -> u8 {
        let mut err = self.open_repeater();
        if err == ERROR_NONE {
            err = self.ftdi.i2c_read_reg8(lna_addr, reg, val);
        }
        if err != ERROR_NONE {
            println!("ERROR: lna read 0x{:02x}, 0x{:02x}", lna_addr, reg);
        }
        err
    }

    pub fn nim_write_lna(&mut self, lna_addr: u8, reg: u8, val: u8) -> u8 {
        let mut err = self.open_repeater();
        if err == ERROR_NONE {
            err = self.ftdi.i2c_write_reg8(lna_addr, reg, val);
        }
        if err != ERROR_NONE {
            println!("ERROR: lna write 0x{:02x}, 0x{:02x},0x{:02x}", lna_addr, reg, val);
        }
        err
    }

    pub fn nim_read_tuner(&mut self, reg: u8, val: &mut u8) -> u8 {
        let mut err = self.open_repeater();
        if err == ERROR_NONE {
            err = self.ftdi.i2c_read_reg8(NIM_TUNER_ADDR, reg, val);
        }
        if err != ERROR_NONE {
            println!("ERROR: tuner read 0x{:02x}", reg);
        }
        err
    }

    pub fn nim_write_tuner(&mut self, reg: u8, val: u8) -> u8 {
        let mut err = self.open_repeater();
        if err == ERROR_NONE {
            err = self.ftdi.i2c_write_reg8(NIM_TUNER_ADDR, reg, val);
        }
        if err != ERROR_NONE {
            println!("ERROR: tuner write {},{}", reg, val);
        }
        err
    }

    /// nim_init(): prove the demodulator answers by writing a register with
    /// alternating bits and reading it back.
    pub fn nim_init(&mut self) -> u8 {
        let mut err = ERROR_NONE;
        let mut val: u8 = 0;
        println!("Flow: NIM init");
        self.repeater_on = false;
        if err == ERROR_NONE {
            err = self.nim_write_demod(0xf536, 0xaa);
        }
        if err == ERROR_NONE {
            err = self.nim_read_demod(0xf536, &mut val);
        }
        if err == ERROR_NONE && val != 0xaa {
            println!("ERROR: nim_init didn't r/w to the modulator");
            err = ERROR_NIM_INIT;
        }
        // "we always want to start with the i2c repeater turned off" - but
        // the C tests err != ERROR_NONE here, so it only does so after a
        // failure, and then overwrites the error with the write's result:
        // a NIM that fails the read-back check is reported as fine and
        // longmynd carries on with it. The write is kept (on the failure
        // path only, as in the C); the error is kept too, unless the write
        // itself fails. Error path only: a working NIM never gets here.
        if err != ERROR_NONE {
            let w = self.nim_write_demod(0xf12a, 0x38);
            if w != ERROR_NONE {
                err = w;
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: nim_init");
        }
        err
    }
}
