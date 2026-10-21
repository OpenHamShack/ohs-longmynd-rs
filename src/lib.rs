//! longmynd-rs: a drop-in Rust port of BATC's longmynd, the receiver
//! software for the MiniTiouner and PicoTuner (Serit NIM: STV0910
//! demodulator, STV6120 tuner, STVVGLNA LNAs).
//!
//! Derived from longmynd, Copyright 2019 Heather Lomond, and BATC's
//! contributors (Phil Crump's libts among them); GPLv3 or later. The module
//! names follow the C files they port, so the two can be read side by side.

pub mod app;
pub mod charset;
#[rustfmt::skip]
mod charset_tables;
pub mod clock;
pub mod cnum;
pub mod config;
pub mod d2xx;
pub mod errors;
pub mod ftdi;
pub mod ftdi_usb;
pub mod libts;
pub mod nim;
pub mod output;
pub mod receiver;
pub mod status;
pub mod stv0910;
pub mod stv6120;
pub mod stvvglna;
pub mod ts;
pub mod usb;

#[rustfmt::skip]
pub mod regs {
    pub mod stv0910_regs;
    pub mod stv0910_regs_init;
    pub mod stv6120_regs;
    pub mod stvvglna_regs;
}
