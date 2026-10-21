//! stvvglna.c: the two STVVGLNA LNAs on newer Serit NIMs (one per input).
//! Older NIMs have none, which longmynd detects by the first read failing.

use crate::errors::*;
use crate::nim::{Nim, NIM_INPUT_TOP, NIM_LNA_0_ADDR, NIM_LNA_1_ADDR};
use crate::regs::stvvglna_regs::*;

const STVVGLNA_AGC_TIMEOUT: u16 = 20;
pub const STVVGLNA_OFF: u8 = 0;
pub const STVVGLNA_ON: u8 = 1;

impl Nim {
    fn stvvglna_read_reg(&mut self, lna_addr: u8, reg: u8, val: &mut u8) -> u8 {
        self.nim_read_lna(lna_addr, reg, val)
    }

    fn stvvglna_write_reg(&mut self, lna_addr: u8, reg: u8, val: u8) -> u8 {
        self.nim_write_lna(lna_addr, reg, val)
    }

    /// Ask the LNA to measure its gain, wait for it, read SWLNAGAIN and
    /// VGO. As in the C, the masks are applied even when a read failed,
    /// and the REG0 read's result is ignored.
    pub fn stvvglna_read_agc(&mut self, input: u8, gain: &mut u8, vgo: &mut u8) -> u8 {
        let mut err;
        let mut timeout: u16 = 0;
        let mut status: u8 = 0;
        let lna_addr = if input == NIM_INPUT_TOP { NIM_LNA_0_ADDR } else { NIM_LNA_1_ADDR };
        // the C keeps this write's result in err, then overwrites it with
        // the first read's before looking at it
        let _ = self.stvvglna_write_reg(lna_addr, STVVGLNA_REG1, STVVGLNA_REG1_GETAGC_START << STVVGLNA_REG1_GETAGC_SHIFT);
        loop {
            err = self.stvvglna_read_reg(lna_addr, STVVGLNA_REG1, &mut status);
            timeout += 1;
            if err == ERROR_NONE && timeout == STVVGLNA_AGC_TIMEOUT {
                err = ERROR_LNA_AGC_TIMEOUT;
                println!("Error: read AGC timeout");
            }
            if !(err == ERROR_NONE && ((status >> STVVGLNA_REG1_GETAGC_SHIFT) & 1) != STVVGLNA_REG1_GETAGC_FORCED) {
                break;
            }
        }
        let _ = self.stvvglna_read_reg(lna_addr, STVVGLNA_REG0, &mut status);
        if err == ERROR_NONE {
            err = self.stvvglna_read_reg(lna_addr, STVVGLNA_REG3, gain);
        }
        *gain = (*gain & STVVGLNA_REG3_SWLNAGAIN_MASK) >> STVVGLNA_REG3_SWLNAGAIN_SHIFT;
        if err == ERROR_NONE {
            err = self.stvvglna_read_reg(lna_addr, STVVGLNA_REG1, vgo);
        }
        *vgo = (*vgo & STVVGLNA_REG1_VGO_MASK) >> STVVGLNA_REG1_VGO_SHIFT;
        if err != ERROR_NONE {
            println!("ERROR: Failed LNA aquire AGC {}", input);
        }
        err
    }

    /// Find out whether this input has an LNA; if so check its ID and turn
    /// it on (fully automatic AGC) or off.
    pub fn stvvglna_init(&mut self, input: u8, state: u8, lna_ok: &mut bool) -> u8 {
        let mut val: u8 = 0;
        println!("Flow: LNA init {}", input);
        let lna_addr = if input == NIM_INPUT_TOP { NIM_LNA_0_ADDR } else { NIM_LNA_1_ADDR };
        let mut err = self.stvvglna_read_reg(lna_addr, STVVGLNA_REG0, &mut val);
        if err != ERROR_NONE {
            println!("      Status: found an older NIM with no LNA");
            *lna_ok = false;
            err = ERROR_NONE;
        } else {
            println!("      Status: found new NIM with LNAs");
            *lna_ok = true;
            if (val & STVVGLNA_REG0_IDENT_MASK) != STVVGLNA_REG0_IDENT_DEFAULT {
                println!("ERROR: failed to recognise LNA ID {} {}", input, val);
                err = ERROR_LNA_ID;
            }
            if state == STVVGLNA_ON {
                // --lna-tlock, --lna-pref, --lna-mode replace the C's choices
                let tlock = match self.ext.lna_tlock_fast {
                    Some(true) => STVVGLNA_REG0_AGC_TLOCK_FAST,
                    _ => STVVGLNA_REG0_AGC_TLOCK_SLOW,
                };
                let pref = self.ext.lna_pref.unwrap_or(STVVGLNA_REG2_RFAGC_PREF_N20DBM);
                let mode = self.ext.lna_mode.unwrap_or(STVVGLNA_REG2_RFAGC_MODE_AUTO_TRACK);
                if err == ERROR_NONE {
                    err = self.stvvglna_write_reg(
                        lna_addr,
                        STVVGLNA_REG0,
                        (STVVGLNA_REG0_AGC_TUPD_FAST << STVVGLNA_REG0_AGC_TUPD_SHIFT)
                            | (tlock << STVVGLNA_REG0_AGC_TLOCK_SHIFT),
                    );
                }
                if err == ERROR_NONE {
                    err = self.stvvglna_write_reg(
                        lna_addr,
                        STVVGLNA_REG1,
                        (STVVGLNA_REG1_LNAGC_PWD_POWER_ON << STVVGLNA_REG1_LNAGC_PWD_SHIFT)
                            | (STVVGLNA_REG1_GETOFF_ACQUISITION_MODE << STVVGLNA_REG1_GETOFF_SHIFT)
                            | (STVVGLNA_REG1_GETAGC_FORCED << STVVGLNA_REG1_GETAGC_SHIFT),
                    );
                }
                if err == ERROR_NONE {
                    err = self.stvvglna_write_reg(
                        lna_addr,
                        STVVGLNA_REG2,
                        (STVVGLNA_REG2_PATH_ACTIVE << STVVGLNA_REG2_PATH2OFF_SHIFT)
                            | (pref << STVVGLNA_REG2_RFAGC_PREF_SHIFT)
                            | (STVVGLNA_REG2_PATH_ACTIVE << STVVGLNA_REG2_PATH1OFF_SHIFT)
                            | (mode << STVVGLNA_REG2_RFAGC_MODE_SHIFT),
                    );
                }
                // SWLNAGAIN is read-only in fully automatic mode
                if err == ERROR_NONE {
                    err = self.stvvglna_write_reg(
                        lna_addr,
                        STVVGLNA_REG3,
                        (STVVGLNA_REG3_LCAL_17KHZ << STVVGLNA_REG3_LCAL_SHIFT)
                            | (STVVGLNA_REG3_RFAGC_UPDATE_FORCED << STVVGLNA_REG3_RFAGC_UPDATE_SHIFT)
                            | (STVVGLNA_REG3_RFAGC_CALSTART_FORCED << STVVGLNA_REG3_RFAGC_CALSTART_SHIFT),
                    );
                }
            } else if err == ERROR_NONE {
                err = self.stvvglna_write_reg(
                    lna_addr,
                    STVVGLNA_REG2,
                    (STVVGLNA_REG2_PATH_OFF << STVVGLNA_REG2_PATH2OFF_SHIFT)
                        | (STVVGLNA_REG2_PATH_OFF << STVVGLNA_REG2_PATH1OFF_SHIFT),
                );
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: LNA init {}", input);
        }
        err
    }
}
