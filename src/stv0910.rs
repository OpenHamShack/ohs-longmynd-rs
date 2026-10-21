//! stv0910.c and stv0910_utils.c: the STV0910 demodulator.
//!
//! The arithmetic is the C's, type for type: where the C does a sum in
//! uint32_t that can wrap, so does this (wrapping_*), and where it goes
//! through double and back, so does this, with the x86-64 conversion
//! behaviour the reference build has (see cnum).

use crate::cnum::{f32_to_u32_x86, f64_to_i32_x86, f64_to_u32_x86};
use crate::errors::*;
use crate::nim::{Nim, NIM_TUNER_XTAL, STV0910_START_ADDR};
use crate::regs::stv0910_regs::*;
use crate::regs::stv0910_regs_init::STV0910_DEF_VAL;

pub const DEMOD_HUNTING: u8 = 0;
pub const DEMOD_FOUND_HEADER: u8 = 1;
pub const DEMOD_S2: u8 = 2;
pub const DEMOD_S: u8 = 3;

const STV0910_PLL_LOCK_TIMEOUT: u16 = 100;
const STV0910_SCAN_BLIND_BEST_GUESS: u8 = 0x15;

pub const STV0910_DEMOD_TOP: u8 = 1;
pub const STV0910_DEMOD_BOTTOM: u8 = 2;

const STV0910_PUNCTURE_1_2: u8 = 0x0d;
const STV0910_PUNCTURE_2_3: u8 = 0x12;
const STV0910_PUNCTURE_3_4: u8 = 0x15;
const STV0910_PUNCTURE_5_6: u8 = 0x18;
const STV0910_PUNCTURE_6_7: u8 = 0x19;
const STV0910_PUNCTURE_7_8: u8 = 0x1a;

/// Pick the top (P2) or bottom (P1) demodulator's register or field.
fn pick<T>(demod: u8, top: T, bottom: T) -> T {
    if demod == STV0910_DEMOD_TOP {
        top
    } else {
        bottom
    }
}

impl Nim {
    // ---- stv0910_utils.c ------------------------------------------------

    /// Change a bitfield via the shadow registers (read-modify-write with
    /// no read).
    pub fn stv0910_write_reg_field(&mut self, field: u32, field_val: u8) -> u8 {
        let mut err = ERROR_NONE;
        let reg = (field >> 16) as u16;
        let idx = (reg - STV0910_START_ADDR) as usize;
        // C: field_val << shift, in int, then truncated to the uint8_t val
        let val = ((self.stv0910_shadow_regs[idx] as u32 & !(field & 0xff))
            | ((field_val as u32) << ((field >> 12) & 0x0f))) as u8;
        if err == ERROR_NONE {
            err = self.nim_write_demod(reg, val);
        }
        if err == ERROR_NONE {
            self.stv0910_shadow_regs[idx] = val;
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 write field");
        }
        err
    }

    /// Read a bitfield from the chip (not the shadows: these are volatile).
    pub fn stv0910_read_reg_field(&mut self, field: u32, field_val: &mut u8) -> u8 {
        let mut err = ERROR_NONE;
        // the C reads into an uninitialised val and masks it even on error
        let mut val: u8 = 0;
        if err == ERROR_NONE {
            err = self.nim_read_demod((field >> 16) as u16, &mut val);
        }
        *field_val = ((val as u32 & (field & 0xff)) >> ((field >> 12) & 0x0f)) as u8;
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read field");
        }
        err
    }

    pub fn stv0910_write_reg(&mut self, reg: u16, val: u8) -> u8 {
        self.stv0910_shadow_regs[(reg - STV0910_START_ADDR) as usize] = val;
        self.nim_write_demod(reg, val)
    }

    pub fn stv0910_read_reg(&mut self, reg: u16, val: &mut u8) -> u8 {
        self.nim_read_demod(reg, val)
    }

    // ---- stv0910.c -------------------------------------------------------

    /// Carrier offset in Hz: a signed 24-bit CFR scaled by MCLK/2^24.
    pub fn stv0910_read_car_freq(&mut self, demod: u8, cf: &mut i32) -> u8 {
        let (mut val_h, mut val_m, mut val_l) = (0u8, 0u8, 0u8);
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_CFR2, RSTV0910_P1_CFR2), &mut val_h);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_CFR1, RSTV0910_P1_CFR1), &mut val_m);
        }
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_CFR0, RSTV0910_P1_CFR0), &mut val_l);
        }
        let raw = (((val_h as u32) << 16).wrapping_add((val_m as u32) << 8).wrapping_add(val_l as u32)) << 8;
        let mut car_offset_freq = (raw as i32) as f64;
        car_offset_freq = 135000000.0 * car_offset_freq / 256.0 / 256.0 / 256.0 / 256.0;
        *cf = f64_to_i32_x86(car_offset_freq);
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read carrier frequency");
        }
        err
    }

    pub fn stv0910_read_constellation(&mut self, demod: u8, i: &mut u8, q: &mut u8) -> u8 {
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_ISYMB, RSTV0910_P1_ISYMB), i);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_QSYMB, RSTV0910_P1_QSYMB), q);
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read constellation");
        }
        err
    }

    /// Symbol rate in symbols/s: SFR (32 bits) scaled by MCLK/2^32.
    pub fn stv0910_read_sr(&mut self, demod: u8, found_sr: &mut u32) -> u8 {
        let (mut val_h, mut val_mu, mut val_ml, mut val_l) = (0u8, 0u8, 0u8, 0u8);
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_SFR3, RSTV0910_P1_SFR3), &mut val_h);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_SFR2, RSTV0910_P1_SFR2), &mut val_mu);
        }
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_SFR1, RSTV0910_P1_SFR1), &mut val_ml);
        }
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_SFR0, RSTV0910_P1_SFR0), &mut val_l);
        }
        let mut sr = (((val_h as u32) << 24)
            .wrapping_add((val_mu as u32) << 16)
            .wrapping_add((val_ml as u32) << 8)
            .wrapping_add(val_l as u32)) as f64;
        sr = 135000000.0 * sr / 256.0 / 256.0 / 256.0 / 256.0;
        *found_sr = f64_to_u32_x86(sr);
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read symbol rate");
        }
        err
    }

    /// The Viterbi puncture rate n of rate n/(n+1). Any other register
    /// value is an error - which, in the HUNTING state, stops longmynd.
    pub fn stv0910_read_puncture_rate(&mut self, demod: u8, rate: &mut u8) -> u8 {
        let mut val: u8 = 0;
        let mut err = self.stv0910_read_reg_field(pick(demod, FSTV0910_P2_VIT_CURPUN, FSTV0910_P1_VIT_CURPUN), &mut val);
        match val {
            STV0910_PUNCTURE_1_2 => *rate = 1,
            STV0910_PUNCTURE_2_3 => *rate = 2,
            STV0910_PUNCTURE_3_4 => *rate = 3,
            STV0910_PUNCTURE_5_6 => *rate = 5,
            STV0910_PUNCTURE_6_7 => *rate = 6,
            STV0910_PUNCTURE_7_8 => *rate = 7,
            _ => err = ERROR_VITERBI_PUNCTURE_RATE,
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read puncture rate");
        }
        err
    }

    pub fn stv0910_read_agc1_gain(&mut self, demod: u8, agc: &mut u16) -> u8 {
        let (mut agc_low, mut agc_high) = (0u8, 0u8);
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_AGCIQIN0, RSTV0910_P1_AGCIQIN0), &mut agc_low);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_AGCIQIN1, RSTV0910_P1_AGCIQIN1), &mut agc_high);
        }
        if err == ERROR_NONE {
            *agc = (agc_high as u16) << 8 | agc_low as u16;
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read agc1 gain");
        }
        err
    }

    pub fn stv0910_read_agc2_gain(&mut self, demod: u8, agc: &mut u16) -> u8 {
        let (mut agc_low, mut agc_high) = (0u8, 0u8);
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_AGC2I0, RSTV0910_P1_AGC2I0), &mut agc_low);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_AGC2I1, RSTV0910_P1_AGC2I1), &mut agc_high);
        }
        if err == ERROR_NONE {
            *agc = (agc_high as u16) << 8 | agc_low as u16;
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read agc2 gain");
        }
        err
    }

    pub fn stv0910_read_power(&mut self, demod: u8, power_i: &mut u8, power_q: &mut u8) -> u8 {
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_POWERI, RSTV0910_P1_POWERI), power_i);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_POWERQ, RSTV0910_P1_POWERQ), power_q);
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read power");
        }
        err
    }

    /// Viterbi error rate in hundredths of a percent (0xff = 6.23%).
    pub fn stv0910_read_err_rate(&mut self, demod: u8, vit_errs: &mut u32) -> u8 {
        let mut val: u8 = 0;
        let err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_VERROR, RSTV0910_P1_VERROR), &mut val);
        *vit_errs = ((val as u32) * 100000 / 4096 + 5) / 10;
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read viterbi error rate");
        }
        err
    }

    /// BER x 10000 from the FEC byte and error counters. Note the C's
    /// second group of reads restarts err from scratch, so an error in the
    /// first group is forgotten - reproduced.
    pub fn stv0910_read_ber(&mut self, demod: u8, ber: &mut u32) -> u8 {
        let (mut high, mut mid_u, mut mid_m, mut mid_l, mut low) = (0u8, 0u8, 0u8, 0u8, 0u8);
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_FBERCPT4, RSTV0910_P1_FBERCPT4), &mut high);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_FBERCPT3, RSTV0910_P1_FBERCPT3), &mut mid_u);
        }
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_FBERCPT2, RSTV0910_P1_FBERCPT2), &mut mid_m);
        }
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_FBERCPT1, RSTV0910_P1_FBERCPT1), &mut mid_l);
        }
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_FBERCPT0, RSTV0910_P1_FBERCPT0), &mut low);
        }
        let _ = err;
        let cpt = high as f64 * 256.0 * 256.0 * 256.0 * 256.0
            + mid_u as f64 * 256.0 * 256.0 * 256.0
            + mid_m as f64 * 256.0 * 256.0
            + mid_l as f64 * 256.0
            + low as f64;
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_FBERERR2, RSTV0910_P1_FBERERR2), &mut high);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_FBERERR1, RSTV0910_P1_FBERERR1), &mut mid_m);
        }
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_FBERERR0, RSTV0910_P1_FBERERR0), &mut low);
        }
        let errs = high as f64 * 256.0 * 256.0 + mid_m as f64 * 256.0 + low as f64;
        *ber = f64_to_u32_x86(10000.0 * errs / (cpt * 8.0));
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read BER");
        }
        err
    }

    /// MER (DVB-S2 CNR) in tenths of a dB from the noise RAM; when no value
    /// is valid it asks for one and reports 0.
    pub fn stv0910_read_mer(&mut self, demod: u8, mer: &mut i32) -> u8 {
        let (mut high, mut low) = (0u8, 0u8);
        let mut err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_NOSRAMPOS, RSTV0910_P1_NOSRAMPOS), &mut high);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_NOSRAMVAL, RSTV0910_P1_NOSRAMVAL), &mut low);
        }
        if (high >> 2) & 0x01 == 1 {
            if (high >> 1) & 0x01 == 1 {
                *mer = ((((high & 0x01) as i32) << 8) | low as i32) - 512;
            } else {
                *mer = (((high & 0x01) as i32) << 8) | low as i32;
            }
        } else {
            *mer = 0;
            if err == ERROR_NONE {
                err = self.stv0910_write_reg_field(
                    pick(demod, FSTV0910_P2_NOSRAM_ACTIVATION, FSTV0910_P1_NOSRAM_ACTIVATION),
                    0x02,
                );
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read DVBS2 MER");
        }
        err
    }

    pub fn stv0910_read_errors_bch_uncorrected(&mut self, _demod: u8, errors_bch_uncorrected: &mut bool) -> u8 {
        let mut result: u8 = 0;
        // chip-wide, not per demodulator
        let err = self.stv0910_read_reg_field(FSTV0910_ERRORFLAG, &mut result);
        *errors_bch_uncorrected = result == 0;
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read BCH Errors Uncorrected");
        }
        err
    }

    pub fn stv0910_read_errors_bch_count(&mut self, _demod: u8, errors_bch_count: &mut u32) -> u8 {
        let mut result: u8 = 0;
        let err = self.stv0910_read_reg_field(FSTV0910_BCH_ERRORS_COUNTER, &mut result);
        *errors_bch_count = result as u32;
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read BCH Errors Count");
        }
        err
    }

    pub fn stv0910_read_errors_ldpc_count(&mut self, _demod: u8, errors_ldpc_count: &mut u32) -> u8 {
        let (mut high, mut low) = (0u8, 0u8);
        let mut err = self.stv0910_read_reg_field(FSTV0910_LDPC_ERRORS1, &mut high);
        if err == ERROR_NONE {
            err = self.stv0910_read_reg_field(FSTV0910_LDPC_ERRORS0, &mut low);
        }
        *errors_ldpc_count = (high as u32) << 8 | low as u32;
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read LDPC Errors Count");
        }
        err
    }

    pub fn stv0910_read_modcod_and_type(&mut self, demod: u8, modcod: &mut u32, short_frame: &mut bool, pilots: &mut bool) -> u8 {
        let mut regval: u8 = 0;
        let err = self.stv0910_read_reg(pick(demod, RSTV0910_P2_DMDMODCOD, RSTV0910_P1_DMDMODCOD), &mut regval);
        *modcod = ((regval & 0x7c) >> 2) as u32;
        *short_frame = (regval & 0x02) != 0;
        *pilots = (regval & 0x01) != 0;
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read MODCOD");
        }
        err
    }

    /// stv0910_setup_clocks(): program the PLL for a 135MHz MCLK from the
    /// 30MHz crystal and wait for it to lock.
    fn stv0910_setup_clocks(&mut self) -> u8 {
        let mut err = ERROR_NONE;
        let mut lock: u8 = 0;
        let mut timeout: u16 = 0;
        println!("Flow: STV0910 set MCLK");
        let odf: u32 = 4;
        let idf: u32 = 1;
        if err == ERROR_NONE {
            err = self.stv0910_write_reg_field(FSTV0910_ODF, odf as u8);
        }
        if err == ERROR_NONE {
            err = self.stv0910_write_reg_field(FSTV0910_IDF, idf as u8);
        }
        let f_xtal = NIM_TUNER_XTAL / 1000;
        let f_phi: u32 = 135000000 / 1000000;
        let ndiv = (f_phi * odf * idf) / f_xtal;
        if err == ERROR_NONE {
            err = self.stv0910_write_reg_field(FSTV0910_N_DIV, ndiv as u8);
        }
        let cp: u8 = 7;
        if err == ERROR_NONE {
            err = self.stv0910_write_reg_field(FSTV0910_CP, cp);
        }
        if err == ERROR_NONE {
            err = self.stv0910_write_reg_field(FSTV0910_STANDBY, 0);
        }
        if err == ERROR_NONE {
            err = self.stv0910_write_reg_field(FSTV0910_BYPASSPLLCORE, 0);
        }
        loop {
            timeout += 1;
            if timeout == STV0910_PLL_LOCK_TIMEOUT {
                err = ERROR_DEMOD_PLL_TIMEOUT;
                println!("ERROR: STV0910 pll lock timeout");
            }
            if err == ERROR_NONE {
                // the read's own error is ignored, as in the C
                let _ = self.stv0910_read_reg_field(FSTV0910_PLLLOCK, &mut lock);
            }
            if !(err == ERROR_NONE && lock == 0) {
                break;
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 set MCLK");
        }
        err
    }

    fn stv0910_setup_equalisers(&mut self, demod: u8) -> u8 {
        println!("Flow: Setup equlaizers {}", demod);
        ERROR_NONE
    }

    /// Carrier search range +/- halfscan_sr (kHz) around the tuned centre.
    /// `halfscan_sr * 65536` is uint32_t arithmetic in the C and wraps for
    /// large -S values; so does this.
    fn stv0910_setup_carrier_loop(&mut self, demod: u8, halfscan_sr: u32) -> u8 {
        println!("Flow: Setup carrier loop {}", demod);
        let mut err = self.stv0910_write_reg(pick(demod, RSTV0910_P2_CFRINIT0, RSTV0910_P1_CFRINIT0), 0);
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(pick(demod, RSTV0910_P2_CFRINIT1, RSTV0910_P1_CFRINIT1), 0);
        }
        let mut temp: i64 = (halfscan_sr.wrapping_mul(65536) / 135000) as i64;
        if err == ERROR_NONE {
            // the C assigns both results to err, so only the second counts
            let _ = self.stv0910_write_reg(pick(demod, RSTV0910_P2_CFRUP0, RSTV0910_P1_CFRUP0), (temp & 0xff) as u8);
            err = self.stv0910_write_reg(pick(demod, RSTV0910_P2_CFRUP1, RSTV0910_P1_CFRUP1), ((temp >> 8) & 0xff) as u8);
        }
        temp = -temp;
        if err == ERROR_NONE {
            let _ = self.stv0910_write_reg(pick(demod, RSTV0910_P2_CFRLOW0, RSTV0910_P1_CFRLOW0), (temp & 0xff) as u8);
            err = self.stv0910_write_reg(pick(demod, RSTV0910_P2_CFRLOW1, RSTV0910_P1_CFRLOW1), ((temp >> 8) & 0xff) as u8);
        }
        err
    }

    /// Initial symbol rate SFRINIT = sr(kHz) * 2^16 / 135000. For the
    /// bottom demodulator the C writes the high byte to SFRINIT0 and the
    /// low to SFRINIT1 (swapped). The C never starts the bottom one, so
    /// the bug has never shown; longmynd-rs does only for --second, and
    /// there writes them the right way round (high byte to SFRINIT1, as
    /// for the top one). No default path reaches the bottom branch.
    fn stv0910_setup_timing_loop(&mut self, demod: u8, sr: u32) -> u8 {
        let mut err = ERROR_NONE;
        println!("Flow: Setup timing loop {}", demod);
        let sr_reg = ((sr << 16) / 135 / 1000) as u16;
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(pick(demod, RSTV0910_P2_SFRINIT1, RSTV0910_P1_SFRINIT1), (sr_reg >> 8) as u8);
        }
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(pick(demod, RSTV0910_P2_SFRINIT0, RSTV0910_P1_SFRINIT0), (sr_reg & 0xFF) as u8);
        }
        err
    }

    /// --fast-retune: move the top demodulator and tuner 1 to a new symbol
    /// rate and frequency without reloading anything else. Between scan
    /// steps only the tuner's PLL, the demodulator's initial symbol rate
    /// (SFRINIT) and its carrier search range change; the rest of what
    /// nim_init() writes is the same table every time. So: stop the
    /// demodulator (DMDISTATE 0x1c, as stv0910_init does), set the carrier
    /// and timing loops, reset the LDPC decoder as the init does, and
    /// re-tune (set_freq recalibrates the VCO and waits for lock). The
    /// caller then starts the scan as after a full init. Equaliser and AGC
    /// state from the previous step is not reset - ST's own drivers re-tune
    /// this way, but it is why this needs a scan test on hardware.
    pub fn stv0910_fast_retune(&mut self, sr: u32, halfscan_ratio: f32, freq: u32) -> u8 {
        println!("Flow: Fast re-tune to {} KHz, {} KSymbols/s", freq as i32, sr as i32);
        let mut err = self.stv0910_write_reg(RSTV0910_P2_DMDISTATE, 0x1c);
        if err == ERROR_NONE {
            err = self.stv0910_setup_carrier_loop(STV0910_DEMOD_TOP, f32_to_u32_x86(sr as f32 * halfscan_ratio));
        }
        if err == ERROR_NONE {
            err = self.stv0910_setup_timing_loop(STV0910_DEMOD_TOP, sr);
        }
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(RSTV0910_TSTRES0, 0x80);
        }
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(RSTV0910_TSTRES0, 0x00);
        }
        if err == ERROR_NONE {
            err = self.stv6120_set_freq(crate::stv6120::TUNER_1, freq);
        }
        err
    }

    /// Start a blind search with best guess (DMDISTATE = 0x15).
    pub fn stv0910_start_scan(&mut self, demod: u8) -> u8 {
        let mut err = ERROR_NONE;
        println!("Flow: STV0910 start scan");
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(pick(demod, RSTV0910_P2_DMDISTATE, RSTV0910_P1_DMDISTATE), STV0910_SCAN_BLIND_BEST_GUESS);
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 start scan");
        }
        err
    }

    /// HEADER_MODE: hunting, found header, DVB-S2 or DVB-S.
    pub fn stv0910_read_scan_state(&mut self, demod: u8, state: &mut u8) -> u8 {
        let mut err = ERROR_NONE;
        if err == ERROR_NONE {
            err = self.stv0910_read_reg_field(pick(demod, FSTV0910_P2_HEADER_MODE, FSTV0910_P1_HEADER_MODE), state);
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 read scan state");
        }
        err
    }

    /// Check the chip IDs and write the whole power-on table.
    fn stv0910_init_regs(&mut self) -> u8 {
        let (mut val1, mut val2) = (0u8, 0u8);
        println!("Flow: stv0910 init regs");
        let mut err = self.nim_read_demod(0xf100, &mut val1);
        if err == ERROR_NONE {
            err = self.nim_read_demod(0xf101, &mut val2);
        }
        println!("      Status: STV0910 MID = 0x{:02x}, DID = 0x{:02x}", val1, val2);
        if val1 != 0x51 || val2 != 0x20 {
            print!("ERROR: read the wrong stv0910 MID/DID");
            return ERROR_DEMOD_INIT;
        }
        if self.ext.fast_i2c {
            if err == ERROR_NONE {
                for &(reg, val) in STV0910_DEF_VAL.iter() {
                    self.stv0910_shadow_regs[(reg - STV0910_START_ADDR) as usize] = val;
                }
                err = self.nim_write_demod_batch(&STV0910_DEF_VAL);
            }
        } else {
            for &(reg, val) in STV0910_DEF_VAL.iter() {
                if err == ERROR_NONE {
                    err = self.stv0910_write_reg(reg, val);
                }
            }
        }
        // a PicoTuner in serial TS mode: the defaults above are parallel
        if self.stv0910_serial_ts {
            if err == ERROR_NONE {
                err = self.stv0910_write_reg(RSTV0910_P1_TSCFGH, 0x40);
            }
            if err == ERROR_NONE {
                err = self.stv0910_write_reg(RSTV0910_P2_TSCFGH, 0x40);
            }
        }
        // from ST's example code: reset the LDPC decoder
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(RSTV0910_TSTRES0, 0x80);
        }
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(RSTV0910_TSTRES0, 0x00);
        }
        err
    }

    /// stv0910_init(): stop both demodulators, load the defaults, start the
    /// clocks, set up whichever demodulators have a symbol rate (kS/s).
    pub fn stv0910_init(&mut self, sr1: u32, sr2: u32, halfscan_ratio1: f32, halfscan_ratio2: f32) -> u8 {
        let mut err = ERROR_NONE;
        println!("Flow: STV0910 init");
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(RSTV0910_P1_DMDISTATE, 0x1c);
        }
        if err == ERROR_NONE {
            err = self.stv0910_write_reg(RSTV0910_P2_DMDISTATE, 0x1c);
        }
        if err == ERROR_NONE {
            err = self.stv0910_init_regs();
        }
        if err == ERROR_NONE {
            err = self.stv0910_setup_clocks();
        }
        if sr1 != 0 {
            if err == ERROR_NONE {
                err = self.stv0910_setup_equalisers(STV0910_DEMOD_TOP);
            }
            if err == ERROR_NONE {
                // uint32 * float is a float multiply in C, then truncated
                err = self.stv0910_setup_carrier_loop(STV0910_DEMOD_TOP, f32_to_u32_x86(sr1 as f32 * halfscan_ratio1));
            }
            if err == ERROR_NONE {
                err = self.stv0910_setup_timing_loop(STV0910_DEMOD_TOP, sr1);
            }
        }
        if sr2 != 0 {
            if err == ERROR_NONE {
                err = self.stv0910_setup_equalisers(STV0910_DEMOD_BOTTOM);
            }
            if err == ERROR_NONE {
                err = self.stv0910_setup_carrier_loop(STV0910_DEMOD_BOTTOM, f32_to_u32_x86(sr2 as f32 * halfscan_ratio2));
            }
            if err == ERROR_NONE {
                err = self.stv0910_setup_timing_loop(STV0910_DEMOD_BOTTOM, sr2);
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: STV0910 init");
        }
        err
    }
}
