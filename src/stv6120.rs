//! stv6120.c: the STV6120 dual tuner. longmynd uses tuner 1 only (tuner 2
//! is powered down), fed from the TOP F-connector, or the BOTTOM one with
//! -w.

use crate::errors::*;
use crate::nim::{Nim, NIM_TUNER_XTAL};
use crate::regs::stv6120_regs::*;

pub const TUNER_1: u8 = 0;
pub const TUNER_2: u8 = 1;

const STV6120_RDIV_THRESHOLD: u32 = 27000;
const STV6120_P_THRESHOLD_1: u32 = 299000;
const STV6120_P_THRESHOLD_2: u32 = 596000;
const STV6120_P_THRESHOLD_3: u32 = 1191000;
const STV6120_CAL_TIMEOUT_MS: u64 = 50;
const STV6120_PLL_TIMEOUT_MS: u64 = 50;
pub const STV6120_PLL_ATTEMPTS: i32 = 10;
const STV6120_LPFCAL_TIMEOUT_MS: u64 = 50;

/// Charge pump current by VCO frequency (kHz), from the datasheet.
const STV6120_ICP_LOOKUP: [[u32; 3]; 7] = [
    [2380000, 2472000, 0],
    [2473000, 2700000, 1],
    [2701000, 3021000, 2],
    [3022000, 3387000, 3],
    [3388000, 3845000, 5],
    [3846000, 4394000, 6],
    [4395000, 4760000, 7],
];

/// Cut-off of the HF filter for each CFHF setting, MHz.
const STV6120_CFHF: [u16; 32] = [
    6796, 5828, 4778, 4118, 3513, 3136, 2794, 2562, 2331, 2169, 2006, 1890, 1771, 1680, 1586, 1514, 1433, 1374, 1310,
    1262, 1208, 1167, 1122, 1087, 1049, 1018, 983, 956, 926, 902, 875, 854,
];

/// PLL settings for a frequency, as stv6120_calc_pll() works them out.
struct Pll {
    p: u8,
    f_vco: u32,
    n: u16,
    f: u32,
    icp: u8,
    cfhf: u8,
}

impl Nim {
    fn stv6120_read_reg(&mut self, reg: u8, val: &mut u8) -> u8 {
        self.nim_read_tuner(reg, val)
    }

    fn stv6120_write_reg(&mut self, reg: u8, val: u8) -> u8 {
        self.nim_write_tuner(reg, val)
    }

    /// Low-pass filter calibration: enable its clock, start, wait for the
    /// start bit to clear, restore the clock setting. The C polls STAT1
    /// for either tuner, so for tuner 2 it waits on tuner 1's bit, which
    /// is already clear; it never calibrates tuner 2, so that has never
    /// mattered. Tuner 2 (only ever used by --second) polls STAT2 here.
    fn stv6120_cal_lowpass(&mut self, tuner: u8) -> u8 {
        let mut err;
        let mut val: u8 = 0;
        println!("Flow: Tuner cal lowpass");
        if tuner == TUNER_1 {
            err = self.stv6120_write_reg(STV6120_CTRL7, self.ctrl7 & !(1 << STV6120_CTRL7_RCCLKOFF_SHIFT));
        } else {
            err = self.stv6120_write_reg(STV6120_CTRL16, self.ctrl16 & !(1 << STV6120_CTRL7_RCCLKOFF_SHIFT));
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                if tuner == TUNER_1 { STV6120_STAT1 } else { STV6120_STAT2 },
                STV6120_STAT1_CALRCSTRT_START << STV6120_STAT1_CALRCSTRT_SHIFT,
            );
        }
        if err == ERROR_NONE {
            let timeout = self.ftdi.clock.monotonic_ms() + STV6120_LPFCAL_TIMEOUT_MS;
            loop {
                err = self.stv6120_read_reg(if tuner == TUNER_1 { STV6120_STAT1 } else { STV6120_STAT2 }, &mut val);
                if self.ftdi.clock.monotonic_ms() >= timeout {
                    err = ERROR_TUNER_CAL_LOWPASS_TIMEOUT;
                    println!("ERROR: tuner wait on CAL_lowpass timed out");
                }
                if !(err == ERROR_NONE
                    && (val & (1 << STV6120_STAT1_CALRCSTRT_SHIFT)) == (1 << STV6120_STAT1_CALRCSTRT_SHIFT))
                {
                    break;
                }
            }
        }
        if err == ERROR_NONE {
            let (reg, v) = if tuner == TUNER_1 { (STV6120_CTRL7, self.ctrl7) } else { (STV6120_CTRL16, self.ctrl16) };
            err = self.stv6120_write_reg(reg, v);
        }
        if err != ERROR_NONE {
            println!("ERROR: Failed to cal lowpass filter");
        }
        err
    }

    fn stv6120_calc_pll(&self, freq: u32) -> Pll {
        println!("Flow: Tuner set freq");
        let p: u8 = if freq <= STV6120_P_THRESHOLD_1 {
            3 // P=16
        } else if freq <= STV6120_P_THRESHOLD_2 {
            2 // P=8
        } else if freq <= STV6120_P_THRESHOLD_3 {
            1 // P=4
        } else {
            0 // P=2
        };
        // F.vco = F.rf * P, kHz, in uint32_t
        let f_vco = freq << (p + 1);
        let n = ((f_vco << self.rdiv) / NIM_TUNER_XTAL) as u16;
        let f = (((((f_vco << self.rdiv) % NIM_TUNER_XTAL) as u64) << 18) / NIM_TUNER_XTAL as u64) as u32;
        // the C's lookup runs off the end of its table for a VCO above
        // 4760000kHz, which longmynd's 2450MHz limit never reaches
        let mut pos = 0;
        loop {
            let row = pos.min(STV6120_ICP_LOOKUP.len() - 1);
            pos += 1;
            if f_vco <= STV6120_ICP_LOOKUP[row][1] || pos >= STV6120_ICP_LOOKUP.len() {
                break;
            }
        }
        let icp = STV6120_ICP_LOOKUP[pos - 1][2] as u8;
        let mut cfhf: u8 = 0;
        while (cfhf as usize) < STV6120_CFHF.len() && (3 * freq / 1000) <= STV6120_CFHF[cfhf as usize] as u32 {
            cfhf += 1;
        }
        // "we are sure it isn't greater than the first array element" - but
        // above 2265MHz (3 x f > 6796MHz) it is, the loop never runs, and
        // 0 - 1 wraps to 255: CTRL8 then gets 0xff, the narrowest filter
        // (854MHz) for a 2.4GHz signal plus every TCAL bit. --hf-filter-fix
        // takes the widest filter instead.
        let cfhf = if cfhf == 0 && self.ext.hf_filter_fix { 0 } else { cfhf.wrapping_sub(1) };
        Pll { p, f_vco, n, f, icp, cfhf }
    }

    /// Program one tuner's PLL to `freq` kHz, recalibrate the VCO and wait
    /// for lock (each with a 50ms timeout on the monotonic clock).
    pub(crate) fn stv6120_set_freq(&mut self, tuner: u8, freq: u32) -> u8 {
        let mut err = ERROR_NONE;
        let mut val: u8 = 0;
        println!("Flow: Tuner set freq");
        let Pll { p, f_vco, n, f, icp, cfhf } = self.stv6120_calc_pll(freq);
        println!(
            "      Status: tuner:{}, f_vco=0x{:x}, icp=0x{:x}, f=0x{:x}, n=0x{:x},",
            tuner, f_vco, icp, f, n
        );
        println!(
            "              rdiv=0x{:x}, p=0x{:x}, freq={}, cfhf={}",
            self.rdiv,
            p,
            freq as i32,
            STV6120_CFHF.get(cfhf as usize).copied().unwrap_or(0)
        );
        let t1 = tuner == TUNER_1;
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(if t1 { STV6120_CTRL3 } else { STV6120_CTRL12 }, (n & 0x00ff) as u8);
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                if t1 { STV6120_CTRL4 } else { STV6120_CTRL13 },
                (((f & 0x0000007f) << 1) | ((n as u32 & 0x0100) >> 8)) as u8,
            );
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(if t1 { STV6120_CTRL5 } else { STV6120_CTRL14 }, ((f & 0x00007f80) >> 7) as u8);
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                if t1 { STV6120_CTRL6 } else { STV6120_CTRL15 },
                (((f & 0x00038000) >> 15) as u8) | (icp << STV6120_CTRL6_ICP_SHIFT) | STV6120_CTRL6_RESERVED,
            );
        }
        if t1 {
            if err == ERROR_NONE {
                err = self.stv6120_write_reg(STV6120_CTRL7, (p << STV6120_CTRL7_PDIV_SHIFT) | self.ctrl7);
            }
            if err == ERROR_NONE {
                err = self.stv6120_write_reg(STV6120_CTRL8, (cfhf << STV6120_CTRL8_CFHF_SHIFT) | self.ctrl8);
            }
        } else {
            if err == ERROR_NONE {
                err = self.stv6120_write_reg(STV6120_CTRL16, (p << STV6120_CTRL7_PDIV_SHIFT) | self.ctrl16);
            }
            if err == ERROR_NONE {
                err = self.stv6120_write_reg(STV6120_CTRL17, (cfhf << STV6120_CTRL8_CFHF_SHIFT) | self.ctrl17);
            }
        }
        let stat = if t1 { STV6120_STAT1 } else { STV6120_STAT2 };
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                stat,
                (STV6120_STAT1_CALVCOSTRT_START << STV6120_STAT1_CALVCOSTRT_SHIFT) | STV6120_STAT1_RESERVED,
            );
        }
        if err == ERROR_NONE {
            let timeout = self.ftdi.clock.monotonic_ms() + STV6120_CAL_TIMEOUT_MS;
            loop {
                err = self.stv6120_read_reg(stat, &mut val);
                if !(err == ERROR_NONE
                    && self.ftdi.clock.monotonic_ms() < timeout
                    && (val & (1 << STV6120_STAT1_CALVCOSTRT_SHIFT))
                        != (STV6120_STAT1_CALVCOSTRT_FINISHED << STV6120_STAT1_CALVCOSTRT_SHIFT))
                {
                    break;
                }
            }
            if err == ERROR_NONE && self.ftdi.clock.monotonic_ms() >= timeout {
                println!("ERROR: tuner wait on CAL timed out");
                err = ERROR_TUNER_CAL_TIMEOUT;
            }
        }
        if err == ERROR_NONE {
            let timeout = self.ftdi.clock.monotonic_ms() + STV6120_PLL_TIMEOUT_MS;
            loop {
                err = self.stv6120_read_reg(stat, &mut val);
                if !(err == ERROR_NONE
                    && self.ftdi.clock.monotonic_ms() < timeout
                    && (val & (1 << STV6120_STAT1_LOCK_SHIFT)) != (STV6120_STAT1_LOCK_LOCKED << STV6120_STAT1_LOCK_SHIFT))
                {
                    break;
                }
            }
            if err == ERROR_NONE && self.ftdi.clock.monotonic_ms() >= timeout {
                println!("ERROR: tuner wait on lock timed out");
                err = ERROR_TUNER_LOCK_TIMEOUT;
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: Tuner set freq {}", freq as i32);
        }
        err
    }

    /// stv6120_init(): set up both paths (a frequency of 0 powers one
    /// down), route TOP/BOTTOM inputs, calibrate and tune.
    pub fn stv6120_init(&mut self, freq_tuner_1: u32, freq_tuner_2: u32, swap: bool) -> u8 {
        let mut err = ERROR_NONE;
        println!("Flow: Tuner init");
        // K from F_xtal/(K+16) = 1MHz
        let k: u8 = (NIM_TUNER_XTAL / 1000 - 16) as u8;
        self.rdiv = if NIM_TUNER_XTAL >= STV6120_RDIV_THRESHOLD { 1 } else { 0 };
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                STV6120_CTRL1,
                (k << STV6120_CTRL1_K_SHIFT)
                    | (self.rdiv << STV6120_CTRL1_RDIV_SHIFT)
                    | (STV6120_CTRL1_OSHAPE_SINE << STV6120_CTRL1_OSHAPE_SHIFT)
                    | (STV6120_CTRL1_MCLKDIV_4 << STV6120_CTRL1_MCLKDIV_SHIFT),
            );
        }
        let path_on_bbgain = |bbgain: u8| {
            (STV6120_CTRL2_DCLOOPOFF_ENABLE << STV6120_CTRL2_DCLOOPOFF_SHIFT)
                | (STV6120_CTRL2_SDOFF_OFF << STV6120_CTRL2_SDOFF_SHIFT)
                | (STV6120_CTRL2_SYN_ON << STV6120_CTRL2_SYN_SHIFT)
                | (STV6120_CTRL2_REFOUTSEL_1_25V << STV6120_CTRL2_REFOUTSEL_SHIFT)
                | (bbgain << STV6120_CTRL2_BBGAIN_SHIFT)
        };
        let path_off = (STV6120_CTRL2_DCLOOPOFF_DISABLE << STV6120_CTRL2_DCLOOPOFF_SHIFT)
            | (STV6120_CTRL2_SDOFF_ON << STV6120_CTRL2_SDOFF_SHIFT)
            | (STV6120_CTRL2_SYN_OFF << STV6120_CTRL2_SYN_SHIFT)
            | (STV6120_CTRL2_REFOUTSEL_1_25V << STV6120_CTRL2_REFOUTSEL_SHIFT)
            | (STV6120_CTRL2_BBGAIN_0DB << STV6120_CTRL2_BBGAIN_SHIFT);
        // path 1 (both branches leave the RC clock off for now)
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                STV6120_CTRL2,
                if freq_tuner_1 > 0 { path_on_bbgain(STV6120_CTRL2_BBGAIN_0DB) } else { path_off },
            );
        }
        if err == ERROR_NONE {
            self.ctrl7 = (STV6120_CTRL7_RCCLKOFF_DISABLE << STV6120_CTRL7_RCCLKOFF_SHIFT)
                | (STV6120_CTRL7_CF_5MHZ << STV6120_CTRL7_CF_SHIFT);
            err = self.stv6120_write_reg(STV6120_CTRL7, self.ctrl7);
        }
        // tcal for both tuners; note the C shifts CALTIME by TCAL_SHIFT too
        if err == ERROR_NONE {
            self.ctrl8 = (STV6120_CTRL8_TCAL_DIV_2 << STV6120_CTRL8_TCAL_SHIFT)
                | (STV6120_CTRL8_CALTIME_500US << STV6120_CTRL8_TCAL_SHIFT);
            err = self.stv6120_write_reg(STV6120_CTRL8, self.ctrl8);
        }
        // RFB is the TOP input, RFC the BOTTOM
        let lna = |on: bool| if on { STV6120_CTRL10_LNA_ON } else { STV6120_CTRL10_LNA_OFF };
        let path = |on: bool| if on { STV6120_CTRL10_PATH_ON } else { STV6120_CTRL10_PATH_OFF };
        if swap {
            if err == ERROR_NONE {
                err = self.stv6120_write_reg(
                    STV6120_CTRL9,
                    (STV6120_CTRL9_RFSEL_RFC_IN << STV6120_CTRL9_RFSEL_1_SHIFT)
                        | (STV6120_CTRL9_RFSEL_RFB_IN << STV6120_CTRL9_RFSEL_2_SHIFT)
                        | STV6120_CTRL9_RESERVED,
                );
            }
            if err == ERROR_NONE {
                err = self.stv6120_write_reg(
                    STV6120_CTRL10,
                    (STV6120_CTRL10_LNA_OFF << STV6120_CTRL10_LNADON_SHIFT)
                        | (lna(freq_tuner_2 > 0) << STV6120_CTRL10_LNABON_SHIFT)
                        | (lna(freq_tuner_1 > 0) << STV6120_CTRL10_LNACON_SHIFT)
                        | (STV6120_CTRL10_LNA_OFF << STV6120_CTRL10_LNAAON_SHIFT)
                        | (path(freq_tuner_2 > 0) << STV6120_CTRL10_PATHON_2_SHIFT)
                        | (path(freq_tuner_1 > 0) << STV6120_CTRL10_PATHON_1_SHIFT),
                );
            }
        } else {
            if err == ERROR_NONE {
                err = self.stv6120_write_reg(
                    STV6120_CTRL9,
                    (STV6120_CTRL9_RFSEL_RFB_IN << STV6120_CTRL9_RFSEL_1_SHIFT)
                        | (STV6120_CTRL9_RFSEL_RFC_IN << STV6120_CTRL9_RFSEL_2_SHIFT)
                        | STV6120_CTRL9_RESERVED,
                );
            }
            if err == ERROR_NONE {
                err = self.stv6120_write_reg(
                    STV6120_CTRL10,
                    (STV6120_CTRL10_LNA_OFF << STV6120_CTRL10_LNADON_SHIFT)
                        | (lna(freq_tuner_2 > 0) << STV6120_CTRL10_LNACON_SHIFT)
                        | (lna(freq_tuner_1 > 0) << STV6120_CTRL10_LNABON_SHIFT)
                        | (STV6120_CTRL10_LNA_OFF << STV6120_CTRL10_LNAAON_SHIFT)
                        | (path(freq_tuner_2 > 0) << STV6120_CTRL10_PATHON_2_SHIFT)
                        | (path(freq_tuner_1 > 0) << STV6120_CTRL10_PATHON_1_SHIFT),
                );
            }
        }
        // path 2
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                STV6120_CTRL11,
                if freq_tuner_2 > 0 { path_on_bbgain(STV6120_CTRL2_BBGAIN_6DB) } else { path_off },
            );
        }
        if err == ERROR_NONE {
            self.ctrl16 = if freq_tuner_2 > 0 {
                (STV6120_CTRL7_RCCLKOFF_ENABLE << STV6120_CTRL7_RCCLKOFF_SHIFT) | (STV6120_CTRL7_CF_5MHZ << STV6120_CTRL7_CF_SHIFT)
            } else {
                (STV6120_CTRL7_RCCLKOFF_DISABLE << STV6120_CTRL7_RCCLKOFF_SHIFT) | (STV6120_CTRL7_CF_5MHZ << STV6120_CTRL7_CF_SHIFT)
            };
            err = self.stv6120_write_reg(STV6120_CTRL16, self.ctrl16);
        }
        // the C remembers ctrl17 but writes ctrl8 to CTRL17
        if err == ERROR_NONE {
            self.ctrl17 = STV6120_CTRL8_CALTIME_500US << STV6120_CTRL8_TCAL_SHIFT;
            err = self.stv6120_write_reg(STV6120_CTRL17, self.ctrl8);
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(STV6120_CTRL18, STV6120_CTRL18_DEFAULT);
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(STV6120_CTRL19, STV6120_CTRL19_DEFAULT);
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                STV6120_CTRL20,
                (STV6120_CTRL20_VCOAMP_NORMAL << STV6120_CTRL20_VCOAMP_SHIFT) | STV6120_CTRL20_RESERVED,
            );
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(STV6120_CTRL21, STV6120_CTRL21_DEFAULT);
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(STV6120_CTRL22, STV6120_CTRL22_DEFAULT);
        }
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(
                STV6120_CTRL23,
                (STV6120_CTRL20_VCOAMP_NORMAL << STV6120_CTRL20_VCOAMP_SHIFT) | STV6120_CTRL20_RESERVED,
            );
        }
        if err == ERROR_NONE && freq_tuner_1 > 0 {
            err = self.stv6120_cal_lowpass(TUNER_1);
            if err == ERROR_NONE {
                err = self.stv6120_set_freq(TUNER_1, freq_tuner_1);
            }
        }
        if err == ERROR_NONE && freq_tuner_2 > 0 {
            err = self.stv6120_cal_lowpass(TUNER_2);
            if err == ERROR_NONE {
                err = self.stv6120_set_freq(TUNER_2, freq_tuner_2);
            }
        }
        if err != ERROR_NONE {
            println!("ERROR: Failed to init Tuner {}, {}", freq_tuner_1 as i32, freq_tuner_2 as i32);
        }
        err
    }

    /// Power down both signal paths (synthesizer, mixer, gain stage), used
    /// between attempts when the PLL will not lock.
    pub fn stv6120_powerdown_both_paths(&mut self) -> u8 {
        println!("Flow: Powering down both stv6120 signal paths");
        let off = (STV6120_CTRL2_DCLOOPOFF_DISABLE << STV6120_CTRL2_DCLOOPOFF_SHIFT)
            | (STV6120_CTRL2_SDOFF_ON << STV6120_CTRL2_SDOFF_SHIFT)
            | (STV6120_CTRL2_SYN_OFF << STV6120_CTRL2_SYN_SHIFT)
            | (STV6120_CTRL2_REFOUTSEL_1_25V << STV6120_CTRL2_REFOUTSEL_SHIFT)
            | (STV6120_CTRL2_BBGAIN_0DB << STV6120_CTRL2_BBGAIN_SHIFT);
        let mut err = self.stv6120_write_reg(STV6120_CTRL2, off);
        if err == ERROR_NONE {
            err = self.stv6120_write_reg(STV6120_CTRL11, off);
        }
        err
    }
}
