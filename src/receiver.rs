//! main.c's loop_i2c() and do_report(): the receiver thread. Every 100ms it
//! applies a new configuration if there is one (re-initialising the whole
//! NIM), reads the demodulator's state and measurements, and publishes them
//! for main() to write out.

use std::sync::atomic::Ordering;
use std::sync::Arc;

use crate::clock::Clock;
use crate::errors::*;
use crate::nim::{Nim, NIM_INPUT_BOTTOM, NIM_INPUT_TOP};
use crate::status::*;
use crate::stv0910::*;
use crate::stv6120::STV6120_PLL_ATTEMPTS;
use crate::stvvglna::{STVVGLNA_OFF, STVVGLNA_ON};

/// Milliseconds between receiver iterations.
pub const I2C_LOOP_MS: u64 = 100;

/// do_report(): read everything the status output shows, for `demod`
/// (the C only ever reports the top one; --second the bottom too).
/// `port_swap` is the configuration's -w, which picks the LNA read.
fn do_report(nim: &mut Nim, status: &mut Status, port_swap: bool, demod: u8) -> u8 {
    let mut err = ERROR_NONE;

    // The C always reads the TOP input's LNA, even with -w; its return
    // value is ignored and its uninitialised outputs used regardless.
    // --lna-follow-input reads the one in use; --no-lna-poll none.
    if status.lna_ok && nim.ext.lna_poll {
        let input = if demod != STV0910_DEMOD_TOP {
            // --second: tuner 2 takes the input tuner 1 does not
            if port_swap {
                NIM_INPUT_TOP
            } else {
                NIM_INPUT_BOTTOM
            }
        } else if nim.ext.lna_follow_input && port_swap {
            NIM_INPUT_BOTTOM
        } else {
            NIM_INPUT_TOP
        };
        let (mut lna_gain, mut lna_vgo) = (0u8, 0u8);
        if err == ERROR_NONE {
            let _ = nim.stvvglna_read_agc(input, &mut lna_gain, &mut lna_vgo);
        }
        status.lna_gain = ((lna_gain as u16) << 5) | lna_vgo as u16;
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_agc1_gain(demod, &mut status.agc1_gain);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_agc2_gain(demod, &mut status.agc2_gain);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_power(demod, &mut status.power_i, &mut status.power_q);
    }
    if err == ERROR_NONE {
        for count in 0..NUM_CONSTELLATIONS {
            if err != ERROR_NONE {
                break;
            }
            let [mut i, mut q] = status.constellation[count];
            err = nim.stv0910_read_constellation(demod, &mut i, &mut q);
            status.constellation[count] = [i, q];
        }
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_puncture_rate(demod, &mut status.puncture_rate);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_car_freq(demod, &mut status.frequency_offset);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_sr(demod, &mut status.symbolrate);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_err_rate(demod, &mut status.viterbi_error_rate);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_ber(demod, &mut status.bit_error_rate);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_errors_bch_uncorrected(demod, &mut status.errors_bch_uncorrected);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_errors_bch_count(demod, &mut status.errors_bch_count);
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_errors_ldpc_count(demod, &mut status.errors_ldpc_count);
    }
    if status.state == STATE_DEMOD_S || status.state == STATE_DEMOD_S2 {
        if err == ERROR_NONE {
            err = nim.stv0910_read_mer(demod, &mut status.modulation_error_rate);
        }
    } else {
        status.modulation_error_rate = 0;
    }
    if err == ERROR_NONE {
        err = nim.stv0910_read_modcod_and_type(
            demod,
            &mut status.modcod,
            &mut status.short_frame,
            &mut status.pilots,
        );
    }
    if status.state != STATE_DEMOD_S2 {
        // short frames and pilots only mean something in DVB-S2
        status.short_frame = false;
        status.pilots = false;
    }
    // --extra-status: roll-off from TMGOBS (one more register read per
    // report, which is why it is not done without the option)
    if nim.ext.extra_status && err == ERROR_NONE {
        let mut ro = 0u8;
        use crate::regs::stv0910_regs::{FSTV0910_P1_ROLLOFF_STATUS, FSTV0910_P2_ROLLOFF_STATUS};
        let field = if demod == STV0910_DEMOD_TOP { FSTV0910_P2_ROLLOFF_STATUS } else { FSTV0910_P1_ROLLOFF_STATUS };
        err = nim.stv0910_read_reg_field(field, &mut ro);
        status.rolloff = rolloff_hundredths(ro);
    }
    err
}

/// TMGOBS ROLLOFF_STATUS as hundredths: 0.35, 0.25, 0.20 (ST's STV0910
/// register description); the fourth code is not a DVB-S2 roll-off, 0.
pub fn rolloff_hundredths(field: u8) -> u8 {
    match field {
        0 => 35,
        1 => 25,
        2 => 20,
        _ => 0,
    }
}

/// --extra-status's TS rate: bytes per second of TS payload, from the TS
/// thread's running count, over windows of at least a second.
struct TsRate {
    at_ms: u64,
    bytes: u64,
    rate: u32,
}

impl TsRate {
    fn update(&mut self, now_ms: u64, bytes: u64) -> u32 {
        if self.at_ms == 0 || bytes < self.bytes {
            (self.at_ms, self.bytes) = (now_ms, bytes);
        } else if now_ms >= self.at_ms + 1000 {
            self.rate = ((bytes - self.bytes) * 1000 / (now_ms - self.at_ms)) as u32;
            (self.at_ms, self.bytes) = (now_ms, bytes);
        }
        self.rate
    }
}

/// The state machine step for one state: report, read the demodulator's
/// scan state, move. `strict` is the HUNTING case, the only one where the
/// C guards the scan-state read with the report's error; in the others it
/// overwrites whatever error the report (or a failed re-init) left - so a
/// re-init that fails while locked is forgotten and longmynd runs on with
/// a half-initialised NIM, still showing the old lock. --strict-errors
/// guards every state, as HUNTING is.
fn step(nim: &mut Nim, s: &mut Status, err: &ErrCell, here: u8, strict: bool, port_swap: bool, demod: u8) {
    let strict = strict || nim.ext.strict_errors;
    if err.ok() {
        err.set(do_report(nim, s, port_swap, demod));
    }
    if strict {
        if err.ok() {
            err.set(nim.stv0910_read_scan_state(demod, &mut s.demod_state));
        }
    } else {
        err.set(nim.stv0910_read_scan_state(demod, &mut s.demod_state));
    }
    let next = match s.demod_state {
        DEMOD_HUNTING => Some(STATE_DEMOD_HUNTING),
        DEMOD_FOUND_HEADER => Some(STATE_DEMOD_FOUND_HEADER),
        DEMOD_S2 => Some(STATE_DEMOD_S2),
        DEMOD_S => Some(STATE_DEMOD_S),
        _ => None,
    };
    match next {
        Some(n) => {
            if n != here {
                s.state = n;
            }
        }
        None => {
            if err.ok() {
                println!("ERROR: demodulator returned a bad scan state");
                err.set(ERROR_BAD_DEMOD_HUNT_STATE);
            }
        }
    }
}

/// One receiver iteration's state machine, for one demodulator.
fn state_step(nim: &mut Nim, s: &mut Status, err: &ErrCell, port_swap: bool, demod: u8) {
    match s.state {
        STATE_DEMOD_HUNTING => step(nim, s, err, STATE_DEMOD_HUNTING, true, port_swap, demod),
        STATE_DEMOD_FOUND_HEADER => step(nim, s, err, STATE_DEMOD_FOUND_HEADER, false, port_swap, demod),
        STATE_DEMOD_S2 => step(nim, s, err, STATE_DEMOD_S2, false, port_swap, demod),
        STATE_DEMOD_S => step(nim, s, err, STATE_DEMOD_S, false, port_swap, demod),
        _ => err.set(ERROR_STATE),
    }
}

/// loop_i2c()
pub fn loop_i2c(nim: &mut Nim, shared: &Shared, clock: &Arc<dyn Clock>) {
    let err = ErrCell(&shared.thread_err_i2c);
    err.set(ERROR_NONE);

    // The C's status_cpy is an uninitialised local; everything the output
    // shows is set before it is published, except on error paths.
    let mut status_cpy = Status::default();
    let mut last_ts_packet_count: u32 = 0;
    let mut port_swap = false;
    // whether the last configuration applied turned the LNB supply on
    let mut lnb_supply_on = false;
    // the NIM has had a full init that succeeded (for --fast-retune)
    let mut nim_up = false;
    let mut ts_rate = TsRate { at_ms: 0, bytes: 0, rate: 0 };
    // --second: the bottom demodulator and tuner 2, and what they report
    let second = nim.ext.second.clone();
    let (sr2, f2) = second.as_ref().map(|s| (s.sr, s.freq)).unwrap_or((0, 0));
    let mut status2_cpy = Status::default();

    let mut last_i2c_loop = clock.monotonic_ms();
    while err.ok() && shared.main_err() == ERROR_NONE {
        loop {
            clock.usleep(10 * 1000);
            if clock.monotonic_ms() >= last_i2c_loop + I2C_LOOP_MS {
                break;
            }
        }

        status_cpy.last_ts_or_reinit_monotonic = 0;

        let new_config = {
            let mut cfg = shared.config.lock().unwrap();
            if cfg.new {
                let copy = cfg.clone();
                cfg.new = false;
                shared.ts_reset.store(true, Ordering::SeqCst);
                Some(copy)
            } else {
                None
            }
        };

        // --ts-timer-after-init: main() holds the no-TS timeout while this
        // runs, and the timer restarts from its end (below)
        let timer_after_init = nim.ext.ts_timer_after_init && new_config.is_some();
        if timer_after_init {
            shared.init_in_progress.store(true, Ordering::SeqCst);
        }

        // --fast-retune: with the NIM already up from a full init, and only
        // the frequency or symbol rate changed (all a scan step or a no-TS
        // re-init changes), re-tune in place. Any failure - the tuner not
        // locking included - falls back to the full init below, which has
        // the C's retries.
        let mut new_config = new_config;
        if let Some(config_cpy) = new_config.as_ref() {
            if nim.ext.fast_retune && nim_up && err.ok() {
                let fi = config_cpy.freq_index as usize;
                let si = config_cpy.sr_index as usize;
                let (freq, sr) = (config_cpy.freq_requested[fi], config_cpy.sr_requested[si]);
                let mut e = nim.stv0910_fast_retune(sr, config_cpy.halfscan_ratio, freq);
                if e == ERROR_NONE {
                    e = nim.stv0910_start_scan(STV0910_DEMOD_TOP);
                }
                if e == ERROR_NONE {
                    status_cpy.frequency_requested = freq;
                    status_cpy.symbolrate_requested = sr;
                    status_cpy.state = STATE_DEMOD_HUNTING;
                    status_cpy.last_ts_or_reinit_monotonic = clock.monotonic_ms();
                    new_config = None;
                } else {
                    println!("Flow: Fast re-tune failed, full re-init");
                }
            }
        }

        if let Some(config_cpy) = new_config {
            port_swap = config_cpy.port_swap;
            let fi = config_cpy.freq_index as usize;
            let si = config_cpy.sr_index as usize;
            status_cpy.frequency_requested = config_cpy.freq_requested[fi];
            status_cpy.symbolrate_requested = config_cpy.sr_requested[si];

            // kept apart so a handled tuner error does not stop longmynd
            let mut tuner_err = ERROR_NONE;
            let mut tuner_lock_attempts: i32 = STV6120_PLL_ATTEMPTS;
            loop {
                if err.ok() {
                    err.set(nim.nim_init());
                }
                // only the top demodulator and tuner 1 are used - unless
                // --second, which starts the bottom one and tuner 2 too
                // (sr2 and f2 are 0 without it, as the C passes)
                let halfscan2 = if sr2 != 0 { config_cpy.halfscan_ratio } else { 0.0 };
                if err.ok() {
                    err.set(nim.stv0910_init(config_cpy.sr_requested[si], sr2, config_cpy.halfscan_ratio, halfscan2));
                }
                if err.ok() {
                    tuner_err = nim.stv6120_init(config_cpy.freq_requested[fi], f2, config_cpy.port_swap);
                }
                // some NIMs' tuners fail to lock: power down and retry
                if err.ok() && tuner_err == ERROR_TUNER_LOCK_TIMEOUT {
                    println!(
                        "Flow: Caught tuner lock timeout, {} attempts at stv6120_init() remaining.",
                        tuner_lock_attempts as u32
                    );
                    err.set(nim.stv6120_powerdown_both_paths());
                    if err.ok() {
                        clock.usleep(200 * 1000);
                    }
                }
                let again = shared.main_err() == ERROR_NONE
                    && err.ok()
                    && tuner_err == ERROR_TUNER_LOCK_TIMEOUT
                    && {
                        let a = tuner_lock_attempts;
                        tuner_lock_attempts -= 1;
                        a > 0
                    };
                if !again {
                    break;
                }
            }

            if err.ok() {
                err.set(tuner_err);
            }

            // the LNA on the input in use on, the other off (if fitted);
            // with --second both inputs are in use
            let both = second.is_some();
            if err.ok() {
                let st = if config_cpy.port_swap && !both { STVVGLNA_OFF } else { STVVGLNA_ON };
                err.set(nim.stvvglna_init(NIM_INPUT_TOP, st, &mut status_cpy.lna_ok));
            }
            if err.ok() {
                let st = if config_cpy.port_swap || both { STVVGLNA_ON } else { STVVGLNA_OFF };
                err.set(nim.stvvglna_init(NIM_INPUT_BOTTOM, st, &mut status_cpy.lna_ok));
            }

            if !err.ok() {
                println!("ERROR: failed to init a device - is the NIM powered on?");
            }

            if err.ok() {
                err.set(
                    nim.ftdi
                        .set_polarisation_supply(config_cpy.polarisation_supply, config_cpy.polarisation_horizontal),
                );
            }
            if err.ok() {
                status_cpy.polarisation_supply = config_cpy.polarisation_supply;
                status_cpy.polarisation_horizontal = config_cpy.polarisation_horizontal;
                lnb_supply_on = config_cpy.polarisation_supply;
            }

            if err.ok() {
                err.set(nim.stv0910_start_scan(STV0910_DEMOD_TOP));
                status_cpy.state = STATE_DEMOD_HUNTING;
            }
            if second.is_some() && err.ok() {
                err.set(nim.stv0910_start_scan(STV0910_DEMOD_BOTTOM));
                status2_cpy.state = STATE_DEMOD_HUNTING;
                status2_cpy.frequency_requested = f2;
                status2_cpy.symbolrate_requested = sr2;
                status2_cpy.lna_ok = status_cpy.lna_ok;
                status2_cpy.polarisation_supply = status_cpy.polarisation_supply;
                status2_cpy.polarisation_horizontal = status_cpy.polarisation_horizontal;
            }
            nim_up = err.ok();

            status_cpy.last_ts_or_reinit_monotonic = clock.monotonic_ms();
        }

        if timer_after_init {
            // published now, not with the rest of the status at the end of
            // this iteration, so the timeout cannot fire in between
            shared.status.lock().unwrap().last_ts_or_reinit_monotonic = clock.monotonic_ms();
            shared.init_in_progress.store(false, Ordering::SeqCst);
        }

        state_step(nim, &mut status_cpy, &err, port_swap, STV0910_DEMOD_TOP);
        // --second: the bottom demodulator, the same way, after the top
        if second.is_some() && err.ok() {
            state_step(nim, &mut status2_cpy, &err, port_swap, STV0910_DEMOD_BOTTOM);
        }

        let count = shared.ts_packet_count_nolock.load(Ordering::SeqCst);
        if count > 0 && last_ts_packet_count != count {
            status_cpy.last_ts_or_reinit_monotonic = clock.monotonic_ms();
            last_ts_packet_count = count;
        }

        {
            let mut st = shared.status.lock().unwrap();
            st.state = status_cpy.state;
            st.demod_state = status_cpy.demod_state;
            st.lna_ok = status_cpy.lna_ok;
            st.lna_gain = status_cpy.lna_gain;
            st.agc1_gain = status_cpy.agc1_gain;
            st.agc2_gain = status_cpy.agc2_gain;
            st.power_i = status_cpy.power_i;
            st.power_q = status_cpy.power_q;
            st.frequency_requested = status_cpy.frequency_requested;
            st.frequency_offset = status_cpy.frequency_offset;
            st.polarisation_supply = status_cpy.polarisation_supply;
            st.polarisation_horizontal = status_cpy.polarisation_horizontal;
            st.symbolrate_requested = status_cpy.symbolrate_requested;
            st.symbolrate = status_cpy.symbolrate;
            st.viterbi_error_rate = status_cpy.viterbi_error_rate;
            st.bit_error_rate = status_cpy.bit_error_rate;
            st.modulation_error_rate = status_cpy.modulation_error_rate;
            st.errors_bch_uncorrected = status_cpy.errors_bch_uncorrected;
            st.errors_bch_count = status_cpy.errors_bch_count;
            st.errors_ldpc_count = status_cpy.errors_ldpc_count;
            st.constellation = status_cpy.constellation;
            st.puncture_rate = status_cpy.puncture_rate;
            st.modcod = status_cpy.modcod;
            st.short_frame = status_cpy.short_frame;
            st.pilots = status_cpy.pilots;
            if nim.ext.extra_status {
                st.rolloff = status_cpy.rolloff;
                st.ts_rate = ts_rate.update(clock.monotonic_ms(), shared.ts_payload_bytes.load(Ordering::SeqCst));
            }
            if status_cpy.last_ts_or_reinit_monotonic != 0 {
                st.last_ts_or_reinit_monotonic = status_cpy.last_ts_or_reinit_monotonic;
            }
            st.last_updated_monotonic = clock.monotonic_ms();
        }

        if second.is_some() {
            let mut st = shared.status2.lock().unwrap();
            let names = (std::mem::take(&mut st.service_name), std::mem::take(&mut st.service_provider_name));
            let (null, es) = (st.ts_null_percentage, st.ts_elementary_streams);
            // everything but what the second TS's parse fills in
            *st = Status {
                service_name: names.0,
                service_provider_name: names.1,
                ts_null_percentage: null,
                ts_elementary_streams: es,
                last_updated_monotonic: clock.monotonic_ms(),
                ..status2_cpy.clone()
            };
        }

        last_i2c_loop = clock.monotonic_ms();
    }

    // --lnb-off-at-exit: whatever stopped the loop (a signal, another
    // thread's error, this thread's), take the LNB supply off the feed.
    // Only after -p turned it on, so a run without -p ends exactly as the
    // C's does. If the USB has gone the write fails, and is ignored as
    // every GPIO write is; there is nothing else left to try.
    if nim.ext.lnb_off_at_exit && lnb_supply_on {
        println!("Flow: LNB supply off at exit");
        let _ = nim.ftdi.set_polarisation_supply(false, false);
    }
}
