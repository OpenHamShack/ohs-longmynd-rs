# longmynd-rs: improvements found while porting

The port is faithful first: with no `--` options, longmynd-rs does to the
USB bus, and prints, exactly what the C does (`cargo test`,
tests/equivalence.rs). Everything below was found on the way. Each entry
says what is wrong, the evidence, what a user sees, the risk of changing
it, and whether a change moves the USB/I2C trace or the status/TS output.

Status:

- **FIXED** - fixed in longmynd-rs already. Only bugs whose fix changes
  nothing on a working path (an error path, or undefined behaviour in the C).
  Each has a test that shows the C with the bug and the Rust without.
- **OPT-IN** - implemented behind a `--` option, off by default. Brian
  decides which become defaults.
- **PROPOSED** - not implemented; recommendation given.

C line numbers are in the reference tree (BATC longmynd b9a325b plus the
PicoTuner patch, `reference/`).

## Decisions (Brian, 06/10/2026)

- **To become defaults once the MiniTiouner hardware test confirms them:**
  item 7 `--fast-i2c`, items 3 and 5 (2m and 13cm HF filter: the overrun
  fix and `--hf-filter-fix`), item 10 `--lna-follow-input`, item 17 LNB
  supply off at exit. Not flipped until then; the flip is one prepared
  change with `--c-compat` to get the C's behaviour back (HANDOVER.md,
  "Apply after hardware test").
- **Every PROPOSED item is built behind an opt-in flag**, default
  byte-identical to the C. Item 21 (second tuner) is built if the
  PicoTuner firmware allows it, otherwise planned.
- **Windows uses FTDI's D2XX driver** (`ftd2xx.dll`, loaded at run time)
  as a second USB backend, so no driver swap is needed (README, Windows).
  Built: `--usb-backend auto|libusb|d2xx`, `src/d2xx.rs`, proven against
  the C through a simulated D2XX library (`equiv/tests/d2xx.rs`). The DLL
  is not shipped: FTDI's licence does not allow a software project to
  (README, "D2XX licence"), and Windows Update installs it anyway.

## Summary

| # | Item | Status | Changes default trace/output? | Recommendation |
|---|------|--------|------------------------------|----------------|
| 1 | One short USB read leaves I2C reading stale bytes forever | FIXED | error path only | keep |
| 2 | nim_init's read-back check can never fail | FIXED | error path only | keep |
| 3 | HF filter lookup reads past its table below ~285 MHz (2m band) | FIXED | C value is undefined | keep; check on 146.5 MHz hardware |
| 4 | Charge-pump lookup reads past its table above 2380 MHz | FIXED | C value is undefined | keep |
| 5 | HF filter wraps to the narrowest above 2265 MHz (CTRL8 = 0xff) | OPT-IN `--hf-filter-fix` | yes, above 2265 MHz | make default after a 13cm test |
| 6 | Status buffer, service-name and PMT overflows | FIXED | only with malformed TS | keep |
| 7 | Init takes ~9,000 USB round trips; every scan step pays it | OPT-IN `--fast-i2c` | trace (same I2C content) | make default after hardware test |
| 8 | Re-tune reloads the whole demodulator table | OPT-IN `--fast-retune` | trace (same final tuning) | scan test on hardware, then decide |
| 9 | LNA AGC steps every ~15 s (breaks 16APSK) | OPT-IN diagnostics | yes | bench test plan below |
| 10 | $2 reports the TOP LNA even with `-w` | OPT-IN `--lna-follow-input` | yes, with -w | make default |
| 11 | Errors after a failed re-init are overwritten in locked states | OPT-IN `--strict-errors` | error path, but process lifetime | daemon opts in once it handles the exit |
| 12 | TS interface latency timer never set (requests go to interface A) | OPT-IN `--ts-latency` | trace | measure; maybe default 2-4 ms |
| 13 | TS parser misses the last packet of every chunk and syncs on 0x47 inside packets | OPT-IN `--ts-parse-fix` | status ($13-17) | default once a consumer has checked `$15` |
| 14 | SDT parser walks the wrong offsets after a charset selector | OPT-IN `--ts-parse-fix` | status ($13/$14) | with 13 |
| 15 | A full TS FIFO drops 510-byte pieces silently | counted; reported by `--extra-status` ($31) | - | keep dropping |
| 16 | USB unplug ends longmynd; no reconnect | OPT-IN `--reconnect` | - | for users without a supervisor |
| 17 | LNB supply stays on after longmynd exits | OPT-IN `--lnb-off-at-exit` | trace at exit, only after `-p` | default after hardware test (decided) |
| 18 | 33 kS/s accepted, the STV0910 floor is ~66 kS/s | OPT-IN `--warn-low-sr` | stdout | consumers opt in |
| 19 | Status units and gaps ($6 kHz truncated, $9 in S/s, no roll-off) | OPT-IN `--extra-status` ($28-$31) | adds lines after the C's | consumers opt in |
| 20 | No-TS timer runs during init and retries | OPT-IN `--ts-timer-after-init` | timing | default candidate; OpenHamShack to try |
| 21 | Second demodulator / tuner (PicoTuner) | OPT-IN `--second FREQ SR` | - | hardware test on a PicoTuner with both TS wired |
| 22 | Races between threads | FIXED by construction | none | - |
| 23 | MER beep (`-b`) | not ported | - | port with an audio crate if anyone uses it |

## 1. A short USB read leaves the I2C layer reading stale bytes - FIXED

- **Wrong:** `ftdi_usb_i2c_read()` keeps a read-ahead buffer with two
  statics, `rxed` and `posn` (ftdi_usb.c:146-176). When both retries of a
  USB read return only the FTDI's two status bytes, `rxed` becomes 2 while
  `posn` stays put. From then on `posn != rxed` on every call, so every
  byte comes from the old buffer at `posn++` - walking past the end of the
  4096-byte array - and the USB is never read again.
- **Evidence:** tests/fixes.rs, `a_short_usb_read_does_not_leave_i2c_reading_stale_bytes`:
  with two short reads injected during the STV0910 table load, the C makes
  no further USB read for the rest of the init (about 900 register writes
  made blind, every ACK "seen" from stale data) and fails much later at
  "STV0910 pll lock timeout" (exit 42).
- **User-visible:** after a USB hiccup the receiver either fails with a
  misleading error or, worse, runs on with registers it never verified.
- **Fix:** a failed read empties the read-ahead; the I2C layer's own retry
  then reads afresh and the init completes.
- **Risk:** none on a working device (the path is never taken).
- **Trace/output:** error path only.

## 2. nim_init's read-back check can never fail - FIXED

- **Wrong:** nim.c:201 is `if (err!=ERROR_NONE) err=nim_write_demod(...)`
  where the comment says the repeater is "always" turned off. On a
  read-back mismatch the error is overwritten by that write's success.
- **Evidence:** tests/fixes.rs, `a_nim_failing_the_readback_check_is_an_error`:
  the C prints "nim_init didn't r/w to the modulator" and carries on to
  "start scan".
- **Fix:** the error is kept (the write still happens, on that path only).
  longmynd then stops with "failed to init a device", exit 42.
- **Not changed:** making the write unconditional as the comment intends -
  one extra write per init, so it would change the default trace. Harmless
  either way: every demod access closes the repeater anyway.

## 3. HF filter lookup reads past its table below ~285 MHz - FIXED

- **Wrong:** stv6120.c:151 searches `stv6120_cfhf[]` for the last cut-off
  at or above 3 x RF. Below 284.67 MHz every entry qualifies and the loop
  runs past index 31 into whatever the compiler put next.
- **Evidence:** equivalence scenario `low_band_146`: at 146.5 MHz this
  build writes CTRL8 = 0x60 - CFHF index 0 (the *widest* filter, 6.8 GHz)
  plus a stray CALTIME bit (bit 5) - and prints `cfhf=20704`, a value from
  outside the table. Another compiler or ARM build writes something else.
- **User-visible:** on 2m (146.5 MHz) the tuner's harmonic filter is
  effectively off, and the calibration-time bit is wrong. Possibly part of
  why 2m reception is poorer than expected; unmeasured.
- **Fix:** the Rust takes the last entry (854 MHz, the narrowest available,
  which is the table's intent for low frequencies). The equivalence test
  masks just this value.
- **Risk:** low, but it is a real change of tuner setting on 2m: include
  146.5 MHz in the hardware test.

## 4. Charge-pump lookup reads past its table above 2380 MHz - FIXED

- **Wrong:** stv6120.c:146 `while (*f_vco > stv6120_icp_lookup[pos++][1]);`
  has no bound; above 2380 MHz the VCO (2 x RF) exceeds the last row
  (4760 MHz) and the ICP value comes from beyond the table.
- **Evidence:** scenario `top_band_2400` (and an earlier 2395 MHz run):
  this build prints `icp=0x64` and writes ICP = 4 (the value truncated into
  a 3-bit field).
- **Fix:** the last row (ICP 7, the highest current, matching the
  highest VCO band). Masked in the equivalence test.

## 5. HF filter wraps to the narrowest above 2265 MHz - OPT-IN `--hf-filter-fix`

- **Wrong:** above 2265.3 MHz (3 x RF > 6796 MHz) the same search finds no
  entry; `(*cfhf)--` (stv6120.c:154) wraps 0 to 255 and CTRL8 becomes
  `255 | ctrl8` = 0xff: CFHF 31 - the narrowest filter, 854 MHz, on a 2.4 GHz
  signal - and every TCAL bit set. The comment says this cannot happen.
- **Evidence:** scenario `top_band_2400` writes `i2c c0 07 ff`;
  tests/extensions.rs shows `--hf-filter-fix` writing 0x40 (CFHF 0).
- **User-visible:** 13cm (2.3-2.45 GHz) reception through a filter set
  for 854 MHz. Probably large loss of sensitivity; unmeasured.
- **Risk:** low; well-defined in the C (unsigned wrap), so it changes the
  default trace - hence opt-in.
- **Recommendation:** test on 2395 MHz (or via a 13cm source) with and
  without; make default if it helps or is neutral.

## 6. Status buffer, service-name and PMT overflows - FIXED

- `fifo_status_string_write` / `udp_status_string_write` format into
  `char status_message[5+128]` (fifo.c:147, udp.c:132); a service name over
  127 bytes overflows the stack.
- The SDT callback copies names into `char service_name[255]` and writes
  the NUL at `[len]` (ts.c:402); a 255-byte name, or a shorter one that
  grows in UTF-8 conversion (6 x for some sets), overflows into the rest of
  the status.
- `ts_callback_pmt_pids` (ts.c:424) indexes `ts_elementary_streams[16]`
  with an unbounded PMT index: a PMT with more than 16 streams writes over
  modcod, short-frame, pilots and beyond.
- **Fix:** lines are written whole, names cut at 254 bytes, PMT entries
  past 16 ignored. Unit test `oversized_pmt_is_cut_at_sixteen_streams`.
- **Trace/output:** only with malformed or exotic TS.

## 7. Init costs ~9,000 USB round trips - OPT-IN `--fast-i2c`

- **Wrong:** every I2C byte is a USB write plus a USB read for its ACK
  (ftdi.c:207, `ftdi_i2c_send_byte_check_ack`): a 16-bit register write is
  four round trips and a fifth write. Every (re-)initialisation writes the
  935-register STV0910 table (stv0910.c, `stv0910_init_regs`).
- **Evidence:** tests/extensions.rs measures start-up to "start scan" on the
  simulator: **8,957 USB transfers and 1.8 s** of USB time for the C;
  **602 transfers and 0.29 s** with `--fast-i2c`, with the decoded I2C
  transactions identical in content and order. (Simulated costs: 125 us per
  write, 250 us per read; a real FT2232H is in the same range, which is
  why longmynd takes 1.5-2 s to start.)
- **User-visible:** every frequency/symbol-rate step of a scan (`-r` with
  lists, or OpenHamShack DATV's band scanner restarting longmynd) pays the full
  init. `--fast-i2c` cuts that by roughly 6x.
- **How:** the same MPSSE commands, queued for 32 registers with one
  "send immediate"; all 128 ACK bits read back and checked; any NAK in a
  batch rewrites that batch through the C's per-register path.
- **Risk:** the bus waveform is unchanged, but the FT2232H (and the
  PicoTuner's firmware MPSSE emulation) now see 3 KB command bursts. The
  PicoTuner is the one to watch.
- **Recommendation:** hardware-test on both a MiniTiouner and a PicoTuner
  (steps in HANDOVER.md); make default if both lock identically.

## 8. Re-tune reloads the whole demodulator table - OPT-IN `--fast-retune`

- Every config change (each scan step, each no-TS timeout) runs nim_init,
  the full 935-register STV0910 load, PLL setup, tuner init and LNA init
  (main.c:611-657). Between steps only the tuner frequency, SFRINIT, the
  carrier range and DMDISTATE actually change.
- **Built:** `--fast-retune`: on a config change with the NIM already up
  from a successful full init, stop the top demodulator (DMDISTATE 0x1c),
  set its carrier and timing loops, reset the LDPC decoder (as the init
  does), retune tuner 1 (set_freq, which recalibrates the VCO and waits for
  lock) and start the scan. Any failure, the tuner not locking included,
  prints "Fast re-tune failed, full re-init" and does the C's full init
  with its retries.
- **Evidence:** tests/extensions.rs `fast_retune_tunes_the_same_and_skips_the_reload`
  runs the 2 x 4 scan of `scan_rates_and_freqs`: every step leaves the
  tuner PLL words (CTRL3-8), SFRINIT and CFRUP/CFRLOW exactly as the C's
  full init leaves them, in the same scan order, for **196 USB transfers
  per step instead of 8,929** (the demodulator ID is read once in the
  whole run). `fast_retune_falls_back_to_a_full_init_when_the_tuner_will_not_lock`
  covers the fallback.
- **Risk:** medium - state left over from the previous step (equaliser,
  AGC, LDPC decoder beyond the reset) is not reset; ST's own drivers do
  retune this way, but it needs a scan test on hardware (does each step
  still lock as fast as after a full init?). Changes the trace (opt-in
  only). With `--fast-i2c` too the one full init is also cheap.

## 9. The LNA AGC steps every ~15 s - OPT-IN diagnostics

- **Observation (OpenHamShack HANDOVER, bench, 16APSK at -20 dB):** every
  15-17 s `$2` goes 127 -> 109 and back: SWLNAGAIN stays 3, VGO drops from
  31 (the top of its range) to 13 for ~0.6 s; AGC1 and input power fall
  with it, and the demodulator loses a few frames on the way back.
- **What the code says:** longmynd sets the LNA to automatic tracking,
  target -20 dBm, fast update, slow lock (stvvglna.c, `stvvglna_init`), and
  then, every 100 ms, writes REG1 = GETAGC_START and polls it
  (stvvglna.c:62, `stvvglna_read_agc`). Nothing in longmynd has a 15 s
  period: no timer, no counter of that length; the 100 ms loop and the 5 s
  no-TS timer are the only clocks. So the step is the LNA's own AGC
  deciding, not longmynd commanding it. VGO = 31 means the LNA was at the
  rail - the input sits on a gain-stage boundary, as the bench note says.
  Two longmynd actions could still disturb it: the 100 ms GETAGC
  request (whose write also zeroes REG1's other fields - harmless only
  because their current values are zero), and opening and closing the
  demodulator's I2C repeater for it every loop.
- **Options added for the bench:**
  - `--no-lna-poll` - no LNA traffic after init. If the steps continue
    (watch AGC1 `$26` and power `$4`, which come from the demodulator),
    longmynd's polling is cleared.
  - `--lna-mode max` (or `min`, `request`) - a fixed gain: if 16APSK is then
    clean, the AGC loop is the cause.
  - `--lna-pref -25..-18` - move the AGC target off the boundary.
  - `--lna-tlock fast` - the other lock time constant.
- **Recommendation:** run the four in HANDOVER.md's bench test. If a fixed
  or offset setting fixes 16APSK without hurting 8PSK, expose it to OpenHamShack
  as a setting rather than changing the default.

## 10. `$2` reports the TOP LNA even with `-w` - OPT-IN `--lna-follow-input`

- main.c:504 always calls `stvvglna_read_agc(NIM_INPUT_TOP, ...)`; with `-w`
  the BOTTOM LNA is the one in use (and the TOP one is switched off).
- **User-visible:** with `-w`, `$2` reports a switched-off LNA.
- **Recommendation:** make default; no reason to keep the wrong one.

## 11. Errors after a failed re-init are overwritten - OPT-IN `--strict-errors`

- In every receiver state except HUNTING, the scan-state read is
  `*err=stv0910_read_scan_state(...)` without the `if (*err==ERROR_NONE)`
  guard (main.c:684, 703, 722). If a re-init fails (e.g. the tuner never
  locks after 11 attempts) while the previous state was S/S2/FOUND_HEADER,
  the error is overwritten by the successful read and longmynd runs on with
  a half-initialised NIM, still reporting the old state.
- **Risk of changing:** longmynd would then exit (42) where it now keeps
  going. The OpenHamShack daemon restarts it, but a scanner might prefer the
  current "carry on" - so this is a decision, not a fix.
- **Built:** `--strict-errors` guards the scan-state read in every state,
  as the C does in HUNTING; the failed re-init's error stands and longmynd
  exits 42.
- **Evidence:** tests/extensions.rs `strict_errors_keeps_a_failed_reinit_error`,
  scenario `ext_strict_errors`: locked to DVB-S, the TS stops, the no-TS
  re-init finds the tuner dead (all 11 attempts fail). The C prints
  "failed to init a device", never starts a new scan, and runs to the end
  of the scenario (43) reporting `$1,3`; with the option, exit 42.
- **Recommendation:** have the OpenHamShack daemon opt in once it restarts
  longmynd on 42 (it already does for unplug).

## 12. The TS interface's latency timer is never set - OPT-IN `--ts-latency`

- ftdi_usb.c:192-221 sends every control request with wIndex 1 -
  interface A - for both handles. The TS handle's purge, latency (16 ms)
  and bitmode requests all go to A again; interface B keeps the chip's
  default latency timer (16 ms on an FT2232H).
- **User-visible:** at low symbol rates (333 kS/s QPSK is ~40 kB/s) a
  partly filled 512-byte USB packet waits up to 16 ms; with the 10 KB TS
  frame that is most of longmynd's own latency.
- **Recommendation:** measure end-to-end latency with `--ts-latency 2`
  against default on a MiniTiouner; make it default if lower with no TS
  errors. (The PicoTuner may ignore the request.)

## 13. TS parser misses packets and syncs on 0x47 - OPT-IN `--ts-parse-fix`

- After each packet libts.c advances one byte (`ts_packet_ptr++`,
  libts.c:176-353) and searches for the next 0x47 with `memchr`, limited to
  `remaining - 188` bytes (libts.c:159). So any 0x47 inside a payload is
  taken as a packet start, and the last packet of every chunk is never
  reached. It is also handed the FTDI's status bytes (a pair at 510 + 512k
  of the parse buffer) as if they were TS, so a PSI section crossing a USB
  packet fails its CRC.
- **Evidence:** unit test `parses_pmt_sdt_and_null_share` (src/libts.rs):
  four packets in (PMT, SDT, two nulls) count as four with a 25 % null
  share - the second null is missed and the "G" of "GB3HV" counted.
- **User-visible:** `$15` (null %) is wrong - in `ts_fifo_and_names`, a
  stream that is 24 % nulls reads as 12-17 % - and PAT/PMT/SDT in the last
  packet of a read are missed (only delays them, normally).
- **Built:** `--ts-parse-fix` (`libts::ts_parse_fixed`): the status pairs
  are taken out first; sync wants a 0x47 with another 188 bytes on (or a
  0x47 where the previous packet said the next would be); after a packet
  the walk moves 188; it parses to the end of the chunk. What is done with
  each packet, and the callbacks, are the C's.
- **Evidence:** unit tests `fixed_parse_counts_every_packet_and_no_payload_bytes`,
  `fixed_parse_skips_the_ftdi_status_bytes` (the C on the same bytes shown
  beside each), and tests/extensions.rs
  `ts_parse_fix_reports_the_true_null_share_and_changes_nothing_else`: on
  `ts_fifo_and_names`'s stream the TS written out is the C's byte for byte,
  the names are the same, and `$15` is 24-25 % (true: 14/57) against the
  C's 12-17 %.

## 14. SDT parser walks wrong offsets after a charset selector - OPT-IN `--ts-parse-fix`

- The SDT callback shortens the name lengths it is given by the selector
  bytes (ts.c:263-291, `*service_name_length_ptr -= 1/3`), and libts then
  uses those shortened lengths to find the next service (libts.c:342).
  It also assumes each service's first descriptor is the service
  descriptor. With more than one service in an SDT, and a selector on a
  name, the second service is parsed from the wrong offset.
- **Built (same option):** services are walked by their own descriptor
  loop lengths, the service descriptor (0x48) is looked for among the
  descriptors, and the callback gets copies of the lengths.
- **Evidence:** unit test `fixed_sdt_walk_is_not_moved_by_a_charset_selector`:
  two services, the first name with an ISO-8859-9 selector; the C does not
  find "TWO", the fixed walk does.

## 15. A full TS FIFO drops data silently - counted, reported with `--extra-status`

- fifo.c:84: on EAGAIN the comment says "try again", but `remaining_len`
  has already been decremented, so the 510-byte piece is dropped and the
  next one written. For live video that is arguably right (better to drop
  than block the USB reader), but nothing reports it.
- **Done:** still dropped exactly as the C does; the FIFO writer now counts
  the drops (no effect on anything the C does), and `--extra-status`
  reports the count as `$31`. Unit test
  `a_full_ts_fifo_drops_pieces_and_counts_them` fills a real FIFO nobody
  reads.

## 16. USB unplug ends longmynd - OPT-IN `--reconnect`

- A TS read error is fatal to the TS thread (ftdi_usb.c:381), which ends
  longmynd (exit 42); I2C errors after 1,000 retries end the receiver
  thread the same way. There is no reconnect. With fix 1 a single stalled
  read now recovers; a real unplug still exits.
- **Built:** `--reconnect`: when a session ends on a worker thread's error
  (anything but the TS output failing; never on a signal or a status
  output error), main() joins the threads - which closes the old handles -
  prints `Flow: Tuner lost, waiting for it to come back (--reconnect)`,
  looks for the tuner once a second without a word (VID/PID, or `-u`'s
  bus/device), then opens it exactly as at start and runs a full init with
  the configuration as it stands (`Flow: Tuner is back, reconnecting`). A
  signal while waiting exits 43. The status output stays open throughout;
  no status is written while the tuner is away.
- Because any worker failure counts, a NIM that stops answering with the
  USB still there is retried too (about every init + 1 s); with
  `--strict-errors` a failed re-init is then retried rather than fatal.
- **Evidence:** tests/extensions.rs `reconnect_waits_for_the_tuner_and_starts_again`
  (scenario `ext_reconnect`: unplugged 3.7 s while hunting; the simulator
  kills every open handle with LIBUSB_ERROR_NO_DEVICE and powers the NIM up
  fresh on return): the C exits 42; with the option it polls 2-4 times,
  reconnects, inits, scans and runs to the end of the scenario.
- **Not covered:** the harness drives the receiver thread itself, so the
  test runs `app::reconnect_wait`/`reconnectable`/`reset_for_reconnect`
  in the same order `app::run` does rather than `run()`'s real threads.
  The OpenHamShack daemon restarting longmynd remains the better supervisor
  where there is one.

## 17. The LNB supply stays on after exit - OPT-IN `--lnb-off-at-exit`

- longmynd never turns the RT5047A LNB supply off on exit; the GPIO keeps
  its state while the FTDI is powered. After `-p h` the feed keeps 18 V
  until the next longmynd start (whose FTDI setup turns it off).
- **User-visible:** the OpenHamShack rule "never put LNB voltage on the feed
  without an offset" can be broken by a stopped receiver.
- **Built:** with `--lnb-off-at-exit`, when the receiver thread stops for
  any reason (SIGTERM/SIGINT, another thread's error, its own) and the
  last configuration applied had `-p`, it writes the LNB enable pin low
  (`Flow: LNB supply off at exit`). Without `-p` nothing is written, so the
  run is the C's exactly. If the USB has already gone the write fails
  silently, as every GPIO write does - nothing more can be done then.
- **Evidence:** tests/extensions.rs `lnb_off_at_exit_turns_the_supply_off_when_stopped`
  (the C's last GPIO write leaves AC4 high; with the option it is low) and
  `lnb_off_at_exit_does_nothing_without_p` (trace identical to the C's).
- **Decision:** default after the hardware test (one GPIO write at exit;
  check with a meter that the feed drops to 0 V on Ctrl+C and on `kill`).

## 18. 33 kS/s is accepted; the STV0910's floor is ~66 kS/s - OPT-IN `--warn-low-sr`

- main.c:424 accepts down to 33 kS/s. OpenHamShack DATV's research (§ summary
  point 6) found a MiniTiouner cannot go below ~66 kS/s; RB-TV at 25/33
  kS/s needs software decode.
- **Built:** `--warn-low-sr` still accepts the rate (scripts pass 33) and
  prints, after the configuration summary, one line per requested rate
  below 66: `WARNING: SR (33) is below 66 KSymbols/s: the STV0910 is not
  known to lock that low (...)`. Stdout only.
- **Evidence:** tests/extensions.rs `warn_low_sr_warns_where_the_c_is_silent`
  (and the unit test in config.rs).

## 19. Status units and gaps - OPT-IN `--extra-status`

What the existing lines mean (unchanged, documented in the README):
`$6` is the tuner frequency in kHz plus the carrier offset divided by
1000 with C truncation (so offsets under 1 kHz vanish and rounding is
towards zero both sides); `$9` is the found symbol rate in **S/s** while the
command line takes kS/s; `$10` Viterbi errors in 0.01 %; `$11` BER x 10^4,
which an empty counter makes 0; `$12` MER in 0.1 dB, 0 outside S/S2; `$18`
the raw MODCOD field even in DVB-S; `$19`/`$20` forced to 0 outside DVB-S2.
Not reported at all: the carrier offset in Hz, roll-off, the TS rate.

- **Built:** `--extra-status` writes four more lines at the end of each
  block, after `$20`, on numbers BATC's longmynd does not use (master
  stops at 27): `$28` carrier offset from the requested frequency in Hz,
  signed (register resolution 135 MHz / 2^24, about 8 Hz); `$29` roll-off
  in hundredths (35, 25, 20; 0 when TMGOBS says something else) - one more
  register read per report; `$30` TS payload bytes per second (FTDI status
  bytes excluded) over windows of at least 1 s, 0 until the first; `$31`
  TS pieces dropped by a full FIFO since start (item 15). No existing line
  changes number, value or place.
- **Evidence:** tests/extensions.rs
  `extra_status_adds_four_lines_after_the_cs_and_changes_none`: every
  block is one the C writes with `$28`-`$31` appended; `$28` follows the
  simulated carrier (-73,000 Hz hunting, 1,830 Hz locked) where `$6` shows
  -72 and +1 kHz.

## 20. The no-TS timer runs during init and retries - OPT-IN `--ts-timer-after-init`

- main.c:1043-1051 compares against `last_ts_or_reinit_monotonic`, which
  the receiver only refreshes after an init completes (main.c:657). An init
  is ~2 s, and tuner retries (up to 11 x (init + 200 ms)) can exceed the
  5 s timeout, so a second re-init is queued while the first is still
  running. Seen in scenario `tuner_lock_retries`.
- **Built:** `--ts-timer-after-init`: the receiver flags an init (full or
  `--fast-retune`) as running, main() does not time out while it is, and
  the receiver restarts the timer the moment the init ends (not with the
  rest of the status at the end of the iteration, so it cannot fire in
  between).
- **Evidence:** tests/extensions.rs `ts_timer_after_init_does_not_fire_during_an_init`
  on `tuner_lock_retries`: the C times out before its first scan starts and
  inits more often; with the option no timeout precedes the first scan.

## 21. Second demodulator and tuner - OPT-IN `--second FREQ SR` (PicoTuner)

- The STV0910 and STV6120 are dual, and the C has the bottom-path code
  (`stv0910_init(sr1, sr2, ...)`, `stv6120_init(f1, f2, ...)`), but
  longmynd only ever starts the top demodulator and tuner 1.
- **The firmware (g4eml/PicoTuner, read 06/10/2026):** interface 1 has two
  bulk IN endpoints, `0x83` "TS2" and `0x84` "TS1"
  (`dev_lowlevel_descriptors.h`, README "USB device description"), fed by
  two PIO serial-TS receivers on separate pins (`PicoTuner.ino`:
  TS1DAT/CLK/VALID 10-12, TS2DAT/CLK/VALID 13-15) with their own DMA and
  512-byte buffers, each with a 2-byte status header (0xAA 0xAA) and a
  status-only packet every 16 ms when idle. longmynd's top demodulator is
  the STV0910's P2 and reads `0x83` (TS2), so the bottom one (P1) is `0x84`
  (TS1). The README: "provide access to the second Transport Stream ...
  two signals to be received at the same time"; OpenTuner (ZR6TG) already
  shows two channels from it. The command buffer is 64 KiB and the result
  buffer 1 KiB, so `--fast-i2c`'s bursts (3 KB of commands, 130 bytes of
  ACKs per batch) fit. On a MiniTiouner only one TS reaches the FTDI, so
  this is PicoTuner-only.
- **Built:** `--second FREQ SR` (kHz, kS/s; one of each, no scanning) with
  `--second-ts-fifo PATH | --second-ts-ip IP PORT` and
  `--second-status-fifo PATH | --second-status-ip IP PORT` (defaults the
  FIFOs `longmynd_second_ts`, `longmynd_second_status`). Each init starts
  both demodulators and both tuners (they share the chips, so a re-init of
  the main receiver re-inits both); tuner 2 takes the input tuner 1 does
  not, both LNAs are on; every 100 ms the bottom demodulator is reported
  after the top one, into its own status (same `$` lines, its own output);
  one TS thread reads `0x83` and `0x84` in turn, the second TS going to its
  own output and parsed in that thread. `-r` cycling, the no-TS timer and
  the beep belong to the main receiver only. On anything but a PicoTuner
  it stops after opening the tuner: "--second needs a PicoTuner" (exit 2).
- **Two C bugs on the bottom path, fixed** (never reached by the C, which
  never starts it; reached only with `--second`): the bottom timing loop
  wrote the SFRINIT bytes swapped; the tuner 2 low-pass calibration polled
  STAT1 instead of STAT2.
- **Evidence:** tests/extensions.rs `second_receiver_*`: tuner 2 programmed
  and its calibration polled on STAT2, P1 SFRINIT right way round, both
  scans started, each status from its own demodulator (the simulator's
  bottom demodulator now has phases of its own), the main TS byte-identical
  to the C's while the second goes to its own file and status; refused on
  a MiniTiouner. (The simulator also had STAT2 at 0x0C, which is CTRL12;
  moved to 0x12 - the C path never touches either, the goldens are
  unchanged.)
- **Not proven:** what the real STV0910 puts on its second TS pins with
  the default register table plus the serial-TS writes the PicoTuner patch
  already makes to both P1 and P2 - the hardware test owes a PicoTuner
  with two signals.

## 22. Races between threads - FIXED by construction

The C shares `config->new`, `config->ts_reset` and
`status->ts_packet_count_nolock` between threads without locks or
atomics (data races, benign on x86 in practice). In the Rust they are a
mutex and atomics; behaviour is otherwise the same. One remaining logical
race, kept: a second config change arriving while the TS thread is
draining after the first is merged into one reset.

## 23. MER beep (`-b`) - not ported

`-b` is accepted, printed in the summary as the C does, and followed by a
warning that it is not available. Porting needs an audio output on both
platforms (cpal would do; it links ALSA on Linux, which the Rust binary
otherwise does not need).
