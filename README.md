# longmynd-rs

A Rust port of BATC's [longmynd](https://github.com/BritishAmateurTelevisionClub/longmynd),
the receiver for the MiniTiouner and PicoTuner (Serit NIM: STV0910
demodulator, STV6120 tuner, STVVGLNA LNAs) used for amateur DATV.

It is a **drop-in replacement**: the same command line, the same status
lines on the same FIFO or UDP port, the same TS output, the same exit
codes - proven against the C byte for byte on a simulated MiniTiouner
(below). It builds for **Linux and Windows** from one source, so OpenHamShack
DATV, OpenHamShack and OpenHamShack Desktop can use a MiniTiouner on either.

The C it matches is the one every OpenHamShack ships: BATC longmynd
`b9a325b` (master since January 2022) plus the OpenHamShack appliance's
PicoTuner patch, both in `reference/`.

Status: complete and tested against the C in simulation; **not yet run
on a real tuner** - the receive-only hardware test is written up in
HANDOVER.md. The MER beep (`-b`) is not ported yet.

## Usage

Exactly as longmynd (`docs/longmynd.1`):

```
longmynd [-u USB_BUS USB_DEVICE]
         [-i MAIN_IP_ADDR MAIN_PORT | -t MAIN_TS_FIFO]
         [-I STATUS_IP_ADDR STATUS_PORT | -s MAIN_STATUS_FIFO]
         [-w] [-b] [-p h | -p v] [-r TS_TIMEOUT_PERIOD] [-S HALFSCAN_WIDTH]
         MAIN_FREQ[,ALT_FREQ...] MAIN_SR[,ALT_SR...]
```

Frequencies in kHz (144000-2450000), symbol rates in kS/s (33-27500), up
to four of each; with lists, `-r` (ms, default 5000) cycles through them
when no TS arrives. `-w` uses the BOTTOM input, `-p h|v` turns on an
RT5047A LNB supply at 18/13 V, `-S` sets the carrier search half-width as
a multiple of the symbol rate (default 1.5), `-u` picks a tuner by USB bus
and device number. TS defaults to the FIFO `longmynd_main_ts`, status to
`longmynd_main_status`; `-i` / `-I` send them over UDP instead.

```
longmynd 437000 333                                  # 70cm, 333 kS/s
longmynd -i 127.0.0.1 1234 -I 127.0.0.1 4002 1255000 1500
longmynd -p h 741500 1500                            # QO-100 via LNB, 18 V
longmynd -r 3000 437000,1255000 333,1500             # scan 2 x 2
```

The command-line parser is the C's, quirks included, because scripts
already depend on it: options are only looked for while two arguments
follow them, an unknown option swallows the argument after it, numbers
are read as `strtol`/`strtof` prefixes.

### Status lines

`$<n>,<value>` per line, the whole set written after every receiver
iteration (every 100 ms), in this order. Units as the C has them:

| n | value |
|---|-------|
| 1 | state: 0 init, 1 hunting, 2 found header, 3 DVB-S, 4 DVB-S2 |
| 2 | LNA gain: SWLNAGAIN << 5 \| VGO (only on NIMs with LNAs; always the TOP input's) |
| 26, 27 | AGC1, AGC2 gain (raw) |
| 4, 5 | I, Q power (raw) |
| 7, 8 | 16 constellation points, alternately I and Q |
| 3 | Viterbi puncture rate: n of rate n/(n+1) |
| 6 | carrier: requested kHz + offset Hz / 1000 (truncated towards zero) |
| 24, 25 | LNB supply on; LNB supply horizontal (18 V) |
| 9 | symbol rate found, in **S/s** (the command line takes kS/s) |
| 10 | Viterbi error rate, 0.01 % |
| 11 | BER x 10^4 |
| 12 | MER, 0.1 dB (0 outside DVB-S/S2) |
| 23, 22, 21 | BCH uncorrected flag, BCH errors, LDPC errors |
| 13, 14 | service name, provider name (UTF-8) |
| 15 | null packets, % |
| 16, 17 | per elementary stream: PID, type |
| 18 | MODCOD (raw field; meaningful in DVB-S2) |
| 19, 20 | short frames, pilots (DVB-S2 only, else 0) |

With `--extra-status`, four more lines follow `$20` in every block, on
numbers the C does not use:

| n | value |
|---|-------|
| 28 | carrier offset from the requested frequency, **Hz**, signed |
| 29 | roll-off, hundredths: 35, 25, 20 (0 = not known) |
| 30 | TS payload arriving, bytes/s (1 s windows; 0 until the first) |
| 31 | 510-byte TS pieces dropped because the TS FIFO was full, since start |

### Options beyond the C

Off unless given; each changes what goes to the NIM or what is printed,
which is why none is on by default. The case for each is in
`docs/improvements.md`. Four - `--fast-i2c`, `--hf-filter-fix`,
`--lna-follow-input`, `--lnb-off-at-exit` - are decided to become the
defaults once the hardware test confirms them; `--c-compat` will then give
the C's behaviour back (it is accepted, and changes nothing, today).

| option | does |
|--------|------|
| `--fast-i2c` | load the STV0910's 935-register table in batches: ~6x fewer USB round trips per (re-)init - faster start-up and scanning |
| `--hf-filter-fix` | above 2265 MHz use the widest STV6120 HF filter (the C wraps to the narrowest) |
| `--lna-follow-input` | `$2` from the LNA in use (the C always reads TOP, even with `-w`) |
| `--no-lna-poll` | stop the 100 ms LNA gain request |
| `--lna-mode track\|request\|min\|max`, `--lna-pref -25..-18`, `--lna-tlock fast\|slow` | LNA AGC mode, target dBm, lock speed (C: track, -20, slow) |
| `--ts-latency MS` | FTDI latency timer of the TS interface (the C never sets it) |
| `--fast-retune` | scan steps and no-TS re-inits re-tune the demodulator and tuner in place (~200 USB transfers) instead of the C's full ~9,000-transfer re-init; falls back to the full init on any failure |
| `--ts-parse-fix` | parse the TS for `$13`-`$17` without the C's faults (FTDI status bytes read as TS, 0x47 in payloads taken as packet starts, last packet of each read skipped, wrong SDT offsets): a correct `$15` |
| `--extra-status` | `$28`-`$31` after the C's lines (table above) |
| `--warn-low-sr` | warn (stdout) for each symbol rate below 66 kS/s, which the STV0910 is not known to lock; still accepted |
| `--ts-timer-after-init` | the no-TS timeout (`-r`) does not run during a NIM (re-)init and starts when it ends; the C's can fire mid-init and queue a second one |
| `--reconnect` | when the tuner is lost (unplug, or any receiver/TS-read failure), wait for it - checked once a second - and start again, instead of exiting 42 |
| `--second FREQ SR` | a PicoTuner's second receiver (bottom demodulator, tuner 2, TS on endpoint 0x84) beside the main one; outputs `--second-ts-fifo PATH`/`--second-ts-ip IP PORT`, `--second-status-fifo PATH`/`--second-status-ip IP PORT` (default FIFOs `longmynd_second_ts`, `longmynd_second_status`); same status lines |
| `--usb-backend auto\|libusb\|d2xx` | how to reach the tuner (Windows, below); not a change to what is sent to it |
| `--strict-errors` | a re-init that fails while locked stops longmynd (exit 42); the C forgets the error and runs on |
| `--lnb-off-at-exit` | after `-p`, turn the LNB supply off when longmynd stops (the C leaves 13/18 V on the feed) |

Only these exact names are taken; any other `--x` is handled as the C
handles it.

## Linux

Same as the C: FIFOs are created by the reader (`mkfifo`) and opened by
longmynd non-blocking; a missing FIFO is an error, one nobody is reading
yet is retried. Install `packaging/minitiouner.rules` in
`/etc/udev/rules.d/` to run without root. The binary needs glibc 2.34 or
newer (Debian 12, Ubuntu 22.04) and libusb-1.0; it does not need ALSA.

## Windows

`longmynd.exe` is self-contained (libusb is linked in; FTDI's D2XX library
is used if Windows has it). Two differences:

**FIFOs are named pipes.** A FIFO name means `\\.\pipe\<name>` (a name
already starting with `\\` is used as given), so the defaults are
`\\.\pipe\longmynd_main_ts` and `\\.\pipe\longmynd_main_status`. As on
Linux the reader creates it - `CreateNamedPipe` + `ConnectNamedPipe`
where Linux has `mkfifo` + `open` - and longmynd opens it as a client;
"no pipe yet" is "no reader yet" and is retried. Or use UDP (`-i`/`-I`),
which needs no change at all: VLC can play `udp://@:1234` directly.

**The tuner: FTDI's own driver (D2XX), or WinUSB.** Windows binds a
MiniTiouner (FT2232H, 0403:6010) to FTDI's driver, as F6DZP's MiniTioune
expects. longmynd.exe talks to that driver through FTDI's D2XX library,
`ftd2xx.dll`, loaded at run time - so nothing has to be swapped, and
MiniTioune and longmynd can take turns with one tuner (not use it at the
same time: whichever opens it first has it). Windows Update installs the
library with the driver the first time a MiniTiouner is plugged in
(`System32\ftd2xx.dll`, from FTDI's "CDM" package); see "D2XX licence"
below for shipping it.

`--usb-backend auto|libusb|d2xx` chooses; `auto`, the default, uses D2XX
when `ftd2xx.dll` loads and it sees the tuner, and libusb otherwise. So
these all work with no option:

- a MiniTiouner on FTDI's driver - D2XX;
- a MiniTiouner already switched to WinUSB (Zadig, as the older design
  needed) - libusb;
- a PicoTuner (2E8A:BA2C), which is not an FTDI device and which D2XX
  never lists - libusb, with WinUSB on it (Zadig, Options > List All
  Devices, the PicoTuner, WinUSB), unless its firmware advertises WinUSB
  itself.

`--usb-backend d2xx` without the library says so and exits 101 (the C's
"Unable to initialise LIBUSB"). With D2XX, `-u BUS DEVICE` numbers the
tuners D2XX lists as bus 1, devices 1, 2, ... in its order; a tuner another
program has open is not seen (D2XX lists it without its details), so
longmynd says "Unable to open device" (102) and adds a line that an FTDI
device is open elsewhere.

How the mapping works, and what of it is proven: `src/d2xx.rs` (module
comment) and `equiv/tests/d2xx.rs` - every applicable scenario run through
D2XX over a simulated library puts the same MPSSE bytes, I2C transactions
and sleeps on the bus as the C, at the same virtual times, with the same
output and exit code. The library is loaded from the exe's folder or
System32 only, never the current directory. It is the one place
longmynd-rs uses `unsafe`.

### D2XX licence: the DLL is not shipped

FTDI's driver licence lets a *manufacturer or seller of a device with a
genuine FTDI chip* distribute the driver with that device, and lets a
*user* install it "onto a computer system in order to use the Device"
(clause 1.3); it must only be used with genuine FTDI parts (1.4.2), not be
modified (3.1.2), keep FTDI's copyright notice (3.1.6), and not otherwise
be made available to anyone (3.1.7). longmynd-rs is software, not a device
seller, so it **does not ship `ftd2xx.dll`** and an installer for it should
not bundle it. It does not need to: Windows Update installs FTDI's driver
package, `ftd2xx.dll` included, the first time a MiniTiouner is plugged in
(checked 06/10/2026: the Microsoft Update Catalog's "FTDI - USB -
2.12.36.20" package binds `USB\VID_0403&PID_6010&MI_00`/`MI_01` and copies
`ftd2xx.dll`/`ftd2xx64.dll` to System32), and FTDI's own CDM installer does
the same. A kit seller such as BATC could ship it with the hardware under
1.3. Loading it at run time, unshipped, keeps it outside the GPL work.

Sources: FTDI "Driver Licence Terms", ftdichip.com (live page behind a
browser check; text as archived at
`web.archive.org/web/20150930071826/http://www.ftdichip.com:80/Drivers/FTDriverLicenceTerms.htm`);
Microsoft Update Catalog, search "FTDI", package
0d361860-6a8b-42f9-a22f-6499571bae54 (`ftdibus.inf`, DriverVer
10/28/2024,2.12.36.20).

## Building

Everything builds in a container (`tools/Containerfile`: Rust on Debian
12, mingw-w64); cargo is not needed on the host.

```
tools/build.sh            # dist/longmynd (Linux x86-64), dist/longmynd.exe (Windows x64)
tools/cargo.sh test --workspace
```

`tools/gen-regs.sh` regenerates the register tables from the reference
headers, `tools/gen-charsets.sh` the character-set tables from glibc.

## Proof of equivalence

`cargo test` (the `equiv` crate) compiles the pinned C longmynd with libusb
replaced by a simulated MiniTiouner and its clock by a virtual one, runs
the Rust port on the same simulator, and requires byte-identical results
for 29 scenarios: every libusb call, MPSSE byte, decoded I2C transaction
and sleep with its virtual time; stdout; the status output; the TS output;
UDP datagrams; the exit code. Golden copies from the C sit in
`equiv/golden/`. The same scenarios also run through the D2XX backend over
a simulated D2XX library (`equiv/tests/d2xx.rs`: everything above the USB
call layer identical, with its timing), and every option beyond the C has
a test that runs the C beside it. HANDOVER.md explains the harness and
what it does not cover.

## Differences from the C

On a working path, none - that is what the tests hold. Elsewhere:

- **Fixed:** an I2C layer that reads stale bytes forever after one short
  USB read; a NIM read-back check that cannot fail; two tuner table
  lookups that read past their tables (below ~285 MHz, above 2380 MHz);
  three buffer overflows reachable from TS content. (docs/improvements.md)
- **Opt-in:** everything under "Options beyond the C"; with none given the
  binary is the C. Four are decided to become defaults after the hardware
  test (`--c-compat` will then restore the C).
- **Not ported:** the MER beep (`-b` is accepted and says so).
- Service names in EUC-KR, GB2312 or Big5 are shown raw rather than
  converted.
- On Windows: named pipes for FIFOs; FTDI's D2XX driver (or WinUSB)
  rather than libusb alone; libusb logging at WARNING rather than INFO
  (INFO is a line per USB hub there).
- The status line for a name longer than 127 bytes is written whole (the
  C overflows a buffer).

## Licence and attribution

GPLv3 or later (LICENSE), as a derived work of longmynd: Copyright 2019
Heather Lomond, with BATC's contributors (libts: Copyright 2021 Phil
Crump); STV0910 register definitions (c) 2013 Digital Devices GmbH; the
PicoTuner support follows BATC's portsdown/ryde copies and G4EML's
PicoTuner firmware. The pinned C source this port derives from is in
`reference/`, checked by `reference/SHA256SUMS`. Thank you to Heather
Lomond and the British Amateur Television Club for longmynd.
