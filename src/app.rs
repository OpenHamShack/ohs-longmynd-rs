//! main.c's main(): parse the command line, open the status output and the
//! tuner, start the TS, TS-parse, receiver and beep threads, then write the
//! status out whenever the receiver has published a new one, and re-tune
//! when no TS has arrived for the timeout.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crate::clock::Clock;
use crate::config::{config_reinit, process_command_line, Config};
use crate::errors::*;
use crate::ftdi::{ftdi_init, FtdiOpened};
use crate::nim::Nim;
use crate::output::{Fifo, FifoStatus, StatusSink, Udp, UdpStatus};
use crate::status::{status_all_write, Shared};
use crate::ts::{loop_ts, loop_ts_parse, TsOut, TsParseHandoff};
use crate::usb::UsbBackend;

/// What the program runs on: USB and time, real or simulated.
pub struct Platform {
    pub usb: Arc<dyn UsbBackend>,
    pub clock: Arc<dyn Clock>,
    /// Set by SIGINT/SIGTERM (Ctrl+C on Windows); main() turns it into
    /// ERROR_SIGNAL_TERMINATE, which every thread is watching for.
    pub terminate: Arc<AtomicBool>,
}

/// The status side of main()'s loop.
pub struct MainLoop {
    pub sink: Option<Box<dyn StatusSink>>,
    pub status_output_ready: bool,
    pub last_status_sent_monotonic: u64,
    pub status_use_ip: bool,
    pub ts_timeout: i32,
    /// --extra-status
    pub extra_status: bool,
    /// --ts-timer-after-init
    pub ts_timer_after_init: bool,
    /// --second: the second receiver's status output (its sink, whether a
    /// FIFO one is ready, the status last written)
    pub second: Option<(Box<dyn StatusSink>, bool, bool, u64)>,
}

impl MainLoop {
    pub fn new(config: &Config) -> Self {
        MainLoop {
            sink: None,
            status_output_ready: true,
            last_status_sent_monotonic: 0,
            status_use_ip: config.status_use_ip,
            ts_timeout: config.ts_timeout,
            extra_status: config.ext.extra_status,
            ts_timer_after_init: config.ext.ts_timer_after_init,
            second: None,
        }
    }

    /// Open the status output: fifo_status_init() or udp_status_init();
    /// with --second its status output too.
    pub fn init_output(&mut self, config: &Config) -> u8 {
        let err = self.init_main_output(config);
        if err != ERROR_NONE {
            return err;
        }
        if let Some(s) = &config.ext.second {
            let mut ready = true;
            let sink: Result<Box<dyn StatusSink>, u8> = match &s.status {
                crate::config::Out::Udp(ip, port) => Udp::init(ip, *port).map(|u| Box::new(UdpStatus(u)) as Box<dyn StatusSink>),
                crate::config::Out::Fifo(p) => Fifo::init(p, &mut ready).map(|f| Box::new(FifoStatus(f)) as Box<dyn StatusSink>),
            };
            match sink {
                Ok(k) => self.second = Some((k, matches!(s.status, crate::config::Out::Udp(..)), ready, 0)),
                Err(e) => return e,
            }
        }
        ERROR_NONE
    }

    fn init_main_output(&mut self, config: &Config) -> u8 {
        if config.status_use_ip {
            match Udp::init(&config.status_ip_addr, config.status_ip_port) {
                Ok(u) => {
                    self.sink = Some(Box::new(UdpStatus(u)));
                    ERROR_NONE
                }
                Err(e) => e,
            }
        } else {
            match Fifo::init(&config.status_fifo_path, &mut self.status_output_ready) {
                Ok(f) => {
                    self.sink = Some(Box::new(FifoStatus(f)));
                    ERROR_NONE
                }
                Err(e) => e,
            }
        }
    }

    /// One pass of main()'s loop, without its 10ms sleep: returns whether
    /// there was new status (the C sleeps only when there was not).
    pub fn poll(&mut self, shared: &Shared, clock: &dyn Clock) -> bool {
        let mut err = shared.main_err();
        let mut had_new = false;
        let last_updated = shared.status.lock().unwrap().last_updated_monotonic;
        if last_updated != self.last_status_sent_monotonic {
            let mut cpy = shared.status.lock().unwrap().clone();
            cpy.ts_dropped = shared.ts_dropped.load(Ordering::SeqCst);
            if let Some(sink) = self.sink.as_mut() {
                if self.status_use_ip || self.status_output_ready {
                    err = status_all_write(&cpy, &mut **sink, &mut self.status_output_ready, self.extra_status);
                } else {
                    err = sink.reopen(&mut self.status_output_ready);
                }
            }
            self.last_status_sent_monotonic = cpy.last_updated_monotonic;
            had_new = true;
        }
        // --second: its status, the same lines on its own output
        if err == ERROR_NONE {
            if let Some((sink, use_ip, ready, last)) = self.second.as_mut() {
                let st = shared.status2.lock().unwrap().clone();
                if st.last_updated_monotonic != *last {
                    err = if *use_ip || *ready { status_all_write(&st, &mut **sink, ready, self.extra_status) } else { sink.reopen(ready) };
                    *last = st.last_updated_monotonic;
                }
            }
        }
        if err == ERROR_NONE && shared.any_thread_err() {
            err = ERROR_THREAD_ERROR;
        }
        shared.main_err.store(err, Ordering::SeqCst);

        let last_ts = shared.status.lock().unwrap().last_ts_or_reinit_monotonic;
        // --ts-timer-after-init: no timeout while an init is running (the
        // receiver restarts the timer when it finishes)
        let initialising = self.ts_timer_after_init && shared.init_in_progress.load(Ordering::SeqCst);
        if self.ts_timeout != -1 && !initialising && clock.monotonic_ms() > last_ts.wrapping_add(self.ts_timeout as i64 as u64) {
            // a while with no TS: re-tune, which also pulls a NIM that has
            // wandered off in its search loops back
            println!("Flow: No-data timeout, re-init config.");
            let line = config_reinit(&mut shared.config.lock().unwrap(), true);
            if let Some(line) = line {
                println!("{}", line);
            }
            shared.status.lock().unwrap().last_ts_or_reinit_monotonic = clock.monotonic_ms();
        }
        had_new
    }
}

/// loop_beep(). The MER-pitched tone (-b) is not ported yet: it needs an
/// audio output on both platforms (ALSA in the C). The thread keeps the
/// C's shape so -b costs nothing and changes nothing else.
fn loop_beep(shared: &Shared, clock: &dyn Clock, beep_enabled: bool) {
    shared.thread_err_beep.store(ERROR_NONE, Ordering::SeqCst);
    if beep_enabled {
        println!("WARNING: the MER beep (-b) is not available in longmynd-rs yet; continuing without it");
    }
    while shared.thread_err_beep.load(Ordering::SeqCst) == ERROR_NONE && shared.main_err() == ERROR_NONE {
        clock.usleep(10 * 1000);
    }
}

/// --reconnect: whether what ended a session is worth reconnecting for.
/// Anything a worker thread failed on counts - an unplug shows up as a USB
/// error in the TS thread or the receiver, and a NIM that stops answering
/// is as well retried as given up on - except the TS output failing,
/// which a new tuner would not mend. A signal, or a status output error
/// (main()'s own), is never a reason.
pub fn reconnectable(shared: &Shared) -> bool {
    let ts = shared.thread_err_ts.load(Ordering::SeqCst);
    let output = matches!(ts, ERROR_TS_FIFO_WRITE | ERROR_OPEN_TS_FIFO | ERROR_UDP_WRITE | ERROR_UDP_SOCKET_OPEN);
    shared.main_err() == ERROR_THREAD_ERROR && !output
}

/// --reconnect: clear every thread's error and flag a full re-init with the
/// configuration as it stands (the current frequency and rate of a scan).
pub fn reset_for_reconnect(shared: &Shared) {
    for e in [&shared.thread_err_ts, &shared.thread_err_ts_parse, &shared.thread_err_i2c, &shared.thread_err_beep] {
        e.store(ERROR_NONE, Ordering::SeqCst);
    }
    shared.main_err.store(ERROR_NONE, Ordering::SeqCst);
    shared.config.lock().unwrap().new = true;
    shared.init_in_progress.store(false, Ordering::SeqCst);
}

/// Whether the tuner can be seen on the bus, asked quietly (no messages,
/// nothing claimed): by VID/PID as ftdi_usb_init() opens it, or by -u.
fn tuner_present(usb: &dyn UsbBackend, bus: u8, addr: u8) -> bool {
    use crate::ftdi::{FTDI_PID, FTDI_VID, PICO_PID, PICO_VID};
    let Ok(mut ctx) = usb.init() else { return false };
    if bus == 0 && addr == 0 {
        ctx.open_device_with_vid_pid(FTDI_VID, FTDI_PID).is_some() || ctx.open_device_with_vid_pid(PICO_VID, PICO_PID).is_some()
    } else {
        ctx.get_device_list()
            .map(|l| l.iter().any(|d| d.bus_number() == bus && d.device_address() == addr))
            .unwrap_or(false)
    }
}

/// --reconnect: wait for the tuner to come back - looked for once a second,
/// quietly - and open it as at start. Err is the error to stop with: a
/// signal (`stop()` true) while waiting, or ftdi_init() failing on a tuner
/// that is there (then it is not coming back by waiting).
pub fn reconnect_wait(usb: &dyn UsbBackend, clock: &Arc<dyn Clock>, config: &Config, stop: &dyn Fn() -> bool) -> Result<FtdiOpened, u8> {
    println!("Flow: Tuner lost, waiting for it to come back (--reconnect)");
    loop {
        if stop() {
            return Err(ERROR_SIGNAL_TERMINATE);
        }
        if tuner_present(usb, config.device_usb_bus, config.device_usb_addr) {
            println!("Flow: Tuner is back, reconnecting");
            return ftdi_init(usb, clock.clone(), config.device_usb_bus, config.device_usb_addr, config.ext.ts_latency_ms);
        }
        clock.usleep(1000 * 1000);
    }
}

/// --second needs a PicoTuner: a MiniTiouner's FTDI carries one TS only.
/// Checked once the tuner is open; the C's error for bad arguments (2).
pub fn second_needs_pico(config: &Config, opened: &FtdiOpened) -> u8 {
    if config.ext.second.is_some() && !opened.pico {
        println!("ERROR: --second needs a PicoTuner: a MiniTiouner carries only one TS to the USB");
        return ERROR_ARGS_INPUT;
    }
    ERROR_NONE
}

/// Start the TS, TS-parse, receiver and beep threads on an opened tuner.
fn start_threads(o: FtdiOpened, config: &Config, shared: &Arc<Shared>, clock: &Arc<dyn Clock>) -> Vec<std::thread::JoinHandle<()>> {
    let mut threads = Vec::new();
    let handoff = Arc::new(TsParseHandoff::new());
    let ts_out = if config.ts_use_ip {
        TsOut::Udp { ip: config.ts_ip_addr.clone(), port: config.ts_ip_port }
    } else {
        TsOut::Fifo { path: config.ts_fifo_path.clone() }
    };
    let mut ts_usb = o.ts;
    let mut nim = Nim::new(o.ftdi, o.serial_ts, config.ext.clone());

    let (s, h) = (shared.clone(), handoff.clone());
    let second_out = config.ext.second.as_ref().map(|s| TsOut::from_out(&s.ts));
    threads.push(
        std::thread::Builder::new()
            .name("TS Transport".into())
            .spawn(move || match second_out {
                Some(out2) => crate::ts::loop_ts_dual(&s, &mut ts_usb, ts_out, out2, &h),
                None => loop_ts(&s, &mut ts_usb, ts_out, &h),
            })
            .expect("Error creating loop_ts thread"),
    );
    let (s, h) = (shared.clone(), handoff.clone());
    threads.push(
        std::thread::Builder::new()
            .name("TS Parse".into())
            .spawn(move || loop_ts_parse(&s, &h))
            .expect("Error creating loop_ts_parse thread"),
    );
    let (s, c) = (shared.clone(), clock.clone());
    threads.push(
        std::thread::Builder::new()
            .name("Receiver".into())
            .spawn(move || crate::receiver::loop_i2c(&mut nim, &s, &c))
            .expect("Error creating loop_i2c thread"),
    );
    let (s, c, beep) = (shared.clone(), clock.clone(), config.beep_enabled);
    threads.push(
        std::thread::Builder::new()
            .name("Beep Audio".into())
            .spawn(move || loop_beep(&s, &*c, beep))
            .expect("Error creating loop_beep thread"),
    );
    // stop an immediate no-TS re-init
    shared.status.lock().unwrap().last_ts_or_reinit_monotonic = clock.monotonic_ms();
    threads
}

/// main(): returns the exit status.
pub fn run(args: &[Vec<u8>], platform: &Platform) -> u8 {
    let clock = platform.clock.clone();
    let shared = Arc::new(Shared::new(Config::default()));
    let check_signal = || {
        if platform.terminate.load(Ordering::SeqCst) {
            shared.main_err.store(ERROR_SIGNAL_TERMINATE, Ordering::SeqCst);
        }
    };

    println!("Flow: main");

    let mut config = Config::default();
    check_signal();
    if shared.main_err() == ERROR_NONE {
        let e = process_command_line(args, &mut config);
        shared.main_err.store(e, Ordering::SeqCst);
    }
    *shared.config.lock().unwrap() = config.clone();

    let mut main_loop = MainLoop::new(&config);
    if shared.main_err() == ERROR_NONE {
        let e = main_loop.init_output(&config);
        shared.main_err.store(e, Ordering::SeqCst);
    }

    let mut opened = None;
    check_signal();
    if shared.main_err() == ERROR_NONE {
        match ftdi_init(&*platform.usb, clock.clone(), config.device_usb_bus, config.device_usb_addr, config.ext.ts_latency_ms) {
            Ok(o) => match second_needs_pico(&config, &o) {
                ERROR_NONE => opened = Some(o),
                e => shared.main_err.store(e, Ordering::SeqCst),
            },
            Err(e) => shared.main_err.store(e, Ordering::SeqCst),
        }
    }

    // One pass is the C's whole life; --reconnect makes more of them.
    loop {
        let threads = match opened.take() {
            Some(o) => start_threads(o, &config, &shared, &clock),
            None => Vec::new(),
        };

        while shared.main_err() == ERROR_NONE {
            check_signal();
            if !main_loop.poll(&shared, &*clock) {
                clock.usleep(10 * 1000);
            }
        }

        println!("Flow: Main loop aborted, waiting for threads.");
        for t in threads {
            let _ = t.join();
        }
        // the threads have ended, and with them the old tuner's handles

        if !(config.ext.reconnect && reconnectable(&shared)) {
            break;
        }
        reset_for_reconnect(&shared);
        let stop = || platform.terminate.load(Ordering::SeqCst);
        match reconnect_wait(&*platform.usb, &clock, &config, &stop) {
            Ok(o) => opened = Some(o),
            Err(e) => {
                shared.main_err.store(e, Ordering::SeqCst);
                break;
            }
        }
    }
    println!("Flow: All threads accounted for. Exiting cleanly.");
    shared.main_err()
}
