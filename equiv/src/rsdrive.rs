//! The Rust port on the simulator: its UsbBackend and Clock traits
//! implemented over the shared Sim, and the three run modes, mirroring
//! c/harness.c step for step.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use longmynd::app::{reconnect_wait, reconnectable, reset_for_reconnect, run, second_needs_pico, MainLoop, Platform};
use longmynd::clock::Clock;
use longmynd::config::{process_command_line, Config};
use longmynd::errors::*;
use longmynd::ftdi::ftdi_init;
use longmynd::nim::Nim;
use longmynd::receiver::loop_i2c;
use longmynd::status::{status_all_write, Shared};
use longmynd::ts::{loop_ts, parse_chunk, TsOut, TsParseHandoff};
use longmynd::usb::{DeviceDescriptor, UsbBackend, UsbContext, UsbDevice, UsbHandle};

use longmynd::d2xx::D2xxBackend;

use crate::d2xxsim::SimD2xx;
use crate::scenario::Mode;
use crate::sim;

type Hook = Box<dyn FnMut() + Send>;

/// Called (outside the simulator's lock) before each TS endpoint read.
static TS_HOOK: Mutex<Option<Hook>> = Mutex::new(None);

/// Run the TS-mode hook (outside the simulator's lock): before each read of
/// the TS endpoint, through libusb or D2XX.
pub fn run_ts_hook() {
    let hook = TS_HOOK.lock().unwrap().take();
    if let Some(mut h) = hook {
        h();
        let mut slot = TS_HOOK.lock().unwrap();
        if slot.is_none() {
            *slot = Some(h);
        }
    }
}

pub struct SimBackend;
struct SimCtx;
struct SimDev(usize);
struct SimHandle(usize);

impl UsbBackend for SimBackend {
    fn init(&self) -> Result<Box<dyn UsbContext>, i32> {
        sim::with(|s| s.libusb_init());
        Ok(Box::new(SimCtx))
    }
}

impl UsbContext for SimCtx {
    fn set_log_level_info(&mut self) {
        sim::with(|s| s.set_option_log_info());
    }
    fn open_device_with_vid_pid(&mut self, vid: u16, pid: u16) -> Option<Box<dyn UsbHandle>> {
        sim::with(|s| s.open_vid_pid(vid, pid)).map(|h| Box::new(SimHandle(h)) as Box<dyn UsbHandle>)
    }
    fn get_device_list(&mut self) -> Result<Vec<Box<dyn UsbDevice>>, i32> {
        let n = sim::with(|s| s.device_list());
        Ok((0..n).map(|i| Box::new(SimDev(i)) as Box<dyn UsbDevice>).collect())
    }
}

impl UsbDevice for SimDev {
    fn bus_number(&self) -> u8 {
        sim::with(|s| s.dev(self.0).bus)
    }
    fn device_address(&self) -> u8 {
        sim::with(|s| s.dev(self.0).addr)
    }
    fn device_descriptor(&self) -> Result<DeviceDescriptor, i32> {
        Ok(sim::with(|s| {
            let d = s.dev(self.0);
            DeviceDescriptor { id_vendor: d.vid, id_product: d.pid, bcd_device: d.bcd }
        }))
    }
    fn open(&self) -> Result<Box<dyn UsbHandle>, i32> {
        sim::with(|s| s.open_dev(self.0)).map(|h| Box::new(SimHandle(h)) as Box<dyn UsbHandle>)
    }
}

impl UsbHandle for SimHandle {
    fn kernel_driver_active(&mut self, interface: u8) -> i32 {
        sim::with(|s| s.kernel_driver_active(self.0, interface))
    }
    fn detach_kernel_driver(&mut self, interface: u8) -> i32 {
        sim::with(|s| s.detach_kernel_driver(self.0, interface))
    }
    fn claim_interface(&mut self, interface: u8) -> i32 {
        sim::with(|s| s.claim_interface(self.0, interface))
    }
    fn device_descriptor(&mut self) -> Result<DeviceDescriptor, i32> {
        let (v, p, b) = sim::with(|s| s.handle_descriptor(self.0));
        Ok(DeviceDescriptor { id_vendor: v, id_product: p, bcd_device: b })
    }
    fn bulk_transfer(&mut self, endpoint: u8, data: &mut [u8], _timeout_ms: u32) -> (i32, usize) {
        if endpoint == 0x83 {
            run_ts_hook();
        }
        sim::with(|s| s.bulk(self.0, endpoint, data))
    }
    fn control_transfer(&mut self, request_type: u8, request: u8, value: u16, index: u16, _timeout_ms: u32) -> i32 {
        sim::with(|s| s.control(self.0, request_type, request, value, index))
    }
}

/// The virtual clock, with the receiver-mode hook run after each sleep.
pub struct SimClock {
    pub hook: Mutex<Option<Hook>>,
}

impl Clock for SimClock {
    fn monotonic_ms(&self) -> u64 {
        sim::with(|s| s.monotonic_us()) / 1000
    }
    fn usleep(&self, us: u64) {
        sim::with(|s| s.usleep(us));
        let hook = self.hook.lock().unwrap().take();
        if let Some(mut h) = hook {
            h();
            let mut slot = self.hook.lock().unwrap();
            if slot.is_none() {
                *slot = Some(h);
            }
        }
    }
}

/// Run through longmynd's D2XX backend over the simulated D2XX library
/// (d2xxsim.rs) instead of the libusb-shaped SimBackend.
static USE_D2XX: AtomicBool = AtomicBool::new(false);
static D2XX_BACKEND: std::sync::OnceLock<D2xxBackend> = std::sync::OnceLock::new();

pub fn use_d2xx() {
    USE_D2XX.store(true, Ordering::SeqCst);
}

/// The USB backend for this run: one D2XX backend for the whole run, as
/// the real program has (it remembers which chips it has open).
fn backend() -> Arc<dyn UsbBackend> {
    if USE_D2XX.load(Ordering::SeqCst) {
        Arc::new(D2XX_BACKEND.get_or_init(|| D2xxBackend::new(Arc::new(SimD2xx))).clone())
    } else {
        Arc::new(SimBackend)
    }
}

fn platform() -> (Platform, Arc<SimClock>) {
    let clock = Arc::new(SimClock { hook: Mutex::new(None) });
    let p = Platform { usb: backend(), clock: clock.clone(), terminate: Arc::new(AtomicBool::new(false)) };
    (p, clock)
}

/// c/harness.c setup(): main() from the command line to just before the
/// threads. Returns the config, the main loop (status output) and what
/// ftdi_init() opened.
fn setup(args: &[Vec<u8>], shared: &Shared, clock: &Arc<dyn Clock>) -> (Config, MainLoop, Option<longmynd::ftdi::FtdiOpened>) {
    println!("Flow: main");
    let mut config = Config::default();
    let mut err = process_command_line(args, &mut config);
    *shared.config.lock().unwrap() = config.clone();
    let mut ml = MainLoop::new(&config);
    if err == ERROR_NONE {
        err = ml.init_output(&config);
    }
    let mut opened = None;
    if err == ERROR_NONE {
        match ftdi_init(&*backend(), clock.clone(), config.device_usb_bus, config.device_usb_addr, config.ext.ts_latency_ms) {
            Ok(o) => match second_needs_pico(&config, &o) {
                ERROR_NONE => opened = Some(o),
                e => err = e,
            },
            Err(e) => err = e,
        }
    }
    shared.main_err.store(err, Ordering::SeqCst);
    (config, ml, opened)
}

fn finish() {
    println!("Flow: Main loop aborted, waiting for threads.");
    println!("Flow: All threads accounted for. Exiting cleanly.");
}

fn run_receiver(args: &[Vec<u8>]) -> u8 {
    let (_, sim_clock) = platform();
    let clock: Arc<dyn Clock> = sim_clock.clone();
    let shared = Arc::new(Shared::new(Config::default()));
    let (config, ml, opened) = setup(args, &shared, &clock);
    if let Some(mut o) = opened {
        let ml = Arc::new(Mutex::new(ml));
        // app::run()'s session loop: one pass unless --reconnect
        loop {
            let mut nim = Nim::new(o.ftdi, o.serial_ts, config.ext.clone());
            shared.status.lock().unwrap().last_ts_or_reinit_monotonic = clock.monotonic_ms();
            {
                let (shared, ml, clock) = (shared.clone(), ml.clone(), clock.clone());
                *sim_clock.hook.lock().unwrap() = Some(Box::new(move || {
                    let add = sim::with(|s| s.ts_flow());
                    if add != 0 {
                        shared.ts_packet_count_nolock.fetch_add(add, Ordering::SeqCst);
                        shared.ts_payload_bytes.fetch_add(add as u64, Ordering::SeqCst);
                    }
                    if shared.main_err() == ERROR_NONE {
                        ml.lock().unwrap().poll(&shared, &*clock);
                    }
                    if sim::with(|s| s.terminate_due()) && shared.main_err() == ERROR_NONE {
                        shared.main_err.store(ERROR_SIGNAL_TERMINATE, Ordering::SeqCst);
                    }
                }));
            }
            loop_i2c(&mut nim, &shared, &clock);
            *sim_clock.hook.lock().unwrap() = None;
            if shared.main_err() == ERROR_NONE {
                ml.lock().unwrap().poll(&shared, &*clock);
            }
            drop(nim);
            if !(config.ext.reconnect && reconnectable(&shared)) {
                break;
            }
            println!("Flow: Main loop aborted, waiting for threads.");
            reset_for_reconnect(&shared);
            let stop = || sim::with(|s| s.terminate_due());
            match reconnect_wait(&*backend(), &clock, &config, &stop) {
                Ok(n) => o = n,
                Err(e) => {
                    shared.main_err.store(e, Ordering::SeqCst);
                    break;
                }
            }
        }
    }
    finish();
    shared.main_err()
}

fn run_ts(args: &[Vec<u8>]) -> u8 {
    let (_, sim_clock) = platform();
    let clock: Arc<dyn Clock> = sim_clock;
    let shared = Arc::new(Shared::new(Config::default()));
    let (config, ml, opened) = setup(args, &shared, &clock);
    if let Some(o) = opened {
        let handoff = Arc::new(TsParseHandoff::new());
        let ml = Arc::new(Mutex::new(ml));
        shared.ts_reset.store(true, Ordering::SeqCst);
        let armed = Arc::new(AtomicBool::new(false));
        let parse_pending = {
            let (shared, handoff, ml, armed) = (shared.clone(), handoff.clone(), ml.clone(), armed.clone());
            move || {
                if armed.load(Ordering::SeqCst) && !handoff.waiting.load(Ordering::SeqCst) {
                    {
                        let inner = handoff.inner.lock().unwrap();
                        let fixed = shared.config.lock().unwrap().ext.ts_parse_fix;
                        parse_chunk(&shared, &inner.buffer, inner.length, fixed);
                    }
                    let cpy = shared.status.lock().unwrap().clone();
                    let mut ml = ml.lock().unwrap();
                    let ml = &mut *ml;
                    if ml.status_use_ip || ml.status_output_ready {
                        if let Some(sink) = ml.sink.as_mut() {
                            let e = status_all_write(&cpy, &mut **sink, &mut ml.status_output_ready, ml.extra_status);
                            if e != ERROR_NONE && shared.main_err() == ERROR_NONE {
                                shared.main_err.store(e, Ordering::SeqCst);
                            }
                        }
                    }
                    // --second: its TS was parsed in the TS thread itself
                    let extra = ml.extra_status;
                    if let Some((sink, use_ip, ready, _)) = ml.second.as_mut() {
                        if *use_ip || *ready {
                            let st2 = shared.status2.lock().unwrap().clone();
                            let _ = status_all_write(&st2, &mut **sink, ready, extra);
                        }
                    }
                }
            }
        };
        {
            let (shared, handoff, armed, parse_pending) = (shared.clone(), handoff.clone(), armed.clone(), parse_pending.clone());
            *TS_HOOK.lock().unwrap() = Some(Box::new(move || {
                parse_pending();
                handoff.waiting.store(true, Ordering::SeqCst);
                armed.store(true, Ordering::SeqCst);
                if sim::with(|s| s.terminate_due()) && shared.main_err() == ERROR_NONE {
                    shared.main_err.store(ERROR_SIGNAL_TERMINATE, Ordering::SeqCst);
                }
            }));
        }
        let out = if config.ts_use_ip {
            TsOut::Udp { ip: config.ts_ip_addr.clone(), port: config.ts_ip_port }
        } else {
            TsOut::Fifo { path: config.ts_fifo_path.clone() }
        };
        let mut ts = o.ts;
        match config.ext.second.as_ref() {
            Some(s) => longmynd::ts::loop_ts_dual(&shared, &mut ts, out, TsOut::from_out(&s.ts), &handoff),
            None => loop_ts(&shared, &mut ts, out, &handoff),
        }
        *TS_HOOK.lock().unwrap() = None;
        parse_pending();
        if shared.main_err() == ERROR_NONE && shared.any_thread_err() {
            shared.main_err.store(ERROR_THREAD_ERROR, Ordering::SeqCst);
        }
    }
    finish();
    shared.main_err()
}

/// Run `mode` with `args` (argv, program name first) on the simulator,
/// which the caller has started.
///
/// Once the decided defaults are switched on (config::
/// DEFAULTS_AFTER_HW_TEST) the Rust side runs every scenario with
/// `--c-compat`: the equivalence scenarios are then still the C's
/// behaviour, and each extension scenario still the C plus only the
/// option it is about. Until then the command line is used as it is, so
/// the proof is of the plain default.
pub fn run_mode(mode: Mode, args: &[Vec<u8>]) -> u8 {
    let mut args = args.to_vec();
    if longmynd::config::DEFAULTS_AFTER_HW_TEST {
        args.insert(1, b"--c-compat".to_vec());
    }
    let args = &args[..];
    match mode {
        Mode::Main => {
            let (p, _) = platform();
            run(args, &p)
        }
        Mode::Receiver => run_receiver(args),
        Mode::Ts => run_ts(args),
    }
}
