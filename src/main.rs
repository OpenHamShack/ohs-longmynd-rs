//! The longmynd executable: the real USB backend, the real clock, and the
//! termination signals, around app::run().

use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use longmynd::app::{run, Platform};
use longmynd::clock::SystemClock;
use longmynd::config::split_extensions;
use longmynd::d2xx::D2xxBackend;
use longmynd::usb::choose_backend;

#[cfg(unix)]
fn args() -> Vec<Vec<u8>> {
    use std::os::unix::ffi::OsStringExt;
    std::env::args_os().map(|a| a.into_vec()).collect()
}

#[cfg(windows)]
fn args() -> Vec<Vec<u8>> {
    std::env::args_os().map(|a| a.to_string_lossy().into_owned().into_bytes()).collect()
}

/// SIGTERM and SIGINT stop longmynd cleanly, as the C's handler does.
/// SIGPIPE is already ignored by the Rust runtime (the C ignores it
/// explicitly), so a reader closing a FIFO is an EPIPE, not a death.
#[cfg(unix)]
fn install_signals(flag: &Arc<AtomicBool>) {
    for sig in [signal_hook::consts::SIGTERM, signal_hook::consts::SIGINT] {
        signal_hook::flag::register(sig, flag.clone()).expect("cannot install signal handler");
    }
}

#[cfg(windows)]
fn install_signals(flag: &Arc<AtomicBool>) {
    let f = flag.clone();
    ctrlc::set_handler(move || f.store(true, std::sync::atomic::Ordering::SeqCst))
        .expect("cannot install Ctrl+C handler");
}

fn main() {
    let terminate = Arc::new(AtomicBool::new(false));
    install_signals(&terminate);
    let args = args();
    // --usb-backend is needed before run() parses the command line (which
    // reports any error in it, so a bad one here just means the default)
    let choice = split_extensions(&args).map(|(_, e)| e.usb_backend).unwrap_or(longmynd::config::UsbBackendChoice::Auto);
    let (usb, problem) = choose_backend(choice, D2xxBackend::load);
    if let Some(p) = problem {
        println!("{}", p);
    }
    let platform = Platform { usb, clock: Arc::new(SystemClock::new()), terminate };
    let code = run(&args, &platform);
    std::process::exit(code as i32);
}
