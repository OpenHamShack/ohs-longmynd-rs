//! Runs one scenario through the Rust port on the simulator.
//!   rs_reference <scenario> <outdir> [d2xx]
//! With "d2xx", through longmynd's D2XX backend over a simulated D2XX
//! library instead of the libusb-shaped one.
//! stdout is the program's stdout; <outdir> gets trace.txt, the files the
//! command line wrote, UDP captures and exit.txt.

use longmynd_equiv::{capture, rsdrive, scenario};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let scn = scenario::find(&a[1]).unwrap_or_else(|| panic!("no scenario {}", a[1]));
    let dir = std::path::PathBuf::from(&a[2]);
    if a.get(3).map(String::as_str) == Some("d2xx") {
        rsdrive::use_d2xx();
    }
    let run = capture::start(&scn, &dir);
    let argv: Vec<Vec<u8>> = scn.argv(dir.to_str().unwrap()).into_iter().map(String::into_bytes).collect();
    let code = rsdrive::run_mode(scn.mode, &argv);
    use std::io::Write;
    std::io::stdout().flush().unwrap();
    run.finish(code);
    std::process::exit(code as i32);
}
