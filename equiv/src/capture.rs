//! Running one scenario: the output directory, the files the command line
//! points at, UDP listeners, and writing down what happened.

use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use crate::scenario::Scenario;
use crate::sim::{Sim, SIM};

pub struct Run {
    pub dir: PathBuf,
    udp: Vec<(u16, Arc<AtomicBool>, JoinHandle<Vec<Vec<u8>>>)>,
}

/// Prepare `dir` for `scn` and start the simulator.
pub fn start(scn: &Scenario, dir: &Path) -> Run {
    let _ = std::fs::remove_dir_all(dir);
    std::fs::create_dir_all(dir).unwrap();
    for f in &scn.files {
        std::fs::write(dir.join(f), b"").unwrap();
    }
    let mut udp = Vec::new();
    for &port in &scn.udp_ports {
        let sock = UdpSocket::bind(("127.0.0.1", port)).unwrap_or_else(|e| panic!("bind 127.0.0.1:{}: {}", port, e));
        sock.set_read_timeout(Some(Duration::from_millis(50))).unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let st = stop.clone();
        let h = std::thread::spawn(move || {
            let mut got = Vec::new();
            let mut buf = vec![0u8; 65536];
            let mut idle = 0;
            loop {
                match sock.recv(&mut buf) {
                    Ok(n) => {
                        got.push(buf[..n].to_vec());
                        idle = 0;
                    }
                    Err(_) => {
                        if st.load(Ordering::SeqCst) {
                            idle += 1;
                            if idle >= 4 {
                                break;
                            }
                        }
                    }
                }
            }
            got
        });
        udp.push((port, stop, h));
    }
    Sim::start(scn.clone());
    Run { dir: dir.to_path_buf(), udp }
}

impl Run {
    /// Write the trace, the UDP captures and the exit status.
    pub fn finish(self, exit: u8) {
        let trace = Sim::take_trace();
        std::fs::write(self.dir.join("trace.txt"), trace).unwrap();
        for (port, stop, h) in self.udp {
            stop.store(true, Ordering::SeqCst);
            let got = h.join().unwrap();
            let mut out = Vec::new();
            for d in got {
                out.extend_from_slice(format!("{}\n", d.len()).as_bytes());
                out.extend_from_slice(&d);
                out.push(b'\n');
            }
            std::fs::write(self.dir.join(format!("udp_{}.bin", port)), out).unwrap();
        }
        std::fs::write(self.dir.join("exit.txt"), format!("{}\n", exit)).unwrap();
        *SIM.lock().unwrap() = None;
    }
}
