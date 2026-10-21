//! Time, behind a trait so the equivalence harness can run the receiver on
//! a virtual clock.
//!
//! The C calls `usleep()` and its own `monotonic_ms()` (CLOCK_MONOTONIC in
//! milliseconds) directly. Every one of those calls in the port goes
//! through [`Clock`], in the same places and with the same arguments, which
//! is what lets the harness drive C and Rust through identical time and
//! compare them sleep for sleep.

use std::time::{Duration, Instant};

pub trait Clock: Send + Sync {
    /// longmynd's `monotonic_ms()`.
    fn monotonic_ms(&self) -> u64;
    /// `usleep(us)`.
    fn usleep(&self, us: u64);
}

/// The real clock. Instant is CLOCK_MONOTONIC on Linux and
/// QueryPerformanceCounter on Windows; only differences between readings
/// are ever used, so it counts from process start plus a fixed offset. The
/// offset matters: the C treats a timestamp of 0 as "not set"
/// (`last_ts_or_reinit_monotonic != 0`), which CLOCK_MONOTONIC never is
/// but a count from process start would be for the first millisecond.
pub struct SystemClock {
    start: Instant,
}

const SYSTEM_CLOCK_OFFSET_MS: u64 = 1_000_000;

impl SystemClock {
    pub fn new() -> Self {
        SystemClock { start: Instant::now() }
    }
}

impl Default for SystemClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for SystemClock {
    fn monotonic_ms(&self) -> u64 {
        SYSTEM_CLOCK_OFFSET_MS + self.start.elapsed().as_millis() as u64
    }
    fn usleep(&self, us: u64) {
        std::thread::sleep(Duration::from_micros(us));
    }
}
