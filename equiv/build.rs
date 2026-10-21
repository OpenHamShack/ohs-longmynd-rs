//! Builds the reference C longmynd (../reference: the pinned BATC tarball
//! plus the PicoTuner patch) into a static library for the c_reference
//! binary, with libusb and the clock replaced by the simulator.
//!
//! Compiled as OpenHamShack builds it for x86-64 (-O3 -march=x86-64-v2, no
//! fast-math), so the floating-point paths - carrier offset, symbol rate,
//! BER - are the shipped binary's.

use std::path::{Path, PathBuf};
use std::process::Command;

fn run(cmd: &mut Command) {
    let st = cmd.status().unwrap_or_else(|e| panic!("{:?}: {}", cmd, e));
    assert!(st.success(), "{:?} failed", cmd);
}

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let reference = manifest.join("../reference").canonicalize().unwrap();
    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    println!("cargo:rerun-if-changed={}", reference.display());
    println!("cargo:rerun-if-changed=c");

    // the exact bytes, or nothing
    run(Command::new("sha256sum").arg("-c").arg("--quiet").arg("SHA256SUMS").current_dir(&reference));

    let tree = out.join("longmynd");
    let _ = std::fs::remove_dir_all(&tree);
    std::fs::create_dir_all(&tree).unwrap();
    let tarball = std::fs::read_dir(&reference)
        .unwrap()
        .map(|e| e.unwrap().path())
        .find(|p| p.extension().map(|e| e == "gz").unwrap_or(false))
        .expect("no tarball in reference/");
    run(Command::new("tar").arg("-xzf").arg(&tarball).arg("--strip-components=1").arg("-C").arg(&tree));
    run(Command::new("patch")
        .args(["-s", "-p1", "--forward", "--fuzz=0", "-i"])
        .arg(reference.join("longmynd-picotuner.patch"))
        .current_dir(&tree));

    let sources = [
        "nim.c", "ftdi.c", "stv0910.c", "stv0910_utils.c", "stvvglna.c", "stvvglna_utils.c", "stv6120.c",
        "stv6120_utils.c", "ftdi_usb.c", "fifo.c", "udp.c", "libts.c",
    ];
    let c = manifest.join("c");
    let mut b = cc::Build::new();
    for s in sources {
        b.file(tree.join(s));
    }
    b.file(c.join("harness.c"))
        .include(c.join("include"))
        .include(&tree)
        .flag("-include")
        .flag(c.join("lmsim_hooks.h").to_str().unwrap())
        .define("_GNU_SOURCE", None)
        .define("VERSION", "\"b9a325b+picotuner\"")
        .opt_level(3)
        .flag("-march=x86-64-v2")
        .flag("-pthread")
        .warnings(false)
        .flag("-w");
    // Linked into c_reference only. Attached to the library instead, its
    // libusb_set_option would also be pulled into rs_reference ahead of
    // the real libusb that rusb links there.
    b.cargo_metadata(false);
    b.compile("lmref");
    let lib = out.join("liblmref.a");
    assert!(Path::new(&lib).exists());
    println!("cargo:rustc-link-arg-bin=c_reference={}", lib.display());
    println!("cargo:rustc-link-arg-bin=c_reference=-lpthread");
    println!("cargo:rustc-link-arg-bin=c_reference=-lm");
}
