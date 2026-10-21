#!/usr/bin/env bash
# Build the release binaries in the build container:
#   dist/longmynd       Linux x86-64 (glibc 2.36+, links libusb-1.0 dynamically)
#   dist/longmynd.exe   Windows x64 (libusb built in statically; needs the
#                       WinUSB driver on the MiniTiouner - README "Windows")
#
#   tools/build.sh [--keep]
#
# The per-target build directories are several hundred MB and have filled
# this disk before, so they are deleted once the binaries are copied out
# unless --keep is given (keep them while iterating: rebuilds are then
# incremental).
set -euo pipefail
repo="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
keep=0
[ "${1:-}" = "--keep" ] && keep=1

"$repo/tools/cargo.sh" build --release --locked -p longmynd --target x86_64-unknown-linux-gnu
"$repo/tools/cargo.sh" build --release --locked -p longmynd --target x86_64-pc-windows-gnu

mkdir -p "$repo/dist"
install -m 0755 "$repo/target/x86_64-unknown-linux-gnu/release/longmynd" "$repo/dist/longmynd"
install -m 0644 "$repo/target/x86_64-pc-windows-gnu/release/longmynd.exe" "$repo/dist/longmynd.exe"

if [ "$keep" = 0 ]; then
    rm -rf "$repo/target/x86_64-unknown-linux-gnu" "$repo/target/x86_64-pc-windows-gnu"
fi
ls -l "$repo/dist"
