#!/usr/bin/env bash
# One receive-only run of a longmynd binary against a real tuner, for the
# side-by-side test in HANDOVER.md. Run it once with the C binary and once
# with longmynd-rs, same arguments, same signal, then compare.py.
#
#   sudo tools/hwtest/run.sh <longmynd binary> <outdir> <seconds> <longmynd args...>
#
# Records in <outdir>: stdout.txt, exit.txt, status.txt (each status line
# with the time it was read), ts.ts (the transport stream), usb.pcapng (the
# USB traffic, if dumpcap is installed: apt install tshark). TS and status
# go to FIFOs here; do not pass -t/-s/-i/-I. Never pass -p unless the
# input really is an LNB: it puts 13/18 V on the feed.
set -euo pipefail
[ $# -ge 4 ] || { sed -n '2,13p' "$0"; exit 2; }
bin="$(readlink -f "$1")"; out="$2"; secs="$3"; shift 3
mkdir -p "$out"; out="$(readlink -f "$out")"
rm -f "$out"/ts.fifo "$out"/status.fifo
mkfifo "$out/ts.fifo" "$out/status.fifo"

# which USB bus the tuner is on, for the capture, and its address on that
# bus: usbmon captures the whole bus, and anything else on it (a Pluto's
# USB Ethernet, say) also has an endpoint 0x02. compare.py reads usb.txt to
# keep only the tuner's traffic.
bus=""; dev=""
for id in 0403:6010 2e8a:ba2c; do
    l="$(lsusb -d "$id" | head -1)"
    [ -n "$l" ] && bus="$(echo "$l" | awk '{print $2+0}')" && dev="$(echo "$l" | awk '{print $4+0}')" && break
done
[ -n "$bus" ] || { echo "no MiniTiouner or PicoTuner on USB"; exit 1; }
echo "$bus $dev" > "$out/usb.txt"

cap=""
if command -v dumpcap >/dev/null; then
    modprobe usbmon 2>/dev/null || true
    # dumpcap gives up root's file privileges once usbmon is open, so an
    # absolute path through a 0700 home is refused ("Permission denied")
    # and the run silently has no capture. From inside $out the name is
    # relative and it works. Its stderr is kept so a failure shows.
    (cd "$out" && exec dumpcap -q -i "usbmon$bus" -w usb.pcapng 2>dumpcap.log) &
    cap=$!
    sleep 1
fi

cat "$out/ts.fifo" > "$out/ts.ts" &
tsr=$!
python3 -u -c '
import sys, time
t0 = time.monotonic()
with open(sys.argv[1]) as f, open(sys.argv[2], "w") as o:
    for line in f:
        o.write("%.3f %s" % (time.monotonic() - t0, line))
' "$out/status.fifo" "$out/status.txt" &
str=$!

set +e
timeout --preserve-status -s INT "$secs" "$bin" -t "$out/ts.fifo" -s "$out/status.fifo" "$@" > "$out/stdout.txt" 2>&1
echo $? > "$out/exit.txt"
set -e

sleep 1
kill "$tsr" "$str" 2>/dev/null || true
[ -n "$cap" ] && kill "$cap" 2>/dev/null || true
wait 2>/dev/null || true
rm -f "$out/ts.fifo" "$out/status.fifo"
echo "$(basename "$bin"): exit $(cat "$out/exit.txt"), $(wc -l < "$out/status.txt") status lines, $(stat -c %s "$out/ts.ts") TS bytes -> $out"
