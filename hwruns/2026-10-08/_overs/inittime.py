# Time from the first MPSSE OUT to the OUT carrying the scan start (d2 f2 16 15) - the real start-up cost.
import sys, subprocess, os
sys.path.insert(0, "tools/hwtest"); import compare as c
for r in sys.argv[1:]:
    bus, dev = open(os.path.join(r, "usb.txt")).read().split()
    out = subprocess.run(["tshark", "--disable-protocol", "ftdi-ft", "-r", os.path.join(r, "usb.pcapng"), "-Y",
        "usb.endpoint_address == 0x02 && usb.urb_type == 83 && usb.device_address == %s" % dev,
        "-T", "fields", "-e", "frame.time_relative", "-e", "usb.capdata"], capture_output=True, text=True).stdout
    rows = [(float(l.split()[0]), bytes.fromhex(l.split()[1])) for l in out.splitlines() if len(l.split()) == 2]
    # find the payload index where the scan-start transaction completes, by decoding cumulatively
    t0 = rows[0][0]; stream = []; tscan = None
    for k, (t, p) in enumerate(rows):
        stream.append(p)
        if k % 50 == 0 or True:
            pass
    # decode once, then map transaction count to payload by incremental decode on prefixes (binary search)
    lo, hi = 0, len(rows)
    def has(n): return "d2 f2 16 15" in c.i2c_transactions([p for _, p in rows[:n]])
    while lo < hi:
        mid = (lo + hi) // 2
        if has(mid): hi = mid
        else: lo = mid + 1
    print("%-14s start-up (first OUT -> scan start): %.3f s, %d OUT transfers" % (os.path.basename(r), rows[lo - 1][0] - t0, lo))
