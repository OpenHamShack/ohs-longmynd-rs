#!/usr/bin/env python3
"""Compare two tools/hwtest/run.sh runs (the C longmynd and longmynd-rs on
the same tuner, same signal, one after the other) - HANDOVER.md, hardware
test.

    tools/hwtest/compare.py <C run dir> <Rust run dir>

Reports, side by side: exit status; time to lock and share of time
locked; MER, symbol rate and carrier while locked; service names and
PIDs; TS bytes and continuity errors; and, when both runs have a USB
capture, whether the I2C register writes from start-up to the scan start
are the same, in the same order.

Also accepts a simulator trace.txt in place of usb.pcapng (equiv tests),
which is how the I2C decoding here was checked.
"""
import os
import statistics
import subprocess
import sys


def status(run):
    """[(t, n, value)] from status.txt."""
    out = []
    p = os.path.join(run, "status.txt")
    if not os.path.exists(p):
        return out
    for line in open(p, errors="replace"):
        try:
            t, rest = line.rstrip("\n").split(" ", 1)
            n, v = rest[1:].split(",", 1)
            out.append((float(t), int(n), v))
        except ValueError:
            pass
    return out


def summary(run):
    st = status(run)
    s = {"exit": open(os.path.join(run, "exit.txt")).read().strip()}
    locked_at = next((t for t, n, v in st if n == 1 and v in ("3", "4")), None)
    s["first lock (s)"] = "never" if locked_at is None else "%.1f" % locked_at
    states = [v for _, n, v in st if n == 1]
    s["share locked"] = "%.0f%%" % (100 * sum(v in ("3", "4") for v in states) / len(states)) if states else "-"
    # values reported while locked
    cur, mer, sr, car, names, pids = None, [], [], [], set(), set()
    for _, n, v in st:
        if n == 1:
            cur = v
        elif cur in ("3", "4"):
            if n == 12:
                mer.append(int(v) / 10)
            elif n == 9:
                sr.append(int(v))
            elif n == 6:
                car.append(int(v))
        if n in (13, 14) and v:
            names.add(v)
        if n == 16:
            pids.add(int(v))
    for k, xs, f in (("MER dB", mer, "%.1f"), ("SR S/s", sr, "%d"), ("carrier kHz", car, "%d")):
        s[k] = "-" if not xs else ("median " + f + " (" + f + ".." + f + ")") % (statistics.median(xs), min(xs), max(xs))
    s["names"] = ", ".join(sorted(names)) or "-"
    s["PIDs"] = ", ".join(str(p) for p in sorted(pids)) or "-"
    ts = os.path.join(run, "ts.ts")
    data = open(ts, "rb").read() if os.path.exists(ts) else b""
    s["TS bytes"] = str(len(data))
    s["TS CC errors"] = str(cc_errors(data))
    return s


def cc_errors(data):
    """Continuity-counter jumps, after re-syncing on 0x47 at 188 spacing."""
    i, last, errs = 0, {}, 0
    while i + 188 <= len(data):
        if data[i] != 0x47 or (i + 188 < len(data) and data[i + 188] != 0x47):
            i += 1
            continue
        pid = (data[i + 1] & 0x1F) << 8 | data[i + 2]
        cc, has_payload = data[i + 3] & 0x0F, data[i + 3] & 0x10
        if pid != 0x1FFF and has_payload:
            if pid in last and cc != (last[pid] + 1) & 0x0F and cc != last[pid]:
                errs += 1
            last[pid] = cc
        i += 188
    return errs


def out_payloads(run):
    """The bulk OUT payloads to endpoint 0x02, in order."""
    pcap = os.path.join(run, "usb.pcapng")
    trace = os.path.join(run, "trace.txt")
    if os.path.exists(pcap):
        flt = "usb.endpoint_address == 0x02 && usb.urb_type == 83"
        where = os.path.join(run, "usb.txt")  # "<bus> <device>" from run.sh
        if os.path.exists(where):
            bus, dev = open(where).read().split()
            flt += " && usb.bus_id == %s && usb.device_address == %s" % (bus, dev)
        # Wireshark 4.x dissects FTDI traffic itself (ftdi-ft) and then
        # leaves usb.capdata empty, which decodes to no I2C at all and an
        # empty-equals-empty "identical". Turn that dissector off.
        r = subprocess.run(
            ["tshark", "--disable-protocol", "ftdi-ft", "-r", pcap, "-Y", flt,
             "-T", "fields", "-e", "usb.capdata"],
            capture_output=True, text=True)
        return [bytes.fromhex(l.replace(":", "")) for l in r.stdout.split() if l]
    if os.path.exists(trace):
        out = []
        for l in open(trace):
            if " bulk 02 out " in l:
                hexpart = l.split("] ", 1)[1].split(" -> ")[0]
                out.append(bytes.fromhex(hexpart.replace(" ", "")))
        return out
    return None


def i2c_transactions(payloads):
    """Decode MPSSE back into I2C transactions (bytes written; reads shown
    as '<'), as the simulator does: START/STOP are SDA edges with SCL high
    on 0x80 commands, 0x11 clocks bytes out."""
    out, cur = [], None
    scl = sda = True
    stream = b"".join(payloads)
    i = 0
    lens = {0x80: 3, 0x82: 3, 0x86: 3, 0x25: 3, 0x24: 3, 0x20: 3, 0x27: 2, 0x26: 2, 0x22: 2}
    while i < len(stream):
        c = stream[i]
        if c in (0x11, 0x10):
            n = stream[i + 1] | stream[i + 2] << 8
            data = stream[i + 3:i + 4 + n]
            if cur is not None:
                cur.extend("%02x" % b for b in data)
            i += 4 + n
            continue
        if c == 0x80:
            val, d = stream[i + 1], stream[i + 2]
            nscl, nsda = bool(val & 1), (bool(val & 2) if d & 2 else True)
            if scl and nscl and sda and not nsda:
                cur = (cur + ["|"]) if cur is not None else []
            elif scl and nscl and not sda and nsda and cur is not None:
                out.append(" ".join(cur))
                cur = None
            scl, sda = nscl, nsda
        elif c in (0x25, 0x24, 0x20) and cur is not None:
            cur.append("<")
        i += lens.get(c, 1)
    return out


def collapse_retries(t):
    """A NAKed address phase is re-sent after a repeated START with no STOP,
    so one transaction reads "d2 f1 b8 | d2 f1 b8 | d3 <". How many NAKs a
    chip gives depends on how long the USB round trips take (the STV0910
    NAKs while its PLL settles after the MCLK change: 3 for the C, 4 for
    the Rust on the first real-hardware run), so repeats are timing, not a
    difference in what is written. Collapse them like polls."""
    segs = []
    for seg in t.split(" | "):
        if not segs or segs[-1] != seg:
            segs.append(seg)
    return " | ".join(segs)


def until_scan_collapsed(tx):
    end = next((k for k, t in enumerate(tx) if t == "d2 f2 16 15"), len(tx) - 1)
    out = []
    for t in tx[:end + 1]:
        t = collapse_retries(t)
        # A NAK in a read's second phase (address|1, read, STOP) retries
        # just that phase, as a transaction of its own: "d3 <" straight
        # after "d2 f5 36 | d3 <". Seen once after the NIM reset, in one
        # run and not the other. Timing again, so drop the repeat.
        if out and "|" not in t and out[-1].endswith(" | " + t):
            continue
        if not out or out[-1] != t:  # polling loops differ in length with timing
            out.append(t)
    return out


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    a, b = sys.argv[1], sys.argv[2]
    sa, sb = summary(a), summary(b)
    w = max(len(k) for k in sa)
    print("%-*s  %-40s  %s" % (w, "", "C", "longmynd-rs"))
    for k in sa:
        mark = "" if sa[k] == sb[k] else "   <-- differs"
        print("%-*s  %-40s  %s%s" % (w, k, sa[k][:40], sb[k][:40], mark))
    pa, pb = out_payloads(a), out_payloads(b)
    if pa is None or pb is None:
        print("\nno USB capture in one or both runs: I2C comparison skipped")
        return
    ta, tb = until_scan_collapsed(i2c_transactions(pa)), until_scan_collapsed(i2c_transactions(pb))
    if not ta or not tb:
        print("\nI2C comparison FAILED: no I2C decoded from %s (capture or tshark problem)"
              % " and ".join(n for n, t in (("C", ta), ("Rust", tb)) if not t))
        return
    if ta == tb:
        print("\nI2C from start-up to scan start: identical (%d transactions after collapsing polls)" % len(ta))
    else:
        k = next((k for k, (x, y) in enumerate(zip(ta, tb)) if x != y), min(len(ta), len(tb)))
        print("\nI2C from start-up to scan start DIFFERS at transaction %d:" % k)
        print("  C:    %s" % (ta[k] if k < len(ta) else "(end)"))
        print("  Rust: %s" % (tb[k] if k < len(tb) else "(end)"))


if __name__ == "__main__":
    main()
