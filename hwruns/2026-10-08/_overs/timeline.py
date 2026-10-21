import sys, statistics, os
sys.path.insert(0, "tools/hwtest"); import compare as c
for r in sys.argv[1:]:
    st = c.status(r); bins = {}
    for t, n, v in st:
        b = int(t // 5) * 5
        d = bins.setdefault(b, {})
        d.setdefault(n, []).append(v)
    print(r)
    for b in sorted(bins):
        d = bins[b]
        mer = [int(x)/10 for x in d.get(12, [])]
        f = lambda n: statistics.median([int(x) for x in d[n]]) if d.get(n) else "-"
        print("  %3ds MER %-5s AGC1 %-6s AGC2 %-6s pow_i %-4s state %s" % (b, statistics.median(mer) if mer else "-", f(26), f(27), f(10), sorted(set(d.get(1, [])))))
    data = open(os.path.join(r, "ts.ts"), "rb").read()
    # CC error positions
    i, last, pos = 0, {}, []
    while i + 188 <= len(data):
        if data[i] != 0x47 or (i + 188 < len(data) and data[i + 188] != 0x47): i += 1; continue
        pid = (data[i+1] & 0x1F) << 8 | data[i+2]; cc, hp = data[i+3] & 15, data[i+3] & 0x10
        if pid != 0x1FFF and hp:
            if pid in last and cc != (last[pid]+1) & 15 and cc != last[pid]: pos.append((i, pid))
            last[pid] = cc
        i += 188
    print("  CC errors at byte offsets:", [(p, "%.0f%%" % (100*p/len(data)), pid) for p, pid in pos])
