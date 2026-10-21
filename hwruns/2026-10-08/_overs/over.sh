#!/usr/bin/env bash
# One over through the OpenHamShack DATV app, with one longmynd run inside it.
#   over.sh <tag> <binary> <rundir> <longmynd args...>
# Terms: 1255.000 MHz, 333 kS QPSK 2/3, -30 dB, <=55 s held (app time-out 60).
set -u
tag="$1"; bin="$2"; run="$3"; shift 3
cd ~/lmtest
LOG=~/ohsdatv-test/over.jsonl
lo() { iio_attr -u ip:192.168.2.1 -c ad9361-phy altvoltage1 powerdown 2>&1; }
cfg=$(python3 -c 'import json;c=json.load(open("/home/user/.config/openhamshack-datv/config.json"));print(c["rxDevice"],c["txGain"],c["txFreq"],c["txSr"],c["constellation"],c["fec"],c["txTimeoutSec"])')
[ "$cfg" = "sim -30.0 1255000000.0 333000 qpsk 2/3 60.0" ] || { echo "ABORT config: $cfg"; exit 3; }
pgrep -f '^python3 -m ohsdatv$' && { echo "ABORT app already running"; exit 3; }
[ "$(lo)" = 1 ] || { echo "ABORT LO before: $(lo)"; exit 3; }
[ -e "$LOG" ] && { echo "ABORT over.jsonl exists"; exit 3; }
echo "LO before: $(lo)"
(cd ~/ohsdatv-test && PYTHONPATH=/home/user/ohsdatv-test/drv:/home/user/ohsdatv-test LBDRV_OVER=55 LBDRV_DELAY=12 XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 QT_QPA_PLATFORM=wayland setsid nohup python3 -m ohsdatv > app.log 2>&1 < /dev/null &)
# wait for keyed (max 60 s)
for i in $(seq 120); do
  if [ -e "$LOG" ] && grep -q '"tag": "refused"' "$LOG"; then echo "REFUSED"; grep refused "$LOG"; break; fi
  if [ -e "$LOG" ] && grep '"tag": "air"' "$LOG" | grep -q '"keyed": true'; then break; fi
  sleep 0.5
done
if ! grep '"tag": "air"' "$LOG" 2>/dev/null | grep -q '"keyed": true'; then
  echo "NOT KEYED - stopping app"; pkill -TERM -f '^python3 -m ohsdatv$'; sleep 5; echo "LO: $(lo)"; mv "$LOG" ~/ohsdatv-test/over-lmrs-$tag.jsonl 2>/dev/null; exit 4
fi
echo "keyed at $(grep '"tag": "air"' "$LOG" | grep '"keyed": true' | head -1 | python3 -c 'import json,sys;print(json.loads(sys.stdin.readline())["utc"])') UTC"
sleep 3
if [ "$bin" != none ]; then sudo -n tools/hwtest/run.sh "$bin" "$run" 40 "$@"; fi
# wait for PTT off and the post snaps
for i in $(seq 60); do grep -q '"tag": "PTT off"' "$LOG" && break; sleep 1; done
sleep 6
pkill -TERM -f '^python3 -m ohsdatv$'
for i in $(seq 30); do pgrep -f '^python3 -m ohsdatv$' >/dev/null || break; sleep 1; done
pgrep -af '^python3 -m ohsdatv$' && echo "APP STILL RUNNING"
python3 - "$LOG" <<'PY'
import json,sys
rows=[json.loads(l) for l in open(sys.argv[1])]
on=[r for r in rows if r.get("tag")=="PTT on"]; off=[r for r in rows if r.get("tag")=="PTT off"]
k=[r for r in rows if r.get("keyed")]
print("PTT on", on[0]["utc"] if on else None, "| PTT off", off[0]["utc"] if off else None,
      "| keyed first", k[0]["utc"] if k else None, "last", k[-1]["utc"] if k else None,
      "| txSeconds", max((r.get("txSeconds") or 0) for r in rows),
      "| gain", sorted({r.get("txGain") for r in rows if "txGain" in r}),
      "| freq", sorted({r.get("txFreq") for r in rows if "txFreq" in r}),
      "| txVerified", sorted({str(r.get("txVerified")) for r in k}),
      "| err", sorted({r.get("error") for r in rows if r.get("error")}), sorted({r.get("txStuck") for r in rows if r.get("txStuck")}))
PY
mv "$LOG" ~/ohsdatv-test/over-lmrs-$tag.jsonl
sleep 2
echo "LO after: $(lo)"
