#!/usr/bin/env python3
"""
LoRa link checker — sniff the ground LoRa serial and report telemetry health.

Opens the port directly (so the GUI must be CLOSED — it holds the port
exclusively) and decodes incoming telemetry with the SAME helpers the ground
app uses (control.py: _clean_json_str -> _unwrap_and_verify_crc), so what you
see here is exactly what main.py would parse.

USAGE
    # close main.py first, keep the drone node running, then:
    python3 lora_link_check.py                       # /dev/lora_ground, 14 s
    python3 lora_link_check.py --secs 20
    python3 lora_link_check.py --port /dev/lora_ground --secs 30

REPORT
    goodput, CRC status, valid/malformed counts, heartbeat rate + cadence
    (avg/min/max gap), loss rate, and a few sample decoded packets.
"""


"""
# Tắt main.py trước (nó độc chiếm cổng), giữ drone node chạy, rồi:
cd ~/ground_gui
python3 lora_link_check.py                       # /dev/lora_ground, 14s
python3 lora_link_check.py --secs 20             # đo lâu hơn
python3 lora_link_check.py --port /dev/lora_ground --secs 30

"""

import argparse
import collections
import json
import time

import serial

from control import _clean_json_str, _unwrap_and_verify_crc, _as_true


def check(port: str, secs: float, link_capacity: int = 500):
    try:
        ser = serial.Serial(port, 9600, timeout=0.2)
    except Exception as e:
        print(f"⚠️  Cannot open {port}: {e}")
        print("    Is main.py still running? It holds the port exclusively — close it first.")
        return

    print(f"Listening on {port} @9600 for ~{secs:.0f}s "
          f"(ground parse path)...\n")

    t0 = time.time()
    buf = ""
    raw_bytes = 0
    status = collections.Counter()
    valid = malformed = hb = 0
    hb_times = []
    samples = []

    try:
        while time.time() - t0 < secs:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            raw_bytes += len(chunk)
            buf += chunk.decode("utf-8", errors="replace")
            while "\n" in buf:
                line, _, buf = buf.partition("\n")
                line = line.strip()
                if not line:
                    continue
                clean = _clean_json_str(line)
                if not clean:
                    malformed += 1
                    continue
                try:
                    raw = json.loads(clean)
                except Exception:
                    malformed += 1
                    continue
                data, st = _unwrap_and_verify_crc(raw)
                status[st] += 1
                if st == "bad_crc" or data is None:
                    continue
                valid += 1
                if _as_true(data.get("hb", 0)):
                    hb += 1
                    hb_times.append(time.time())
                if len(samples) < 4:
                    samples.append({k: data[k] for k in
                                    ("hb", "state", "gps_ok", "x", "y", "z", "speed", "battery")
                                    if k in data})
    finally:
        ser.close()

    dur = time.time() - t0
    print(f"goodput      : {raw_bytes} bytes ({raw_bytes / dur:.0f} B/s)"
          f"   [≈{100 * (raw_bytes / dur) / link_capacity:.0f}% of ~{link_capacity} B/s]")
    print(f"CRC status   : {dict(status)}")
    print(f"valid packets: {valid}    malformed/truncated: {malformed}")
    print(f"heartbeats   : {hb}  (~{hb / dur:.1f} Hz)")
    if len(hb_times) >= 2:
        gaps = [hb_times[i + 1] - hb_times[i] for i in range(len(hb_times) - 1)]
        print(f"hb cadence   : avg {sum(gaps) / len(gaps):.2f}s  "
              f"min {min(gaps):.2f}  max {max(gaps):.2f}")
    total = valid + malformed
    loss = (malformed / total * 100) if total else 0.0
    print(f"loss rate    : {loss:.0f}%")
    print("\nsample decoded packets:")
    for sd in samples:
        print("  ", sd)

    healthy = valid >= 2 and hb >= 2 and status.get("bad_crc", 0) == 0 and loss < 5
    print("\n" + ("✅ LINK HEALTHY — telemetry flows & parses cleanly"
                  if healthy else
                  "⚠️  check: low valid count / high loss / bad CRC"))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/lora_ground", help="serial port (default: /dev/lora_ground)")
    ap.add_argument("--secs", type=float, default=14.0, help="listen duration in seconds (default: 14)")
    ap.add_argument("--capacity", type=int, default=500,
                    help="link goodput ceiling for the %% figure (default: 500 = 19.2k air-rate)")
    args = ap.parse_args()
    check(args.port, args.secs, args.capacity)


if __name__ == "__main__":
    main()
