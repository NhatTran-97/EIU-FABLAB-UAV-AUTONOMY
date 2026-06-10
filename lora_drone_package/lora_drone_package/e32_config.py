#!/usr/bin/env python3
"""
EBYTE E32-433T20D (SX1278) configuration helper.

Reads / sets the module's air data rate over UART while it is in
CONFIGURATION mode (M0=HIGH, M1=HIGH). In config mode the UART is ALWAYS
9600 8N1 regardless of the operating baud.

USAGE
  Read current config (safe, no write):
      python3 e32_config.py --port /dev/lora_ground

  Set air rate to 19.2k (max) and save permanently:
      python3 e32_config.py --port /dev/lora_ground --air 19.2k

IMPORTANT
  * Put the module in CONFIG mode first: M0=3V3 and M1=3V3 (both HIGH).
    M0/M1 are 3.3 V logic inputs — do NOT drive them with 5 V.
  * Configure BOTH modules (drone + ground) to the SAME air rate and the
    SAME channel, or they stop talking to each other.
  * After configuring, return M0=M1=LOW (GND) for normal operation.
"""

import argparse
import time
import serial

# --- SPED byte fields (bit7-6 parity | bit5-3 UART baud | bit2-0 air rate) ---
UART_BAUD = {0: "1200", 1: "2400", 2: "4800", 3: "9600",
             4: "19200", 5: "38400", 6: "57600", 7: "115200"}
PARITY    = {0: "8N1", 1: "8O1", 2: "8E1", 3: "8N1"}
AIR_RATE  = {0: "0.3k", 1: "1.2k", 2: "2.4k", 3: "4.8k",
             4: "9.6k", 5: "19.2k", 6: "19.2k", 7: "19.2k"}
AIR_CODE  = {"0.3k": 0, "1.2k": 1, "2.4k": 2, "4.8k": 3, "9.6k": 4, "19.2k": 5}
# OPTION byte: bit7 fixed/transparent | bit2 FEC | bit1-0 TX power
TXPOWER   = {0: "20dBm", 1: "17dBm", 2: "14dBm", 3: "10dBm"}


def decode(cfg: bytes) -> dict:
    head, addh, addl, sped, chan, option = cfg[:6]
    parity = (sped >> 6) & 0b11
    baud   = (sped >> 3) & 0b111
    air    = sped & 0b111
    print(f"  header   : 0x{head:02X} ({'saved' if head == 0xC0 else 'temp' if head == 0xC2 else '??'})")
    print(f"  address  : 0x{addh:02X}{addl:02X}")
    print(f"  channel  : {chan} (0x{chan:02X})  ->  {410 + chan} MHz")
    print(f"  UART     : {UART_BAUD[baud]} {PARITY[parity]}")
    print(f"  air rate : {AIR_RATE[air]}   <== throughput knob")
    print(f"  TX power : {TXPOWER[option & 0b11]}   "
          f"FEC: {'on' if (option >> 2) & 1 else 'off'}   "
          f"mode: {'fixed' if (option >> 7) & 1 else 'transparent'}")
    return dict(addh=addh, addl=addl, sped=sped, chan=chan, option=option, air=air)


def read_config(ser: serial.Serial) -> bytes:
    ser.reset_input_buffer()
    ser.write(bytes([0xC1, 0xC1, 0xC1]))   # read operating parameters
    ser.flush()
    time.sleep(0.2)
    resp = ser.read(6)
    if len(resp) != 6 or resp[0] not in (0xC0, 0xC2):
        raise RuntimeError(
            f"no valid config response (got {resp!r}). "
            "Is the module in CONFIG mode?  M0=HIGH, M1=HIGH (3.3 V)."
        )
    return resp


def write_config(ser: serial.Serial, addh, addl, sped, chan, option) -> bytes:
    cmd = bytes([0xC0, addh, addl, sped, chan, option])   # C0 = save permanently
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.3)
    resp = ser.read(6)
    if len(resp) != 6 or resp[0] != 0xC0:
        raise RuntimeError(f"write not confirmed (got {resp!r})")
    return resp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/lora_ground")
    ap.add_argument("--air", choices=sorted(AIR_CODE),
                    help="target air rate to SET (omit = read only)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 9600, bytesize=8, parity="N", stopbits=1, timeout=1.0)
    try:
        print(f"== current config on {args.port} ==")
        info = decode(read_config(ser))

        if not args.air:
            print("\n(read-only; add  --air 19.2k  to change)")
            return

        target = AIR_CODE[args.air]
        if target == info["air"]:
            print(f"\nair rate already {args.air}; nothing to do.")
            return

        new_sped = (info["sped"] & 0b11111000) | target   # keep parity+UART, change air bits only
        print(f"\n== writing air rate -> {args.air}  (SPED 0x{info['sped']:02X} -> 0x{new_sped:02X}) ==")
        write_config(ser, info["addh"], info["addl"], new_sped, info["chan"], info["option"])

        print("== verify (read back) ==")
        d = decode(read_config(ser))
        if AIR_RATE[d["air"]] == args.air:
            print(f"\n✅ air rate is now {args.air}. Set M0=M1=LOW (GND) and power-cycle.")
            print("   Configure the OTHER module to the SAME air rate + channel.")
        else:
            print("\n⚠️ readback mismatch — change not applied.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
