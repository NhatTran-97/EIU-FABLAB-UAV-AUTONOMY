#!/usr/bin/env python3
"""Monitor du lieu drone_telemetry phia ground (khong can ROS).

    python3 telemetry_monitor.py --port /dev/ttyUSB0      # cong cua module Ground

Cho cong xuat hien neu module Ground chua cam, va tu mo lai khi bi rut giua chung.
Node ground that su co the dung lai dung cach parse o day: drone_dialect.MAVLink.parse_buffer().
"""

import argparse
import os
import sys
import time

import serial
from serial.tools import list_ports

try:
    from drone_telemetry import drone_dialect as mavlink
except ImportError:   # chay tu source
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from drone_telemetry import drone_dialect as mavlink

NAMES = {mavlink.DRONE_COMPONENT_LIDAR: 'Lidar', mavlink.DRONE_COMPONENT_GIMBAL: 'Gimbal',
         mavlink.DRONE_COMPONENT_CAMERA: 'Camera', mavlink.DRONE_COMPONENT_PX4: 'PX4'}
LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
STALE_S = 3.0


def open_port(port, baud):
    warned = False
    while True:
        try:
            return serial.Serial(port, baud, timeout=0.1)
        except (serial.SerialException, OSError) as e:
            if not warned:
                warned = True
                print(f'Khong mo duoc {port}: {e}')
                usb = [p.device for p in list_ports.comports()
                       if 'USB' in p.device or 'ACM' in p.device]
                print('Cong USB serial dang co: '
                      + (', '.join(usb) if usb else 'khong co (module Ground chua cam?)'))
                print('Dang cho cong xuat hien... (Ctrl+C de thoat)')
            time.sleep(1.0)


def show(now, comps, heartbeat_at, radio):
    print('\033[2J\033[H', end='')   # xoa man hinh
    hb = f'{now - heartbeat_at:.1f}s truoc' if heartbeat_at else 'chua nhan'
    print(f'Heartbeat drone: {hb}')
    if radio:
        print(f'Radio: RSSI {radio.rssi}/{radio.remrssi}  noise {radio.noise}/{radio.remnoise}'
              f'  rxerrors {radio.rxerrors}')
    print(f"\n{'Thiet bi':<8} {'Level':<6} {'State':<16} {'Hz':>6}  Cap nhat")
    for cid in sorted(comps):
        t, m = comps[cid]
        age = now - t
        level = 'STALE' if age > STALE_S else LEVELS.get(m.level, m.level)
        print(f"{NAMES.get(cid, cid):<8} {level:<6} {m.state:<16} {m.rate_hz:>6.1f}  {age:.1f}s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, default=57600)
    args = ap.parse_args()

    parser = mavlink.MAVLink(None)
    parser.robust_parsing = True
    comps, heartbeat_at, radio = {}, None, None
    next_print = 0.0

    while True:
        ser = open_port(args.port, args.baud)
        try:
            while True:
                data = ser.read(ser.in_waiting or 1)
                now = time.monotonic()
                for m in (parser.parse_buffer(data) or []) if data else []:
                    t = m.get_type()
                    if t == 'DRONE_COMPONENT_STATUS':
                        comps[m.component_id] = (now, m)
                    elif t == 'HEARTBEAT':
                        heartbeat_at = now
                    elif t == 'RADIO_STATUS':
                        radio = m
                if now >= next_print:
                    next_print = now + 1.0
                    show(now, comps, heartbeat_at, radio)
        except (serial.SerialException, OSError) as e:
            print(f'\nMat ket noi {args.port}: {e}. Cho cam lai...')
        finally:
            ser.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
