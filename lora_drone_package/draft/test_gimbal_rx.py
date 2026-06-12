#!/usr/bin/env python3
"""
test_gimbal_rx.py  —  SX1278 LoRa edition
──────────────────────────────────────────
Nhận packet lệnh từ UI (TX), gửi ngược feedback giả lập (RX).
Thiết kế cho SX1278 half-duplex: đọc trước, gửi feedback khi kênh rảnh.

TX packet (UI → gimbal): 32 bytes
  [0-1]   0xAA 0x55
  [2]     payload length = 29
  [3]     control_mode   0=MANUAL 1=AUTO
  [4]     gimbal_mode    0=PTZ 1=POSITION 2=VELOCITY
  [5]     ptz_cmd        0=STOP 1=UP 2=DOWN 3=CENTER 4=FOLLOW 5=LOCK
  [6-9]   pitch_deg      float32 LE
  [10-13] yaw_deg        float32 LE
  [14-17] pitch_speed    float32 LE
  [18-21] yaw_speed      float32 LE
  [22]    enable_flags   bit0=pitch  bit1=yaw
  [23-26] pitch_vel      float32 LE
  [27-30] yaw_vel        float32 LE
  [31]    CRC-8

RX packet (gimbal → UI): 18 bytes
  [0-1]   0xBB 0x66
  [2]     payload length = 14
  [3]     control_mode
  [4-7]   yaw_deg        float32 LE
  [8-11]  pitch_deg      float32 LE
  [12-15] roll_deg       float32 LE
  [16]    is_connected   1=online
  [17]    CRC-8
"""

import serial
import struct
import time
import threading
import argparse
from typing import Optional

# ── Mặc định ──────────────────────────────────────────────────────
PORT            = '/dev/ttyUSB0'
BAUD            = 9600       # baud UART ↔ SX1278 module (thường 9600 với E32)
FB_INTERVAL     = 1.0        # giây giữa 2 lần gửi feedback (LoRa-friendly)
TX_LEN          = 32
RX_LEN          = 18
TX_STX          = b'\xAA\x55'

GIMBAL_MODE = {0: 'PTZ', 1: 'POSITION', 2: 'VELOCITY'}
CTRL_MODE   = {0: 'MANUAL', 1: 'AUTO'}
PTZ_CMD     = {0: 'STOP', 1: 'UP', 2: 'DOWN', 3: 'CENTER', 4: 'FOLLOW', 5: 'LOCK'}


# ── CRC-8 (polynomial 0x07) ───────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


# ── Parse TX packet ───────────────────────────────────────────────
def parse_tx(pkt: bytes) -> Optional[dict]:
    if len(pkt) != TX_LEN:
        return None
    if pkt[0] != 0xAA or pkt[1] != 0x55:
        return None

    expected = crc8(pkt[2:TX_LEN - 1])
    if expected != pkt[TX_LEN - 1]:
        print(f"  [CRC ERR] expect={expected:02X} got={pkt[TX_LEN-1]:02X}")
        return None

    pitch_deg, = struct.unpack_from('<f', pkt, 6)
    yaw_deg,   = struct.unpack_from('<f', pkt, 10)
    pitch_spd, = struct.unpack_from('<f', pkt, 14)
    yaw_spd,   = struct.unpack_from('<f', pkt, 18)
    pitch_vel, = struct.unpack_from('<f', pkt, 23)
    yaw_vel,   = struct.unpack_from('<f', pkt, 27)

    return {
        'ctrl':      CTRL_MODE.get(pkt[3],  f'?{pkt[3]}'),
        'mode':      GIMBAL_MODE.get(pkt[4], f'?{pkt[4]}'),
        'ptz':       PTZ_CMD.get(pkt[5],    f'?{pkt[5]}'),
        'pitch_deg': pitch_deg,
        'yaw_deg':   yaw_deg,
        'pitch_spd': pitch_spd,
        'yaw_spd':   yaw_spd,
        'en_pitch':  bool(pkt[22] & 0x01),
        'en_yaw':    bool(pkt[22] & 0x02),
        'pitch_vel': pitch_vel,
        'yaw_vel':   yaw_vel,
    }


# ── Build RX feedback packet ──────────────────────────────────────
def build_rx(yaw: float, pitch: float, roll: float,
             ctrl_mode: int = 0) -> bytes:
    payload  = struct.pack('<Bfff', ctrl_mode, yaw, pitch, roll)
    payload += struct.pack('B', 1)           # is_connected = True
    pkt      = bytes([0xBB, 0x66, len(payload)]) + payload
    pkt     += bytes([crc8(pkt[2:])])
    assert len(pkt) == RX_LEN
    return pkt


# ── In lệnh nhận được ─────────────────────────────────────────────
def print_cmd(d: dict, idx: int):
    mode = d['mode']
    ctrl = d['ctrl']

    if mode == 'PTZ':
        detail = f"ptz={d['ptz']}"
    elif mode == 'POSITION':
        detail = (f"pitch={d['pitch_deg']:+7.2f}°  yaw={d['yaw_deg']:+7.2f}°  "
                  f"spd=({d['pitch_spd']:.0f},{d['yaw_spd']:.0f})  "
                  f"en=({'P' if d['en_pitch'] else '-'}{'Y' if d['en_yaw'] else '-'})")
    else:
        detail = (f"vel_pitch={d['pitch_vel']:+6.1f}°/s  "
                  f"vel_yaw={d['yaw_vel']:+6.1f}°/s")

    ts = time.strftime('%H:%M:%S')
    print(f"[{ts}][#{idx:04d}][{ctrl:6s}|{mode:8s}] {detail}")


# ── Shared state (thread-safe) ────────────────────────────────────
class GimbalState:
    def __init__(self):
        self._lock   = threading.Lock()
        self.yaw     = 0.0
        self.pitch   = 0.0
        self.roll    = 0.0
        self.ctrl    = 0

    def update(self, yaw: float, pitch: float, roll: float = 0.0, ctrl: int = 0):
        with self._lock:
            self.yaw   = yaw
            self.pitch = pitch
            self.roll  = roll
            self.ctrl  = ctrl

    def get_rx_packet(self) -> bytes:
        with self._lock:
            return build_rx(self.yaw, self.pitch, self.roll, self.ctrl)


# ── Feedback thread — gửi khi kênh rảnh (half-duplex safe) ───────
class FeedbackSender(threading.Thread):
    def __init__(self, ser: serial.Serial, state: GimbalState,
                 interval: float, serial_lock: threading.Lock):
        super().__init__(daemon=True)
        self.ser         = ser
        self.state       = state
        self.interval    = interval
        self.serial_lock = serial_lock  # dùng chung lock với main reader
        self._stop       = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            time.sleep(self.interval)
            # Thử lấy lock — nếu main thread đang đọc thì bỏ qua lần này
            acquired = self.serial_lock.acquire(blocking=False)
            if not acquired:
                continue
            try:
                pkt = self.state.get_rx_packet()
                self.ser.write(pkt)
                # Chờ UART flush hết trước khi main thread có thể đọc
                # (SX1278 half-duplex: sau TX cần vài ms để switch về RX)
                self.ser.flush()
                time.sleep(0.02)   # 20ms guard time
            except Exception as e:
                print(f"[FB ERR] {e}")
            finally:
                self.serial_lock.release()


# ── Main ──────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Gimbal packet monitor — SX1278 LoRa')
    parser.add_argument('--port',        default=PORT,        help='Serial port')
    parser.add_argument('--baud',        default=BAUD,        type=int)
    parser.add_argument('--fb-interval', default=FB_INTERVAL, type=float,
                        help='Giây giữa 2 lần gửi feedback (default 1.0s)')
    parser.add_argument('--no-feedback', action='store_true',
                        help='Chỉ đọc, không gửi feedback')
    args = parser.parse_args()

    # Ước tính airtime SX1278 (SF7/BW125 transparent module)
    bytes_per_sec = args.baud / 10          # 8N1
    tx_air_ms     = TX_LEN / bytes_per_sec * 1000
    rx_air_ms     = RX_LEN / bytes_per_sec * 1000
    print(f"Port  : {args.port} @ {args.baud} baud")
    print(f"TX pkt: {TX_LEN} bytes  ≈ {tx_air_ms:.1f} ms UART")
    print(f"RX pkt: {RX_LEN} bytes  ≈ {rx_air_ms:.1f} ms UART")
    print(f"FB    : {'OFF' if args.no_feedback else f'mỗi {args.fb_interval}s'}")
    print()

    try:
        ser = serial.Serial(
            args.port, args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05       # 50ms read timeout
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"Lỗi mở port: {e}")
        return

    state       = GimbalState()
    serial_lock = threading.Lock()
    fb_thread   = None

    if not args.no_feedback:
        fb_thread = FeedbackSender(ser, state, args.fb_interval, serial_lock)
        fb_thread.start()
        print(f"Feedback thread ON — mỗi {args.fb_interval}s\n")

    print("Đang lắng nghe... (Ctrl+C để thoát)\n")
    buf       = b''
    pkt_count = 0
    err_count = 0

    try:
        while True:
            # Lấy lock trước khi đọc (không tranh với feedback TX)
            with serial_lock:
                chunk = ser.read(ser.in_waiting or 1)

            if not chunk:
                continue
            buf += chunk

            while len(buf) >= TX_LEN:
                idx = buf.find(TX_STX)
                if idx < 0:
                    buf = b''
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < TX_LEN:
                    break

                result = parse_tx(buf[:TX_LEN])
                if result is not None:
                    pkt_count += 1
                    print_cmd(result, pkt_count)
                    buf = buf[TX_LEN:]

                    # Cập nhật state để feedback echo lại (thread-safe)
                    if result['mode'] == 'POSITION':
                        state.update(result['yaw_deg'], result['pitch_deg'])
                    elif result['mode'] == 'VELOCITY':
                        state.update(state.yaw, state.pitch)  # giữ vị trí cũ
                else:
                    err_count += 1
                    buf = buf[1:]

    except KeyboardInterrupt:
        print(f"\nDừng.")
        print(f"  Packets OK : {pkt_count}")
        print(f"  Lỗi parse : {err_count}")
    finally:
        if fb_thread:
            fb_thread.stop()
        ser.close()


if __name__ == '__main__':
    main()
