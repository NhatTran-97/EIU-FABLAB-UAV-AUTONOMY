#!/usr/bin/env python3
"""
lora_gimbal_bridge_node.py
LoRa (SX1278 half-duplex UART) ↔ ROS2 bridge cho gimbal.

  UART RX  32-byte TX packet  →  parse  →  publish /gimbal_command
  UART TX  subscribe /gimbal_state  →  build 18-byte RX packet  →  UART

Topics:
  pub  /gimbal_command  skydroid_msgs/msg/GimbalCommand
  sub  /gimbal_state    skydroid_msgs/msg/GimbalState

Parameters:
  port  (string)  serial port, default /dev/ttyUSB0
  baud  (int)     baud rate,   default 9600
"""

import threading
import struct
import time
from typing import Optional

import serial
import rclpy
from rclpy.node import Node
from skydroid_msgs.msg import GimbalCommand, GimbalState

# ── Packet constants ──────────────────────────────────────────────
TX_LEN = 32          # UI → gimbal (LoRa TX)
RX_LEN = 18          # gimbal → UI (LoRa RX / feedback)
TX_STX = b'\xAA\x55'


# ── CRC-8 (polynomial 0x07) ───────────────────────────────────────
def _crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


# ── Parse incoming TX packet (UI → gimbal) ────────────────────────
def _parse_tx(pkt: bytes) -> Optional[dict]:
    if len(pkt) != TX_LEN:
        return None
    if pkt[0] != 0xAA or pkt[1] != 0x55:
        return None
    if _crc8(pkt[2:TX_LEN - 1]) != pkt[TX_LEN - 1]:
        return None

    pitch_deg, = struct.unpack_from('<f', pkt, 6)
    yaw_deg,   = struct.unpack_from('<f', pkt, 10)
    pitch_spd, = struct.unpack_from('<f', pkt, 14)
    yaw_spd,   = struct.unpack_from('<f', pkt, 18)
    pitch_vel, = struct.unpack_from('<f', pkt, 23)
    yaw_vel,   = struct.unpack_from('<f', pkt, 27)

    return {
        'ctrl_mode':  pkt[3],
        'mode':       pkt[4],
        'ptz_cmd':    pkt[5],
        'pitch_deg':  pitch_deg,
        'yaw_deg':    yaw_deg,
        'pitch_spd':  pitch_spd,
        'yaw_spd':    yaw_spd,
        'en_pitch':   bool(pkt[22] & 0x01),
        'en_yaw':     bool(pkt[22] & 0x02),
        'pitch_vel':  pitch_vel,
        'yaw_vel':    yaw_vel,
    }


# ── Build outgoing RX feedback packet (gimbal → UI) ───────────────
def _build_rx(yaw: float, pitch: float, roll: float,
              ctrl_mode: int, is_connected: bool) -> bytes:
    payload  = struct.pack('<Bfff', ctrl_mode, yaw, pitch, roll)
    payload += struct.pack('B', 1 if is_connected else 0)
    pkt      = bytes([0xBB, 0x66, len(payload)]) + payload
    pkt     += bytes([_crc8(pkt[2:])])
    assert len(pkt) == RX_LEN
    return pkt


# ── ROS2 node ─────────────────────────────────────────────────────
class LoraGimbalBridgeNode(Node):
    def __init__(self):
        super().__init__('lora_gimbal_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self._serial_lock = threading.Lock()

        try:
            self._ser = serial.Serial(
                port, baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,   # 50 ms — đủ ngắn để không chặn lock lâu
            )
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self.get_logger().info(f'Serial opened: {port} @ {baud} baud')
        except Exception as e:
            self.get_logger().fatal(f'Cannot open serial port: {e}')
            raise

        self._cmd_pub = self.create_publisher(GimbalCommand, '/gimbal_command', 10)
        self._state_sub = self.create_subscription(
            GimbalState, '/gimbal_state', self._state_callback, 10)

        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(
            'LoRa gimbal bridge ready  '
            '| pub /gimbal_command  sub /gimbal_state')

    # ── /gimbal_state → UART feedback ────────────────────────────
    def _state_callback(self, msg: GimbalState):
        acquired = self._serial_lock.acquire(blocking=False)
        if not acquired:
            return  # reader thread đang dùng port, bỏ qua lần này
        try:
            pkt = _build_rx(
                float(msg.yaw_deg),
                float(msg.pitch_deg),
                float(msg.roll_deg),
                int(msg.control_mode),
                bool(msg.is_connected),
            )
            self._ser.write(pkt)
            self._ser.flush()
            time.sleep(0.02)    # 20 ms: chờ SX1278 switch TX→RX
        except Exception as e:
            self.get_logger().error(f'UART write error: {e}')
        finally:
            self._serial_lock.release()

    # ── UART reader → /gimbal_command ────────────────────────────
    def _reader_loop(self):
        buf       = b''
        pkt_count = 0
        err_count = 0

        while rclpy.ok():
            try:
                with self._serial_lock:
                    chunk = self._ser.read(self._ser.in_waiting or 1)
            except Exception as e:
                self.get_logger().error(f'UART read error: {e}')
                time.sleep(0.1)
                continue

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

                d = _parse_tx(buf[:TX_LEN])
                if d is not None:
                    pkt_count += 1
                    self._cmd_pub.publish(self._make_cmd(d))
                    buf = buf[TX_LEN:]
                    if pkt_count % 100 == 0:
                        self.get_logger().info(
                            f'LoRa: {pkt_count} packets OK, {err_count} CRC errors')
                else:
                    err_count += 1
                    buf = buf[1:]

    # ── dict → GimbalCommand ──────────────────────────────────────
    def _make_cmd(self, d: dict) -> GimbalCommand:
        cmd = GimbalCommand()
        cmd.control_mode    = d['ctrl_mode']
        cmd.mode            = d['mode']
        cmd.ptz_cmd         = d['ptz_cmd']
        cmd.pitch_deg       = d['pitch_deg']
        cmd.yaw_deg         = d['yaw_deg']
        cmd.pitch_speed_dps = d['pitch_spd']
        cmd.yaw_speed_dps   = d['yaw_spd']
        cmd.enable_pitch    = d['en_pitch']
        cmd.enable_yaw      = d['en_yaw']
        cmd.pitch_vel_dps   = d['pitch_vel']
        cmd.yaw_vel_dps     = d['yaw_vel']
        return cmd

    def destroy_node(self):
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoraGimbalBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
