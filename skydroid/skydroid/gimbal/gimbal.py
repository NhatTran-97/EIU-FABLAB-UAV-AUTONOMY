
class GimbalMixin:
    """Mixin that requires self.send_cmd() from the driver class."""

    PTZ = {
        "stop": "00", "up": "01", "down": "02",
        "left": "03", "right": "04", "center": "05",
        "follow": "06", "lock": "07",
    }

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _s8_hex(self, v: int) -> str:
        return f"{self._clamp(int(v), -127, 127) & 0xFF:02X}"

    def _angle_hex(self, deg: float) -> str:
        deg = self._clamp(deg, -90.0, 90.0)
        return f"{int(round(deg * 100)) & 0xFFFF:04X}"

    def _speed_hex(self, dps: float) -> str:
        raw = self._clamp(int(round(dps * 10)), 0, 99)
        return f"{raw:02X}"

    def g_ptz(self, action: str):
        code = self.PTZ.get(action)
        if code is None:
            raise ValueError(f"Unknown PTZ action: {action}")
        self.send_cmd(f"#TPUG2wPTZ{code}", expect_reply=False)

    def g_speed(self, yaw=0, pitch=0):
        """
        Set gimbal speed independently per axis.
        g_speed(pitch=-30)         # pitch only
        g_speed(yaw=20)            # yaw only
        g_speed(yaw=20, pitch=-30) # both axes
        g_speed()                  # stop both axes
        """
        if yaw == 0 and pitch == 0:
            # Stop both axes
            self.send_cmd("#TPUG4wGSM0000", expect_reply=False)

        elif pitch == 0:
            # Yaw only — use GSY
            self.send_cmd(f"#TPUG2wGSY{self._s8_hex(yaw)}", expect_reply=False)

        elif yaw == 0:
            # Pitch only — use GSP
            self.send_cmd(f"#TPUG2wGSP{self._s8_hex(pitch)}", expect_reply=False)

        else:
            # Both axes — use GSM
            self.send_cmd(
                f"#TPUG4wGSM{self._s8_hex(yaw)}{self._s8_hex(pitch)}",
                expect_reply=False)


    def g_angle(self, yaw=None, pitch=None,
                speed_dps=3.0,
                yaw_speed_dps=None,
                pitch_speed_dps=None):
        """
        Move gimbal to an absolute angle with per-axis speed control.
        g_angle(pitch=-30)                          # pitch only, shared speed
        g_angle(pitch=-30, pitch_speed_dps=5.0)     # pitch with custom speed
        g_angle(yaw=45, yaw_speed_dps=8.0)          # yaw with custom speed
        g_angle(yaw=30, pitch=-20,
                yaw_speed_dps=5.0,
                pitch_speed_dps=3.0)                # both axes, independent speeds
        """
        if yaw is None and pitch is None:
            raise ValueError("At least one of yaw or pitch must be provided")

        # Fall back to shared speed_dps if per-axis speed is not specified
        y_spd = self._speed_hex(yaw_speed_dps   if yaw_speed_dps   is not None else speed_dps)
        p_spd = self._speed_hex(pitch_speed_dps if pitch_speed_dps is not None else speed_dps)

        if yaw is not None and pitch is not None:
            self.send_cmd(
                f"#TPUGCwGAM{self._angle_hex(yaw)}{y_spd}{self._angle_hex(pitch)}{p_spd}",
                expect_reply=False)

        elif yaw is not None:
            self.send_cmd(
                f"#TPUG6wGAY{self._angle_hex(yaw)}{y_spd}",
                expect_reply=False)

        elif pitch is not None:
            self.send_cmd(
                f"#TPUG6wGAP{self._angle_hex(pitch)}{p_spd}",
                expect_reply=False)

    def g_enable_attitude_push(self, freq_hz: int):
        """
        Enable or disable attitude push packets.
        freq_hz=0 disables, freq_hz=1..100 sets the push rate in Hz.
        """
        if freq_hz == 0:
            data = "00"
        else:
            freq_hz = max(1, min(100, freq_hz))
            data = f"{freq_hz:02X}"
        self.send_cmd(f"#TPUG2wGAA{data}", expect_reply=False)

    def parse_gac_packet(self, reply: str):
        """Parse a GAC attitude packet and return a dict with yaw/pitch/roll in degrees."""
        if not reply or "GAC" not in reply:
            return None

        # Locate the data field immediately after "GAC"
        idx = reply.find("GAC")
        if idx == -1:
            return None

        data = reply[idx+3 : idx+15]   # 12 chars: YYYYPPPPRRR
        if len(data) < 12:
            return None

        def hex4_to_deg(h: str) -> float:
            v = int(h, 16)
            if v & 0x8000:
                v -= 0x10000
            return v / 100.0

        try:
            return {
                "yaw_deg":   hex4_to_deg(data[0:4]),
                "pitch_deg": hex4_to_deg(data[4:8]),
                "roll_deg":  hex4_to_deg(data[8:12]),
            }
        except ValueError:
            return None
