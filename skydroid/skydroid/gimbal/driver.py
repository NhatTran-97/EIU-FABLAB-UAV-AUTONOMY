import time
import logging

try:
    from .protocol import build_cmd, decode_frame
    from .transport import UdpTransport
    from .gimbal import GimbalMixin
except ImportError:  # pragma: no cover - supports running files directly
    from protocol import build_cmd, decode_frame
    from transport import UdpTransport
    from gimbal import GimbalMixin

log = logging.getLogger(__name__)

class SkydroidDriver(GimbalMixin):
    def __init__(self, sky_ip, sky_port=5000,
                 local_ip="0.0.0.0", local_port=5000, timeout=1.0):
        self._transport = UdpTransport(
            local_addr=(local_ip, local_port),
            remote_addr=(sky_ip, sky_port),
            timeout=timeout,
        )
        self._timeout = timeout

    def recv_attitude(self, timeout: float | None = None) -> dict | None:
        raw = self._transport.recv(timeout=timeout)
        if raw is None:
            return None
        return self.parse_gac_packet(raw)

    def send_cmd(self, body: str, delay=0.1, expect_reply=True):
        packet = build_cmd(body)
        log.debug("TX: %s", packet.decode())
        self._transport.send(packet)

        if not expect_reply:
            time.sleep(delay)
            return None

        deadline = time.monotonic() + self._timeout
        frame = None
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            raw = self._transport.recv(timeout=remaining)
            if raw is None:
                break

            if self.parse_gac_packet(raw) is not None:
                log.debug("RX async attitude packet while waiting for %s", body)
                continue

            log.debug("RX: %s", raw)
            frame = decode_frame(raw)
            if frame is not None:
                break

        time.sleep(delay)
        if frame is None:
            log.warning("No valid response for: %s", body)
            return None

        return frame

    def check_connection(self) -> str | None:
        frame = self.send_cmd("#TPUD2rVER00")
        if frame and frame["cmd"] == "VER" and frame["crc_ok"]:
            return frame["data"]
        return None

    def __enter__(self): return self
    def __exit__(self, *_): self._transport.close()

    def close(self):
        self._transport.close()
