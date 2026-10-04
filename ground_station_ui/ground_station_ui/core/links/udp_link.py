"""Link UDP: PX4 SITL (mac dinh gui toi udp 14550), hoac MAVLink qua mang.

GCS nghe tren `listen` (host:port); dia chi phia drone hoc tu goi tin dau tien nhan duoc,
moi goi gui di (heartbeat, lenh) tra ve dia chi do.
"""

import socket

from ground_station_ui.core.links.base import Link


class UdpLink(Link):
    kind = 'udp'

    def __init__(self, name, cfg, on_messages, on_state):
        super().__init__(name, cfg, on_messages, on_state)
        host, _, port = str(cfg.get('listen', '0.0.0.0:14550')).rpartition(':')
        self.listen = (host or '0.0.0.0', int(port))
        self._peer = None

    def describe(self):
        return f'udp {self.listen[0]}:{self.listen[1]}'

    def _open(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(self.listen)
        sock.settimeout(0.1)
        self._peer = None
        return sock

    def _read(self, sock):
        try:
            data, peer = sock.recvfrom(65535)
        except socket.timeout:
            return b''
        self._peer = peer
        return data

    def _write(self, sock, data):
        if self._peer is not None:      # chua biet drone o dau -> bo qua (vd heartbeat dau tien)
            sock.sendto(data, self._peer)

    def _close(self, sock):
        sock.close()
