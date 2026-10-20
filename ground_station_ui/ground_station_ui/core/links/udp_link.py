"""Link UDP: PX4 SITL (mac dinh gui toi udp 14550), hoac MAVLink qua mang.

GCS nghe tren `listen` (host:port); dia chi phia drone hoc tu goi tin dau tien nhan duoc,
moi goi gui di (heartbeat, lenh) tra ve dia chi do.

`remote` (host:port, tuy chon): dich gui khi chua co peer, vd PX4 SITL cong 18570. Instance MAVLink GCS
cua PX4 gui toi peer gui goi gan nhat; neu chuong trinh khac (QGC, MAVROS) da gui toi 18570, app chi
nghe se nhan 0 msg/s. `remote` -> heartbeat 1 Hz toi PX4 -> PX4 chuyen luong ve app.
"""

import socket
from ground_station_ui.core.links.base import Link

class UdpLink(Link):
    kind = 'udp'

    def __init__(self, name, cfg, on_messages, on_state):
        super().__init__(name, cfg, on_messages, on_state)
        host, _, port = str(cfg.get('listen', '0.0.0.0:14550')).rpartition(':')
        self.listen = (host or '0.0.0.0', int(port))
        remote = str(cfg.get('remote') or '')
        rhost, _, rport = remote.rpartition(':')
        self.remote = (rhost or '127.0.0.1', int(rport)) if remote else None
        self._peer = None

    def describe(self):
        return f'udp {self.listen[0]}:{self.listen[1]}'

    def _open(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
        peer = self._peer or self.remote   
        if peer is not None:     
            sock.sendto(data, peer)

    def _close(self, sock):
        sock.close()
