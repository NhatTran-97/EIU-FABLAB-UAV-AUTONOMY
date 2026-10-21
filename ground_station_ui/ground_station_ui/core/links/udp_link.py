"""UDP link: PX4 SITL (sends to udp 14550 by default) or MAVLink over a network.

The GCS listens on `listen` (host:port); the drone address is learned from the first received packet and
every outgoing packet (heartbeat, commands) goes back to it.

`remote` (host:port, optional): destination before any peer is known, e.g. PX4 SITL GCS port 18570. The PX4
GCS MAVLink instance streams to the peer that sent it the latest packet; if another program (QGC, MAVROS)
has sent to 18570, a listen-only app gets 0 msg/s. With `remote`, a 1 Hz heartbeat to PX4 pulls the stream
back to this app.
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
