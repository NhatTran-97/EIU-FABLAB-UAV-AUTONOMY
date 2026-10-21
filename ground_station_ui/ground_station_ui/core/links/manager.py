"""LinkManager: creates links from the configuration and tracks the health of each link.

Per-link (not per-drone) state: open or not, heartbeat received from the drone side, RSSI
(RADIO_STATUS injected by the SiK radio), messages per second.
QML reads `links.values.<name>.<key>` and `links.items` (list for the Links tab).
"""

import time

from ground_station_ui.core.facts import FactGroup
from ground_station_ui.core.links.serial_link import SerialLink
from ground_station_ui.core.links.udp_link import UdpLink
from ground_station_ui.core.mavlink import drone_dialect as mavlink

LINK_TYPES = {'serial': SerialLink, 'udp': UdpLink}
ALIVE_TIMEOUT_S = 3.0


def sik_rssi_dbm(raw):
    """SiK radio RSSI (0-255) to dBm, per the SiK documentation: raw / 1.9 - 127."""
    return round(raw / 1.9 - 127)


class _LinkStats:
    def __init__(self):
        self.last_remote_hb = None
        self.msg_count = 0
        self.rate = 0.0
        self.radio = {}


class LinkManager(FactGroup):
    def __init__(self, link_cfgs, on_messages, on_state):
        super().__init__()
        self.links = {}
        self._stats = {}
        self._rate_t0 = time.monotonic()
        for cfg in link_cfgs:
            if not cfg.get('enabled', True):
                continue
            cls = LINK_TYPES[cfg.get('type', 'serial')]
            link = cls(cfg['name'], cfg, on_messages, on_state)
            self.links[link.name] = link
            self._stats[link.name] = _LinkStats()

    def attach(self, router):
        router.subscribe('*', self._on_any)

    def get(self, name):
        return self.links.get(name)

    def stop_all(self):
        for link in self.links.values():
            link.stop()

    def _on_any(self, msg, link_name):
        st = self._stats.get(link_name)
        if st is None:
            return
        st.msg_count += 1
        t = msg.get_type()
        if t == 'HEARTBEAT' and msg.type != mavlink.MAV_TYPE_GCS:
            st.last_remote_hb = time.monotonic()
        elif t == 'RADIO_STATUS':
            st.radio = {'rssiDbm': sik_rssi_dbm(msg.rssi), 'remoteRssiDbm': sik_rssi_dbm(msg.remrssi),
                        'noise': msg.noise, 'remoteNoise': msg.remnoise, 'rxErrors': msg.rxerrors,
                        'txbuf': msg.txbuf}     # % of FREE transmit buffer in the ground radio (uplink)

    def tick(self, now):
        dt = now - self._rate_t0
        if dt >= 1.0:
            for st in self._stats.values():
                st.rate, st.msg_count = st.msg_count / dt, 0
            self._rate_t0 = now

        values, items = {}, []
        for name, link in self.links.items():
            st = self._stats[name]
            age = None if st.last_remote_hb is None else now - st.last_remote_hb
            info = {
                'name': name,
                'title': link.cfg.get('title', name),
                'hint': link.cfg.get('hint', ''),
                'kind': link.kind,
                'port': getattr(link, 'port', ''),
                'baud': getattr(link, 'baud', 0),
                'status': link.status,
                'isOpen': link.is_open,
                'active': link.active,
                'alive': link.is_open and age is not None and age < ALIVE_TIMEOUT_S,
                'heartbeatAge': None if age is None else round(age),   # whole seconds: changes once per second
                'msgRate': round(st.rate),
                'rssiDbm': st.radio.get('rssiDbm'),
                'remoteRssiDbm': st.radio.get('remoteRssiDbm'),
                'txbuf': st.radio.get('txbuf'),
            }
            values[name] = info
            items.append(info)
        if values != self.values:      # unchanged link state: QML bindings are not re-evaluated
            self.values = values
            self.dirty = True
        self.set_items(items)
