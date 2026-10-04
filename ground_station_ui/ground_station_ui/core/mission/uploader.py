"""Gui danh sach waypoint bang giao thuc mission MAVLink (Python thuan).

    GCS ── MISSION_COUNT(n) ─────────────► dich (Jetson; sau nay co the la PX4)
        ◄─ MISSION_REQUEST_INT(seq) ─────┘   (dich xin tung diem, tu toc do cua no)
        ── MISSION_ITEM_INT(seq) ────────►
        ...
        ◄─ MISSION_ACK(ACCEPTED) ─────────  xong

Khong co phan hoi sau `timeout_s` -> gui lai message cuoi (COUNT hoac ITEM), het `retries` -> timeout.
Danh sach duoc chup lai luc bam gui: sua waypoint trong luc dang gui khong lam lech du lieu.
"""

import time

from ground_station_ui.core.facts import FactGroup
from ground_station_ui.core.mavlink import drone_dialect as mavlink

MISSION_RESULT = {
    mavlink.MAV_MISSION_ACCEPTED: 'done',
    mavlink.MAV_MISSION_NO_SPACE: 'No space',
    mavlink.MAV_MISSION_UNSUPPORTED: 'Unsupported',
    mavlink.MAV_MISSION_DENIED: 'Denied',
    mavlink.MAV_MISSION_INVALID_SEQUENCE: 'Invalid sequence',
}


def to_items(waypoints, target_system, target_component):
    """[{lat, lon, alt}] -> [MISSION_ITEM_INT]; do cao tuong doi diem Home."""
    out = []
    for i, wp in enumerate(waypoints):
        out.append(mavlink.MAVLink_mission_item_int_message(
            target_system, target_component, i,
            mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mavlink.MAV_CMD_NAV_WAYPOINT,
            1 if i == 0 else 0, 1,                       # current, autocontinue
            0, 0, 0, float('nan'),                       # hold, accept radius, pass radius, yaw
            int(round(wp['lat'] * 1e7)), int(round(wp['lon'] * 1e7)), float(wp['alt']),
            mavlink.MAV_MISSION_TYPE_MISSION))
    return out


class MissionUploader(FactGroup):
    """values: busy, state (idle|sending|done|timeout|no_link|error), sent, total, message."""

    def __init__(self, links, timeout_s=1.5, retries=5):
        super().__init__(busy=False, state='idle', sent=0, total=0, message='')
        self.links = links
        self.timeout_s = float(timeout_s)
        self.retries = int(retries)
        self._job = None

    def attach(self, router):
        router.subscribe('MISSION_REQUEST_INT', self.on_request)
        router.subscribe('MISSION_REQUEST', self.on_request)    # dich cu (khong INT) -- van tra ITEM_INT
        router.subscribe('MISSION_ACK', self.on_ack)

    def upload(self, target, waypoints):
        items = to_items(waypoints, target.system, target.component)
        self._job = {'target': target, 'items': items, 'last': None, 'sent_at': 0.0, 'tries': 0,
                     'requested': set()}
        self.set(busy=True, state='sending', sent=0, total=len(items), message='')
        count = mavlink.MAVLink_mission_count_message(target.system, target.component, len(items),
                                                      mavlink.MAV_MISSION_TYPE_MISSION)
        self._transmit(count)

    def cancel(self):
        if self._job:
            self._finish('idle', 'Cancelled')

    # --- noi bo -----------------------------------------------------------------------------
    def _transmit(self, msg, retry=False):
        job = self._job
        link = self.links.get(job['target'].link)
        job['last'] = msg
        job['sent_at'] = time.monotonic()
        job['tries'] = job['tries'] + 1 if retry else 1
        if link is None or not link.send(msg):
            self._finish('no_link', 'Link not open: ' + job['target'].link)

    def _finish(self, state, message=''):
        self._job = None
        self.set(busy=False, state=state, message=message)

    def _from_target(self, m, link_name):
        job = self._job
        if job is None or link_name != job['target'].link:
            return False
        t = job['target']
        if m.get_srcSystem() != t.system or m.get_srcComponent() != t.component:
            return False
        return getattr(m, 'mission_type', 0) == mavlink.MAV_MISSION_TYPE_MISSION

    def on_request(self, m, link_name):
        if not self._from_target(m, link_name):
            return
        job = self._job
        if not 0 <= m.seq < len(job['items']):
            self._finish('error', f'Target requested missing item #{m.seq}')
            return
        job['requested'].add(m.seq)
        self.set(sent=len(job['requested']))
        self._transmit(job['items'][m.seq])

    def on_ack(self, m, link_name):
        if not self._from_target(m, link_name):
            return
        result = MISSION_RESULT.get(m.type, f'Error {m.type}')
        if result == 'done':
            self._finish('done', f'Uploaded {len(self._job["items"])} waypoints')
        else:
            self._finish('error', result)

    def tick(self, now):
        job = self._job
        if job is None or now - job['sent_at'] < self.timeout_s:
            return
        if job['tries'] >= self.retries:
            self._finish('timeout', 'Target not responding')
        else:
            self._transmit(job['last'], retry=True)
