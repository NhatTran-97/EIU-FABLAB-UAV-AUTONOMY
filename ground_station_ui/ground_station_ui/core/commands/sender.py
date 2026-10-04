"""Gui lenh MAVLink COMMAND_LONG co cho ACK + gui lai (Python thuan).

    GCS ──COMMAND_LONG(confirmation=0,1,2..)──► dich (Jetson / PX4)
        ◄──────────── COMMAND_ACK(result) ────┘
    Khong co ACK sau ack_timeout_s -> gui lai (tang `confirmation`), het `retries` -> timeout.

Moi luc chi 1 lenh dang cho (bam Take-off roi Land ngay -> lenh sau thay lenh truoc).
Retry chay trong tick() cua UI ticker nen khong can thread rieng.
"""

import time

from ground_station_ui.core.facts import FactGroup
from ground_station_ui.core.mavlink import drone_dialect as mavlink

RESULT_TEXT = {
    mavlink.MAV_RESULT_ACCEPTED: 'accepted',
    mavlink.MAV_RESULT_TEMPORARILY_REJECTED: 'rejected',
    mavlink.MAV_RESULT_DENIED: 'denied',
    mavlink.MAV_RESULT_UNSUPPORTED: 'unsupported',
    mavlink.MAV_RESULT_FAILED: 'failed',
    mavlink.MAV_RESULT_IN_PROGRESS: 'in_progress',
}


class CommandTarget:
    """Dich cua lenh: link nao + sysid/compid nao (vd Jetson qua radio 433)."""

    def __init__(self, link, system, component, ack_timeout_s=1.5, retries=3):
        self.link = link
        self.system = int(system)
        self.component = int(component)
        self.ack_timeout_s = float(ack_timeout_s)
        self.retries = int(retries)


class CommandSender(FactGroup):
    """values: busy, label, state (idle|sending|accepted|rejected|...|timeout|no_link), attempt."""

    def __init__(self, links):
        super().__init__(busy=False, label='', state='idle', attempt=0, retries=0)
        self.links = links              # LinkManager (.get(name) -> Link)
        self._pending = None            # dict: target, command, params, label, sent_at, attempt

    def attach(self, router):
        router.subscribe('COMMAND_ACK', self.on_ack)

    # --- gui --------------------------------------------------------------------------------
    def send(self, target, command, params=(), label=''):
        params = list(params) + [0.0] * (7 - len(params))
        self._pending = {'target': target, 'command': int(command), 'params': params[:7],
                         'label': label or str(command), 'attempt': 0, 'sent_at': 0.0}
        self.set(busy=True, label=self._pending['label'], state='sending', attempt=0, retries=target.retries)
        self._transmit(time.monotonic())

    def _transmit(self, now):
        p = self._pending
        t = p['target']
        link = self.links.get(t.link)
        p['attempt'] += 1
        p['sent_at'] = now
        msg = mavlink.MAVLink_command_long_message(t.system, t.component, p['command'],
                                                   min(p['attempt'] - 1, 255), *p['params'])
        if link is None or not link.send(msg):
            self._finish('no_link')
            return
        self.set(attempt=p['attempt'])

    def _finish(self, state):
        self._pending = None
        self.set(busy=False, state=state)

    # --- nhan ACK ---------------------------------------------------------------------------
    def on_ack(self, m, link_name):
        p = self._pending
        if p is None or link_name != p['target'].link or m.command != p['command']:
            return
        t = p['target']
        if m.get_srcSystem() != t.system or m.get_srcComponent() != t.component:
            return
        state = RESULT_TEXT.get(m.result, 'failed')
        if state == 'in_progress':      # dich dang thuc hien: cho them, khong gui lai
            p['sent_at'] = time.monotonic()
            self.set(state='in_progress')
            return
        self._finish(state)

    def tick(self, now):
        p = self._pending
        if p is None or now - p['sent_at'] < p['target'].ack_timeout_s:
            return
        if self.values['state'] == 'in_progress' or p['attempt'] >= p['target'].retries:
            self._finish('timeout')
        else:
            self._transmit(now)
