"""Gui lenh MAVLink COMMAND_LONG co cho ACK + gui lai (Python thuan).

    GCS ──COMMAND_LONG(confirmation=0,1,2..)──► dich (Jetson / PX4)
        ◄──────────── COMMAND_ACK(result) ────┘
    Im lang ack_timeout_s -> gui lai (tang `confirmation`); `retries` lan gui lien tiep KHONG nghe
    gi -> timeout. Bat ky ACK nao (ke ca IN_PROGRESS) dem lai tu dau.
    Dang IN_PROGRESS ma im lang -> VAN gui lai (hoi trang thai): dich thay confirmation > 0 thi
    khong chay lai -- tra IN_PROGRESS, hoac ket qua cu neu vua xong (ACK ket qua bi rot tren radio).
    Mat vai goi IN_PROGRESS lien tiep tren radio khong con bi bao nham "not responding".

Lenh nhieu buoc (vd gui thang PX4: ARM -> NAV_TAKEOFF): buoc sau chi gui khi buoc truoc ACCEPTED;
mot buoc bi tu choi / het han -> ca lenh dung o do.

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
    """Dich cua lenh: link nao + sysid/compid nao (vd Jetson qua radio 433). `name`: hien tren UI."""

    def __init__(self, link, system, component, ack_timeout_s=1.5, retries=3, name=''):
        self.link = link
        self.system = int(system)
        self.component = int(component)
        self.ack_timeout_s = float(ack_timeout_s)
        self.retries = int(retries)
        self.name = name or str(link)


class CommandSender(FactGroup):
    """values: busy, label, state (idle|sending|in_progress|accepted|rejected|denied|unsupported|
    failed|timeout|no_link|blocked), attempt (so lan gui lien tiep chua nghe gi), retries,
    route (ten dich), message (ly do blocked), step / steps (lenh nhieu buoc),
    elapsed (giay tu luc bam den khi co ket qua; -1 = chua xong)."""

    def __init__(self, links):
        super().__init__(busy=False, label='', state='idle', attempt=0, retries=0,
                         route='', message='', step=0, steps=0, elapsed=-1.0)
        self.links = links              # LinkManager (.get(name) -> Link)
        self._pending = None            # dict: target, steps, i, label, sent_at, silent, tx, started

    def attach(self, router):
        router.subscribe('COMMAND_ACK', self.on_ack)

    # --- gui --------------------------------------------------------------------------------
    def send(self, target, command, params=(), label=''):
        self.send_steps(target, [(command, params)], label or str(command))

    def send_steps(self, target, steps, label):
        steps = [(int(c), (list(p) + [0.0] * 7)[:7]) for c, p in steps]
        now = time.monotonic()
        self._pending = {'target': target, 'steps': steps, 'i': 0, 'label': label,
                         'silent': 0, 'tx': 0, 'sent_at': 0.0, 'started': now}
        self.set(busy=True, label=label, state='sending', attempt=0, retries=target.retries,
                 route=target.name, message='', step=1, steps=len(steps), elapsed=-1.0)
        self._transmit(now)

    def block(self, label, route, message):
        """Lenh khong gui (GCS tu chan: sai trang thai bay, khong co duong gui...)."""
        self._pending = None
        self.set(busy=False, label=label, state='blocked', route=route, message=message,
                 attempt=0, step=0, steps=0, elapsed=-1.0)

    def _transmit(self, now):
        p = self._pending
        t = p['target']
        command, params = p['steps'][p['i']]
        link = self.links.get(t.link)
        p['silent'] += 1
        p['sent_at'] = now
        msg = mavlink.MAVLink_command_long_message(t.system, t.component, command,
                                                   min(p['tx'], 255), *params)   # confirmation
        p['tx'] += 1
        if link is None or not link.send(msg):
            self._finish('no_link')
            return
        self.set(attempt=p['silent'])

    def _finish(self, state):
        p, self._pending = self._pending, None
        self.set(busy=False, state=state, elapsed=round(time.monotonic() - p['started'], 1) if p else -1.0)

    # --- nhan ACK ---------------------------------------------------------------------------
    def on_ack(self, m, link_name):
        p = self._pending
        if p is None or link_name != p['target'].link or m.command != p['steps'][p['i']][0]:
            return
        t = p['target']
        if m.get_srcSystem() != t.system or m.get_srcComponent() != t.component:
            return
        state = RESULT_TEXT.get(m.result, 'failed')
        p['silent'] = 0                 # dich con song: dem lai so lan im lang
        if state == 'in_progress':      # dich dang thuc hien: cho them
            p['sent_at'] = time.monotonic()
            self.set(state='in_progress', attempt=0)
            return
        if state == 'accepted' and p['i'] + 1 < len(p['steps']):
            p['i'] += 1                 # buoc tiep theo (vd ARM xong -> NAV_TAKEOFF)
            p['tx'] = 0
            self.set(state='sending', step=p['i'] + 1)
            self._transmit(time.monotonic())
            return
        self._finish(state)

    def tick(self, now):
        p = self._pending
        if p is None or now - p['sent_at'] < p['target'].ack_timeout_s:
            return
        if p['silent'] >= p['target'].retries:
            self._finish('timeout')
        else:
            self._transmit(now)         # gui lai / hoi trang thai (ca khi dang IN_PROGRESS)
