"""Send MAVLink COMMAND_LONG with ACK wait and retries (pure Python).

    GCS ──COMMAND_LONG(confirmation=0,1,2..)──► target (Jetson / PX4)
        ◄──────────── COMMAND_ACK(result) ────┘
    No ACK within ack_timeout_s -> resend (incrementing `confirmation`); `retries` silent sends in a row
    -> timeout. Any ACK (including IN_PROGRESS) resets the counter.
    Silence after IN_PROGRESS still triggers resends to poll the state; a target seeing confirmation > 0
    does not rerun the command and replies IN_PROGRESS or the final result (when that ACK was lost).

Multi-step commands (e.g. direct to PX4: ARM -> NAV_TAKEOFF) send the next step only after the previous
one is ACCEPTED; a rejected or timed-out step stops the whole command.

At most one pending command; a new one replaces it. Retries run in tick() on the main thread.
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
    """Command target: link and sysid/compid (e.g. the Jetson over radio 433). `name` is shown in the UI."""

    def __init__(self, link, system, component, ack_timeout_s=1.5, retries=3, name=''):
        self.link = link
        self.system = int(system)
        self.component = int(component)
        self.ack_timeout_s = float(ack_timeout_s)
        self.retries = int(retries)
        self.name = name or str(link)


class CommandSender(FactGroup):
    """values: busy, label, state (idle|sending|in_progress|accepted|rejected|denied|unsupported|
    failed|timeout|no_link|blocked), attempt (consecutive sends without reply), retries,
    route (target name), message (blocked reason, or the target's STATUSTEXT explaining a refusal),
    step / steps (multi-step commands),
    elapsed (seconds from press to result; -1 = pending)."""

    def __init__(self, links):
        super().__init__(busy=False, label='', state='idle', attempt=0, retries=0,
                         route='', message='', step=0, steps=0, elapsed=-1.0)
        self.links = links              # LinkManager (.get(name) -> Link)
        self._pending = None            # dict: target, steps, i, label, sent_at, silent, tx, started

    def attach(self, router):
        router.subscribe('COMMAND_ACK', self.on_ack)
        router.subscribe('STATUSTEXT', self.on_text)

    def on_text(self, m, link_name):
        """The target explains a refusal with a STATUSTEXT sent just before its COMMAND_ACK: keep the latest one
        received while the command is pending, to show it with the result."""
        p = self._pending
        if p is None or link_name != p['target'].link:
            return
        if m.get_srcSystem() == p['target'].system and m.get_srcComponent() == p['target'].component:
            p['reason'] = m.text

    # --- send ---------------------------------------------------------------------------
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
        """Command not sent (blocked by the GCS: wrong flight state, no route...)."""
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
        reason = (p or {}).get('reason', '') if state != 'accepted' else ''
        self.set(busy=False, state=state, message=reason,
                 elapsed=round(time.monotonic() - p['started'], 1) if p else -1.0)

    # --- ACK handling ---------------------------------------------------------------------
    def on_ack(self, m, link_name):
        p = self._pending
        if p is None or link_name != p['target'].link or m.command != p['steps'][p['i']][0]:
            return
        t = p['target']
        if m.get_srcSystem() != t.system or m.get_srcComponent() != t.component:
            return
        state = RESULT_TEXT.get(m.result, 'failed')
        p['silent'] = 0                 # reset the silence counter
        if state == 'in_progress':
            p['sent_at'] = time.monotonic()
            self.set(state='in_progress', attempt=0)
            return
        if state == 'accepted' and p['i'] + 1 < len(p['steps']):
            p['i'] += 1                 # next step (ARM -> NAV_TAKEOFF)
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
            self._transmit(now)         # resend / poll state
