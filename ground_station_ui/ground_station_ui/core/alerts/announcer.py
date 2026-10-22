"""Voice announcements: watch the drone state and produce the sentences to speak when it changes.

Pure Python (testable): tick(now) reads the FactGroups and returns new sentences.
The Qt layer (qt/voice.py) feeds them to QTextToSpeech.

Events: flight mode change, arm / disarm, PX4 and Jetson link lost / restored, battery thresholds, the result
of every flight command and of Send WP, and mission progress (paused, complete).
"""

from ground_station_ui.core.alerts.health import BATTERY_CRIT, BATTERY_WARN

PHRASES = {
    'mode': '{mode} mode', 'armed': 'Armed', 'disarmed': 'Disarmed',
    'px4_lost': 'PX4 connection lost', 'px4_back': 'PX4 connected',
    'jetson_lost': 'Jetson connection lost', 'jetson_back': 'Jetson connected',
    'battery': 'Battery {pct} percent',
    'battery_low': 'Battery low, {pct} percent',
    'battery_critical': 'Battery critical, {pct} percent. Land now',
    'cmd_accepted': '{label} accepted', 'cmd_failed': '{label} failed', 'cmd_denied': '{label} denied',
    'cmd_timeout': '{label}: no response', 'cmd_no_link': '{label}: no link', 'cmd_blocked': '{label} blocked',
    'upload_done': '{n} waypoints sent', 'upload_failed': 'Waypoint upload failed',
    'mission_started': 'Mission started, flying to waypoint {seq} of {total}',
    'mission_resumed': 'Mission resumed',
    'mission_paused': 'Mission paused', 'mission_complete': 'Mission complete. Hovering at the last waypoint',
}

# command sender state -> phrase key (final states only; sending / in progress are not spoken)
CMD_RESULT = {'accepted': 'cmd_accepted', 'denied': 'cmd_denied', 'rejected': 'cmd_failed', 'failed': 'cmd_failed',
              'unsupported': 'cmd_failed', 'timeout': 'cmd_timeout', 'no_link': 'cmd_no_link',
              'blocked': 'cmd_blocked'}
SPOKEN = {'RTL': 'Return to launch', 'Send WP': 'Send waypoints'}


def spoken_label(label):
    """'Start mission 2 m/s + avoidance' -> 'Start mission': the command name without its numbers / options."""
    words = []
    for w in str(label).replace('[TEST]', '').split():
        if any(c.isdigit() for c in w) or w == '+':
            break
        words.append(w)
    name = ' '.join(words)
    return SPOKEN.get(name, name)

BATTERY_STEPS = (30, 20, 15, 10)     # announced once when the battery drops below each level


class Announcer:
    def __init__(self, vehicle, senders=(), uploader=None):
        """senders: CommandSender groups whose results are spoken; uploader: MissionUploader (Send WP)."""
        self.vehicle = vehicle
        self.senders = list(senders)
        self.uploader = uploader
        self._cmd = [None] * len(self.senders)        # last (label, state) of each sender
        self._upload = None
        self._mission = 'unset'           # mission state at the previous tick; 'unset' before the first one
        self._mode = None
        self._armed = None
        self._px4 = None
        self._jetson = None
        self._bat_step = None

    def _say(self, key, **kw):
        return PHRASES[key].format(**kw)

    def tick(self, now=None):
        g = self.vehicle.groups
        st, comp, bat = g['status'].values, g['companion'].values, g['battery'].values
        out = []

        px4 = bool(st.get('linkAlive'))
        if self._px4 is not None and px4 != self._px4:
            out.append(self._say('px4_back' if px4 else 'px4_lost'))
        self._px4 = px4

        if px4:
            mode = st.get('mode')
            if mode and mode != '—':
                if self._mode is not None and mode != self._mode:
                    out.append(self._say('mode', mode=mode))
                self._mode = mode
            armed = bool(st.get('armed'))
            if self._armed is not None and armed != self._armed:
                out.append(self._say('armed' if armed else 'disarmed'))
            self._armed = armed

            pct = bat.get('percent', -1)
            if pct is not None and pct >= 0:
                step = next((x for x in sorted(BATTERY_STEPS) if pct <= x), None)   # lowest level reached
                if self._bat_step is None:
                    # first sample: announce only when flying and already below a level
                    if step is not None and self._armed:
                        out.append(self._battery(pct))
                    self._bat_step = step if step is not None else 101
                elif step is not None and step < self._bat_step:
                    out.append(self._battery(pct))
                    self._bat_step = step
                elif pct > self._bat_step + 5:          # battery swapped (5 % hysteresis)
                    self._bat_step = step if step is not None else 101

        jetson = bool(comp.get('linkAlive'))
        if self._jetson is not None and jetson != self._jetson:
            out.append(self._say('jetson_back' if jetson else 'jetson_lost'))
        self._jetson = jetson

        out += self._commands()
        out += self._mission_progress(g.get('missionProgress'))
        return out

    def _battery(self, pct):
        key = 'battery_critical' if pct <= BATTERY_CRIT else 'battery_low' if pct <= BATTERY_WARN else 'battery'
        return self._say(key, pct=int(pct))

    def _commands(self):
        """Final result of each command (accepted, denied, no response...) and of Send WP, spoken once."""
        out = []
        for i, sender in enumerate(self.senders):
            v = sender.values
            key = (v.get('label'), v.get('state'))
            if self._cmd[i] is not None and key != self._cmd[i] and key[1] in CMD_RESULT and key[0]:
                out.append(self._say(CMD_RESULT[key[1]], label=spoken_label(key[0])))
            self._cmd[i] = key
        if self.uploader is not None:
            v = self.uploader.values
            key = (v.get('state'), self.uploader.revision)
            if self._upload is not None and key != self._upload:
                if key[0] == 'done':
                    out.append(self._say('upload_done', n=v.get('total', 0)))
                elif key[0] in ('timeout', 'no_link', 'error'):
                    out.append(self._say('upload_failed'))
            self._upload = key
        return out

    def _mission_progress(self, group):
        if group is None:
            return []
        v = group.values
        state = v.get('state') if v.get('valid') else None
        out = []
        if self._mission != 'unset' and state != self._mission:
            if state == 'active':
                out.append(self._say('mission_resumed') if self._mission == 'paused'
                           else self._say('mission_started', seq=v.get('seq', 1), total=v.get('total', 0)))
            elif state == 'paused':
                out.append(self._say('mission_paused'))
            elif state == 'complete':
                out.append(self._say('mission_complete'))
        self._mission = state
        return out
