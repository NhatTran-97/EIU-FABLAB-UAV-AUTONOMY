"""Voice announcements: watch the drone state and produce the sentences to speak when it changes.

Pure Python (testable): tick(now) reads the FactGroups and returns new sentences.
The Qt layer (qt/voice.py) feeds them to QTextToSpeech.

Events: flight mode change, arm / disarm, PX4 and Jetson link lost / restored, battery thresholds.
"""

PHRASES = {
    'mode': '{mode} mode', 'armed': 'Armed', 'disarmed': 'Disarmed',
    'px4_lost': 'PX4 connection lost', 'px4_back': 'PX4 connected',
    'jetson_lost': 'Jetson connection lost', 'jetson_back': 'Jetson connected',
    'battery': 'Battery {pct} percent',
}

BATTERY_STEPS = (30, 20, 15, 10)     # announced once when the battery drops below each level


class Announcer:
    def __init__(self, vehicle):
        self.vehicle = vehicle
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
                        out.append(self._say('battery', pct=int(pct)))
                    self._bat_step = step if step is not None else 101
                elif step is not None and step < self._bat_step:
                    out.append(self._say('battery', pct=int(pct)))
                    self._bat_step = step
                elif pct > self._bat_step + 5:          # battery swapped (5 % hysteresis)
                    self._bat_step = step if step is not None else 101

        jetson = bool(comp.get('linkAlive'))
        if self._jetson is not None and jetson != self._jetson:
            out.append(self._say('jetson_back' if jetson else 'jetson_lost'))
        self._jetson = jetson
        return out
