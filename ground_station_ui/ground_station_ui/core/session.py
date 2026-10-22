"""Session state kept across restarts of the app (pure Python).

Saved to a JSON file and restored at start-up, so closing the app (or a crash) during a flight loses nothing
the operator needs:
    mission   waypoint list being edited and the mission options (speed, tolerances, default altitude)
    uploaded  waypoints of the last successful Send WP (planned path in the 3D view)
    track     flown path shown on the map, and the flight being recorded (continued after a restart)
    trail3d   flown trail of the 3D view (EKF frame; dropped if the EKF was reset in the meantime)
    status    last result of each command sender and of Send WP, and the message list (STATUSTEXT)

Live data (position, mission progress, link state) is not saved: it comes back from the drone within a second.

save_if_changed() is called from the UI tick; it writes at most every SAVE_PERIOD_S and only when something
changed. Writes are atomic (temporary file + rename), so a crash never leaves a half-written file.
"""

import json
import logging
import os

log = logging.getLogger(__name__)

SAVE_PERIOD_S = 2.0
VERSION = 1
BUSY_STATES = ('sending', 'in_progress')     # a command still running when the app closed is not restored


class SessionStore:
    def __init__(self, path, mission, uploader, track, status_groups=None, messages=None, pose=None):
        """status_groups: {name: FactGroup} whose last result is shown (command senders, uploader);
        messages: the STATUSTEXT group (its items are kept)."""
        self.path = str(path)
        self.mission = mission
        self.uploader = uploader
        self.track = track
        self.status_groups = status_groups or {}
        self.messages = messages
        self.pose = pose
        self._last_key = None
        self._last_save = -SAVE_PERIOD_S

    # --- state ---------------------------------------------------------------------------
    def snapshot(self):
        m = self.mission
        return {
            'version': VERSION,
            'mission': {'items': m.items, 'values': {k: m.values[k] for k in
                                                     ('defaultAltitude', 'speed', 'xyTolerance', 'zTolerance')}},
            'uploaded': {'waypoints': self.uploader.uploaded, 'revision': self.uploader.revision},
            'track': self.track.snapshot(),
            'status': {name: g.values for name, g in self.status_groups.items()},
            'messages': self.messages.items if self.messages is not None else [],
            'trail3d': self.pose.snapshot() if self.pose is not None else {},
        }

    def _key(self):
        """Cheap change detector: the mission is small, the track is summarised by its counters."""
        t = self.track
        return (json.dumps([self.mission.items, self.mission.values], sort_keys=True),
                self.uploader.revision, t.path.count, len(t.path.segments), len(t._flight),
                t.values['lengthM'], t._armed,
                json.dumps([g.values for g in self.status_groups.values()], sort_keys=True, default=str),
                len(self.messages.items) and self.messages.items[0].get('time') if self.messages is not None else 0,
                (self.pose.trail.count, len(self.pose.trail.segments)) if self.pose is not None else 0)

    # --- load / save ---------------------------------------------------------------------
    def load(self):
        """Restore the last session; a missing or unreadable file starts empty."""
        try:
            with open(self.path) as f:
                data = json.load(f)
        except FileNotFoundError:
            return False
        except (OSError, ValueError) as e:
            log.warning('Session file %s unreadable, starting empty: %s', self.path, e)
            return False
        if data.get('version') != VERSION:
            return False
        try:
            self.mission.restore(data.get('mission') or {})
            up = data.get('uploaded') or {}
            self.uploader.uploaded = list(up.get('waypoints') or [])
            self.uploader.revision = int(up.get('revision') or 0)
            self.track.restore(data.get('track') or {})
            self._restore_status(data.get('status') or {}, data.get('messages') or [])
            if self.pose is not None:
                self.pose.restore(data.get('trail3d') or {})
        except (KeyError, TypeError, ValueError) as e:
            log.warning('Session file %s invalid, partly restored: %s', self.path, e)
        self._last_key = self._key()
        log.info('Session restored: %d waypoints, %d uploaded, track %.0f m',
                 len(self.mission.items), len(self.uploader.uploaded), self.track.values['lengthM'])
        return True

    def _restore_status(self, status, messages):
        for name, group in self.status_groups.items():
            values = status.get(name)
            if not isinstance(values, dict) or values.get('state') in BUSY_STATES:
                continue
            group.set(**{k: v for k, v in values.items() if k in group.values})
            group.set(busy=False)
        if self.messages is not None and isinstance(messages, list):
            self.messages.set_items([m for m in messages if isinstance(m, dict)])

    def save(self):
        tmp = self.path + '.tmp'
        try:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            with open(tmp, 'w') as f:
                json.dump(self.snapshot(), f)
            os.replace(tmp, self.path)
        except OSError as e:
            log.warning('Session not saved: %s', e)
            return
        self._last_key = self._key()

    def save_if_changed(self, now):
        if now - self._last_save < SAVE_PERIOD_S:
            return
        self._last_save = now
        if self._key() != self._last_key:
            self.save()
