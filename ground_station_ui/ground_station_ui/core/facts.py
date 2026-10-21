"""FactGroup: one data group (GPS, battery, link...) = `values` dict + `items` list + `dirty` flag.

Written at message rate (up to ~50 Hz); the Qt layer reads it only when `dirty`, at most 5-10 Hz.
"""


class FactGroup:
    def __init__(self, **defaults):
        self.values = dict(defaults)
        self.items = []
        self.dirty = True

    def set(self, **kv):
        for k, v in kv.items():
            if self.values.get(k) != v:
                self.values[k] = v
                self.dirty = True

    def set_items(self, items):
        if items != self.items:
            self.items = items
            self.dirty = True

    def attach(self, router):
        """Subscribe to messages on the router (subclasses)."""

    def tick(self, now):
        """Time-based update (staleness, data age); called from the UI ticker."""


class PathLog:
    """Multi-segment polyline read incrementally.

    Write: start_segment(), add(point), clear(), trim_front(n), thin()
    Read (single consumer, once per UI flush):
        take() -> None | ('reset', segments) | ('append', [(seg_index, point), ...])
    clear / trim_front / thin produce 'reset'; add produces 'append'.
    """

    def __init__(self):
        self.segments = []
        self.count = 0
        self._pending = []
        self._reset = True

    def start_segment(self):
        if not self.segments or self.segments[-1]:
            self.segments.append([])

    def add(self, point):
        if not self.segments:
            self.segments.append([])
        self.segments[-1].append(point)
        self.count += 1
        if not self._reset:
            self._pending.append((len(self.segments) - 1, point))

    def last(self):
        for seg in reversed(self.segments):
            if seg:
                return seg[-1]
        return None

    def clear(self):
        self.segments, self.count = [], 0
        self._mark_reset()

    def trim_front(self, n):
        """Drop the first n points."""
        while n > 0 and self.segments:
            seg = self.segments[0]
            k = min(n, len(seg))
            del seg[:k]
            n -= k
            self.count -= k
            if not seg:
                self.segments.pop(0)
        self._mark_reset()

    def thin(self):
        """Halve the number of points, keeping the first and last point of each segment."""
        self.segments = [s[:-1:2] + s[-1:] if len(s) > 2 else s for s in self.segments]
        self.count = sum(len(s) for s in self.segments)
        self._mark_reset()

    def _mark_reset(self):
        self._reset, self._pending = True, []

    def take(self):
        if self._reset:
            self._reset, self._pending = False, []
            return 'reset', self.segments
        if self._pending:
            out, self._pending = self._pending, []
            return 'append', out
        return None
