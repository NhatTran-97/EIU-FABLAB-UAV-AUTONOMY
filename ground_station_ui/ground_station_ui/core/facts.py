"""FactGroup: nhom du lieu (GPS, pin, link...) = dict `values` + list `items` + co `dirty`.

Ghi o tan so message (toi ~50 Hz); tang Qt doc khi `dirty`, toi da 5-10 Hz.
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
        """Dang ky message voi router (lop con)."""

    def tick(self, now):
        """Cap nhat theo thoi gian (stale, tuoi du lieu); goi tu UI ticker."""


class PathLog:
    """Polyline nhieu doan, doc tang dan.

    Ghi: start_segment(), add(point), clear(), trim_front(n), thin()
    Doc (1 consumer, moi lan flush UI):
        take() -> None | ('reset', segments) | ('append', [(seg_index, point), ...])
    clear / trim_front / thin -> 'reset'; add -> 'append'.
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
        """Bo n diem dau."""
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
        """Giam 1/2 so diem, giu diem dau / cuoi moi doan."""
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
