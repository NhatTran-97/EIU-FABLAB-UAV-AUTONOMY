"""Tan so + tuoi tung loai message (de the Telemetry hien 'x Hz' hoac STALE thay vi so cu)."""

import time
from collections import deque


class MessageStats:
    WINDOW_S = 3.0

    def __init__(self):
        self._last = {}       # type -> thoi diem nhan cuoi
        self._times = {}      # type -> [thoi diem trong cua so]

    def attach(self, router):
        router.subscribe('*', self.on_msg)

    def on_msg(self, m, _link):
        now = time.monotonic()
        t = m.get_type()
        self._last[t] = now
        lst = self._times.setdefault(t, [])
        lst.append(now)
        if len(lst) > 400:
            del lst[:200]

    def age(self, msg_type, now):
        last = self._last.get(msg_type)
        return float('inf') if last is None else now - last

    def snapshot(self, now):
        """-> {type: {'age': s, 'rate': Hz}}"""
        out = {}
        for t, last in self._last.items():
            recent = [x for x in self._times.get(t, ()) if now - x <= self.WINDOW_S]
            self._times[t] = recent
            out[t] = {'age': round(now - last, 1), 'rate': round(len(recent) / self.WINDOW_S, 1)}
        return out


class History:
    """Lich su cac tin hieu (lay mau moi tick UI) cho do thi Real-time."""

    def __init__(self, signals, max_s=300.0):
        self.signals = signals        # name -> ham () -> float | None
        self.max_s = max_s
        self.data = {k: deque() for k in signals}

    def sample(self, now):
        for k, fn in self.signals.items():
            try:
                v = fn()
            except Exception:
                v = None
            if v is not None:
                lst = self.data[k]
                lst.append((now, float(v)))
                while lst and now - lst[0][0] > self.max_s:
                    lst.popleft()

    def series(self, name, now, window_s):
        return [[round(t - now, 2), v] for t, v in self.data.get(name, ()) if now - t <= window_s]
