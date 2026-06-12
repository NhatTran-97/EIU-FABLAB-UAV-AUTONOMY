"""Command idempotency / dedupe — pure, NO ROS dependency (unit-testable).

The ground re-sends the SAME `seq` when it loses an ACK. This cache lets the drone re-ACK a
retried command WITHOUT executing it twice (no double mission publish, no double mode trigger).
It also resets itself when the ground session id (`sid`) changes — i.e. the ground app
restarted and its seq counter went back to 0 — so a fresh seq is not mistaken for a duplicate.
"""

from collections import deque
from threading import Lock


class IdempotencyCache:
    def __init__(self, max_entries: int = 128):
        self._lock = Lock()
        self._processed = {}        # seq -> ACK payload (None while in-flight)
        self._order = deque()       # insertion order, for bounded eviction
        self._max = max_entries
        self._cur_sid = None        # current ground session id

    def reset_session(self, sid) -> bool:
        """If `sid` is new (ground restarted), clear the cache. Returns True when it reset, so
        the caller can log it."""
        if sid is None or sid == self._cur_sid:
            return False
        with self._lock:
            self._processed.clear()
            self._order.clear()
            self._cur_sid = sid
        return True

    def check(self, seq):
        """Register a new seq or detect a duplicate. Returns (status, payload):
          ("new", None)           -> first time; caller should EXECUTE the command.
          ("dup_acked", payload)  -> already handled; caller re-sends `payload`, skips exec.
          ("dup_inflight", None)  -> handled but ACK not produced yet; caller skips.
          ("untracked", None)     -> seq is None (no dedup possible); caller executes.
        """
        if seq is None:
            return "untracked", None
        with self._lock:
            if seq in self._processed:
                cached = self._processed[seq]
                return ("dup_acked", cached) if cached is not None else ("dup_inflight", None)
            if len(self._order) >= self._max:
                self._processed.pop(self._order.popleft(), None)
            self._order.append(seq)
            self._processed[seq] = None   # in-flight; ACK cached when produced
            return "new", None

    def cache_ack(self, seq, payload):
        """Cache the FIRST ACK produced for a seq so a retry can be re-ACKed without re-executing.
        Do NOT overwrite — e.g. the auto-OFFBOARD that follows a mission must not clobber the
        mission 'uploaded' ACK that shares the same seq."""
        if seq is None:
            return
        with self._lock:
            if seq in self._processed and self._processed[seq] is None:
                self._processed[seq] = payload
