"""Per-component state and rate tracking. No ROS dependency.

Each component has a ComponentMonitor fed by one of two sources:
  * diagnostics : a DiagnosticStatus on /diagnostics (matched by `diag_name`).
                  state = values[state_key], level = the driver's level,
                  rate = values[rate_key], or derived from the counter values[count_key].
  * topic       : the monitor counts messages on a topic (e.g. PX4, which has no diagnostics).

snapshot(now) returns (level, rate_hz, state); no data for `stale_timeout` gives LEVEL_STALE.
"""

LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2
LEVEL_STALE = 3

STATE_MAX_LEN = 16   # char[16] in DRONE_COMPONENT_STATUS


def _to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


class ComponentMonitor:
    def __init__(self, comp_id, name, stale_timeout, *, state_key="state",
                 rate_key="", count_key="", min_rate_hz=0.0, rate_window=1.0):
        self.comp_id = comp_id
        self.name = name
        self.stale_timeout = stale_timeout
        self.state_key = state_key
        self.rate_key = rate_key
        self.count_key = count_key
        self.min_rate_hz = min_rate_hz        # topic source only; 0 = disabled
        self.rate_window = rate_window

        self._last_update = None
        self._level = LEVEL_STALE
        self._state = "NO_DATA"
        self._rate = 0.0

        self._prev_count = None               # count_key: previous (now, count)
        self._msg_count = 0                   # topic source
        self._window_start = None

    # --- diagnostics source --------------------------------------------------------------
    def update_from_diag(self, now, level, message, values):
        """values: dict key -> str (DiagnosticStatus.values)."""
        self._last_update = now
        self._level = level if level in (LEVEL_OK, LEVEL_WARN, LEVEL_ERROR) else LEVEL_ERROR
        self._state = values.get(self.state_key) or message or "UNKNOWN"

        if self.rate_key:
            rate = _to_float(values.get(self.rate_key))
            self._rate = rate if rate is not None and rate >= 0 else 0.0
        elif self.count_key:
            count = _to_float(values.get(self.count_key))
            if count is not None and self._prev_count is not None:
                t0, c0 = self._prev_count
                dt = now - t0
                # counter went down: the node restarted, skip this sample
                if dt > 0 and count >= c0:
                    self._rate = (count - c0) / dt
            if count is not None:
                self._prev_count = (now, count)

    # --- topic source --------------------------------------------------------------------
    def on_message(self, now):
        if self._window_start is None:
            self._window_start = now
        self._msg_count += 1
        self._last_update = now

    def tick(self, now):
        """Call periodically (topic source): compute the rate over rate_window."""
        if self._window_start is None:
            return
        dt = now - self._window_start
        if dt >= self.rate_window:
            self._rate = self._msg_count / dt
            self._msg_count = 0
            self._window_start = now
            if self.min_rate_hz > 0 and self._rate < self.min_rate_hz:
                self._level, self._state = LEVEL_WARN, "LOW_RATE"
            else:
                self._level, self._state = LEVEL_OK, "STREAMING"

    # --- result ------------------------------------------------------------------------
    def snapshot(self, now):
        if self._last_update is None or now - self._last_update > self.stale_timeout:
            return LEVEL_STALE, 0.0, "NO_DATA"
        return self._level, round(self._rate, 2), self._state[:STATE_MAX_LEN]
