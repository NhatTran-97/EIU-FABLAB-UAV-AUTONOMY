"""Vehicle: one drone = its FactGroups + identification of the data sources (gcs.yaml `vehicle:`).

Autopilot = (link, sysid, compid) of the autopilot HEARTBEAT, accepted only on `autopilot_links`
  (order = priority; empty = every link except the companion's). Switches when a higher-priority link
  appears or the current autopilot is silent for REIDENTIFY_S.
Companion (Jetson) = messages on `companion_links` with compid `companion_compid` and the autopilot's
  sysid (or `companion_sysid`). DRONE_* messages from other sources are dropped.

Groups filter by source with is_autopilot(msg, link) / is_companion(msg, link). One drone only.
"""

import time

from ground_station_ui.core.vehicle import groups

REIDENTIFY_S = 5.0


class Vehicle:
    def __init__(self, router, cfg=None):
        cfg = cfg or {}
        self.autopilot_links = list(cfg.get('autopilot_links') or [])   # [] = every link except the companion's
        self.fixed_sysid = cfg.get('sysid')
        # companion_link: link of the latest companion heartbeat, used for commands / waypoints
        links = cfg.get('companion_links') or [cfg.get('companion_link', 'jetson')]
        self.companion_links = [str(ln) for ln in links]
        self.companion_link = self.companion_links[0]
        self.companion_compid = int(cfg.get('companion_compid', 191))
        self.companion_sysid = cfg.get('companion_sysid')
        self.sysid = None
        self.compid = None
        self.link = None
        self._seen = None            # time of the last autopilot heartbeat (monotonic)
        self.on_identity_change = []
        self.groups = {
            'status': groups.StatusGroup(self),
            'gps': groups.GpsGroup(self),
            'home': groups.HomeGroup(self),
            'battery': groups.BatteryGroup(self),
            'sensors': groups.SensorsGroup(self),
            'flight': groups.FlightGroup(self),
            'attitude': groups.AttitudeGroup(self),
            'messages': groups.MessagesGroup(self),
            'companion': groups.CompanionGroup(self),
            'localPose': groups.LocalPoseGroup(self),
            'gimbal': groups.GimbalGroup(self),
            'track': groups.TrackGroup(self, cfg.get('track_dir')),
            'missionProgress': groups.MissionProgressGroup(self),
        }
        for g in self.groups.values():
            g.attach(router)

    # --- autopilot ---------------------------------------------------------------------------
    def link_allowed(self, link):
        if link in self.companion_links:
            return False
        return not self.autopilot_links or link in self.autopilot_links

    def _priority(self, link):
        return self.autopilot_links.index(link) if link in self.autopilot_links else len(self.autopilot_links)

    def identify(self, sysid, compid, link, now=None):
        """Handle an autopilot HEARTBEAT. True if (link, sysid, compid) is the selected autopilot."""
        now = time.monotonic() if now is None else now
        if not self.link_allowed(link) or (self.fixed_sysid is not None and sysid != self.fixed_sysid):
            return False
        key = (link, sysid, compid)
        if self.sysid is not None and key == (self.link, self.sysid, self.compid):
            self._seen = now
            return True
        switch = (self.sysid is None
                  or now - (self._seen or 0) > REIDENTIFY_S
                  or (link != self.link and self._priority(link) < self._priority(self.link)))
        if not switch:
            return False
        changed = self.sysid is not None
        self.link, self.sysid, self.compid, self._seen = link, sysid, compid, now
        if changed:
            for cb in self.on_identity_change:
                cb()
        return True

    def is_autopilot(self, msg, link=None):
        return (self.sysid is not None and msg.get_srcSystem() == self.sysid
                and msg.get_srcComponent() == self.compid
                and (link is None or link == self.link))

    # --- global position: autopilot preferred, companion as fallback ------------------------
    GLOBAL_FALLBACK_S = 2.0

    def accept_global(self, msg, link):
        """Accept GLOBAL_POSITION_INT always from the autopilot, and from the companion only when the
        autopilot's GLOBAL_POSITION_INT is stale > GLOBAL_FALLBACK_S."""
        now = time.monotonic()
        if self.is_autopilot(msg, link):
            self._ap_global_at = now
            return True
        if self.is_companion(msg, link):
            t = getattr(self, '_ap_global_at', None)
            return t is None or now - t > self.GLOBAL_FALLBACK_S
        return False

    # --- companion (Jetson) ------------------------------------------------------------------
    def is_companion(self, msg, link):
        if link not in self.companion_links or msg.get_srcComponent() != self.companion_compid:
            return False
        want = self.companion_sysid if self.companion_sysid is not None else self.sysid
        return want is None or msg.get_srcSystem() == want

    def tick(self, now):
        for g in self.groups.values():
            g.tick(now)
