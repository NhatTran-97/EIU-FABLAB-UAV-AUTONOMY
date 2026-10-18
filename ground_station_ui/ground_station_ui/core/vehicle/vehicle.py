"""Vehicle: 1 drone = tap hop cac FactGroup + DANH TINH nguon du lieu.

Danh tinh (cau hinh `vehicle:` trong gcs.yaml):
  * Autopilot = (link, sysid, compid) cua heartbeat autopilot. Chi nhan tu cac link trong
    `autopilot_links` (thu tu = uu tien; mac dinh: moi link tru companion_link), `sysid` co dinh
    neu cau hinh. Doi sang autopilot khac khi: link moi uu tien hon (vd radio that hon SITL),
    hoac autopilot hien tai im lang > REIDENTIFY_S. Khong con "heartbeat dau tien thang mai mai".
  * Companion (Jetson) = message den tu `companion_link`, compid `companion_compid`, sysid trung
    autopilot (hoac `companion_sysid`). Custom message (DRONE_*) tu nguon khac bi bo.

Moi group loc bang is_autopilot(msg, link) / is_companion(msg, link) -- khong tu quyet nguon.
Hien tai 1 drone (khong ho tro nhieu drone).
"""

import time

from ground_station_ui.core.vehicle import groups

REIDENTIFY_S = 5.0


class Vehicle:
    def __init__(self, router, cfg=None):
        cfg = cfg or {}
        self.autopilot_links = list(cfg.get('autopilot_links') or [])   # [] = moi link tru companion
        self.fixed_sysid = cfg.get('sysid')
        self.companion_link = cfg.get('companion_link', 'jetson')
        self.companion_compid = int(cfg.get('companion_compid', 191))
        self.companion_sysid = cfg.get('companion_sysid')
        self.sysid = None
        self.compid = None
        self.link = None
        self._seen = None            # thoi diem heartbeat autopilot cuoi
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
        }
        for g in self.groups.values():
            g.attach(router)

    # --- autopilot ---------------------------------------------------------------------------
    def link_allowed(self, link):
        if link == self.companion_link:
            return False
        return not self.autopilot_links or link in self.autopilot_links

    def _priority(self, link):
        return self.autopilot_links.index(link) if link in self.autopilot_links else len(self.autopilot_links)

    def identify(self, sysid, compid, link, now=None):
        """Goi khi co heartbeat autopilot. True neu (link, sysid, compid) la autopilot dang chon."""
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

    # --- companion (Jetson) ------------------------------------------------------------------
    def is_companion(self, msg, link):
        if link != self.companion_link or msg.get_srcComponent() != self.companion_compid:
            return False
        want = self.companion_sysid if self.companion_sysid is not None else self.sysid
        return want is None or msg.get_srcSystem() == want

    def tick(self, now):
        for g in self.groups.values():
            g.tick(now)
