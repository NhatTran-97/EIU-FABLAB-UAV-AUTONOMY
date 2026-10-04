"""Vehicle: 1 drone = tap hop cac FactGroup, nhan dien autopilot theo (sysid, compid).

Hien tai 1 drone. Them drone sau nay: tao nhieu Vehicle theo sysid (VehicleManager),
cac group khong can doi vi da loc theo vehicle.is_autopilot().
"""

from ground_station_ui.core.vehicle import groups


class Vehicle:
    def __init__(self, router):
        self.sysid = None
        self.compid = None
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

    def identify(self, sysid, compid):
        """Autopilot dau tien gui heartbeat duoc chon (1 drone)."""
        if self.sysid is None:
            self.sysid, self.compid = sysid, compid

    def is_autopilot(self, msg):
        return (self.sysid is not None and msg.get_srcSystem() == self.sysid
                and msg.get_srcComponent() == self.compid)

    def tick(self, now):
        for g in self.groups.values():
            g.tick(now)
