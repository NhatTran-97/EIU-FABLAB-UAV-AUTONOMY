"""offboard_control node tests: callbacks are called directly and PX4 publishers are intercepted.

Nothing is spun. The test runs in an isolated DDS domain on localhost so it can never talk to a real drone.
"""

import os
import sys

import pytest

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '199'
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'px4_offboard'))

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs.msg')

from geometry_msgs.msg import Point  # noqa: E402
from px4_msgs.msg import VehicleCommand, VehicleControlMode, VehicleLocalPosition  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

import offboard_control as oc  # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Sim:
    def __init__(self):
        self.node = oc.OffboardControlNode()
        self.t = 100.0
        self.node.now_sec = lambda: self.t
        self.commands, self.setpoints, self.flight = [], [], []
        self.node.vehicle_command_pub.publish = lambda m: self.commands.append(m.command)
        self.node.trajectory_setpoint_pub.publish = lambda m: self.setpoints.append(list(m.position))
        self.node.flight_active_pub.publish = lambda m: self.flight.append(m.data)
        pos = VehicleLocalPosition()
        pos.xy_valid = pos.z_valid = True
        pos.x, pos.y, pos.z, pos.heading = 1.0, 2.0, 0.0, 0.5
        self.node.vehicle_local_position_callback(pos)

    def mode(self, armed, offboard):
        m = VehicleControlMode()
        m.flag_armed, m.flag_control_offboard_enabled = armed, offboard
        self.node.vehicle_control_mode_callback(m)

    def ticks(self, n):
        for _ in range(n):
            self.node.timer_callback()
            self.t += self.node.timer_period

    def close(self):
        self.node.destroy_node()


@pytest.fixture
def sim():
    s = Sim()
    yield s
    s.close()


def test_start_warmup_then_offboard_and_arm(sim):
    sim.mode(False, False)
    sim.node.start_request_callback(Bool(data=True))
    assert sim.flight[-1] is True
    sim.ticks(sim.node.warmup_ticks)
    assert sim.commands == []                                          # only streaming during warmup
    sim.ticks(1)
    assert sim.commands == [VehicleCommand.VEHICLE_CMD_DO_SET_MODE, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM]
    assert sim.setpoints[0] == [1.0, 2.0, 0.0]                         # holds where it started


def test_arm_timeout_uses_vehicle_control_mode(sim):
    sim.mode(False, False)
    sim.node.start_request_callback(Bool(data=True))
    sim.ticks(sim.node.warmup_ticks + 1 + int(sim.node.arm_timeout_s * sim.node.setpoint_rate_hz) + 2)
    assert sim.flight[-1] is False and not sim.node.flight_active


def test_disarm_ends_the_flight(sim):
    sim.mode(False, False)
    sim.node.start_request_callback(Bool(data=True))
    sim.ticks(sim.node.warmup_ticks + 1)
    sim.mode(True, True)
    assert sim.node.was_armed and sim.node.offboard_seen
    sim.mode(False, False)                                             # kill switch
    assert sim.flight[-1] is False and not sim.node.flight_active


def test_unchanged_target_is_not_published_twice(sim):
    sim.mode(False, False)
    sim.node.start_request_callback(Bool(data=True))
    sim.ticks(sim.node.warmup_ticks + 1)
    sim.mode(True, True)
    sim.setpoints.clear()
    for _ in range(5):
        sim.node.target_position_callback(Point(x=1.0, y=2.0, z=-3.0))
    assert sim.setpoints == [[1.0, 2.0, -3.0]]                        # timer stream does the rest
