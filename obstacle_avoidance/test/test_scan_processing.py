"""Kiem tra doi he lidar -> NED bang so.

Sai he toa do la loi bay thu gan nhu khong phat hien duoc: drone ne SANG
PHIA vat can. Test o day thuan numpy, khong can ROS.

    colcon test --packages-select obstacle_avoidance
"""
import math

import numpy as np
import pytest

from obstacle_avoidance.scan_processing import (ScanConfig, process_scan,
                                                quat_to_rotmat, yaw_to_quat)

N_BEAMS = 360
ANGLE_MIN = -math.pi
INC = math.radians(1.0)


def make_scan(hits):
    """hits: {goc_do: range}. Cac tia khac = inf (khong cham gi)."""
    r = np.full(N_BEAMS, np.inf)
    for deg, rng in hits.items():
        r[int(round(deg + 180)) % N_BEAMS] = rng
    return r


def run(hits, q=yaw_to_quat(0.0), agl=None, **cfg):
    cfg.setdefault('despeckle_tol', 0.0)
    return process_scan(make_scan(hits), ANGLE_MIN, INC, 0.1, 30.0, q, agl, ScanConfig(**cfg))


def only_point(res):
    assert res.points_ne.shape == (1, 2)
    return res.points_ne[0]


def pitch_quat(pitch):
    return (math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0)


def test_rotmat_is_proper_rotation():
    R = quat_to_rotmat((0.3, -0.2, 0.5, 0.7))
    assert np.linalg.det(R) == pytest.approx(1.0)
    assert R.T @ R == pytest.approx(np.eye(3))


@pytest.mark.parametrize('beam_deg, heading_deg, expected_ne', [
    (0.0, 0.0, (5.0, 0.0)),      # truoc mui, mui huong Bac     -> Bac
    (0.0, 90.0, (0.0, 5.0)),     # truoc mui, mui huong Dong    -> Dong
    (90.0, 0.0, (0.0, -5.0)),    # tia +90 (ben TRAI, REP-103)  -> Tay
    (-90.0, 0.0, (0.0, 5.0)),    # tia -90 (ben phai)           -> Dong
    (90.0, 90.0, (5.0, 0.0)),    # ben trai khi mui huong Dong  -> Bac
])
def test_beam_to_ned(beam_deg, heading_deg, expected_ne):
    res = run({beam_deg: 5.0}, q=yaw_to_quat(math.radians(heading_deg)))
    assert only_point(res) == pytest.approx(expected_ne, abs=1e-9)


def test_upside_down_lidar_mirrors_left_right():
    res = run({90.0: 5.0}, upside_down=True)
    assert only_point(res) == pytest.approx((0.0, 5.0), abs=1e-9)   # tia +90 thanh ben phai


def test_mount_yaw_offset():
    # Truc x lidar quay 90 do sang trai: tia 0 cua lidar nhin sang trai drone.
    res = run({0.0: 5.0}, lidar_yaw_offset=math.radians(90.0))
    assert only_point(res) == pytest.approx((0.0, -5.0), abs=1e-9)


def test_tilted_beam_uses_horizontal_distance():
    res = run({0.0: 5.0}, q=pitch_quat(math.radians(-20.0)), agl=10.0)
    assert only_point(res) == pytest.approx((5.0 * math.cos(math.radians(20.0)), 0.0), abs=1e-9)


def test_ground_hit_is_removed_when_low():
    # Chui mui 20 do o 1.5m: tia truoc mui cham dat o ~1.71m phia duoi.
    res = run({0.0: 5.0}, q=pitch_quat(math.radians(-20.0)), agl=1.5)
    assert res.points_ne.shape == (0, 2)
    assert res.ground_hits == 1


def test_self_mask_and_range_limits():
    res = run({0.0: 5.0, 180.0: 0.2, 90.0: 3.0},
              self_mask=[(math.radians(80.0), math.radians(100.0))], min_range=0.3)
    assert only_point(res) == pytest.approx((5.0, 0.0), abs=1e-9)


def test_self_mask_wraps_through_180():
    res = run({179.0: 5.0, -179.0: 5.0, 0.0: 5.0},
              self_mask=[(math.radians(170.0), math.radians(-170.0))])
    assert only_point(res) == pytest.approx((5.0, 0.0), abs=1e-9)


def test_despeckle_drops_single_beam_noise_but_keeps_objects():
    hits = {10.0: 4.0}                                   # nhieu 1 tia
    hits.update({d: 6.0 for d in (-2.0, -1.0, 0.0, 1.0, 2.0)})   # vat that 5 tia
    res = run(hits, despeckle_tol=0.3)
    assert res.points_ne.shape[0] == 5
    assert np.hypot(res.points_ne[:, 0], res.points_ne[:, 1]) == pytest.approx(np.full(5, 6.0))


def test_no_return_beams_are_free_bearings():
    res = run({0.0: 5.0})
    assert res.free_bearings.size == N_BEAMS - 1
