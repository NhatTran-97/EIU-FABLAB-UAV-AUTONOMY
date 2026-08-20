"""Kiem tra phep bien doi he toa do bang so.

Sai he toa do la loai loi bay thu gan nhu khong phat hien duoc: so trong hop
ly, drone chi lech sai huong. geometry.py thuan numpy nen test duoc o day,
khong can ROS, khong can drone, khong can camera.

    colcon test --packages-select aruco_detection
"""
import math

import numpy as np
import pytest

from aruco_detection.geometry import (CameraExtrinsic, R_FLU_FROM_CAM_DOWN,
                                      euler_to_rotmat, marker_relative_enu,
                                      rotmat_to_quat)

LEVEL = (0.0, 0.0, 0.0, 1.0)      # quaternion don vi: drone thang bang


def test_mount_matrix_is_a_proper_rotation():
    assert np.linalg.det(R_FLU_FROM_CAM_DOWN) == pytest.approx(1.0)
    assert R_FLU_FROM_CAM_DOWN.T @ R_FLU_FROM_CAM_DOWN == pytest.approx(np.eye(3))


@pytest.mark.parametrize("tvec, expected", [
    ((0.0, 0.0, 3.0), (0.0,  0.0, -3.0)),   # thang duoi 3m
    ((1.0, 0.0, 3.0), (0.0, -1.0, -3.0)),   # lech phai 1m  -> Bac am
    ((0.0, -1.0, 3.0), (1.0, 0.0, -3.0)),   # phia mui 1m   -> Dong duong
])
def test_axes_map_correctly_when_level(tvec, expected):
    p, _ = marker_relative_enu(tvec, LEVEL, CameraExtrinsic())
    assert p == pytest.approx(np.array(expected), abs=1e-9)


def test_tilt_does_not_move_a_stationary_marker():
    """Nghieng drone khi marker nam yen thang duoi -> lech ngang phai bang 0.

    Day la ly do ton tai cua marker_localizer_node: tvec tho ghep attitude
    vao vi tri, bo dieu khien se duoi theo mot sai so khong co that.
    """
    R = euler_to_rotmat(0.0, 10.0, 0.0)          # chuc mui xuong 10 do
    p_enu_true = np.array([0.0, 0.0, -3.0])      # marker o thang duoi 3 m
    p_cam = np.linalg.inv(R_FLU_FROM_CAM_DOWN) @ (R.T @ p_enu_true)

    # Neu dua thang tvec vao bo dieu khien: marker "dich" 3*sin(10deg) = 52cm.
    # La sin chu khong phai tan -- marker o cach 3m THANG DUNG, canh huyen la
    # khoang cach xien ma camera nhin thay.
    apparent = 3.0 * math.sin(math.radians(10.0))
    assert math.hypot(p_cam[0], p_cam[1]) == pytest.approx(apparent, abs=1e-9)
    assert apparent == pytest.approx(0.521, abs=1e-3)

    # Sau khi khu nghieng bang attitude
    p, _ = marker_relative_enu(p_cam, rotmat_to_quat(R), CameraExtrinsic())
    assert p == pytest.approx(p_enu_true, abs=1e-9)


def test_mount_error_biases_position():
    """Lech goc lap 2 do o 5 m -> ~17 cm sai so he thong, khong bo loc nao khu duoc."""
    perfect = CameraExtrinsic()
    tilted = CameraExtrinsic(mount_pitch=2.0)
    tvec = (0.0, 0.0, 5.0)
    p_ok, _ = marker_relative_enu(tvec, LEVEL, perfect)
    p_bad, _ = marker_relative_enu(tvec, LEVEL, tilted)
    assert np.linalg.norm(p_bad[:2] - p_ok[:2]) == pytest.approx(
        5.0 * math.sin(math.radians(2.0)), abs=1e-9)      # = 17.4 cm


def test_camera_offset_is_applied_in_body_axes():
    ex = CameraExtrinsic(offset=(0.10, 0.0, -0.05))
    p, _ = marker_relative_enu((0.0, 0.0, 3.0), LEVEL, ex)
    assert p == pytest.approx(np.array([0.10, 0.0, -3.05]), abs=1e-9)


def test_marker_yaw_is_not_drone_yaw():
    """Huong marker phai doc lap voi huong drone.

    Bug da tung co: lay yaw tu mot minh R_enu_from_flu, tuc la publish huong
    cua DRONE ra topic mang ten huong cua MARKER.
    """
    from aruco_detection.geometry import marker_yaw_enu

    ex = CameraExtrinsic()
    R_cam_from_marker = np.eye(3)          # marker khong xoay so voi camera

    # Drone quay 40 do, marker giu nguyen -> yaw marker phai DOI theo drone
    R_drone = euler_to_rotmat(0.0, 0.0, 40.0)
    yaw_a = marker_yaw_enu(R_cam_from_marker, rotmat_to_quat(R_drone), ex)

    # Drone giu nguyen, marker xoay 40 do quanh truc quang hoc
    R_marker = euler_to_rotmat(0.0, 0.0, 40.0)
    yaw_b = marker_yaw_enu(R_marker, LEVEL, ex)

    # Hai truong hop khac nhau -> ket qua phai khac nhau
    assert yaw_a != pytest.approx(yaw_b, abs=1e-6)
