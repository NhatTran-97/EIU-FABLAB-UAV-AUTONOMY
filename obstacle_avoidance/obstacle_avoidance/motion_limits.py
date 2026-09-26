"""Gioi han toc do theo quang duong dung -- thuan python.

Drone phai dung kip truoc khi cham bong bong an toan:

    v * T + v^2 / (2 * a)  <=  D

    T: tong do tre tu luc lidar thay vat can toi luc PX4 bat dau phanh (s)
    a: gia toc phanh ngang (m/s^2), <= MPC_ACC_HOR
    D: quang trong truoc khi cham bong bong (m)

Giai theo v: v = a * (-T + sqrt(T^2 + 2 D / a)).
T lay tu do tre DO DUOC luc chay, nen lidar cham hon / tre hon thi drone tu
bay cham lai, khong can chinh tay.
"""
import math


def stopping_distance(speed: float, latency: float, decel: float) -> float:
    return speed * latency + speed * speed / (2.0 * decel)


def stopping_speed(distance: float, latency: float, decel: float) -> float:
    """Toc do lon nhat con dung kip trong `distance` m."""
    if distance <= 0.0:
        return 0.0
    if math.isinf(distance):
        return math.inf
    return decel * (-latency + math.sqrt(latency * latency + 2.0 * distance / decel))
