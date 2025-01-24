from typing import Tuple

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion


def euler_from_quaternion(quaternion: Quaternion) -> Tuple[float, float, float]:
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def laser_scan_to_polar(message: LaserScan) -> np.ndarray:
    N = len(message.ranges)
    array = np.zeros((N, 2))
    for i in range(len(message.ranges)):
        angle = i * message.angle_increment
        if message.ranges[i] > message.range_max:
            distance = message.range_max
        elif message.ranges[i] < message.range_min:
            distance = message.range_min
        else:
            distance = message.ranges[i]
        array[i, 0] = distance
        array[i, 1] = angle
    return array


def polar_to_cartesian(coordinates: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    N = coordinates.shape[0]
    array = np.zeros((N, 2))
    array[:, 0] = x + coordinates[:, 0] * np.cos(coordinates[:, 1] + theta)
    array[:, 1] = y + coordinates[:, 0] * np.sin(coordinates[:, 1] + theta)
    return array
