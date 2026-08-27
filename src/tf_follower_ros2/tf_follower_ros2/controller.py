import math
from typing import Tuple


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def compute_follow_command(
    x: float,
    y: float,
    stop_distance: float,
    linear_gain: float,
    angular_gain: float,
    max_linear_speed: float,
    max_angular_speed: float,
) -> Tuple[float, float]:
    distance = math.hypot(x, y)
    if distance <= stop_distance:
        return (0.0, 0.0)

    heading_error = math.atan2(y, x)
    linear_x = clamp(distance * linear_gain, 0.0, max_linear_speed)
    angular_z = clamp(heading_error * angular_gain, -max_angular_speed, max_angular_speed)
    return (linear_x, angular_z)
