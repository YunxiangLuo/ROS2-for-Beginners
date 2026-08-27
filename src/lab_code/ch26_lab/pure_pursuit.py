
# Windows GBK 控制台输出 Unicode 符号(勾/叉)会抛 UnicodeEncodeError, 统一切换到 UTF-8
import sys as _sys
if hasattr(_sys.stdout, 'reconfigure'):
    try:
        _sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

import math
import numpy as np


class PurePursuit:
    def __init__(self, wheelbase=2.5, Ld0=2.0, kv=0.5, min_Ld=1.0, max_Ld=15.0):
        self.wheelbase = wheelbase
        self.Ld0 = Ld0
        self.kv = kv
        self.min_Ld = min_Ld
        self.max_Ld = max_Ld

    def compute_lookahead_distance(self, speed):
        Ld = self.Ld0 + self.kv * speed
        return max(self.min_Ld, min(Ld, self.max_Ld))

    def find_nearest_waypoint(self, vehicle_pos, waypoints):
        min_dist = float('inf')
        nearest_idx = 0
        for i, wp in enumerate(waypoints):
            dx = wp[0] - vehicle_pos[0]
            dy = wp[1] - vehicle_pos[1]
            dist = dx * dx + dy * dy
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        return nearest_idx

    def find_lookahead_point(self, waypoints, vehicle_pos, nearest_idx, Ld):
        n = len(waypoints)
        for i in range(nearest_idx, n):
            dx = waypoints[i][0] - vehicle_pos[0]
            dy = waypoints[i][1] - vehicle_pos[1]
            dist = math.hypot(dx, dy)
            if dist >= Ld:
                return waypoints[i], i
        return waypoints[-1], n - 1

    def compute_steering(self, vehicle_pos, vehicle_yaw, waypoints, speed=0.0):
        if len(waypoints) < 2:
            return 0.0

        Ld = self.compute_lookahead_distance(speed)
        nearest_idx = self.find_nearest_waypoint(vehicle_pos, waypoints)

        target_wp, _ = self.find_lookahead_point(
            waypoints, vehicle_pos, nearest_idx, Ld
        )

        dx = target_wp[0] - vehicle_pos[0]
        dy = target_wp[1] - vehicle_pos[1]

        alpha = math.atan2(dy, dx) - vehicle_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        if abs(alpha) < 1e-6:
            return 0.0

        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), Ld)
        steer = max(-1.0, min(steer, 1.0))

        return steer


def unit_test():
    pp = PurePursuit(wheelbase=2.5, Ld0=2.0, kv=0.5)

    waypoints = [(i * 5.0, 0.0) for i in range(20)]

    vehicle_pos = (0.0, 0.5)
    vehicle_yaw = 0.0
    speed = 5.0

    print("=" * 60)
    print("Pure Pursuit 单元测试")
    print(f"轴距: {pp.wheelbase}, Ld0: {pp.Ld0}, kv: {pp.kv}")
    print("-" * 60)
    print(f"{'步数':>6} {'位置x':>8} {'位置y':>8} {'航向':>8} {'转向角':>10} {'Ld':>8}")
    print("-" * 60)

    pos_x, pos_y = vehicle_pos
    yaw = vehicle_yaw

    for step in range(50):
        steer = pp.compute_steering(
            (pos_x, pos_y), yaw, waypoints, speed
        )

        pos_x += speed * 0.05 * math.cos(yaw)
        pos_y += speed * 0.05 * math.sin(yaw)
        yaw += steer * 0.05
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        if step % 5 == 0:
            Ld = pp.compute_lookahead_distance(speed)
            nearest = pp.find_nearest_waypoint((pos_x, pos_y), waypoints)
            dx = waypoints[nearest][0] - pos_x
            lateral_error = abs(pos_y)
            print(f"{step:6d} {pos_x:8.2f} {pos_y:8.3f} {yaw:8.3f} {steer:10.3f} {Ld:8.2f}")

    print("-" * 60)
    lateral_error = abs(pos_y)
    if lateral_error < 0.2:
        print(f"✓ 直线跟踪达标: 横向偏差 {lateral_error:.4f}m (目标 < 0.2m)")
    else:
        print(f"✗ 直线跟踪未达标: 横向偏差 {lateral_error:.4f}m")
    print("=" * 60)

    return True


if __name__ == "__main__":
    import sys
    if "--test" in sys.argv:
        unit_test()
    else:
        print("PurePursuit 使用示例:")
        print("  pp = PurePursuit(wheelbase=2.5)")
        print("  steer = pp.compute_steering(vehicle_pos, yaw, waypoints, speed)")
