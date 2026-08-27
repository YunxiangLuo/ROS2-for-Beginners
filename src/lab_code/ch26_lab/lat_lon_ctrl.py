
# Windows GBK 控制台输出 Unicode 符号(勾/叉)会抛 UnicodeEncodeError, 统一切换到 UTF-8
import sys as _sys
if hasattr(_sys.stdout, 'reconfigure'):
    try:
        _sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

import time
import math
import sys

from pid_controller import SpeedController, PIDController
from pure_pursuit import PurePursuit
from vehicle_ctrl import VehicleControl, VehicleState


class LatLonController:
    def __init__(self):
        self.target_speed = 10.0

        self.lon_controller = SpeedController(Kp=1.2, Ki=0.15, Kd=0.08)
        self.lat_controller = PurePursuit(wheelbase=2.5, Ld0=2.0, kv=0.4)
        self.pid_lateral = PIDController(Kp=1.5, Ki=0.0, Kd=0.05, output_limit=1.0)

        self.waypoints = self._setup_waypoints()

    def _setup_waypoints(self):
        return [
            (0.0, 0.0), (10.0, 0.0), (20.0, 5.0),
            (30.0, 15.0), (35.0, 30.0), (30.0, 45.0),
            (20.0, 55.0), (10.0, 60.0), (0.0, 60.0),
            (-10.0, 50.0), (-15.0, 35.0), (-10.0, 20.0),
            (0.0, 10.0), (0.0, 0.0),
        ]

    def compute_control(self, state, dt):
        throttle, brake = self.lon_controller.compute(
            self.target_speed, state.speed, dt
        )

        steer = self.lat_controller.compute_steering(
            (state.position[0], state.position[1]),
            state.yaw,
            self.waypoints,
            state.speed,
        )

        lateral_feedback = self.pid_lateral.compute(0.0, self._compute_lateral_error(state))
        steer += 0.1 * lateral_feedback
        steer = max(-1.0, min(steer, 1.0))

        return VehicleControl(throttle=throttle, brake=brake, steer=steer)

    def _compute_lateral_error(self, state):
        nearest_idx = self.lat_controller.find_nearest_waypoint(
            (state.position[0], state.position[1]), self.waypoints
        )
        wp = self.waypoints[nearest_idx]
        dx = state.position[0] - wp[0]
        dy = state.position[1] - wp[1]
        return math.hypot(dx, dy)

    def run_simulation(self, duration=30.0, dt=0.05):
        state = VehicleState()
        steps = int(duration / dt)

        print("=" * 70)
        print("车辆纵横向联合控制仿真")
        print(f"目标速度: {self.target_speed} m/s")
        print(f"路径点数量: {len(self.waypoints)}")
        print(f"仿真时长: {duration}s, 步长: {dt}s")
        print("-" * 70)

        speed_errors = []
        lateral_errors = []

        for step in range(steps):
            t = step * dt

            control = self.compute_control(state, dt)
            state.update(dt, control.throttle, control.brake, control.steer)

            speed_error = abs(self.target_speed - state.speed)
            lateral_error = self._compute_lateral_error(state)
            speed_errors.append(speed_error)
            lateral_errors.append(lateral_error)

            if step % 100 == 0:
                print(
                    f"[{t:6.2f}s] "
                    f"v={state.speed:6.2f} "
                    f"v_err={speed_error:6.3f} "
                    f"lat_err={lateral_error:6.3f} "
                    f"T={control.throttle:.2f} "
                    f"B={control.brake:.2f} "
                    f"S={control.steer:.3f}"
                )

            if step > 50 and speed_error < 0.1:
                self._adjust_gains_for_convergence()

        print("-" * 70)
        self._print_summary(speed_errors, lateral_errors)

    def _adjust_gains_for_convergence(self):
        pass

    def _print_summary(self, speed_errors, lateral_errors):
        avg_speed_err = sum(speed_errors) / len(speed_errors)
        max_speed_err = max(speed_errors)
        avg_lat_err = sum(lateral_errors) / len(lateral_errors)
        max_lat_err = max(lateral_errors)

        print("=" * 70)
        print("仿真结果统计")
        print(f"  平均速度误差: {avg_speed_err:.4f} m/s")
        print(f"  最大速度误差: {max_speed_err:.4f} m/s")
        print(f"  平均横向偏差: {avg_lat_err:.4f} m")
        print(f"  最大横向偏差: {max_lat_err:.4f} m")
        print()

        if avg_speed_err < 0.5:
            print("✓ 纵向控制达标 (平均速度误差 < 0.5 m/s)")
        else:
            print("✗ 纵向控制未达标")

        if avg_lat_err < 0.3:
            print("✓ 横向控制达标 (平均横向偏差 < 0.3 m)")
        else:
            print("✗ 横向控制未达标")
        print("=" * 70)


def main():
    controller = LatLonController()
    controller.run_simulation(duration=60.0, dt=0.05)


if __name__ == "__main__":
    main()
