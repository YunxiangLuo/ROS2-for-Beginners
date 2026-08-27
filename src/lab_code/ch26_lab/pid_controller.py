
# Windows GBK 控制台输出 Unicode 符号(勾/叉)会抛 UnicodeEncodeError, 统一切换到 UTF-8
import sys as _sys
if hasattr(_sys.stdout, 'reconfigure'):
    try:
        _sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

import time
import math


class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.05, output_limit=1.0, integral_limit=10.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.alpha = 0.1  # 低通滤波系数

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def compute(self, target, current):
        error = target - current

        self.integral += error * self.dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        raw_derivative = (error - self.prev_error) / self.dt if self.dt > 1e-6 else 0.0
        derivative = self.alpha * raw_derivative + (1.0 - self.alpha) * self.prev_derivative

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(-self.output_limit, min(output, self.output_limit))

        self.prev_error = error
        self.prev_derivative = derivative

        return output

    def throttle_brake_map(self, control_output):
        if control_output >= 0.0:
            throttle = min(control_output, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-control_output, 1.0)
        return throttle, brake


class SpeedController:
    def __init__(self, Kp=1.0, Ki=0.2, Kd=0.1, dt=0.05):
        self.pid = PIDController(Kp, Ki, Kd, dt)

    def compute(self, target_speed, current_speed, dt=None):
        if dt is not None:
            self.pid.dt = dt
        control = self.pid.compute(target_speed, current_speed)
        return self.pid.throttle_brake_map(control)

    def reset(self):
        self.pid.reset()


def unit_test():
    dt = 0.05
    controller = PIDController(Kp=2.0, Ki=0.5, Kd=0.1, dt=dt)

    target = 10.0
    current = 0.0
    total_time = 10.0
    steps = int(total_time / dt)

    print("=" * 60)
    print("PID控制器单元测试: 阶跃响应 (0 -> 10.0)")
    print(f"Kp={controller.Kp}, Ki={controller.Ki}, Kd={controller.Kd}")
    print("-" * 60)
    print(f"{'时间(s)':>8} {'目标':>8} {'当前':>8} {'误差':>8} {'输出':>8}")
    print("-" * 60)

    for i in range(steps):
        t = i * dt
        control = controller.compute(target, current)
        current += control * dt
        current = max(0.0, current)

        if i % 20 == 0:
            error = target - current
            print(f"{t:8.2f} {target:8.2f} {current:8.2f} {error:8.3f} {control:8.3f}")

        if t >= 2.0:
            error = abs(target - current)
            if error < 0.1:
                print("-" * 60)
                print(f"✓ 稳态误差达标: {error:.4f} (目标 < 0.1)")
                break

    print("=" * 60)

    overshoot = max(0, (current - target) / target * 100)
    print(f"超调量: {overshoot:.1f}%")

    return True


if __name__ == "__main__":
    import sys
    if "--test" in sys.argv:
        unit_test()
    else:
        print("PIDController 使用示例:")
        print("  pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)")
        print("  output = pid.compute(target=10.0, current=5.0)")
