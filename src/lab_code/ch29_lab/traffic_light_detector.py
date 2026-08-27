#!/usr/bin/env python3

"""
第29章 练习29.1 - 交通灯检测与响应

检测 CARLA 仿真器中的红绿灯状态，计算车辆到停止线的距离，
并根据灯色输出油门/刹车控制指令。

运行方式:
    python traffic_light_detector.py
"""

import carla
import math
import time
import argparse


class TrafficLightDetector:
    """交通灯检测与响应控制器"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.world = vehicle.get_world()

        # 参数
        self.STOP_LINE_MARGIN = 3.0      # 停止线余量 (m)
        self.DECEL_DISTANCE = 15.0       # 开始减速距离 (m)
        self.YELLOW_TIME_THRESHOLD = 3.0  # 黄灯剩余时间阈值 (s)

        # 状态
        self.last_traffic_light = None
        self.is_stopped = False
        self.stop_start_time = None
        self.total_stop_time = 0.0

        # 统计
        self.stats = {
            'red_stops': 0,
            'green_passes': 0,
            'total_time': 0.0,
        }

    def get_traffic_light_info(self):
        """获取当前影响车辆的交通灯信息"""
        traffic_light = self.vehicle.get_traffic_light()
        info = {
            'exists': False,
            'state': 'NONE',
            'distance': float('inf'),
            'stop_location': None,
            'light_location': None,
            'time_until_change': -1.0,
        }

        if traffic_light is None:
            return info

        info['exists'] = True
        info['state'] = str(traffic_light.get_state()).split('.')[-1]
        info['light_location'] = traffic_light.get_location()

        # 获取停止线位置
        stop_waypoints = traffic_light.get_stop_waypoints()
        if stop_waypoints:
            stop_loc = stop_waypoints[0].transform.location
            info['stop_location'] = stop_loc
            vehicle_loc = self.vehicle.get_location()
            info['distance'] = vehicle_loc.distance(stop_loc)

        # 获取灯色剩余时间
        try:
            info['time_until_change'] = \
                traffic_light.get_green_time()  # 实际API视版本而定
        except Exception:
            # 某些版本不支持此API
            pass

        return info

    def should_stop(self, tl_info):
        """判断是否应该在交通灯前停止"""
        if not tl_info['exists']:
            return False

        state = tl_info['state']
        distance = tl_info['distance']

        # 绿灯 → 正常通过
        if state == 'Green':
            return False

        # 红灯或黄灯
        red_or_yellow = state in ('Red', 'Yellow')

        # 已经过了停止线 → 不停
        if distance < 0.5:
            return False

        # 距离在停止线余量内 → 强制停止
        if distance < self.STOP_LINE_MARGIN:
            return True

        # 距离在减速范围内 → 根据灯色判断
        if distance < self.DECEL_DISTANCE and red_or_yellow:
            return True

        # 红灯且距离较远 → 准备停止
        if state == 'Red' and distance < 30.0:
            return True

        return False

    def compute_control(self, tl_info):
        """根据交通灯计算控制指令"""
        cmd = carla.VehicleControl()
        cmd.throttle = 0.3
        cmd.brake = 0.0
        cmd.steer = 0.0
        cmd.hand_brake = False

        if not tl_info['exists']:
            # 无交通灯，正常行驶
            cmd.throttle = 0.35
            self.is_stopped = False
            return cmd

        state = tl_info['state']
        distance = tl_info['distance']

        if state == 'Green':
            # 绿灯：保持巡航
            cmd.throttle = 0.35
            self.is_stopped = False
            self.stats['green_passes'] += 1

        elif state == 'Red':
            if distance < self.STOP_LINE_MARGIN:
                # 到达停止线：刹停
                cmd.throttle = 0.0
                cmd.brake = 1.0
                self.is_stopped = True
            elif distance < self.DECEL_DISTANCE:
                # 接近中：线性减速
                brake_ratio = 1.0 - (distance / self.DECEL_DISTANCE)
                cmd.throttle = 0.0
                cmd.brake = min(brake_ratio * 0.8, 0.8)
            else:
                # 远处看到红灯：松油门滑行
                cmd.throttle = 0.05
                cmd.brake = 0.0

        elif state == 'Yellow':
            if distance < self.STOP_LINE_MARGIN:
                # 已到停止线：安全停止
                cmd.throttle = 0.0
                cmd.brake = 0.8
                self.is_stopped = True
            elif distance < self.DECEL_DISTANCE:
                # 临近黄灯：减速停止
                cmd.throttle = 0.0
                cmd.brake = 0.5
            else:
                # 远处黄灯：视剩余时间决定
                cmd.throttle = 0.2
                cmd.brake = 0.0

        return cmd

    def format_state_string(self, tl_info):
        """格式化输出状态字符串"""
        if not tl_info['exists']:
            return "无交通灯"

        state = tl_info['state']
        distance = tl_info['distance']

        # 灯色符号
        symbols = {'Red': '🔴', 'Yellow': '🟡', 'Green': '🟢', 'Off': '⚫'}
        symbol = symbols.get(state, '❓')

        # 状态文本
        if state == 'Red':
            if distance < self.STOP_LINE_MARGIN:
                action = "已停车"
            elif distance < self.DECEL_DISTANCE:
                action = "减速中"
            else:
                action = "准备停"
        elif state == 'Yellow':
            action = "注意"
        elif state == 'Green':
            action = "通过"
        else:
            action = "未知"

        return (f"{symbol} {state:6s} | "
                f"距离: {distance:5.1f}m | "
                f"{action}")

    def run(self, duration=60.0):
        """主运行循环"""
        print("=" * 60)
        print("交通灯检测与响应演示")
        print(f"运行时长: {duration} 秒")
        print("=" * 60)
        print()

        start_time = time.time()
        frame_count = 0

        try:
            while time.time() - start_time < duration:
                # 获取交通灯信息
                tl_info = self.get_traffic_light_info()

                # 计算控制
                cmd = self.compute_control(tl_info)
                self.vehicle.apply_control(cmd)

                # 记录统计
                if tl_info['exists'] and tl_info['state'] == 'Red' \
                   and self.is_stopped:
                    if self.stop_start_time is None:
                        self.stop_start_time = time.time()
                        self.stats['red_stops'] += 1
                    else:
                        self.total_stop_time += 0.05

                if not self.is_stopped:
                    self.stop_start_time = None

                # 输出信息（每5帧一次）
                frame_count += 1
                if frame_count % 5 == 0:
                    status = self.format_state_string(tl_info)
                    v = self.vehicle.get_velocity()
                    speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
                    print(f"[{time.time()-start_time:5.1f}s] "
                          f"{status} | "
                          f"车速: {speed:5.1f} km/h | "
                          f"油门: {cmd.throttle:.2f} | "
                          f"刹车: {cmd.brake:.2f}")

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n用户中断")

        finally:
            # 输出统计
            elapsed = time.time() - start_time
            print()
            print("=" * 60)
            print("运行统计")
            print("=" * 60)
            print(f"运行时间: {elapsed:.1f} 秒")
            print(f"红灯停止次数: {self.stats['red_stops']}")
            print(f"绿灯通过次数: {self.stats['green_passes']}")
            print(f"红灯前总停车时间: {self.total_stop_time:.1f} 秒")

            # 停止车辆
            cmd = carla.VehicleControl()
            cmd.throttle = 0.0
            cmd.brake = 1.0
            self.vehicle.apply_control(cmd)


def main():
    parser = argparse.ArgumentParser(
        description='CARLA 交通灯检测与响应')
    parser.add_argument('--host', default='localhost',
                       help='CARLA 主机地址')
    parser.add_argument('--port', type=int, default=2000,
                       help='CARLA 端口')
    parser.add_argument('--duration', type=int, default=60,
                       help='运行时长 (秒)')
    args = parser.parse_args()

    # 连接 CARLA
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    # 设置同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 清理并生成自车
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    spawn_points = world.get_map().get_spawn_points()

    # 选择一个有交通灯的路口附近的出生点
    # 优先选择索引靠前的出生点（通常在路口附近）
    spawn_point = spawn_points[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    if vehicle is None:
        print("错误: 生成车辆失败")
        return

    print(f"自车生成在: ({spawn_point.location.x:.1f}, "
          f"{spawn_point.location.y:.1f})")

    # 设置观察者
    spectator = world.get_spectator()

    # 创建交通灯检测器
    detector = TrafficLightDetector(vehicle)

    try:
        while True:
            world.tick()

            # 更新视角
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location + carla.Location(z=40),
                carla.Rotation(pitch=-90)
            ))

            # 运行检测
            tl_info = detector.get_traffic_light_info()
            cmd = detector.compute_control(tl_info)
            vehicle.apply_control(cmd)

            # 输出
            v = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
            status = detector.format_state_string(tl_info)
            print(f"\r{status} | "
                  f"车速: {speed:5.1f} km/h", end='')

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        settings.synchronous_mode = False
        world.apply_settings(settings)
        vehicle.destroy()
        print("清理完成")


if __name__ == '__main__':
    main()
