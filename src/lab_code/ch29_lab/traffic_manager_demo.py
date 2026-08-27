#!/usr/bin/env python3

"""
第29章 练习29.3 - CARLA Traffic Manager 配置演示

使用 CARLA Traffic Manager 控制多辆 AI 车辆，
展示不同驾驶风格（激进/普通/保守）的配置方法。

运行方式:
    python traffic_manager_demo.py --vehicles 20 --mode normal
    python traffic_manager_demo.py --vehicles 30 --mode aggressive
    python traffic_manager_demo.py --vehicles 50 --mode mixed
"""

import carla
import math
import random
import time
import argparse
from collections import defaultdict


class TrafficManagerDemo:
    """Traffic Manager 配置演示类"""

    def __init__(self, client, tm_port=8000):
        self.client = client
        self.world = client.get_world()
        self.tm = client.get_trafficmanager(tm_port)

        self.vehicles = []
        self.vehicle_configs = {}
        self.stats = defaultdict(lambda: {'speed_sum': 0.0, 'count': 0})

    def spawn_vehicles(self, num_vehicles):
        """在随机出生点生成车辆"""
        blueprint_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()

        if len(spawn_points) < num_vehicles:
            print(f"警告: 只有 {len(spawn_points)} 个出生点，"
                  f"调整为 {len(spawn_points)}")
            num_vehicles = len(spawn_points)

        vehicle_bps = blueprint_library.filter('vehicle.*')
        selected_points = random.sample(spawn_points, num_vehicles)

        for i, sp in enumerate(selected_points):
            bp = random.choice(vehicle_bps)

            # 随机颜色
            if bp.has_attribute('color'):
                color = random.choice(
                    bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', color)

            bp.set_attribute('role_name', f'background_{i}')

            vehicle = self.world.spawn_actor(bp, sp)
            if vehicle is not None:
                self.vehicles.append(vehicle)
                print(f"  [{i+1}/{num_vehicles}] {bp.id}")

        print(f"成功生成 {len(self.vehicles)} 辆车")
        return self.vehicles

    def configure_aggressive(self, vehicle):
        """配置激进驾驶风格"""
        self.tm.set_desired_speed(vehicle, random.uniform(60, 90))
        self.tm.set_distance_to_leading_vehicle(
            vehicle, random.uniform(1.0, 2.5))
        self.tm.set_ignore_traffic_light_percentage(vehicle, 20)
        self.tm.set_ignore_signs_percentage(vehicle, 15)
        self.tm.set_lane_change_behavior(
            vehicle, random.choice([0, 1, 2]))
        self.tm.set_force_lane_change(vehicle, True)
        return 'aggressive'

    def configure_normal(self, vehicle):
        """配置普通驾驶风格"""
        self.tm.set_desired_speed(vehicle, random.uniform(40, 65))
        self.tm.set_distance_to_leading_vehicle(
            vehicle, random.uniform(3.0, 5.0))
        self.tm.set_ignore_traffic_light_percentage(vehicle, 5)
        self.tm.set_ignore_signs_percentage(vehicle, 2)
        self.tm.set_lane_change_behavior(
            vehicle, random.choice([0, 1]))
        return 'normal'

    def configure_conservative(self, vehicle):
        """配置保守驾驶风格"""
        self.tm.set_desired_speed(vehicle, random.uniform(25, 45))
        self.tm.set_distance_to_leading_vehicle(
            vehicle, random.uniform(6.0, 10.0))
        self.tm.set_ignore_traffic_light_percentage(vehicle, 0)
        self.tm.set_ignore_signs_percentage(vehicle, 0)
        self.tm.set_lane_change_behavior(vehicle, 0)
        return 'conservative'

    def configure_traffic(self, mode='normal'):
        """配置所有车辆的 Traffic Manager 参数"""
        config_functions = {
            'aggressive': self.configure_aggressive,
            'normal': self.configure_normal,
            'conservative': self.configure_conservative,
        }

        # 全局设置
        self.tm.set_global_distance_to_leading_vehicle(3.0)
        self.tm.set_random_device_seed(42)
        self.tm.set_synchronous_mode(True)

        for vehicle in self.vehicles:
            vehicle.set_autopilot(True, self.tm.get_port())

            if mode == 'mixed':
                # 混合模式：随机分配
                chosen = random.choice(
                    ['aggressive', 'normal', 'conservative'])
                config_func = config_functions[chosen]
                config_name = config_func(vehicle)
            else:
                config_func = config_functions.get(
                    mode, self.configure_normal)
                config_name = config_func(vehicle)

            self.vehicle_configs[vehicle.id] = {
                'mode': config_name,
            }

        # 统计各模式的车辆数
        mode_counts = defaultdict(int)
        for cfg in self.vehicle_configs.values():
            mode_counts[cfg['mode']] += 1

        print("\n驾驶风格分布:")
        for mode_name, count in mode_counts.items():
            print(f"  {mode_name}: {count} 辆车")

        return self.vehicle_configs

    def get_traffic_stats(self):
        """获取交通流统计信息"""
        speeds_by_mode = defaultdict(list)

        for vehicle in self.vehicles:
            vels = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(
                vels.x**2 + vels.y**2 + vels.z**2)

            config = self.vehicle_configs.get(vehicle.id, {})
            mode = config.get('mode', 'unknown')
            speeds_by_mode[mode].append(speed_kmh)

        stats = {}
        for mode, speeds in speeds_by_mode.items():
            if speeds:
                stats[mode] = {
                    'count': len(speeds),
                    'avg_speed': sum(speeds) / len(speeds),
                    'max_speed': max(speeds),
                    'min_speed': min(speeds),
                }

        return stats

    def print_stats(self, stats):
        """打印交通流统计信息"""
        print("-" * 60)
        print(f"{'风格':<14} {'数量':<6} {'平均(km/h)':<12} "
              f"{'最大(km/h)':<12} {'最小(km/h)':<12}")
        print("-" * 60)
        for mode in ['aggressive', 'normal', 'conservative']:
            if mode in stats:
                s = stats[mode]
                print(f"{mode:<14} {s['count']:<6} "
                      f"{s['avg_speed']:<12.1f} "
                      f"{s['max_speed']:<12.1f} "
                      f"{s['min_speed']:<12.1f}")
        print("-" * 60)

    def update_spectator_view(self):
        """更新观察者视角到俯瞰位置"""
        if not self.vehicles:
            return

        # 计算所有车辆的中心位置
        avg_x = sum(v.get_location().x for v in self.vehicles
                   ) / len(self.vehicles)
        avg_y = sum(v.get_location().y for v in self.vehicles
                   ) / len(self.vehicles)

        spectator = self.world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(x=avg_x, y=avg_y, z=80),
            carla.Rotation(pitch=-90)
        ))

    def run(self, duration=120, print_interval=5.0):
        """主运行循环"""
        print(f"\n交通流仿真开始，运行 {duration} 秒")
        print(f"车辆数: {len(self.vehicles)}")
        print(f"信息更新间隔: {print_interval} 秒")
        print()

        start_time = time.time()
        last_print_time = 0.0

        try:
            while time.time() - start_time < duration:
                self.world.tick()

                current_time = time.time()
                elapsed = current_time - start_time

                # 定期输出统计
                if current_time - last_print_time >= print_interval:
                    last_print_time = current_time
                    stats = self.get_traffic_stats()
                    print(f"\n[{elapsed:5.1f}s] 交通流统计:")
                    self.print_stats(stats)

                # 更新视角
                self.update_spectator_view()

        except KeyboardInterrupt:
            print("\n用户中断")
        except Exception as e:
            print(f"\n错误: {e}")

        finally:
            self.cleanup()

    def cleanup(self):
        """清理所有车辆"""
        print("\n清理车辆...")
        for vehicle in self.vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
        print(f"已清理 {len(self.vehicles)} 辆车")


def main():
    parser = argparse.ArgumentParser(
        description='CARLA Traffic Manager 配置演示')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--tm-port', type=int, default=8000)
    parser.add_argument('--vehicles', type=int, default=20)
    parser.add_argument('--mode', choices=[
        'normal', 'aggressive', 'conservative', 'mixed'
    ], default='normal')
    parser.add_argument('--duration', type=int, default=120)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    # 设置随机种子
    random.seed(args.seed)

    # 连接 CARLA
    print(f"连接 CARLA ({args.host}:{args.port})...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    # 配置同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 清理现有车辆
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()

    print(f"清理完成")

    # 创建 Demo 实例
    print(f"\n[1/3] 生成 {args.vehicles} 辆车...")
    demo = TrafficManagerDemo(client, args.tm_port)
    demo.spawn_vehicles(args.vehicles)

    # 配置 Traffic Manager
    print(f"\n[2/3] 配置 Traffic Manager (模式: {args.mode})...")
    demo.configure_traffic(args.mode)

    # 运行
    print(f"\n[3/3] 开始仿真...")
    demo.run(duration=args.duration, print_interval=5.0)


if __name__ == '__main__':
    main()
