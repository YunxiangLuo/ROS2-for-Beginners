#!/usr/bin/env python3

"""
第29章 练习29.2 - FSM 行为决策节点

基于有限状态机的自动驾驶行为决策实现。
状态: CRUISE → FOLLOW → STOP
使用 CARLA Python API 直接连接仿真器。

运行方式:
    python fsm_decision.py
"""

import carla
import math
import random
import time
from enum import Enum
from collections import deque


class FsmState(Enum):
    """FSM 状态枚举"""
    INIT = 0
    CRUISE = 1
    FOLLOW = 2
    STOP = 3
    AVOID = 4
    COMPLETE = 5


class FSMDecision:
    """有限状态机行为决策类"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.state = FsmState.INIT
        self.state_names = {
            FsmState.INIT: 'INIT',
            FsmState.CRUISE: 'CRUISE',
            FsmState.FOLLOW: 'FOLLOW',
            FsmState.STOP: 'STOP',
            FsmState.AVOID: 'AVOID',
            FsmState.COMPLETE: 'COMPLETE',
        }

        # 参数配置
        self.CRUISE_SPEED = 8.0       # 巡航速度 (m/s) ≈ 29 km/h
        self.SAFE_DISTANCE = 15.0     # 安全跟车距离 (m)
        self.MAX_THROTTLE = 0.6       # 最大油门
        self.BRAKE_STRENGTH = 0.8     # 刹车强度
        self.STOP_LINE_MARGIN = 3.0   # 停止线余量 (m)

        # 状态
        self.target_speed = self.CRUISE_SPEED
        self.current_speed = 0.0
        self.lead_distance = float('inf')
        self.have_red_light = False
        self.target_location = None
        self.past_speeds = deque(maxlen=10)

        # 状态转移处理函数映射
        self.handlers = {
            FsmState.CRUISE: self._handle_cruise,
            FsmState.FOLLOW: self._handle_follow,
            FsmState.STOP: self._handle_stop,
            FsmState.AVOID: self._handle_avoid,
            FsmState.COMPLETE: self._handle_complete,
        }

        # 完成标记
        self.destination_reached = False

    def set_destination(self, location):
        """设置目标位置"""
        self.target_location = location
        self.state = FsmState.CRUISE

    def update(self):
        """主更新函数，执行一次状态转移和动作输出"""
        if self.state == FsmState.INIT:
            self.state = FsmState.CRUISE
            return self._create_stop_cmd()
        if self.state == FsmState.COMPLETE:
            return self._create_stop_cmd()

        # 读取当前状态
        self._update_perception()

        # 委托给当前状态处理器
        handler = self.handlers.get(self.state, self._handle_cruise)
        cmd = handler()

        self._log_state()
        return cmd

    def _update_perception(self):
        """更新感知信息"""
        # 自车速度
        v = self.vehicle.get_velocity()
        self.current_speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

        # 交通灯检测
        traffic_light = self.vehicle.get_traffic_light()
        if traffic_light is not None:
            state = traffic_light.get_state()
            self.have_red_light = (
                state == carla.TrafficLightState.Red or
                state == carla.TrafficLightState.Yellow
            )
            # 检查到停止线的距离
            stop_wps = traffic_light.get_stop_waypoints()
            if stop_wps:
                stop_loc = stop_wps[0].transform.location
                dist = self.vehicle.get_location().distance(stop_loc)
                # 距离较远时忽略红灯
                if dist > 50.0:
                    self.have_red_light = False
        else:
            self.have_red_light = False

        # 前车检测（使用 CARLA 传感器或简化模拟）
        self._detect_lead_vehicle()

        # 目的地检查
        if self.target_location is not None:
            dist_to_dest = self.vehicle.get_location().distance(
                self.target_location)
            if dist_to_dest < 5.0:
                self.destination_reached = True
                self.state = FsmState.COMPLETE

    def _detect_lead_vehicle(self):
        """检测前方车辆（简化版本）"""
        self.lead_distance = float('inf')
        vehicle_loc = self.vehicle.get_location()
        vehicle_forward = self.vehicle.get_transform().get_forward_vector()

        world = self.vehicle.get_world()
        actors = world.get_actors().filter('vehicle.*')

        for actor in actors:
            if actor.id == self.vehicle.id:
                continue

            actor_loc = actor.get_location()
            # 计算相对向量
            dx = actor_loc.x - vehicle_loc.x
            dy = actor_loc.y - vehicle_loc.y

            # 检查是否在前方（点积 > 0 表示在前方）
            dot = dx * vehicle_forward.x + dy * vehicle_forward.y
            if dot <= 0:
                continue

            # 检查是否在同一车道（横向偏移 < 2m）
            # 使用 Carla 地图的车道 API
            waypoint = world.get_map().get_waypoint(vehicle_loc)
            actor_waypoint = world.get_map().get_waypoint(actor_loc)
            if waypoint.lane_id != actor_waypoint.lane_id:
                continue

            distance = math.sqrt(dx*dx + dy*dy)
            if distance < self.lead_distance:
                self.lead_distance = distance

    def _handle_cruise(self):
        """巡航状态处理"""
        cmd = carla.VehicleControl()
        cmd.throttle = 0.4
        cmd.brake = 0.0
        cmd.steer = 0.0

        # 条件1: 检测到前车且距离小于安全距离
        if self.lead_distance < self.SAFE_DISTANCE:
            self.state = FsmState.FOLLOW
            print(f"[CRUISE → FOLLOW] 前车距离: {self.lead_distance:.1f}m")
            return self._handle_follow()

        # 条件2: 检测到红灯(距离过滤已在 _update_perception 中完成)
        if self.have_red_light:
            self.state = FsmState.STOP
            print("[CRUISE → STOP] 红灯检测")
            return self._handle_stop()

        # 速度 PID 控制（简化）
        speed_error = self.CRUISE_SPEED - (self.current_speed / 3.6)
        if speed_error > 1.0:
            cmd.throttle = min(self.MAX_THROTTLE,
                             0.3 + 0.05 * speed_error)
        elif speed_error < -1.0:
            cmd.throttle = 0.1
        else:
            cmd.throttle = 0.25

        self.target_speed = self.CRUISE_SPEED
        return cmd

    def _handle_follow(self):
        """跟车状态处理"""
        cmd = carla.VehicleControl()

        # 前车消失/太远 → 回到巡航
        if self.lead_distance > self.SAFE_DISTANCE * 1.5:
            self.state = FsmState.CRUISE
            print(f"[FOLLOW → CRUISE] 前车消失，距离: {self.lead_distance:.1f}m")
            return self._handle_cruise()

        # 红灯 → 停止
        if self.have_red_light:
            self.state = FsmState.STOP
            print("[FOLLOW → STOP] 红灯")
            return self._handle_stop()

        # 跟车控制：距离越近，刹车越大
        if self.lead_distance < 5.0:
            cmd.throttle = 0.0
            cmd.brake = 0.8
            self.target_speed = 0.0
        elif self.lead_distance < 10.0:
            cmd.throttle = 0.15
            cmd.brake = 0.1
            self.target_speed = self.current_speed / 3.6 * 0.7
        else:
            cmd.throttle = 0.3
            cmd.brake = 0.0
            self.target_speed = self.CRUISE_SPEED

        return cmd

    def _handle_stop(self):
        """停止状态处理"""
        cmd = carla.VehicleControl()
        cmd.throttle = 0.0

        # 红灯变绿 → 恢复巡航
        if not self.have_red_light:
            self.state = FsmState.CRUISE
            print("[STOP → CRUISE] 绿灯")
            return self._handle_cruise()

        # 保持停止
        if self.current_speed / 3.6 > 0.5:
            cmd.brake = 0.7
        else:
            cmd.brake = 1.0

        self.target_speed = 0.0
        return cmd

    def _handle_avoid(self):
        """避障状态处理（预留）"""
        # TODO: 实现紧急避让逻辑
        cmd = carla.VehicleControl()
        cmd.throttle = 0.0
        cmd.brake = 0.9
        cmd.steer = 0.0
        self.state = FsmState.CRUISE
        return cmd

    def _handle_complete(self):
        """完成状态处理"""
        return self._create_stop_cmd()

    def _create_stop_cmd(self):
        """生成停止控制指令"""
        cmd = carla.VehicleControl()
        cmd.throttle = 0.0
        cmd.brake = 1.0
        cmd.hand_brake = True
        return cmd

    def _log_state(self):
        """输出当前状态信息"""
        print(
            f"[{self.state_names[self.state]:8s}] "
            f"速度: {self.current_speed:5.1f} km/h | "
            f"前车: {self.lead_distance:5.1f} m | "
            f"红灯: {'是' if self.have_red_light else '否'} | "
            f"目标: {self.target_speed:4.1f} m/s"
        )

    def get_state_name(self):
        return self.state_names[self.state]


def main():
    """主函数：连接 CARLA 并运行 FSM 决策"""
    # 连接CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    print("=" * 60)
    print("第29章 FSM 行为决策演示")
    print("=" * 60)

    # 获取设置
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 清理现有车辆
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()

    # 生成自车
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    spawn_points = world.get_map().get_spawn_points()

    if not spawn_points:
        print("错误: 没有可用的出生点")
        return

    spawn_point = spawn_points[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    if vehicle is None:
        print("错误: 生成车辆失败")
        return

    print(f"自车生成在: ({spawn_point.location.x:.1f}, "
          f"{spawn_point.location.y:.1f})")

    # 生成背景车辆（用于跟车测试）
    if len(spawn_points) > 1:
        other_bp = blueprint_library.find('vehicle.audi.a2')
        other_sp = spawn_points[3]  # 选择前方的出生点
        other_vehicle = world.spawn_actor(other_bp, other_sp)
        if other_vehicle:
            # 背景车设定为自动驾驶
            tm = client.get_trafficmanager(8000)
            other_vehicle.set_autopilot(True, tm.get_port())
            tm.set_desired_speed(other_vehicle, 20.0)
            print(f"背景车辆生成在: ({other_sp.location.x:.1f}, "
                  f"{other_sp.location.y:.1f})")

    # 初始化 FSM
    fsm = FSMDecision(vehicle)
    # 设置目的地（使用最后一个出生点）
    fsm.set_destination(spawn_points[-1].location)

    # 添加交通灯摄像头（可选）
    spectator = world.get_spectator()

    try:
        running_time = 0.0
        while not fsm.destination_reached:
            world.tick()
            running_time += 0.05

            # 更新观察者视角
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location + carla.Location(z=50),
                carla.Rotation(pitch=-90)
            ))

            # 执行 FSM 决策
            control = fsm.update()
            vehicle.apply_control(control)

            # 超时保护（运行120秒）
            if running_time > 120.0:
                print("\n超时退出")
                break

        print(f"\n到达目的地! 运行时间: {running_time:.1f}s")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 恢复非同步模式
        settings.synchronous_mode = False
        world.apply_settings(settings)

        # 清理
        for actor in world.get_actors().filter('vehicle.*'):
            actor.destroy()
        print("清理完成")


if __name__ == '__main__':
    main()
