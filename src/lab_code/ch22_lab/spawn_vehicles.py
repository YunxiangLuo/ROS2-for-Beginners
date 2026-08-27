#!/usr/bin/env python3
"""在CARLA中生成车辆并设置自动驾驶"""
import argparse
import random
import time

import carla


def spawn_vehicles(client, world, num_vehicles, autopilot=True):
    bp_lib = world.get_blueprint_library()
    carla_map = world.get_map()

    # 获取所有车辆蓝图
    vehicle_bps = bp_lib.filter("vehicle.*")

    # 获取所有生成点
    spawn_points = carla_map.get_spawn_points()
    if len(spawn_points) < num_vehicles:
        print(f"警告: 生成点({len(spawn_points)})少于请求车辆数({num_vehicles})"
              f"，将只生成 {len(spawn_points)} 辆车")
        num_vehicles = len(spawn_points)

    # 随机选择生成点
    selected_spawns = random.sample(spawn_points, num_vehicles)

    # 批量生成车辆
    vehicles = []
    batch = []
    for i, spawn_pt in enumerate(selected_spawns):
        bp = random.choice(vehicle_bps)
        # 避免生成摩托车或自行车（有特殊物理）
        while bp.id in ("vehicle.harley-davidson.low-rider",
                        "vehicle.yamaha.yzf",
                        "vehicle.kawasaki.ninja",
                        "vehicle.bh.crossbike",
                        "vehicle.diamondback.century",
                        "vehicle.gazelle.omafiets"):
            bp = random.choice(vehicle_bps)

        batch.append(carla.command.SpawnActor(bp, spawn_pt))

    # 执行批量生成
    results = client.apply_batch_sync(batch, do_tick=True)
    for i, result in enumerate(results):
        if result.error:
            print(f"  车辆 [{i}] 生成失败: {result.error}")
        else:
            vehicles.append(result.actor_id)
            print(f"  车辆 [{i}] 生成成功: actor_id={result.actor_id}")

    print(f"\n成功生成 {len(vehicles)}/{num_vehicles} 辆车")

    # 获取Actor对象
    vehicle_actors = []
    for actor_id in vehicles:
        actor = world.get_actor(actor_id)
        if actor:
            vehicle_actors.append(actor)

    # 设置自动驾驶
    if autopilot and vehicle_actors:
        tm = client.get_trafficmanager(8000)
        tm.set_global_distance_to_leading_vehicle(2.0)
        tm.set_synchronous_mode(False)

        for actor in vehicle_actors:
            actor.set_autopilot(True, tm.get_port())

        print("所有车辆已设置为自动驾驶模式")

    return vehicle_actors


def main():
    parser = argparse.ArgumentParser(description="在CARLA中生成车辆")
    parser.add_argument("--host", default="localhost", help="CARLA服务器主机地址")
    parser.add_argument("--port", type=int, default=2000, help="CARLA服务器端口")
    parser.add_argument("--timeout", type=float, default=10.0, help="连接超时时间(秒)")
    parser.add_argument("-n", "--num-vehicles", type=int, default=10,
                        help="要生成的车辆数量 (默认: 10)")
    parser.add_argument("--no-autopilot", action="store_true",
                        help="不启用自动驾驶")
    parser.add_argument("--map", default=None,
                        help="指定地图名称 (默认: 使用当前地图)")
    args = parser.parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        print(f"已连接到 CARLA 服务器 {args.host}:{args.port}")

        # 切换地图
        if args.map:
            print(f"切换到地图: {args.map}")
            world = client.load_world(args.map)
            time.sleep(2.0)
        else:
            world = client.get_world()

        print(f"当前地图: {world.get_map().name}")

        # 生成车辆
        vehicles = spawn_vehicles(
            client, world, args.num_vehicles,
            autopilot=not args.no_autopilot
        )

        print(f"\n场景已准备完毕，共 {len(vehicles)} 辆车")
        print("按 Ctrl+C 退出并清除所有车辆")

        # 保持运行，直到用户中断
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n\n正在清除车辆...")
        world = client.get_world()
        actors = world.get_actors()
        vehicle_actors = [a for a in actors
                          if "vehicle" in a.type_id]
        for actor in vehicle_actors:
            actor.destroy()
        print(f"已清除 {len(vehicle_actors)} 辆车")
        print("退出")
    except RuntimeError as e:
        print(f"错误: {e}")
        print("请确保 CARLA 服务器已启动 (./CarlaUE4.sh)")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
