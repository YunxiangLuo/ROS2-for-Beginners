#!/usr/bin/env python3
"""连接到CARLA服务器并打印世界信息"""
import argparse
import carla
from importlib.metadata import PackageNotFoundError, version


def carla_api_version():
    try:
        return version("carla")
    except PackageNotFoundError:
        return getattr(carla, "__version__", "unknown")


def print_separator(title):
    print()
    print("=" * 60)
    print(f"  {title}")
    print("=" * 60)


def explore_world(client, world):
    # 服务器版本
    server_version = client.get_server_version()
    print_separator("服务器信息")
    print(f"  CARLA 版本: {server_version[1]}")
    print(f"  API  版本:  {carla_api_version()}")

    # 地图信息
    print_separator("地图信息")
    carla_map = world.get_map()
    print(f"  地图名称: {carla_map.name}")
    print(f"  地图文件: {carla_map.name}")

    spawn_points = carla_map.get_spawn_points()
    print(f"  可生成点: {len(spawn_points)} 个")

    topology = carla_map.get_topology()
    print(f"  路网段数: {len(topology)}")

    if spawn_points:
        print(f"\n  前5个生成点坐标:")
        for i, pt in enumerate(spawn_points[:5]):
            loc = pt.location
            rot = pt.rotation
            print(f"    [{i}] x={loc.x:.1f} y={loc.y:.1f} z={loc.z:.1f} "
                  f"pitch={rot.pitch:.1f} yaw={rot.yaw:.1f} roll={rot.roll:.1f}")

    # Actor信息
    print_separator("Actor 信息")
    actors = world.get_actors()
    print(f"  当前 Actor 数量: {len(actors)}")
    for actor in actors:
        print(f"    - {actor.type_id}  [id={actor.id}]")

    # 蓝图信息
    print_separator("蓝图信息")
    bp_lib = world.get_blueprint_library()

    vehicle_bps = bp_lib.filter("vehicle.*")
    print(f"  车辆蓝图数量: {len(vehicle_bps)}")
    for bp in vehicle_bps[:8]:
        print(f"    - {bp.id}")

    sensor_bps = bp_lib.filter("sensor.*")
    print(f"\n  传感器蓝图数量: {len(sensor_bps)}")
    for bp in sensor_bps:
        print(f"    - {bp.id}")

    # 天气信息
    print_separator("天气信息")
    weather = world.get_weather()
    print(f"  云量 (cloudiness):           {weather.cloudiness}")
    print(f"  降水 (precipitation):        {weather.precipitation}")
    print(f"  积水 (precipitation_deposits): {weather.precipitation_deposits}")
    print(f"  风速 (wind_intensity):       {weather.wind_intensity}")
    print(f"  太阳方位角 (sun_azimuth):    {weather.sun_azimuth_angle}")
    print(f"  太阳高度角 (sun_altitude):   {weather.sun_altitude_angle}")
    print(f"  雾浓度 (fog_density):        {weather.fog_density}")
    print(f"  湿润度 (wetness):            {weather.wetness}")

    print()
    print("探索完成！")


def main():
    parser = argparse.ArgumentParser(description="探索CARLA世界")
    parser.add_argument("--host", default="localhost", help="CARLA服务器主机地址")
    parser.add_argument("--port", type=int, default=2000, help="CARLA服务器端口")
    parser.add_argument("--timeout", type=float, default=10.0, help="连接超时时间(秒)")
    args = parser.parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        world = client.get_world()
        print(f"已连接到 CARLA 服务器 {args.host}:{args.port}")
        explore_world(client, world)
    except RuntimeError as e:
        print(f"连接 CARLA 服务器失败: {e}")
        print("请确保 CARLA 服务器已启动 (./CarlaUE4.sh)")
        return 1
    except KeyboardInterrupt:
        print("\n用户中断")
        return 0

    return 0


if __name__ == "__main__":
    exit(main())
