#!/usr/bin/env python3
"""
spawn_ego.py — 通过CARLA Python API生成Ego Vehicle。

功能：
1. 连接CARLA仿真服务器
2. 选择指定Blueprint的车辆
3. 设置 role_name 供Bridge识别
4. 在指定Spawn Point生成车辆
5. 附加默认传感器（RGB相机、Lidar）

用法：
    python3 spawn_ego.py --spawn-point 10 --role-name ego_vehicle
"""

import argparse
import sys
import time

import carla


def spawn_ego_vehicle(
    host='localhost',
    port=2000,
    spawn_point_index=0,
    role_name='ego_vehicle',
    blueprint_id='vehicle.tesla.model3',
    color='255,0,0',
    attach_sensors=True,
):
    print(f'Connecting to CARLA server at {host}:{port}...')
    client = carla.Client(host, port)
    client.set_timeout(10.0)

    world = client.get_world()
    bp_library = world.get_blueprint_library()

    # 选择Blueprint
    vehicle_bp = bp_library.find(blueprint_id)
    if not vehicle_bp:
        available = [bp.id for bp in bp_library.filter('vehicle.*')]
        print(f'ERROR: Blueprint "{blueprint_id}" not found.')
        print(f'Available vehicles: {available[:10]}...')
        sys.exit(1)

    # 设置属性
    vehicle_bp.set_attribute('role_name', role_name)
    if vehicle_bp.has_attribute('color'):
        vehicle_bp.set_attribute('color', color)

    # 选取生成点
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print('ERROR: No spawn points available on this map.')
        sys.exit(1)

    if spawn_point_index >= len(spawn_points):
        print(f'WARN: Spawn point {spawn_point_index} out of range '
              f'(max={len(spawn_points)-1}). Using index 0.')
        spawn_point_index = 0

    spawn_point = spawn_points[spawn_point_index]

    # 生成车辆
    print(f'Spawn point [{spawn_point_index}]: '
          f'x={spawn_point.location.x:.2f}, '
          f'y={spawn_point.location.y:.2f}, '
          f'yaw={spawn_point.rotation.yaw:.1f}')

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    if vehicle is None:
        print('ERROR: Failed to spawn vehicle. '
              'Check if spawn point is occupied.')
        sys.exit(1)

    print(f'Ego Vehicle spawned!')
    print(f'  Role name:  {role_name}')
    print(f'  Blueprint:  {blueprint_id}')
    print(f'  Location:   ({spawn_point.location.x:.2f}, '
          f'{spawn_point.location.y:.2f}, '
          f'{spawn_point.location.z:.2f})')
    print(f'  Rotation:   yaw={spawn_point.rotation.yaw:.1f}')

    # 附加传感器
    sensors = []
    if attach_sensors:
        sensors = _attach_sensors(world, vehicle, role_name)

    return vehicle, sensors


def _attach_sensors(world, vehicle, role_name):
    """为车辆附加RGB相机和Lidar传感器"""
    bp_library = world.get_blueprint_library()
    sensors = []

    # RGB相机 (前视)
    camera_bp = bp_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90.0')
    camera_bp.set_attribute('sensor_tick', '0.05')

    camera_transform = carla.Transform(
        carla.Location(x=1.6, z=1.7))
    camera = world.spawn_actor(
        camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: None)  # Bridge 会接管数据流
    sensors.append(camera)
    print(f'  [Sensor] RGB Camera attached')

    # Lidar
    lidar_bp = bp_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '50.0')
    lidar_bp.set_attribute('channels', '32')
    lidar_bp.set_attribute('points_per_second', '100000')
    lidar_bp.set_attribute('sensor_tick', '0.1')

    lidar_transform = carla.Transform(
        carla.Location(x=0.0, z=2.0))
    lidar = world.spawn_actor(
        lidar_bp, lidar_transform, attach_to=vehicle)
    lidar.listen(lambda point_cloud: None)
    sensors.append(lidar)
    print(f'  [Sensor] Lidar attached')

    print(f'  Total sensors: {len(sensors)}')
    return sensors


def main():
    parser = argparse.ArgumentParser(
        description='Spawn an Ego Vehicle in CARLA for ROS2 Bridge')
    parser.add_argument('--host', default='localhost',
                        help='CARLA server host (default: localhost)')
    parser.add_argument('--port', type=int, default=2000,
                        help='CARLA server port (default: 2000)')
    parser.add_argument('--spawn-point', type=int, default=0,
                        help='Spawn point index (default: 0)')
    parser.add_argument('--role-name', default='ego_vehicle',
                        help='Vehicle role name (default: ego_vehicle)')
    parser.add_argument('--blueprint', default='vehicle.tesla.model3',
                        help='Blueprint ID (default: vehicle.tesla.model3)')
    parser.add_argument('--color', default='255,0,0',
                        help='Vehicle color as R,G,B (default: 255,0,0)')
    parser.add_argument('--no-sensors', action='store_true',
                        help='Skip attaching sensors')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Keep the vehicle alive for this many seconds; '
                             '0 keeps running until Ctrl+C (default: 0)')
    args = parser.parse_args()

    if args.duration < 0.0:
        parser.error('--duration must be zero or greater')

    vehicle, sensors = spawn_ego_vehicle(
        host=args.host,
        port=args.port,
        spawn_point_index=args.spawn_point,
        role_name=args.role_name,
        blueprint_id=args.blueprint,
        color=args.color,
        attach_sensors=not args.no_sensors,
    )

    print('\nVehicle is ready. Bridge will publish topics with '
          f'prefix: /carla/{args.role_name}/')
    if args.duration > 0.0:
        print(f'Keeping the vehicle alive for {args.duration:g} seconds.')
    else:
        print('Press Ctrl+C to destroy the vehicle and exit.')

    try:
        if args.duration > 0.0:
            time.sleep(args.duration)
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDestroying vehicle...')
        for sensor in reversed(sensors):
            sensor.stop()
            sensor.destroy()
        vehicle.destroy()
        print('Done.')


if __name__ == '__main__':
    main()
