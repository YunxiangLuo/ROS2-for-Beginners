#!/usr/bin/env python3
"""sensor_config.py — 在CARLA中配置并挂载多传感器套件到自车

运行前提:
  1. CARLA服务器运行在 localhost:2000
  2. carla_ros_bridge 已启动

用法:
  python3 sensor_config.py

传感器配置:
  - RGB相机: 800x600, FOV=90°, 20Hz, 前挡风玻璃位置
  - Depth相机: 800x600, FOV=90°, 20Hz, 与前视同位置
  - SemSeg相机: 800x600, FOV=90°, 20Hz
  - LiDAR: 64通道, 100m, 10Hz, 车顶
  - RADAR: FOV=60°, 50m, 20Hz, 前保险杠
  - GNSS: 10Hz, 车顶
  - IMU: 50Hz, 车辆中心
"""

import carla
import random
import time


def setup_camera(world, blueprint_library, vehicle, camera_type, attr_config):
    """通用相机配置函数

    Args:
        camera_type: 'rgb', 'depth', 或 'semantic_segmentation'
        attr_config: dict 包含图像尺寸、FOV、sensor_tick等
    """
    bp_map = {
        'rgb': 'sensor.camera.rgb',
        'depth': 'sensor.camera.depth',
        'semantic_segmentation': 'sensor.camera.semantic_segmentation',
    }
    camera_bp = blueprint_library.find(bp_map[camera_type])

    camera_bp.set_attribute('image_size_x', str(attr_config.get('width', 800)))
    camera_bp.set_attribute('image_size_y', str(attr_config.get('height', 600)))
    camera_bp.set_attribute('fov', str(attr_config.get('fov', 90)))
    camera_bp.set_attribute('sensor_tick', str(attr_config.get('sensor_tick', 0.05)))
    camera_bp.set_attribute('gamma', str(attr_config.get('gamma', 2.2)))

    transform = carla.Transform(
        carla.Location(x=attr_config.get('x', 1.6),
                       y=attr_config.get('y', 0.0),
                       z=attr_config.get('z', 1.7)),
        carla.Rotation(pitch=attr_config.get('pitch', 0),
                       yaw=attr_config.get('yaw', 0),
                       roll=attr_config.get('roll', 0)),
    )
    camera = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
    print(f"[{camera_type.upper()}相机] 已挂载: {transform.location}")
    return camera


def setup_lidar(world, blueprint_library, vehicle, config):
    """配置LiDAR传感器"""
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

    lidar_bp.set_attribute('channels', str(config.get('channels', 64)))
    lidar_bp.set_attribute('range', str(config.get('range', 100.0)))
    lidar_bp.set_attribute('points_per_second', str(config.get('points_per_second', 1000000)))
    lidar_bp.set_attribute('rotation_frequency', str(config.get('rotation_frequency', 10)))
    lidar_bp.set_attribute('upper_fov', str(config.get('upper_fov', 15)))
    lidar_bp.set_attribute('lower_fov', str(config.get('lower_fov', -25)))
    lidar_bp.set_attribute('horizontal_fov', str(config.get('horizontal_fov', 360)))
    lidar_bp.set_attribute('sensor_tick', str(config.get('sensor_tick', 0.1)))
    lidar_bp.set_attribute('dropoff_general_rate', str(config.get('dropoff_rate', 0.1)))
    lidar_bp.set_attribute('dropoff_intensity_limit', str(config.get('dropoff_intensity', 0.1)))
    lidar_bp.set_attribute('dropoff_zero_intensity', str(config.get('dropoff_zero', 0.0)))

    transform = carla.Transform(
        carla.Location(x=config.get('x', 0.0),
                       y=config.get('y', 0.0),
                       z=config.get('z', 2.0)),
        carla.Rotation(pitch=config.get('pitch', 0),
                       yaw=config.get('yaw', 0),
                       roll=config.get('roll', 0)),
    )
    lidar = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
    print(f"[LiDAR] 已挂载: {transform.location}, 通道数={config.get('channels', 64)}")
    return lidar


def setup_radar(world, blueprint_library, vehicle, config):
    """配置RADAR传感器"""
    radar_bp = blueprint_library.find('sensor.other.radar')

    radar_bp.set_attribute('horizontal_fov', str(config.get('horizontal_fov', 60)))
    radar_bp.set_attribute('vertical_fov', str(config.get('vertical_fov', 10)))
    radar_bp.set_attribute('range', str(config.get('range', 50.0)))
    radar_bp.set_attribute('points_per_second', str(config.get('points_per_second', 2000)))
    radar_bp.set_attribute('sensor_tick', str(config.get('sensor_tick', 0.05)))

    transform = carla.Transform(
        carla.Location(x=config.get('x', 2.0),
                       y=config.get('y', 0.0),
                       z=config.get('z', 0.5)),
        carla.Rotation(pitch=config.get('pitch', 0),
                       yaw=config.get('yaw', 0),
                       roll=config.get('roll', 0)),
    )
    radar = world.spawn_actor(radar_bp, transform, attach_to=vehicle)
    print(f"[RADAR] 已挂载: {transform.location}")
    return radar


def setup_gnss(world, blueprint_library, vehicle, config):
    """配置GNSS传感器"""
    gnss_bp = blueprint_library.find('sensor.other.gnss')

    gnss_bp.set_attribute('noise_alt_bias', str(config.get('noise_alt_bias', 0.0)))
    gnss_bp.set_attribute('noise_alt_stddev', str(config.get('noise_alt_stddev', 0.5)))
    gnss_bp.set_attribute('noise_lat_bias', str(config.get('noise_lat_bias', 0.0)))
    gnss_bp.set_attribute('noise_lat_stddev', str(config.get('noise_lat_stddev', 0.0001)))
    gnss_bp.set_attribute('noise_lon_bias', str(config.get('noise_lon_bias', 0.0)))
    gnss_bp.set_attribute('noise_lon_stddev', str(config.get('noise_lon_stddev', 0.0001)))
    gnss_bp.set_attribute('sensor_tick', str(config.get('sensor_tick', 0.1)))

    transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
    gnss = world.spawn_actor(gnss_bp, transform, attach_to=vehicle)
    print(f"[GNSS] 已挂载, 采样间隔={config.get('sensor_tick', 0.1)}s")
    return gnss


def setup_imu(world, blueprint_library, vehicle, config):
    """配置IMU传感器"""
    imu_bp = blueprint_library.find('sensor.other.imu')

    imu_bp.set_attribute('noise_accel_stddev_x',
                         str(config.get('noise_accel_stddev_x', 0.1)))
    imu_bp.set_attribute('noise_accel_stddev_y',
                         str(config.get('noise_accel_stddev_y', 0.1)))
    imu_bp.set_attribute('noise_accel_stddev_z',
                         str(config.get('noise_accel_stddev_z', 0.1)))
    imu_bp.set_attribute('noise_gyro_stddev_x',
                         str(config.get('noise_gyro_stddev_x', 0.05)))
    imu_bp.set_attribute('noise_gyro_stddev_y',
                         str(config.get('noise_gyro_stddev_y', 0.05)))
    imu_bp.set_attribute('noise_gyro_stddev_z',
                         str(config.get('noise_gyro_stddev_z', 0.05)))
    imu_bp.set_attribute('sensor_tick', str(config.get('sensor_tick', 0.02)))

    transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
    imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
    print(f"[IMU] 已挂载, 采样间隔={config.get('sensor_tick', 0.02)}s")
    return imu


def spawn_ego_vehicle(world, blueprint_library):
    """生成自车并设置为自动驾驶"""
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    if not vehicle_bp:
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))

    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[0] if spawn_points else carla.Transform()

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(True)
    print(f"[自车] 已生成: {vehicle.type_id} @ {spawn_point.location}")
    return vehicle


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # 设置同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # 20Hz
    world.apply_settings(settings)

    # 生成自车
    vehicle = spawn_ego_vehicle(world, blueprint_library)

    sensors = []

    # 配置前视RGB相机
    camera_rgb = setup_camera(world, blueprint_library, vehicle, 'rgb', {
        'width': 800, 'height': 600, 'fov': 90, 'sensor_tick': 0.05,
        'x': 1.6, 'y': 0.0, 'z': 1.7,
    })
    sensors.append(camera_rgb)

    # 配置Depth相机
    camera_depth = setup_camera(world, blueprint_library, vehicle, 'depth', {
        'width': 800, 'height': 600, 'fov': 90, 'sensor_tick': 0.05,
        'x': 1.6, 'y': 0.0, 'z': 1.7,
    })
    sensors.append(camera_depth)

    # 配置SemSeg相机
    camera_semseg = setup_camera(world, blueprint_library, vehicle,
                                 'semantic_segmentation', {
        'width': 800, 'height': 600, 'fov': 90, 'sensor_tick': 0.05,
        'x': 1.6, 'y': 0.0, 'z': 1.7,
    })
    sensors.append(camera_semseg)

    # 配置64线LiDAR
    lidar = setup_lidar(world, blueprint_library, vehicle, {
        'channels': 64,
        'range': 100.0,
        'points_per_second': 1000000,
        'rotation_frequency': 10,
        'upper_fov': 15,
        'lower_fov': -25,
        'sensor_tick': 0.1,
        'dropoff_rate': 0.1,
        'x': 0.0, 'y': 0.0, 'z': 2.0,
    })
    sensors.append(lidar)

    # 配置RADAR
    radar = setup_radar(world, blueprint_library, vehicle, {
        'horizontal_fov': 60,
        'vertical_fov': 10,
        'range': 50.0,
        'points_per_second': 2000,
        'sensor_tick': 0.05,
        'x': 2.0, 'y': 0.0, 'z': 0.5,
    })
    sensors.append(radar)

    # 配置GNSS
    gnss = setup_gnss(world, blueprint_library, vehicle, {
        'noise_alt_stddev': 0.5,
        'noise_lat_stddev': 0.0001,
        'noise_lon_stddev': 0.0001,
        'sensor_tick': 0.1,
    })
    sensors.append(gnss)

    # 配置IMU
    imu = setup_imu(world, blueprint_library, vehicle, {
        'noise_accel_stddev_x': 0.1,
        'noise_accel_stddev_y': 0.1,
        'noise_accel_stddev_z': 0.1,
        'noise_gyro_stddev_x': 0.05,
        'noise_gyro_stddev_y': 0.05,
        'noise_gyro_stddev_z': 0.05,
        'sensor_tick': 0.02,
    })
    sensors.append(imu)

    print(f"\n=== 传感器配置完成 ===")
    print(f"共挂载 {len(sensors)} 个传感器:")
    for s in sensors:
        print(f"  - {s.type_id}")

    try:
        # 主循环: 驱动仿真
        while True:
            world.tick()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n正在清理传感器...")
        for s in sensors:
            s.destroy()
        vehicle.destroy()
        print("清理完成")

    # 恢复异步模式
    settings.synchronous_mode = False
    world.apply_settings(settings)


if __name__ == '__main__':
    main()
