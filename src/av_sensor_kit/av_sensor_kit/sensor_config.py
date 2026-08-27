import yaml
from dataclasses import dataclass, field, asdict
from typing import List, Optional


@dataclass
class SensorConfig:
    type: str
    name: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    width: Optional[int] = None
    height: Optional[int] = None
    fov: Optional[float] = None
    channels: Optional[int] = None
    range: Optional[float] = None
    points_per_second: Optional[float] = None
    rotation_frequency: Optional[float] = None

    def to_dict(self):
        d = {k: v for k, v in asdict(self).items() if v is not None}
        return d


FRONT_CAMERA = SensorConfig(
    type='sensor.camera.rgb',
    name='front_rgb',
    x=1.5, z=1.8,
    width=1280, height=720, fov=90,
)

DEPTH_CAMERA = SensorConfig(
    type='sensor.camera.depth',
    name='front_depth',
    x=1.5, z=1.8,
    width=1280, height=720, fov=90,
)

SEMANTIC_CAMERA = SensorConfig(
    type='sensor.camera.semantic_segmentation',
    name='front_semantic',
    x=1.5, z=1.8,
    width=1280, height=720, fov=90,
)

LIDAR_64 = SensorConfig(
    type='sensor.lidar.ray_cast',
    name='lidar_64',
    z=2.2,
    channels=64, range=120.0, points_per_second=1300000,
    rotation_frequency=0.2,
)

LIDAR_32 = SensorConfig(
    type='sensor.lidar.ray_cast',
    name='lidar_32',
    z=2.2,
    channels=32, range=100.0, points_per_second=650000,
    rotation_frequency=10.0,
)

GNSS = SensorConfig(
    type='sensor.other.gnss',
    name='gnss',
)

IMU = SensorConfig(
    type='sensor.other.imu',
    name='imu',
)


def load_config(yaml_path: str) -> List[SensorConfig]:
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    configs = []
    for item in data.get('sensors', []):
        kwargs = dict(item)
        name = kwargs.pop('name')
        stype = kwargs.pop('type')
        x = kwargs.pop('x', 0.0)
        y = kwargs.pop('y', 0.0)
        z = kwargs.pop('z', 0.0)
        roll = kwargs.pop('roll', 0.0)
        pitch = kwargs.pop('pitch', 0.0)
        yaw = kwargs.pop('yaw', 0.0)
        width = kwargs.pop('width', None)
        height = kwargs.pop('height', None)
        fov = kwargs.pop('fov', None)
        channels = kwargs.pop('channels', None)
        range_val = kwargs.pop('range', None)
        points_per_second = kwargs.pop('points_per_second', None)
        rotation_frequency = kwargs.pop('rotation_frequency', None)
        configs.append(SensorConfig(
            type=stype,
            name=name,
            x=x, y=y, z=z,
            roll=roll, pitch=pitch, yaw=yaw,
            width=width, height=height, fov=fov,
            channels=channels, range=range_val,
            points_per_second=points_per_second,
            rotation_frequency=rotation_frequency,
        ))
    return configs


def save_config(configs: List[SensorConfig], yaml_path: str):
    data = {'sensors': [c.to_dict() for c in configs]}
    with open(yaml_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)


def main():
    import sys
    if len(sys.argv) < 2:
        print('Usage: sensor_config <yaml_path>')
        return
    configs = load_config(sys.argv[1])
    for c in configs:
        print(c)


if __name__ == '__main__':
    main()
