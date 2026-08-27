import math


class VehicleControl:
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0):
        self.throttle = max(0.0, min(throttle, 1.0))
        self.brake = max(0.0, min(brake, 1.0))
        self.steer = max(-1.0, min(steer, 1.0))

    def __repr__(self):
        return (f"VehicleControl(throttle={self.throttle:.3f}, "
                f"brake={self.brake:.3f}, steer={self.steer:.3f})")


class CarlaVehicleWrapper:
    def __init__(self, vehicle=None):
        self.vehicle = vehicle

    def apply_control(self, control):
        if self.vehicle is not None:
            carla_ctrl = self.vehicle.get_control()
            carla_ctrl.throttle = control.throttle
            carla_ctrl.brake = control.brake
            carla_ctrl.steer = control.steer
            self.vehicle.apply_control(carla_ctrl)

    def get_velocity(self):
        if self.vehicle is None:
            return 0.0
        v = self.vehicle.get_velocity()
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def get_position(self):
        if self.vehicle is None:
            return (0.0, 0.0, 0.0)
        t = self.vehicle.get_transform()
        return (t.location.x, t.location.y, t.location.z)

    def get_yaw(self):
        if self.vehicle is None:
            return 0.0
        t = self.vehicle.get_transform()
        return math.radians(t.rotation.yaw)

    @staticmethod
    def spawn_test_vehicle(client, blueprint_name='vehicle.tesla.model3', spawn_point_idx=0):
        world = client.get_world()
        bp_lib = world.get_blueprint_library()
        bp = bp_lib.find(blueprint_name)
        spawn_points = world.get_map().get_spawn_points()

        if spawn_point_idx >= len(spawn_points):
            spawn_point_idx = 0

        vehicle = world.spawn_actor(bp, spawn_points[spawn_point_idx])
        return CarlaVehicleWrapper(vehicle)


class VehicleState:
    def __init__(self, position=(0.0, 0.0, 0.0), yaw=0.0, speed=0.0):
        self.position = position
        self.yaw = yaw
        self.speed = speed

    def update(self, dt, throttle, brake, steer):
        accel = 1.5 * throttle - 3.0 * brake
        self.speed += accel * dt
        self.speed = max(0.0, self.speed)

        angular_vel = 1.0 * steer
        self.yaw += angular_vel * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.position = (
            self.position[0] + self.speed * math.cos(self.yaw) * dt,
            self.position[1] + self.speed * math.sin(self.yaw) * dt,
            0.0
        )
