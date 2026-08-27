import math

from geometry_msgs.msg import PoseStamped


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose_stamped(frame_id: str, x: float, y: float, yaw: float):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    _, _, z, w = quaternion_from_yaw(yaw)
    msg.pose.orientation.z = z
    msg.pose.orientation.w = w
    return msg
