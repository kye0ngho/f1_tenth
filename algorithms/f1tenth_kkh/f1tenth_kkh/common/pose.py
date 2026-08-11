"""TF-based vehicle pose lookup, shared by all f1tenth_kkh controllers.

Ported from LinearMpcNode.quaternion_to_yaw / lookup_vehicle_pose in
algorithms/control/control/linear_mpc_node.py.
"""

import math

from rclpy.duration import Duration
from rclpy.time import Time


def quaternion_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def lookup_vehicle_pose(tf_buffer, global_frame_id, base_frame_id,
                         timeout_sec=0.03):
    """Look up (x, y, yaw) of base_frame_id in global_frame_id.

    Raises tf2_ros.TransformException on failure; callers are expected to
    catch it, matching the existing control-package nodes.
    """
    transform = tf_buffer.lookup_transform(
        global_frame_id,
        base_frame_id,
        Time(),
        timeout=Duration(seconds=timeout_sec),
    )
    translation = transform.transform.translation
    yaw = quaternion_to_yaw(transform.transform.rotation)
    return translation.x, translation.y, yaw
