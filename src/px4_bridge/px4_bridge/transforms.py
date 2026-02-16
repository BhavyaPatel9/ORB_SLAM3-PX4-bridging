"""
px4_bridge/transforms.py

Frame & transform helpers:
- Convert a Twist in FLU (body) frame to PX4 expected NED world velocities (trajectory_setpoint).
- Convert quaternion to yaw for yaw-aware transforms (if needed).
"""

import math
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleAttitude  # type: ignore

# Notes on conventions used here:
# - motion_controller outputs FLU body-frame Twist:
#     linear.x = forward
#     linear.y = left
#     linear.z = up
#     angular.z = yaw_rate (positive = left-turn)
#
# - PX4 expects NED coordinates for trajectory_setpoint.velocity:
#     x = North, y = East, z = Down
#
# We'll implement the same mapping used in the px4_offboard example:
#   1) FLU body -> intermediate NED body mapping:
#        vx_body_ned = -twist.linear.y  (north)
#        vy_body_ned =  twist.linear.x  (east)
#        vz_body_ned = -twist.linear.z  (down)
#   2) Rotate velocities from body frame into world frame using current yaw (true_yaw)
#        vx_world = vx_body_ned * cos(yaw) - vy_body_ned * sin(yaw)
#        vy_world = vx_body_ned * sin(yaw) + vy_body_ned * cos(yaw)
#        vz_world = vz_body_ned  (vertical is already in NED down)
#
# true_yaw should be drone's yaw in radians, with same sign convention as px4_attitude example
# (the px4_offboard example used a negation when extracting yaw; we accept true_yaw as "drone yaw"
# in radians consistent with transformation formulas below).

def quaternion_to_yaw(q):
    """
    Convert quaternion (array/list [w, x, y, z] or msg.q) to yaw (radians).
    This returns yaw in the range [-pi, pi].
    Accepts q as a sequence or as the VehicleAttitude.q field (length 4).
    """
    # Expect q of length 4: [w, x, y, z] or msg.q where msg.q[0] = w in PX4 message
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))  [if quaternion order is w,x,y,z]
    # But the px4 example uses another formula with indices [w, x, y, z], so use standard:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def flu_body_twist_to_ned_world(twist: Twist, true_yaw: float):
    """
    Convert a FLU body-frame Twist (linear m/s, angular rad/s) to PX4 NED world velocities.
    Returns: (vx_world, vy_world, vz_world, yaw_rate)
      - velocities in m/s in NED world frame (vx north, vy east, vz down)
      - yaw_rate left as-is (rad/s); PX4 TrajectorySetpoint uses yawspeed (rad/s)
    """
    # Step 1: FLU body -> NED body (component remapping)
    # px4_offboard mapping:
    #   X(FLU) -> -Y(NED body)
    #   Y(FLU) -> X(NED body)
    #   Z(FLU) -> -Z(NED body)
    vx_body_ned = -float(twist.linear.y)   # North component in body frame
    vy_body_ned = float(twist.linear.x)    # East component in body frame
    vz_body_ned = -float(twist.linear.z)   # Down component in body frame

    # Step 2: rotate body -> world using current yaw (true_yaw)
    cos_yaw = math.cos(true_yaw)
    sin_yaw = math.sin(true_yaw)

    vx_world = vx_body_ned * cos_yaw - vy_body_ned * sin_yaw
    vy_world = vx_body_ned * sin_yaw + vy_body_ned * cos_yaw
    vz_world = vz_body_ned

    yaw_rate = float(twist.angular.z)

    return vx_world, vy_world, vz_world, yaw_rate
