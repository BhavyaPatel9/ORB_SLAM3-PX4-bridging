# Coordinate transforms, timestamps
"""
px4_bridge.utils

Small helper functions used across px4_bridge:
- timestamp helper (microseconds)
- quaternion -> yaw conversion
- simple clamp
- helpers to build/zero TrajectorySetpoint messages
- publish-burst helper to ensure RX on the FMU side (used for safe zeroing)
"""

from typing import Optional
import math
import time

import rclpy
from rclpy.clock import Clock

from px4_msgs.msg import TrajectorySetpoint  # px4_msgs dependency
from geometry_msgs.msg import Quaternion  # type: ignore


def now_us(clock: Optional[Clock] = None) -> int:
    """
    Return current time in microseconds as int.
    If a Clock is provided, it will be used (useful for unit tests); otherwise
    a fresh rclpy Clock() is used.
    """
    if clock is None:
        # lightweight: create ephemeral Clock
        return int(Clock().now().nanoseconds / 1000)
    return int(clock.now().nanoseconds / 1000)


def quaternion_to_yaw(q) -> float:
    """
    Convert a quaternion to yaw (radians).
    Accepts:
      - a sequence-like (w, x, y, z) or
      - a geometry_msgs/Quaternion object
      - a px4 VehicleAttitude.msg.q (list-like [w,x,y,z])
    Returns yaw in range [-pi, pi].
    """
    if q is None:
        return 0.0

    if isinstance(q, Quaternion):
        w = q.w
        x = q.x
        y = q.y
        z = q.z
    else:
        # assume sequence [w, x, y, z] or msg.q style
        w = float(q[0])
        x = float(q[1])
        y = float(q[2])
        z = float(q[3])

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, low: float, high: float) -> float:
    """Clamp value to [low, high]."""
    return max(low, min(high, value))


def make_zero_trajectory_setpoint(timestamp_us: Optional[int] = None) -> TrajectorySetpoint:
    """
    Return a TrajectorySetpoint message that encodes "zero velocity" (world NED).
    - positions and accelerations are left NaN (so PX4 ignores them)
    - velocity components set to 0
    - yaw set to NaN, yawspeed = 0
    If timestamp_us is provided, it will be written into the message.timestamp field.
    """
    sp = TrajectorySetpoint()
    if timestamp_us is None:
        sp.timestamp = now_us()
    else:
        sp.timestamp = int(timestamp_us)

    # positions -> NaN (not used)
    sp.position[0] = float('nan')
    sp.position[1] = float('nan')
    sp.position[2] = float('nan')

    # zero velocities
    sp.velocity[0] = 0.0
    sp.velocity[1] = 0.0
    sp.velocity[2] = 0.0

    # accelerations -> NaN
    sp.acceleration[0] = float('nan')
    sp.acceleration[1] = float('nan')
    sp.acceleration[2] = float('nan')

    sp.yaw = float('nan')
    sp.yawspeed = 0.0

    return sp


def publish_burst(publisher, msg, count: int = 3, delay_s: float = 0.01) -> None:
    """
    Publish the same message `count` times, sleeping `delay_s` between publishes.
    This helps ensure late subscribers (or less reliable links) receive a message burst.
    Caller is responsible for choosing safe values for count/delay.
    """
    for _ in range(count):
        # update timestamp if message has timestamp field (common for PX4 msgs)
        try:
            # prefer to update numeric timestamp fields when present
            if hasattr(msg, "timestamp"):
                msg.timestamp = now_us()
        except Exception:
            pass
        publisher.publish(msg)
        time.sleep(delay_s)
