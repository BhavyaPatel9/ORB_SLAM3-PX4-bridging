# Main MotionControllerNode
#!/usr/bin/env python3
"""
motion_controller/node.py

MotionController Node (complete):
- Subscribes to a "safe" twist topic (default /cmd_vel_safe)
- Applies clamping (max velocity), slew-rate limiting (max accel), watchdog timeout
- Publishes final twist on out_topic (default /cmd_vel)
- Configurable via ROS2 parameters

Usage:
  ros2 run motion_controller node     # after packaging and installing
Or run via a launch file that declares params.

Author: Bhavya Patel
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist
from . import config


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        # ---- parameters (declare defaults) ----
        self.declare_parameter('in_topic', config.SAFE_CMD_VEL_TOPIC)
        self.declare_parameter('out_topic', config.CMD_VEL_TOPIC)
        self.declare_parameter('control_rate', config.CONTROL_RATE)
        self.declare_parameter('max_linear_vel', config.MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_vel', config.MAX_ANGULAR_VEL)
        self.declare_parameter('max_linear_accel', config.MAX_LINEAR_ACCEL)
        self.declare_parameter('max_angular_accel', config.MAX_ANGULAR_ACCEL)
        self.declare_parameter('hold_z', config.HOLD_Z)
        self.declare_parameter('watchdog_timeout', config.WATCHDOG_TIMEOUT)
        self.declare_parameter('publish_zero_on_start', config.PUBLISH_ZERO_ON_START)


        # read parameters to local variables
        self.in_topic: str = self.get_parameter('in_topic').value
        self.out_topic: str = self.get_parameter('out_topic').value
        self.control_rate: float = float(self.get_parameter('control_rate').value)
        self.max_linear_vel: float = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel: float = float(self.get_parameter('max_angular_vel').value)
        self.max_linear_accel: float = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel: float = float(self.get_parameter('max_angular_accel').value)
        self.hold_z: bool = bool(self.get_parameter('hold_z').value)
        self.watchdog_timeout: float = float(self.get_parameter('watchdog_timeout').value)
        self.publish_zero_on_start: bool = bool(self.get_parameter('publish_zero_on_start').value)

        # ---- QoS ----
        qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        durability=QoSDurabilityPolicy.VOLATILE
    )


        # ---- state ----
        self.last_cmd: Twist = Twist()               # last received safe input (target)
        self.last_recv_time: Optional[float] = None  # seconds (monotonic)
        self.last_pub_time: float = self._now_sec()
        # previous published values for slew-rate limiting
        self.prev_linear_x: float = 0.0
        self.prev_linear_y: float = 0.0
        self.prev_linear_z: float = 0.0
        self.prev_angular_z: float = 0.0

        # ---- ROS pubs/subs ----
        self.sub = self.create_subscription(
            Twist, self.in_topic, self._safe_cmd_cb, qos_profile=qos)
        self.pub = self.create_publisher(Twist, self.out_topic, qos)

        # ---- timer ----
        period = 1.0 / max(1.0, float(self.control_rate))
        self.timer = self.create_timer(period, self._control_loop)

        # publish zero initially if requested
        if self.publish_zero_on_start:
            self._publish_zero()

        self.get_logger().info(f"MotionController initialized. in_topic: '{self.in_topic}' -> out_topic: '{self.out_topic}'")
        self.get_logger().info(f"control_rate={self.control_rate}Hz, max_lin={self.max_linear_vel}m/s, max_ang={self.max_angular_vel}rad/s")

    # --------------------------
    # Helpers
    # --------------------------
    def _now_sec(self) -> float:
        """Return current time in seconds (monotonic rcl clock)."""
        return float(self.get_clock().now().nanoseconds) / 1e9

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _sign(x: float) -> float:
        if x > 0.0:
            return 1.0
        if x < 0.0:
            return -1.0
        return 0.0

    # --------------------------
    # Subscriber callback
    # --------------------------
    def _safe_cmd_cb(self, msg: Twist) -> None:
        """Receive safe twist from planner/avoidance and store with timestamp."""
        # make a copy to avoid mutation issues
        self.last_cmd = Twist()
        self.last_cmd.linear.x = float(msg.linear.x)
        self.last_cmd.linear.y = float(msg.linear.y)
        self.last_cmd.linear.z = float(msg.linear.z)
        self.last_cmd.angular.x = float(msg.angular.x)
        self.last_cmd.angular.y = float(msg.angular.y)
        self.last_cmd.angular.z = float(msg.angular.z)
        self.last_recv_time = self._now_sec()
        # debug logging at low frequency only
        # self.get_logger().debug(f"Got safe cmd: lx={self.last_cmd.linear.x:.3f}, ly={self.last_cmd.linear.y:.3f}, az={self.last_cmd.angular.z:.3f}")

    # --------------------------
    # Main control loop
    # --------------------------
    def _control_loop(self) -> None:
        now = self._now_sec()
        dt = now - self.last_pub_time if self.last_pub_time is not None else (1.0 / max(1.0, self.control_rate))
        if dt <= 0.0:
            dt = 1.0 / max(1.0, self.control_rate)

        # determine target command: if watchdog timed out -> zeros
        timed_out = True
        if self.last_recv_time is not None and (now - self.last_recv_time) <= self.watchdog_timeout:
            timed_out = False

        if timed_out:
            target_lin_x = 0.0
            target_lin_y = 0.0
            target_lin_z = 0.0
            target_ang_z = 0.0
            # log once on timeout
            # Use warn level but keep it compact
            # Only warn if cmd wasn't already zero
            if any(abs(v) > 1e-6 for v in [self.prev_linear_x, self.prev_linear_y, self.prev_linear_z, self.prev_angular_z]):
                self.get_logger().warn("Watchdog: no safe command received recently — publishing zero velocities")
        else:
            # Read target from last_cmd and clamp to configured maxes
            target_lin_x = self._clamp(self.last_cmd.linear.x, -self.max_linear_vel, self.max_linear_vel)
            target_lin_y = self._clamp(self.last_cmd.linear.y, -self.max_linear_vel, self.max_linear_vel)
            #target_lin_z = self.last_cmd.linear.z if not self.hold_z else 0.0
            # if hold_z set to zero and last_cmd had something, we ignore vertical component
            if self.hold_z:
                target_lin_z = 0.0
            else:
                target_lin_z = self._clamp(self.last_cmd.linear.z,-self.max_linear_vel,self.max_linear_vel)

            target_ang_z = self._clamp(self.last_cmd.angular.z,-self.max_angular_vel,self.max_angular_vel)
        # Apply slew-rate limiting (acceleration constraints)
        # linear x
        max_dvx = self.max_linear_accel * dt
        dvx = target_lin_x - self.prev_linear_x
        if abs(dvx) > max_dvx:
            dvx = self._sign(dvx) * max_dvx
        new_lin_x = self.prev_linear_x + dvx

        # linear y
        max_dvy = self.max_linear_accel * dt
        dvy = target_lin_y - self.prev_linear_y
        if abs(dvy) > max_dvy:
            dvy = self._sign(dvy) * max_dvy
        new_lin_y = self.prev_linear_y + dvy

        # linear z
        max_dvz = self.max_linear_accel * dt
        dvz = target_lin_z - self.prev_linear_z
        if abs(dvz) > max_dvz:
            dvz = self._sign(dvz) * max_dvz
        new_lin_z = self.prev_linear_z + dvz

        # angular z
        max_daz = self.max_angular_accel * dt
        daz = target_ang_z - self.prev_angular_z
        if abs(daz) > max_daz:
            daz = self._sign(daz) * max_daz
        new_ang_z = self.prev_angular_z + daz

        # Final safety clamp (numerical safety)
        new_lin_x = self._clamp(new_lin_x, -self.max_linear_vel, self.max_linear_vel)
        new_lin_y = self._clamp(new_lin_y, -self.max_linear_vel, self.max_linear_vel)
        new_lin_z = self._clamp(new_lin_z, -self.max_linear_vel, self.max_linear_vel)
        new_ang_z = self._clamp(new_ang_z, -self.max_angular_vel, self.max_angular_vel)

        # Publish Twist
        out = Twist()
        out.linear.x = float(new_lin_x)
        out.linear.y = float(new_lin_y)
        out.linear.z = float(new_lin_z if not self.hold_z else 0.0)
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = float(new_ang_z)

        self.pub.publish(out)

        # update previous and time
        self.prev_linear_x = new_lin_x
        self.prev_linear_y = new_lin_y
        self.prev_linear_z = new_lin_z
        self.prev_angular_z = new_ang_z
        self.last_pub_time = now

    # --------------------------
    # Utility: publish zero
    # --------------------------
    def _publish_zero(self) -> None:
        z = Twist()
        z.linear.x = 0.0
        z.linear.y = 0.0
        z.linear.z = 0.0
        z.angular.x = 0.0
        z.angular.y = 0.0
        z.angular.z = 0.0
        # publish multiple times to ensure late subscribers receive a zero (small burst)
        for _ in range(3):
            self.pub.publish(z)
            time.sleep(0.01)

    # --------------------------
    # Clean shutdown helper
    # --------------------------
    def destroy_node(self):
        # publish zeros and then call base class destroy
        try:
            self.get_logger().info("MotionController shutting down: publishing zero command")
            self._publish_zero()
        except Exception as e:
            self.get_logger().warn(f"Exception while publishing zero on shutdown: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt — shutting down MotionController")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
