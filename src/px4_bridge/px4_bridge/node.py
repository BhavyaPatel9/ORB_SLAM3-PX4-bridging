#!/usr/bin/env python3
"""
px4_bridge/node.py

PX4BridgeNode (FSM Version)
---------------------------
Bridges ROS2 Twist (body FLU) → PX4 Offboard velocity (world NED).

FSM States:
IDLE -> ARMING -> OFFBOARD

Behavior:
- Starts streaming setpoints immediately
- After few setpoints → arms
- After armed → switches to OFFBOARD
- Maintains OFFBOARD while commands active
- On timeout → sends zero velocity (but keeps streaming)
"""

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleAttitude,
)

from . import config
from .utils import now_us, quaternion_to_yaw
from .transforms import body_flu_to_world_ned


class PX4BridgeNode(Node):

    # =============================
    # INIT
    # =============================

    def __init__(self):
        super().__init__("px4_bridge")

        # -------- Parameters --------
        self.declare_parameter("cmd_vel_topic", config.CMD_VEL_TOPIC)
        self.declare_parameter("cmd_timeout", config.WATCHDOG_TIMEOUT)
        self.declare_parameter("control_rate", config.OFFBOARD_PUBLISH_RATE)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.control_rate = float(self.get_parameter("control_rate").value)

        # -------- QoS (PX4 requires BEST_EFFORT) --------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # -------- State --------
        self.current_yaw: float = 0.0
        self.last_cmd: Optional[Twist] = None
        self.last_cmd_time: Optional[float] = None
        self.vehicle_status: Optional[VehicleStatus] = None

        self.state = "IDLE"
        self.setpoint_counter = 0

        # -------- Subscribers --------
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(
            VehicleStatus,
            config.VEHICLE_STATUS_TOPIC,
            self.vehicle_status_callback,
            qos,
        )
        self.create_subscription(
            VehicleAttitude,
            config.VEHICLE_ATTITUDE_TOPIC,
            self.vehicle_attitude_callback,
            qos,
        )

        # -------- Publishers --------
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode,
            config.OFFBOARD_CONTROL_MODE_TOPIC,
            qos,
        )

        self.pub_trajectory = self.create_publisher(
            TrajectorySetpoint,
            config.TRAJECTORY_SETPOINT_TOPIC,
            qos,
        )

        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand,
            config.VEHICLE_COMMAND_TOPIC,
            10,
        )

        # -------- Timer --------
        period = 1.0 / max(1.0, self.control_rate)
        self.create_timer(period, self.control_loop)

        self.get_logger().info("PX4 Bridge (FSM) Initialized")

    # ==========================================================
    # CALLBACKS
    # ==========================================================

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        self.current_yaw = quaternion_to_yaw(msg.q)

    # ==========================================================
    # MAIN CONTROL LOOP
    # ==========================================================

    def control_loop(self):

        now_sec = self.get_clock().now().nanoseconds / 1e9
        timestamp = now_us()

        # -------------------------------------------------
        # Always publish OffboardControlMode (required!)
        # -------------------------------------------------
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = timestamp
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        self.pub_offboard_mode.publish(offboard_msg)

        # -------------------------------------------------
        # Determine target velocity
        # -------------------------------------------------
        vx = vy = vz = 0.0
        yawspeed = 0.0

        if (
            self.last_cmd is not None
            and self.last_cmd_time is not None
            and (now_sec - self.last_cmd_time) <= self.cmd_timeout
        ):
            vx, vy, vz = body_flu_to_world_ned(
                self.last_cmd.linear.x,
                self.last_cmd.linear.y,
                self.last_cmd.linear.z,
                self.current_yaw,
            )
            yawspeed = self.last_cmd.angular.z

        # -------------------------------------------------
        # Publish Trajectory Setpoint
        # -------------------------------------------------
        sp = TrajectorySetpoint()
        sp.timestamp = timestamp

        sp.velocity[0] = vx
        sp.velocity[1] = vy
        sp.velocity[2] = vz

        sp.position[0] = float("nan")
        sp.position[1] = float("nan")
        sp.position[2] = float("nan")

        sp.acceleration[0] = float("nan")
        sp.acceleration[1] = float("nan")
        sp.acceleration[2] = float("nan")

        sp.yaw = float("nan")
        sp.yawspeed = yawspeed

        self.pub_trajectory.publish(sp)

        # -------------------------------------------------
        # FSM Logic
        # -------------------------------------------------

        if self.state == "IDLE":
            self.setpoint_counter += 1
            if self.setpoint_counter > 10:
                self.get_logger().info("Arming vehicle")
                self.arm()
                self.state = "ARMING"

        elif self.state == "ARMING":
            if (
                self.vehicle_status
                and self.vehicle_status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED
            ):
                self.get_logger().info("Switching to OFFBOARD")
                self.set_offboard_mode()
                self.state = "OFFBOARD"

        elif self.state == "OFFBOARD":
            # Stay in OFFBOARD as long as stream continues
            pass

    # ==========================================================
    # COMMAND HELPERS
    # ==========================================================

    def arm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def set_offboard_mode(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = now_us()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_vehicle_cmd.publish(msg)

    # ==========================================================
    # Shutdown
    # ==========================================================

    def destroy_node(self):
        self.get_logger().info("PX4 Bridge shutting down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PX4BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
