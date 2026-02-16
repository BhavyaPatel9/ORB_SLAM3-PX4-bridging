# Topic names / defaults
"""
px4_bridge/config.py

Default configuration values for px4_bridge node.
These are sane defaults and can be overridden via ROS2 params YAML.
"""

# Topics (incoming)
CMD_VEL_TOPIC = "/cmd_vel"                   # input from motion_controller
VEHICLE_STATUS_TOPIC = "/fmu/out/vehicle_status"
VEHICLE_ATTITUDE_TOPIC = "/fmu/out/vehicle_attitude"

# Topics (outgoing to PX4)
OFFBOARD_CONTROL_MODE_TOPIC = "/fmu/in/offboard_control_mode"
TRAJECTORY_SETPOINT_TOPIC = "/fmu/in/trajectory_setpoint"
VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"

# General parameters
OFFBOARD_PUBLISH_RATE = 50.0    # Hz at which OffboardControlMode + setpoints are published
OFFBOARD_ENABLE_ON_CMD = True   # start OFFBOARD when cmd_vel received
WATCHDOG_TIMEOUT = 0.5         # seconds: if no cmd_vel for this long, treat as stopped
MIN_OFFBOARD_RATE = 2.0        # PX4 requires > 2 Hz setpoint stream

# Frame assumptions
# motion_controller publishes FLU (forward-left-up) body-frame twist (linear m/s, angular rad/s)
INPUT_FRAME = "FLU_body"

# Safety / behavior
PUBLISH_ZERO_AFTER_TIMEOUT = True
ZERO_BURST_COUNT = 3           # publish zeros this many times on shutdown / timeout
TIMESTAMP_US = True            # publish timestamps in microseconds (PX4 expects us to)
