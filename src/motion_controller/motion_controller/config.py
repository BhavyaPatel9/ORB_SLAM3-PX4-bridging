# Topic names / defaults
"""
motion_controller/config.py

Central configuration defaults for MotionController.
These values are used as default parameters but can be overridden
via ROS2 parameter YAML or launch files.
"""

# -----------------------------
# Topic Names
# -----------------------------
SAFE_CMD_VEL_TOPIC = "/cmd_vel_safe"
CMD_VEL_TOPIC = "/cmd_vel"

# -----------------------------
# Control Parameters
# -----------------------------
CONTROL_RATE = 20.0           # Hz

# -----------------------------
# Velocity Limits
# -----------------------------
MAX_LINEAR_VEL = 1.5          # m/s
MAX_ANGULAR_VEL = 1.5         # rad/s

# -----------------------------
# Acceleration Limits (Slew)
# -----------------------------
MAX_LINEAR_ACCEL = 1.0        # m/s^2
MAX_ANGULAR_ACCEL = 2.0       # rad/s^2

# -----------------------------
# Safety
# -----------------------------
WATCHDOG_TIMEOUT = 0.5        # seconds
HOLD_Z = True                 # force vertical velocity to zero

# -----------------------------
# Startup Behavior
# -----------------------------
PUBLISH_ZERO_ON_START = True
