# ORB_SLAM3-PX4-bridging


# 1.motion_controller

Lightweight ROS2 node that safely filters velocity commands (`geometry_msgs/Twist`) coming from a planner / SLAM stack and publishes smoothed, limited commands for the low-level controller or PX4.

## Key features
- Velocity clamping (max linear / angular speeds)
- Slew-rate (acceleration) limiting to prevent jerks
- Watchdog timeout → publishes zero if input lost
- Optional vertical hold (ignore Z)
- Safe zero-publish on startup and shutdown

## Topics
- **Subscribed**
  - `<in_topic>` (default: `/cmd_vel_safe`) — `geometry_msgs/Twist`
- **Published**
  - `<out_topic>` (default: `/cmd_vel`) — `geometry_msgs/Twist`

> Defaults are defined in `config.py`. Typical defaults:
> `in_topic=/cmd_vel_safe`, `out_topic=/cmd_vel`, `control_rate=20.0`, `max_linear_vel=1.0` m/s, `max_angular_vel=1.0` rad/s, `max_linear_accel=0.5` m/s², `max_angular_accel=1.0` rad/s, `watchdog_timeout=0.5` s, `hold_z=True`.

## Installation
```bash
# from workspace root
cd ~/your_ros2_ws
# copy repo into src if not already
# then:
colcon build
source install/setup.bash
````

## Launch / Run

Run directly:

```bash
ros2 run motion_controller motion_controller_node
```

Example launch snippet (ROS2 Python launch):

```xml
# (example) launch/motion_controller.launch.py
# set params: in_topic, out_topic, control_rate, etc.
```

## Parameters

| Name                    |   Type | Description                        |
| ----------------------- | -----: | ---------------------------------- |
| `in_topic`              | string | Input safe Twist topic             |
| `out_topic`             | string | Output Twist topic                 |
| `control_rate`          |  float | Control loop frequency (Hz)        |
| `max_linear_vel`        |  float | Max linear speed (m/s)             |
| `max_angular_vel`       |  float | Max angular speed (rad/s)          |
| `max_linear_accel`      |  float | Max linear accel (m/s²)            |
| `max_angular_accel`     |  float | Max angular accel (rad/s²)         |
| `hold_z`                |   bool | If true, vertical commands ignored |
| `watchdog_timeout`      |  float | Seconds before publishing zeros    |
| `publish_zero_on_start` |   bool | Publish zeros at startup           |

## Input / Output examples

**Input** (`/cmd_vel_safe`):

```yaml
geometry_msgs/Twist
linear: {x: 1.2, y: 0.5, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 1.5}
```

**Output** (`/cmd_vel`) — after clamping and slewrate limiting:

```yaml
geometry_msgs/Twist
linear: {x: 0.50, y: 0.20, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.12}
```

## Diagnostics & troubleshooting

* If the node continually publishes zeros: ensure the planner/safe source publishes within `watchdog_timeout`.
* Tune `max_*` params to match robot / PX4 capabilities.
* Check QoS compatibility with publishers/subscribers if messages don't appear.



---

# 2.px4_bridge

Bridge between a SLAM/vision pipeline and PX4 via `px4_msgs` (ROS2). Converts SLAM poses & planner commands into PX4-compatible topics (which map to uORB inside PX4) so the autopilot estimator and offboard controller can consume them.

## Responsibilities
- Convert SLAM pose (`geometry_msgs/PoseStamped` / `nav_msgs/Odometry`) → `px4_msgs/VehicleVisualOdometry` (published to `/fmu/in/vehicle_visual_odometry`)
- Publish position setpoints as `px4_msgs/TrajectorySetpoint` → `/fmu/in/trajectory_setpoint`
- Publish `px4_msgs/OffboardControlMode` → `/fmu/in/offboard_control_mode`
- Publish `px4_msgs/VehicleCommand` → `/fmu/in/vehicle_command` for arm/mode/takeoff commands
- Coordinate frame conversion (common: camera/ENU → PX4 NED). See NOTE below.

## Important topics (ROS2 ↔ PX4)
- `/fmu/in/vehicle_visual_odometry`  ← SLAM pose → used by EKF/VIO
- `/fmu/in/trajectory_setpoint`     ← desired position setpoints (NED)
- `/fmu/in/offboard_control_mode`    ← enable/describe offboard control type
- `/fmu/in/vehicle_command`          ← arm, disarm, set mode, takeoff, land
- `/fmu/out/vehicle_status`          → monitor arming/nav state

## Requirements
- ROS2 (Foxy/Humble) — use ROS2 matching your system
- `px4_msgs` matching your PX4 version (e.g. `release/1.14` for PX4 v1.14) — **must match** PX4-Autopilot
- Micro XRCE-DDS Agent running and PX4 connected (for SITL)
- PX4 SITL (if testing in Gazebo)

## Quick start (SITL + bridge)
1. Start PX4 SITL (example for v1.14 + gazebo):
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
````

2. Start MicroXRCEAgent:

```bash
MicroXRCEAgent udp4 -p 8888
```

3. Build & source your ROS2 workspace (bridge + px4_msgs):

```bash
colcon build
source install/setup.bash
```

4. Run bridge node / launch file:

```bash
ros2 launch px4_bridge px4_bridge.launch.py
```

5. Confirm topics:

```bash
ros2 topic list | grep fmu
# expect /fmu/in/vehicle_visual_odometry, /fmu/in/trajectory_setpoint, ...
```

## Coordinate frames & transformation

* SLAM usually outputs poses in the **camera** or **ENU** frame. PX4 expects **NED** for many uORB messages.
* The bridge performs the required transform (example):

  * `x_ned =  y_enu`
  * `y_ned =  x_enu`
  * `z_ned = -z_enu`
* Verify frame conventions in your SLAM pipeline and adjust conversion accordingly.

## Example usage: takeoff (offboard)

1. Continuously publish `OffboardControlMode` and `TrajectorySetpoint` (2+ Hz) with a safe hover target (e.g., z = -2).
2. Arm:

```bash
ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "command: 400
param1: 1.0" -1
```

3. Set offboard mode (or use `commander mode offboard` in PX4 console). PX4 will then accept setpoints and climb to the requested position.

## Troubleshooting

* **Topics not visible**: ensure `px4_msgs` version in ROS matches PX4 SITL version.
* **Offboard rejected**: make sure setpoints are published BEFORE requesting OFFBOARD and at >= 2 Hz.
* **Time sync / time jump warnings**: start PX4 SITL first, then MicroXRCEAgent, then ROS2 bridge. Avoid restarting the agent while PX4 runs.
* **Yaw estimate / preflight fails**: wait for EKF to initialize, or for SITL testing set `COM_ARM_WO_GPS=1` (only for testing).

## Development notes

* Ensure unit tests for conversion & message fields if you modify message mapping.
* Keep `px4_msgs` as a single copy in your workspace (avoid conflicting underlays/overlays).
