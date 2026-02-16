# Launch file for px4_bridge
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _maybe_start_xrce(context, package_dir):
    """
    Helper executed at launch-time to start XRCE agent if script exists.
    We use ExecuteProcess in a separate terminal to run the script so it's optional.
    """
    script_path = os.path.join(package_dir, 'scripts', 'start_xrce_agent.sh')
    # Only add the ExecuteProcess if the script actually exists
    if os.path.exists(script_path):
        return [ExecuteProcess(
            cmd=['bash', '-c', f"bash '{script_path}'"],
            output='screen',
            shell=False
        )]
    return []


def generate_launch_description():
    package_dir = get_package_share_directory('px4_bridge')
    params_file = os.path.join(package_dir, 'params', 'px4_bridge.yaml')

    launch_actions = []

    # Optionally start XRCE agent (only if the script exists)
    launch_actions += _maybe_start_xrce(None, package_dir)

    # PX4 Bridge node
    bridge_node = Node(
        package='px4_bridge',
        executable='px4_bridge_node',   # must match setup.py entry_point
        name='px4_bridge',
        output='screen',
        parameters=[params_file],
        # remappings if you want to change topic names at launch:
        # remappings=[('cmd_vel', '/cmd_vel')]
    )

    launch_actions.append(bridge_node)

    return LaunchDescription(launch_actions)
