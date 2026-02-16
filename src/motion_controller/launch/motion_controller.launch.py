# Launch file for motion_controller
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_dir = get_package_share_directory('motion_controller')

    params_file = os.path.join(
        package_dir,
        'params',
        'motion_controller.yaml'
    )

    return LaunchDescription([

        Node(
            package='motion_controller',
            executable='motion_controller_node',  # must match setup.py entry_point
            name='motion_controller',
            output='screen',
            parameters=[params_file]
        )

    ])
