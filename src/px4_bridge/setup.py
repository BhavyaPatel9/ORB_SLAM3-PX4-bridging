# setup.py for px4_bridge
from setuptools import setup
from glob import glob
import os

package_name = 'px4_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # Param files
        (os.path.join('share', package_name, 'params'),
         glob('params/*.yaml')),

        # Scripts
        (os.path.join('share', package_name, 'scripts'),
         glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spitze',
    maintainer_email='your@email.com',
    description='PX4 Offboard bridge for ROS2 using XRCE-DDS',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_bridge_node = px4_bridge.node:main',
        ],
    },
)
