"""
Full robot launch: world + kinematic + climb.

Brings up the swerve stack (controller, IK, EKF, odom, CAN bridge) and the
climb node together. Both swerve_can_node and climb_node open independent
SocketCAN raw sockets on the same can0 interface — that is supported by
SocketCAN and not the source of TX errors. If you see "bus not available"
(ENETDOWN), can0 has gone bus-off; bring it up with auto-restart:

    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 restart-ms 100

Usage:
    ros2 launch world robot.launch.py
    ros2 launch world robot.launch.py field_centric:=false
    ros2 launch world robot.launch.py auto:=true   # also launches auto_climb_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_launch = os.path.join(
        get_package_share_directory('world'), 'launch', 'world.launch.py',
    )
    climb_launch = os.path.join(
        get_package_share_directory('climb'), 'launch', 'climb.launch.py',
    )

    auto_arg = DeclareLaunchArgument(
        'auto', default_value='false',
        description='Set to true to also launch auto_climb_node',
    )

    return LaunchDescription([
        auto_arg,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(world_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(climb_launch),
            launch_arguments={'auto': LaunchConfiguration('auto')}.items(),
        ),
    ])
