"""
Full swerve drive launch: world-package nodes + kinematic support nodes.

Pipeline:
    /field_cmd_vel -> swerve_controller_node -> /cmd_vel -> ik_node -> /wheel_cmd -> swerve_can_node -> CAN
    heading_lock_node -> /heading_omega (mixed by swerve_controller_node)
    ekf_node -> /ekf/pose -> odom_node -> /odom_fast (250 Hz)

Usage:
    ros2 launch world world.launch.py

Override a parameter at launch:
    ros2 launch world world.launch.py field_centric:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    world_cfg = os.path.join(
        get_package_share_directory('world'),
        'config', 'swerve.yaml',
    )

    return LaunchDescription([

        # -- world: field-centric swerve controller --
        Node(
            package='world',
            executable='swerve_controller_node',
            name='swerve_controller_node',
            output='screen',
            parameters=[world_cfg],
        ),

        # -- world: heading PID --
        Node(
            package='world',
            executable='heading_lock_node',
            name='heading_lock_node',
            output='screen',
            parameters=[world_cfg],
        ),

        # -- world: high-rate odometry (250 Hz, anchored to ekf/pose) --
        Node(
            package='world',
            executable='odom_node',
            name='odom_node',
            output='screen',
            parameters=[world_cfg],
        ),

        # -- world: position controller (/target_pose -> /field_cmd_vel) --
        Node(
            package='world',
            executable='pos_controller_node',
            name='pos_controller_node',
            output='screen',
            parameters=[world_cfg],
        ),

        # -- kinematic: EKF (fuses /odom_enc + /imu_yaw -> /ekf/pose) --
        Node(
            package='kinematic',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
        ),

        # -- kinematic: IK solver (/cmd_vel -> /wheel_cmd) --
        Node(
            package='kinematic',
            executable='ik_node',
            name='ik_node',
            output='screen',
            parameters=[{'rate_hz': 100.0}],
        ),

        # -- kinematic: CAN bridge (/wheel_cmd -> CAN -> motors) --
        Node(
            package='kinematic',
            executable='swerve_can_node',
            name='swerve_can_node',
            output='screen',
        ),
    ])
