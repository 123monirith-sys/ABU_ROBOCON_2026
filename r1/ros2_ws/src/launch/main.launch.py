from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_pkg',
            executable='canrx_node',
            name='canrx_node'
        ),

        Node(
            package='robot_pkg',
            executable='cantx_node',
            name='cantx_node'
        ),

        Node(
            package='robot_pkg',
            executable='main_node',
            name='main_node'
        )
    ])