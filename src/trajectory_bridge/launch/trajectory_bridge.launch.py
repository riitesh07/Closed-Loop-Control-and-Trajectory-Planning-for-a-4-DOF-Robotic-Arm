from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="trajectory_bridge",
            executable="trajectory_bridge",
            name="trajectory_bridge",
            output="screen",
        )
    ])

