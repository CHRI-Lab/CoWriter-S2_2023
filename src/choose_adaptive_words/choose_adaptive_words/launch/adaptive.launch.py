from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="choose_adaptive_words",
                executable="backend",
                name="backend",
            ),
        ]
    )
