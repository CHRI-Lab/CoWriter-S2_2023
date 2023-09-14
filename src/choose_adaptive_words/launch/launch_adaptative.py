from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="choose_adaptive_words",
                executable="child_ui.py",
                name="child_ui",
            ),
            Node(
                package="choose_adaptive_words",
                executable="manager_ui.py",
                name="manager_ui",
            ),
        ]
    )
