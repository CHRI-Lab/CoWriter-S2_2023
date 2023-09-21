from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="nao_trajectory_following",
                executable="writing_surface_positioner",
                name="writing_surface_positioner",
            ),
            Node(
                package="nao_trajectory_following",
                executable="nao_writer_naoqi",
                name="nao_writer_naoqi",
            ),
        ]
    )
