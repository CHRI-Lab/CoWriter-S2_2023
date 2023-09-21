import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    choose_adaptive_words = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("choose_adaptive_words"),
                    "launch",
                ),
                "/adaptive.launch.py",
            ]
        )
    )
    letter_learning_interaction = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("letter_learning_interaction"),
                    "launch",
                ),
                "/cowriter.launch.py",
            ]
        )
    ),
    nao_trajectory_following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nao_trajectory_following"),
                    "launch",
                ),
                "/trajectory.launch.py",
            ]
        )
    )
    ld.add_action(choose_adaptive_words)
    ld.add_action(letter_learning_interaction)
    ld.add_action(nao_trajectory_following)
    return ld
