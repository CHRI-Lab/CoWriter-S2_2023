from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("choose_adaptative_words"),
                        "/launch/launch_adaptative.py",
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("cowriter_letter_learning"),
                        "letter_learning_interaction/launch/package_2_launch.py",
                    ]
                ),
            ),
        ]
    )
