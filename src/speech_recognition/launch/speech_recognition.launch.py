import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("speech_recognition"),
        "config",
        "speech_recognition.yaml",
    )

    audio_capture_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("audio_capture"), "launch"
                ),
                "/capture.launch.py",
            ]
        ),
        launch_arguments={"format": "wave", "ns": ""}.items(),
    )

    speech_recognition_node = Node(
        package="speech_recognition",
        executable="speech_recognition",
        name="speech_recognition",
        parameters=[config],
    )

    return LaunchDescription(
        [
            audio_capture_node,
            speech_recognition_node,
        ]
    )
