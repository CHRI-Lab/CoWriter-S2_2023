from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="bluering_letter_learning",
                executable="display_manager_server",
                name="display_manager_server",
            ),
            Node(
                package="bluering_letter_learning",
                executable="tablet_input_interpreter",
                name="tablet_input_interpreter",
            ),
            Node(
                package="bluering_letter_learning",
                executable="interactive_learning",
                name="interactive_learning",
            ),
            Node(
                package="bluering_letter_learning",
                executable="diagram_manager",
                name="diagram_manager",
            ),
        ]
    )
