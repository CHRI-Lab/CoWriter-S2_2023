from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="letter_learning_interaction",
                executable="display_manager_server.py",
                name="display_manager_server",
            ),
            Node(
                package="letter_learning_interaction",
                executable="learning_words_nao.py",
                name="learning_words_nao",
                parameters=[
                    {
                        "writing_surface_frame_id": "writing_surface",
                        "dataset_directory": "default",
                    }
                ],
            ),
            Node(
                package="letter_learning_interaction",
                executable="tablet_input_interpreter.py",
                name="tablet_input_interpreter",
            ),
            Node(
                package="letter_learning_interaction",
                executable="word_card_detector.py",
                name="word_card_detector",
            ),
        ]
    )
