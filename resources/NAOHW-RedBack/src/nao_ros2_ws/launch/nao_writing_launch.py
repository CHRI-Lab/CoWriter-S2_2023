from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nao_writing',
            namespace='nao_writing',
            executable='input_interpreter',
            name='input_interpreter'
        ),
        Node(
            package='nao_writing',
            namespace='nao_writing',
            executable='nao_writer_naoqi',
            name='nao_writer_naoqi'
        ),
        Node(
            package='nao_writing',
            namespace='nao_writing',
            executable='learning_word',
            name='learning_word'
        ),
        Node(
            package='nao_writing',
            namespace='nao_writing',
            executable='audio_chat',
            name='audio_chat'
        ),
    ])