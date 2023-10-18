#!/usr/bin/env python3
# coding: utf-8

from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool, Float64MultiArray, String
from .text_shaper import ScreenManager, TextShaper
from .watchdog import Watchdog
from .interaction_settings import InteractionSettings
from .shape_learner_manager import ShapeLearnerManager


class PublisherManager:
    """
    Class to manage publishers, can be passed to other classes as
    necessary in order to avoid passing lots of publishers to
    multiple classes.
    """

    def __init__(self, ros_node: Node):
        self.ros_node = ros_node
        # Controls the camera based on the interaction state
        # (turn it off for writing b/c CPU gets maxed)
        self.PUBLISH_STATUS_TOPIC = "camera_publishing_status"
        # self.ros_node.get_parameter(
        #     "camera_publishing_status_topic", "camera_publishing_status"
        # )
        # Name of topic to publish shapes to
        self.SHAPE_TOPIC = "/write_traj"
        # self.ros_node.get_parameter(
        #     "~trajectory_output_topic", "/write_traj"
        # )

        # Name of topic to publish bounding boxes of letters to
        self.BOUNDING_BOXES_TOPIC = "/boxes_to_draw"
        # self.ros_node.get_parameter(
        #     "~bounding_boxes_topic", "/boxes_to_draw"
        # )

        # Name of topic to publish downsampled shapes to
        self.SHAPE_TOPIC_DOWNSAMPLED = "/write_traj_downsampled"
        # self.ros_node.get_parameter(
        #     "~trajectory_output_nao_topic", "/write_traj_downsampled"
        # )

        # Clear tablet publisher topic
        self.CLEAR_SURFACE_TOPIC = "clear_screen"
        # self.ros_node.get_parameter(
        #     "~clear_writing_surface_topic", "clear_screen"
        # )

        # Signal speech recognition to start listening
        self.LISTENING_SIGNAL_TOPIC = "listening_signal"

    def init_publishers(self):
        """
        Initialize all publishers for the interaction.
        """

        # Init publishers
        self.pub_camera_status = self.ros_node.create_publisher(
            Bool, self.PUBLISH_STATUS_TOPIC, 10
        )

        self.pub_traj = self.ros_node.create_publisher(
            Path, self.SHAPE_TOPIC, 10
        )

        self.pub_bounding_boxes = self.ros_node.create_publisher(
            Float64MultiArray, self.BOUNDING_BOXES_TOPIC, 10
        )

        self.pub_traj_downsampled = self.ros_node.create_publisher(
            Path, self.SHAPE_TOPIC_DOWNSAMPLED, 10
        )

        self.pub_clear = self.ros_node.create_publisher(
            Empty, self.CLEAR_SURFACE_TOPIC, 10
        )

        self.pub_listening_signal = self.ros_node.create_publisher(
            String, self.LISTENING_SIGNAL_TOPIC, 10
        )


class SubscriberTopics:
    """
    Class to store subscriber topic names as constants.
    """

    def __init__(self, ros_node: Node) -> None:
        self.ros_node = ros_node
        self.CLEAR_SURFACE_TOPIC: str = "clear_screen"
        # self.ros_node.get_parameter(
        #     "~clear_writing_surface_topic", "clear_screen"
        # )

        # Welcome a new teacher but don't reset learning algorithm's 'memory'
        self.NEW_CHILD_TOPIC: str = "new_child"
        # self.ros_node.get_parameter(
        #     "~new_teacher_topic", "new_child"
        # )

        self.WORDS_TOPIC: str = "words_to_write"
        # self.ros_node.get_parameter(
        #     "~words_to_write_topic", "words_to_write"
        # )

        # Name of topic to listen for when test card has been shown to the robot
        self.TEST_TOPIC: str = "test_learning"
        # self.ros_node.get_parameter(
        #     "~test_request_topic", "test_learning"
        # )

        # Name of topic to listen for when stop card has been shown to the robot
        self.STOP_TOPIC: str = "stop_learning"
        # self.ros_node.get_parameter(
        #     "~stop_request_topic", "stop_learning"
        # )

        # Name of topic to listen for user shapes
        self.PROCESSED_USER_SHAPE_TOPIC: str = "user_shapes_processed"
        # self.ros_node.get_parameter(
        #     "~processed_user_shape_topic", "user_shapes_processed"
        # )

        # Name of topic to get gestures representing the active shape for demonstration
        self.GESTURE_TOPIC: str = "gesture_info"
        # self.ros_node.get_parameter(
        #     "~gesture_info_topic", "gesture_info"
        # )

        # tablet param
        self.SHAPE_FINISHED_TOPIC = "shape_finished"
        # self.ros_node.get_parameter(
        #     "~shape_writing_finished_topic", "shape_finished"
        # )


class DeviceManager:
    """
    At this stage this class is just a wrapper for various utilities
    related to the tablet and shape management.

    Right now, all params are initialised within the class. It may be
    better to pass these as params to the constructor eventually, but
    the file is currently using default parameters for these, and doing
    it this way makes it easier to update the rest of the file (error
    highlighting should show where they are being called, since they
    are no longer global variables)
    """

    def __init__(self, ros_node: Node):
        self.ros_node = ros_node
        self.SHAPE_LOGGING_PATH = ""
        # self.ros_node.get_parameter("~shape_log", "")
        self.word_manager = ShapeLearnerManager(
            InteractionSettings.generate_settings,  # self.SHAPE_LOGGING_PATH
        )

        self.tablet_watchdog = Watchdog(
            "watchdog_clear/tablet", 0.4, self.ros_node
        )
        self.screen_manager = ScreenManager(0.2, 0.1395)
        self.text_shaper: TextShaper = TextShaper()
