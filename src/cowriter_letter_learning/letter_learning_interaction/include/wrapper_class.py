#!/usr/bin/env python3
# coding: utf-8

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool, Float64MultiArray
import sys
import os.path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from text_shaper import ScreenManager, TextShaper
from watchdog import Watchdog
from interaction_settings import InteractionSettings
from shape_learner_manager import ShapeLearnerManager


class PublisherManager:
    """
    Class to manage publishers, can be passed to other classes as
    necessary in order to avoid passing lots of publishers to
    multiple classes.
    """

    def __init__(self):

        # Controls the camera based on the interaction state
        # (turn it off for writing b/c CPU gets maxed)
        self.PUBLISH_STATUS_TOPIC = rospy.get_param('~camera_publishing_status_topic',
                                                    'camera_publishing_status')
        # Name of topic to publish shapes to
        self.SHAPE_TOPIC = rospy.get_param('~trajectory_output_topic',
                                           '/write_traj')

        # Name of topic to publish bounding boxes of letters to
        self.BOUNDING_BOXES_TOPIC = rospy.get_param('~bounding_boxes_topic',
                                                    '/boxes_to_draw')

        # Name of topic to publish downsampled shapes to
        self.SHAPE_TOPIC_DOWNSAMPLED = rospy.get_param('~trajectory_output_nao_topic',
                                                       '/write_traj_downsampled')

        # Clear tablet publisher topic
        self.CLEAR_SURFACE_TOPIC = rospy.get_param('~clear_writing_surface_topic',
                                                   'clear_screen')

    def init_publishers(self):
        """
        Initialize all publishers for the interaction.
        """

        # Init publishers
        self.pub_camera_status = rospy.Publisher(self.PUBLISH_STATUS_TOPIC,
                                                 Bool, queue_size=10)

        self.pub_traj = rospy.Publisher(self.SHAPE_TOPIC, Path, queue_size=10)

        self.pub_bounding_boxes = rospy.Publisher(self.BOUNDING_BOXES_TOPIC,
                                                  Float64MultiArray,
                                                  queue_size=10)

        self.pub_traj_downsampled = rospy.Publisher(self.SHAPE_TOPIC_DOWNSAMPLED,
                                                    Path, queue_size=10)

        self.pub_clear = rospy.Publisher(self.CLEAR_SURFACE_TOPIC, Empty,
                                         queue_size=10)


class SubscriberTopics:
    """
    Class to store subscriber topic names as constants.
    """

    CLEAR_SURFACE_TOPIC: str = rospy.get_param('~clear_writing_surface_topic',
                                               'clear_screen')

    # Welcome a new teacher but don't reset learning algorithm's 'memory'
    NEW_CHILD_TOPIC: str = rospy.get_param('~new_teacher_topic', 'new_child')

    WORDS_TOPIC: str = rospy.get_param(
        '~words_to_write_topic', 'words_to_write')

    # Name of topic to listen for when test card has been shown to the robot
    TEST_TOPIC: str = rospy.get_param('~test_request_topic', 'test_learning')

    # Name of topic to listen for when stop card has been shown to the robot
    STOP_TOPIC: str = rospy.get_param('~stop_request_topic', 'stop_learning')

    # Name of topic to listen for user shapes
    PROCESSED_USER_SHAPE_TOPIC: str = rospy.get_param(
        '~processed_user_shape_topic', 'user_shapes_processed')

    # Name of topic to get gestures representing the active shape for demonstration
    GESTURE_TOPIC: str = rospy.get_param('~gesture_info_topic', 'gesture_info')

    # tablet param
    SHAPE_FINISHED_TOPIC = rospy.get_param('~shape_writing_finished_topic',
                                           'shape_finished')


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

    def __init__(self):

        self.SHAPE_LOGGING_PATH = rospy.get_param('~shape_log', '')
        self.word_manager = ShapeLearnerManager(
            InteractionSettings.generate_settings,
            self.SHAPE_LOGGING_PATH)

        self.tablet_watchdog = Watchdog('watchdog_clear/tablet', 0.4)
        self.screen_manager = ScreenManager(0.2, 0.1395)
        self.text_shaper: TextShaper = TextShaper()
