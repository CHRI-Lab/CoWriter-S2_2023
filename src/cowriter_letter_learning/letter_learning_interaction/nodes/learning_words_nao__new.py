#!/usr/bin/env python3
# coding: utf-8

import os.path
import logging
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
from scipy import interpolate
from copy import deepcopy
import sys
import rclpy
from rclpy.node import Node

# for normalise_shape_height()
sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../include")
)
from wrapper_class import DeviceManager, PublisherManager, SubscriberTopics
from nao_settings import NaoSettings, PhraseManagerGPT
from watchdog import Watchdog

from interaction_settings import InteractionSettings
from state_machine import StateMachine
from shape_modeler import ShapeModeler

from letter_learning_interaction.msg import Shape as ShapeMsg  # type: ignore
from letter_learning_interaction.srv import *


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from std_msgs.msg import (
    String,
    Empty,
    Bool,
    Float64MultiArray,
    MultiArrayDimension,
)


class LearningWordsNao(Node):
    def __init__(self):
        super().__init__("learning_words_nao")


if __name__ == "__main__":
    # Init node inside main to avoid running node if imported
    # rospy.init_node("learning_words_nao")
    rclpy.init(args=None)
    node = LearningWordsNao()
    node = LearningWordsNao()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    dataset_directory = rospy.get_param("~dataset_directory", "default")
    if dataset_directory.lower() == "default":  # use default
        import inspect

        # TODO: letters templates must be in share files of Allograph
        try:
            file_name = inspect.getsourcefile(ShapeModeler)
            install_directory = file_name.split("/lib")[0]  # type: ignore
            dataset_directory = (
                install_directory
                + "/share/shape_learning/letter_model_datasets/uji_pen_chars2"
            )
        except:
            RuntimeError("Missing Dataset")

    # Init state machine
    state_machine = StateMachine()

    # Init objects for interaction
    nao_settings = NaoSettings()
    device_manager = DeviceManager()
    publish_manager = PublisherManager()
    managerGPT = PhraseManagerGPT("English")

    subscriber_callbacks = SubscriberCallbacks(
        nao_settings, device_manager, managerGPT, publish_manager, state_machine
    )

    generated_word_logger = logging.getLogger("word_logger")
    # HACK: should properly configure the path from an option
    generated_word_logger = configure_logging(generated_word_logger)

    state_manager = StateManager(
        nao_settings,
        device_manager,
        publish_manager,
        subscriber_callbacks,
        generated_word_logger,
    )

    # Add interaction states to state machine
    state_machine.add_state(
        "STARTING_INTERACTION", state_manager.start_interaction
    )
    state_machine.add_state("WAITING_FOR_WORD", state_manager.wait_for_word)
    state_machine.add_state(
        "RESPONDING_TO_NEW_WORD", state_manager.respond_to_new_word
    )
    state_machine.add_state("PUBLISHING_WORD", state_manager.publish_word)
    state_machine.add_state(
        "WAITING_FOR_LETTER_TO_FINISH", state_manager.wait_for_shape_to_finish
    )
    state_machine.add_state(
        "ASKING_FOR_FEEDBACK", state_manager.ask_for_feedback
    )
    state_machine.add_state(
        "WAITING_FOR_FEEDBACK", state_manager.wait_for_feedback
    )
    state_machine.add_state(
        "RESPONDING_TO_DEMONSTRATION_FULL_WORD",
        state_manager.respond_to_demonstration_with_full_word,
    )
    state_machine.add_state("STOPPING", state_manager.stop_interaction)
    state_machine.add_state("EXIT", None, end_state=True)
    state_machine.set_start("STARTING_INTERACTION")
    info_for_start_state = {"state_came_from": None}

    # Set nao up for interaction
    nao_settings.set_nao_interaction()

    # Init subscribers
    # listen for a new child signal
    new_child_subscriber = rospy.Subscriber(
        SubscriberTopics.NEW_CHILD_TOPIC,
        String,
        subscriber_callbacks.on_new_child_received,
    )

    # listen for words to write
    words_subscriber = rospy.Subscriber(
        SubscriberTopics.WORDS_TOPIC,
        String,
        subscriber_callbacks.on_word_received,
    )

    # listen for test time
    test_subscriber = rospy.Subscriber(
        SubscriberTopics.TEST_TOPIC,
        Empty,
        subscriber_callbacks.on_test_request_received,
    )

    # listen for when to stop
    stop_subscriber = rospy.Subscriber(
        SubscriberTopics.STOP_TOPIC,
        Empty,
        subscriber_callbacks.on_stop_request_received,
    )

    # listen for user-drawn shapes
    shape_subscriber = rospy.Subscriber(
        SubscriberTopics.PROCESSED_USER_SHAPE_TOPIC,
        ShapeMsg,
        subscriber_callbacks.on_user_drawn_shape_received,
    )

    # listen for user-drawn finger gestures
    gesture_subscriber = rospy.Subscriber(
        SubscriberTopics.GESTURE_TOPIC,
        PointStamped,
        subscriber_callbacks.on_set_active_shape_gesture,
    )

    shape_finished_subscriber = rospy.Subscriber(
        SubscriberTopics.SHAPE_FINISHED_TOPIC,
        String,
        subscriber_callbacks.on_shape_finished,
    )

    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    # listen for request to clear screen (from tablet)
    # clear_subscriber = rospy.Subscriber(SubscriberTopics.CLEAR_SURFACE_TOPIC,
    #                                     Empty,
    #                                     subscriber_callbacks.on_clear_screen_received)

    TOPIC_GPT_INPUT = "chatgpt_input"
    rospy.Subscriber(
        TOPIC_GPT_INPUT, String, subscriber_callbacks.on_user_chat_received
    )

    START_SENDING_VOICE = "speech_rec"
    rospy.Subscriber(
        START_SENDING_VOICE, String, subscriber_callbacks.on_feedback_received
    )

    # initialise display manager for shapes (manages positioning of shapes)

    rospy.loginfo("Waiting for display manager services to become available")
    rospy.wait_for_service("clear_all_shapes")

    rospy.sleep(2.0)  # Allow some time for the subscribers to do t heir thing,
    # or the first message will be missed (eg. first traj on tablet, first clear request locally)

    # TODO: Make a ROS server so that *everyone* can access the connection statuses

    # robot_watchdog = Watchdog('watchdog_clear/robot', 0.8)

    rospy.loginfo(
        "Nao configuration: writing=%s, speaking=%s (%s), standing=%s, handedness=%s"
        % (
            nao_settings.nao_writing,
            nao_settings.nao_speaking,
            nao_settings.LANGUAGE,
            nao_settings.nao_standing,
            nao_settings.NAO_HANDEDNESS,
        )
    )

    # initialise word manager (passes feedback to shape learners and keeps history of words learnt)
    InteractionSettings.set_dataset_directory(dataset_directory)
    # path to a log file where all learning steps will be stored

    # Start the interaction
    state_machine.run(info_for_start_state)

    rospy.spin()

    device_manager.tablet_watchdog.stop()
    # robot_watchdog.stop()
