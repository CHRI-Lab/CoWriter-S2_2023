#!/usr/bin/env python3
# coding: utf-8

"""
Nao learning words using the shape_learning package.
This node manages the state machine which maintains the interaction
sequence, receives interaction inputs e.g. which words to write and
user demonstrations, passes these demonstrations to the learning
algorithm, and publishes the resulting learned shapes for the robot
and tablet to draw.
"""

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


def configure_logging(logger: logging.Logger, path="/tmp"):
    """
    Configures the logging settings for the provided logger instance.
    If a valid directory path is provided, logs will be written to a
    file named 'words_generated.log' in that directory. If the path
    is not provided or is not a valid directory, a NullHandler will
    be used, effectively disabling logging output.

    Args:
        logger (logging.Logger): The logger instance to configure.
        path (str, optional): The directory path where the log file
        should be created. Defaults to "/tmp".

    Returns:
        logging.Logger: The configured logger instance.
    """
    if path:
        if os.path.isdir(path):
            path = os.path.join(path, "words_generated.log")
        handler = logging.FileHandler(path)
        handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        handler.setFormatter(formatter)
    else:
        handler = logging.NullHandler()

    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)

    return logger


# -- interaction config parameters come from launch file


# -------------------------- CALLBACK METHODS FOR ROS SUBSCRIBERS
class SubscriberCallbacks:
    """
    This class contains a collection of methods to handle incoming ROS
    messages and update instance variables accordingly.
    """

    def __init__(
        self,
        nao_settings: NaoSettings,
        device_manager: DeviceManager,
        managerGPT: PhraseManagerGPT,
        publish_manager: PublisherManager,
        state_machine: StateMachine,
    ):
        self.managerGPT = managerGPT
        self.nao_settings = nao_settings
        self.device_manager = device_manager
        self.publish_manager = publish_manager
        self.state_machine = state_machine

        self.demo_shapes_received: List[ShapeMsg] = []
        self.active_letter: Optional[str] = None
        self.shape_finished: bool = False
        self.test_request_received: bool = False
        self.stop_request_received: bool = False
        self.word_received: Optional[str] = None
        self.feedback_received: Optional[str] = None

        # chatGPT related variables
        # a state should have these variables defined on entry
        # or it will "remain the same as it was"
        self.chatGPT_enabled = True
        self.chatGPT_to_say_enabled = True
        self.response = None

    def on_user_chat_received(self, in_chat: String) -> None:
        """
        if self.chat_enabled is enabled, send the word to Chatgpt,
        and let robot say() the output

        :in_chat: std_msgs.msg.String, input fot chat in .data
        """
        rospy.loginfo("input for GPT: " + in_chat.data)
        if self.chatGPT_enabled:
            self.response = self.managerGPT.get_gpt_response(in_chat.data)
            if self.chatGPT_to_say_enabled:
                self.nao_settings.text_to_speech.say(self.response)

    def on_user_drawn_shape_received(self, shape: ShapeMsg) -> None:
        """
        Processes a user-drawn shape and identifies the corresponding
        letter.

        :param shape: The received shape message from the user.
        """
        if (
            self.state_machine.get_state() == "WAITING_FOR_FEEDBACK"
            or self.state_machine.get_state() == "ASKING_FOR_FEEDBACK"
        ):
            nbpts = int(len(shape.path) / 2)
            # Create a path from the shape by combining x and y coordinates into tuples
            path = list(
                zip(shape.path[:nbpts], [-y for y in shape.path[nbpts:]])
            )
            # Split the path into segments based on a template
            demo_from_template = (
                self.device_manager.screen_manager.split_path_from_template(
                    path
                )
            )  # type: ignore

            # If the demo_from_template is not empty
            if demo_from_template:
                rospy.loginfo(
                    f"Received template demonstration for letters {demo_from_template.keys()}"
                )

                # Iterate through the path segments
                for name, path in demo_from_template.items():
                    # If the path segment corresponds to a multi-stroke letter, ignore it
                    if name in ["i", "j", "t"]:
                        rospy.logwarn(
                            f"Received demonstration for multi-stroke letter {name}: ignoring it."
                        )
                        continue

                    # Flatten the path by combining x and y coordinates into a single list
                    # Note: Unclear why y-coordinate is being flipped
                    flatpath = [x for x, y in path]
                    flatpath.extend([-y for x, y in path])

                    # Append the flattened path as a ShapeMsg to the demo_shapes_received list
                    self.demo_shapes_received.append(
                        ShapeMsg(path=flatpath, shape_type=name)
                    )

            # If demo_from_template is empty
            else:
                # If an active letter exists
                if self.active_letter:
                    # Assign the shape type as the active letter and reset the active letter
                    shape.shape_type = self.active_letter
                    self.active_letter = None
                    rospy.loginfo(
                        f"Received demonstration for selected letter {shape.shape_type}"
                    )
                else:
                    # Find the letter corresponding to the shape path
                    letter, bb = self.device_manager.screen_manager.find_letter(
                        shape.path
                    )

                    # If a letter is found
                    if letter:
                        # Assign the shape type as the found letter
                        shape.shape_type = letter
                        rospy.loginfo(
                            f"Received demonstration for {shape.shape_type}"
                        )
                    else:
                        rospy.logwarn(
                            "Received demonstration, but unable to find the letter that was demonstrated! Ignoring it."
                        )
                        return

                # Replace any existing feedback with the new shape
                self.demo_shapes_received = [shape]

        else:
            pass  # ignore feedback

    def on_shape_finished(self, message: Empty) -> None:
        """
        Callback function that is triggered when the shape drawing is
        finished.

        This function sets the 'shape_finished' instance variable to
        True, indicating that the shape has been completed. The
        shape_finished flag can be used to control the program flow
        depending on whether the shape is finished or not.

        :param message (Empty): An empty message indicating the shape
                                drawing completion event.
        """
        self.shape_finished = True

    def on_new_child_received(self, message: String) -> None:
        """
        Processes a new child message and updates the instance
        variables accordingly.

        :param message: The received new child message.
        """
        if self.nao_settings.nao_writing:
            if self.nao_settings.nao_standing:
                self.nao_settings.posture_proxy.goToPosture("StandInit", 0.3)
            else:
                self.nao_settings.motion_proxy.rest()
                self.nao_settings.motion_proxy.setStiffnesses(
                    ["Head", "LArm", "RArm"], 0.5
                )
                self.nao_settings.motion_proxy.setStiffnesses(
                    [
                        "LHipYawPitch",
                        "LHipRoll",
                        "LHipPitch",
                        "RHipYawPitch",
                        "RHipRoll",
                        "RHipPitch",
                    ],
                    0.8,
                )

        if self.nao_settings.nao_speaking:
            if self.nao_settings.alternate_sides_looking_at:
                self.nao_settings.look_and_ask_for_feedback(
                    self.nao_settings.phrase_manager.intro_phrase,
                    self.nao_settings.next_side_to_look_at,
                )
            else:
                self.nao_settings.look_and_ask_for_feedback(
                    self.nao_settings.phrase_manager.intro_phrase,
                    self.nao_settings.person_side,
                )
        # clear screen
        self.publish_manager.pub_clear.publish(Empty())
        rospy.sleep(0.5)

    def on_word_received(self, message: String) -> None:
        """
        Processes a word received message and updates the instance
        variables accordingly.

        :param message: The received word message.
        """
        if (
            self.state_machine.get_state() == "WAITING_FOR_FEEDBACK"
            or self.state_machine.get_state() == "WAITING_FOR_WORD"
            or self.state_machine.get_state() == "ASKING_FOR_FEEDBACK"
            or self.state_machine.get_state() == "STARTING_INTERACTION"
            or self.state_machine.get_state() is None
        ):  # state machine hasn't started yet - word probably came from input arguments
            self.word_received = message
            rospy.loginfo(f"Received word: {self.word_received}")
        else:
            self.word_received = None  # ignore

    def on_clear_screen_received(self) -> None:
        """
        Processes a clear screen request and updates the instance
        variables accordingly.

        :param message (Empty): The received clear screen request
        message.
        """
        rospy.loginfo("Clearing display")
        try:
            # NOTE: follow up this clear_all_shapes pass to service then call func
            clear_all_shapes = rospy.ServiceProxy("clear_all_shapes", ClearAllShapes)  # type: ignore
            resp1 = clear_all_shapes()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def on_test_request_received(self, message: Empty) -> None:
        """
        Processes a test request message and updates the instance
        variables accordingly.

        :param message (Empty): The received test request message.
        """
        self.test_request_received = True

    def on_stop_request_received(self, message: Empty) -> None:
        """
        Processes a stop request message and updates the instance
        variables accordingly.

        :param message: The received stop request message.
        """
        self.stop_request_received = True

    def on_set_active_shape_gesture(self, message: PointStamped) -> None:
        """
        Processes a user-drawn finger gesture and updates the active
        letter accordingly.

        :param message: The received finger gesture message.
        """
        # NOTE: Need to handle screen_manager here!
        (
            self.active_letter,
            bb,
        ) = self.device_manager.screen_manager.closest_letter(
            message.point.x, message.point.y, strict=True
        )

    def on_feedback_received(self, in_chat) -> None:
        """
        Callback function that is triggered when feedback is received.

        :param in_chat: A message containing the feedback.
        """
        rospy.loginfo("input for GPT: " + in_chat.data)
        if self.chatGPT_enabled:
            self.response = self.managerGPT.get_gpt_response(in_chat.data)
            if self.nao_settings.nao_connected and self.chatGPT_to_say_enabled:
                self.nao_settings.text_to_speech.say(self.response)
        else:
            self.response = "feedback"
            self.nao_settings.text_to_speech.say("Thanks for feedback")

        self.feedback_received = self.response


# ------------------------------- METHODS FOR DIFFERENT STATES IN STATE MACHINE
class StateManager:
    """
    A manager class that handles the states and state transitions in
    the robot's interaction system. It is responsible for processing
    and responding to different events such as demonstrations, word
    reception, and state completion. The class also contains methods
    to manage speech and trajectory generation, as well as checking for
    stop requests.

    Attributes:
        nao_settings (NaoSettings):
            An object containing the settings for the Nao robot.
        device_manager (DeviceManager):
            An object responsible for managing connected devices.
        publish_manager (PublisherManager):
            An object responsible for managing ROS publishers.
        subscriber_callbacks (SubscriberCallbacks):
            An object containing subscriber callback methods.
        generated_word_logger:
            An object responsible for logging generated words.
        t0 (float):
            The initial time for trajectories.
        dt (float):
            The time step for trajectories.
        delay_before_executing (float):
            The delay before executing a trajectory.
        info_to_restore_wait_for_shape_to_finish (Dict[str, Any]):
            A dictionary containing information to restore the state
            when waiting for a shape to finish.
        info_to_restore_wait_for_robot_to_connect (Dict[str, Any]):
            A dictionary containing information to restore the state
            when waiting for the robot to connect.
        info_to_restore_wait_for_tablet_to_connect (Dict[str, Any]):
            A dictionary containing information to restore the state
            when waiting for the tablet to connect.
        drawing_letter_substates (List[str]):
            A list of substates related to drawing letters.
    """

    # shape params
    # Frame ID to publish points in
    FRAME = rospy.get_param("~writing_surface_frame_id", "writing_surface")

    # Name of topic to receive feedback on
    # FEEDBACK_TOPIC = rospy.get_param('~shape_feedback_topic', 'shape_feedback')
    # FEEDBACK_TOPIC not used anywhere, comment out for now

    NUMDESIREDSHAPEPOINTS = (
        7.0  # Number of points to downsample the length of shapes to
    )
    # Number of points used by ShapeModelers (@todo this could vary for each letter)
    NUMPOINTS_SHAPEMODELER = 70
    DOWNSAMPLEFACTOR = float(NUMPOINTS_SHAPEMODELER - 1) / float(
        NUMDESIREDSHAPEPOINTS - 1
    )

    def __init__(
        self,
        nao_settings: NaoSettings,
        device_manager: DeviceManager,
        publish_manager: PublisherManager,
        subscriber_callbacks: SubscriberCallbacks,
        generated_word_logger,
    ):
        self.nao_settings = nao_settings
        self.device_manager = device_manager
        self.publish_manager = publish_manager
        self.publish_manager.init_publishers()
        self.subscriber_callbacks = subscriber_callbacks
        self.generated_word_logger = generated_word_logger

        # Trajectory publishing parameters
        (
            self.t0,
            self.dt,
            self.delay_before_executing,
        ) = InteractionSettings.get_trajectory_timings(
            self.nao_settings.nao_writing
        )

        # Phrase params are in nao_settings.phrase_manager

        # text_to_speech, motion_proxy, effector - all need handled
        # Should these be handled by a class? Number of params being passed here is stupid
        # Phrases could be handled by a class as well

        self.info_to_restore_wait_for_shape_to_finish = None
        self.info_to_restore_wait_for_robot_to_connect = None
        # self.info_to_restore_wait_for_tablet_to_connect = None

        self.drawing_letter_substates: List[str] = [
            "WAITING_FOR_ROBOT_TO_CONNECT",
            "WAITING_FOR_TABLET_TO_CONNECT",
            "PUBLISHING_LETTER",
        ]

    def handle_word_received(
        self, next_state: str, info_for_next_state: Dict[str, str]
    ) -> Tuple[str, Dict[str, str]]:
        """
        Handles the 'word_received' event. If a new word is received &
        the 'word_received' callback is set, updates the
        'info_for_next_state'  dictionary with the received word and
        clears the callback. Sets the next state to
        "RESPONDING_TO_NEW_WORD".

        Args:
            next_state (str):
                The current next state in the state machine.
            info_for_next_state (Dict[str, str]):
                A dictionary containing information to be passed to the
                next state.

        Returns:
            Tuple[str, Dict[str, str]]:
                A tuple containing the updated next state and the
                updated 'info_for_next_state' dictionary.
        """
        if self.subscriber_callbacks.word_received is not None:
            info_for_next_state[
                "word_received"
            ] = self.subscriber_callbacks.word_received
            self.subscriber_callbacks.word_received = None
            next_state = "RESPONDING_TO_NEW_WORD"

        return next_state, info_for_next_state

    def check_stop_request_received(self, next_state: str) -> str:
        """
        Check if a stop request has been received, if so then change
        next_state to "STOPPING", else return next_state unchanged.

        Args:
            next_state (str): The next state to go into, determined
                              before stop request check.

        Returns:
            next_state (str): Either input or "STOPPING".
        """
        if self.subscriber_callbacks.stop_request_received:
            next_state = "STOPPING"

        return next_state

    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    # def respond_to_demonstration(self, info_from_prev_state):
    #     """
    #     Respond to a demonstration by updating shape models and generating speech output (if enabled).

    #     Args:
    #         info_from_prev_state (dict): A dictionary containing information from the previous state.

    #     Returns:
    #         tuple: A tuple containing the next state and information for the next state.
    #     """
    #     rospy.loginfo("STATE: RESPONDING_TO_DEMONSTRATION")
    #     self.subscriber_callbacks.demo_shapes_received = info_from_prev_state[
    #         'demo_shapes_received']

    #     # Update the shape models with the incoming demos
    #     new_shapes = []

    #     letters = "".join([s.shape_type
    #                        for s in self.subscriber_callbacks.demo_shapes_received])

    #     # Probably another NaoSettings method to factor out here
    #     # (Arguably two, get_next_phrase surely is a NaoSettings or PhraseManager method,
    #     # # but PhraseManager is an attribute of NaoSettings anyway)
    #     # If nao is speaking then try get response phrase
    #     if self.nao_settings.nao_speaking:
    #         to_say, self.nao_settings.phrase_manager.demo_response_phrases_counter = self.get_next_phrase(
    #             self.nao_settings.phrase_manager.demo_response_phrases,
    #             self.nao_settings.phrase_manager.demo_response_phrases_counter,
    #             letters
    #         )

    #         # Nao speak and rospy log the phrase
    #         self.nao_settings.nao_speak_and_log_phrase(to_say)

    #     for shape in self.subscriber_callbacks.demo_shapes_received:
    #         # Get shape info, downsample and log
    #         glyph = shape.path
    #         shape_name = shape.shape_type
    #         glyph = self.downsample_shape(glyph)
    #         rospy.loginfo(f"Received demo for {shape_name}")

    #         # Process shape with word manager
    #         shape_index = self.device_manager.word_manager.current_collection.index(
    #             shape_name)
    #         shape = self.device_manager.word_manager.respond_to_demonstration(
    #             shape_index, glyph)

    #         new_shapes.append(shape)

    #     # Pop the next state from the drawing_letter_substates list and
    #     # generate info for the next state
    #     state_go_to = deepcopy(self.drawing_letter_substates)
    #     next_state = state_go_to.pop(0)
    #     info_for_next_state = {'state_go_to': state_go_to,
    #                            'state_came_from': "RESPONDING_TO_DEMONSTRATION",
    #                            'shapes_to_publish': new_shapes}

    #     return next_state, info_for_next_state

    def respond_to_demonstration_with_full_word(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Respond to a demonstration by updating shape models, displaying
        the updated word, and generating speech output (if enabled).

        Args:
            info_from_prev_state (dict):
                A dictionary containing information from the previous
                state.

        Returns:
            tuple:
                A tuple containing the next state and information for
                the next state.
        """
        rospy.loginfo("STATE: RESPONDING_TO_DEMONSTRATION_FULL_WORD")

        demo_shapes_received = info_from_prev_state["demo_shapes_received"]

        letters = "".join([s.shape_type for s in demo_shapes_received])

        if self.nao_settings.nao_speaking:
            (
                to_say,
                self.nao_settings.phrase_manager.demo_response_phrases_counter,
            ) = self.get_next_phrase(
                self.nao_settings.phrase_manager.demo_response_phrases,
                self.nao_settings.phrase_manager.demo_response_phrases_counter,
                letters,
            )

            self.nao_settings.nao_speak_and_log_phrase(to_say)

        # 1- Update the shape models with the incoming demos
        for shape in demo_shapes_received:
            glyph = shape.path
            shape_name = shape.shape_type

            rospy.logdebug(f"Downsampling {shape_name}...")
            glyph = self.downsample_shape(glyph)
            rospy.loginfo(
                f"Downsampling of {shape_name} done. "
                + f"Demo received for {shape_name}"
            )
            shape_index = (
                self.device_manager.word_manager.current_collection.index(
                    shape_name
                )
            )
            self.device_manager.word_manager.respond_to_demonstration(
                shape_index, glyph
            )

        # 2- Display the updated word

        # Clear the screen
        self.device_manager.screen_manager.clear()
        self.publish_manager.pub_clear.publish(Empty())
        rospy.sleep(0.5)

        shapes_to_publish = (
            self.device_manager.word_manager.shapes_of_current_collection()
        )

        next_state = "PUBLISHING_WORD"
        info_for_next_state = {
            "state_came_from": "RESPONDING_TO_DEMONSTRATION_FULL_WORD",
            "state_go_to": "WAITING_FOR_WORD",
            "shapes_to_publish": shapes_to_publish,
            "item_written": self.device_manager.word_manager.current_collection,
        }

        return next_state, info_for_next_state

    def publish_word(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Publishes the word on the screen.

        Args:
            info_from_prev_state:
                A dictionary containing information from the previous
                state.

        Returns:
            A tuple containing the next state and information for the
            next state.
        """
        # Log the current state
        rospy.loginfo("STATE: PUBLISHING_WORD")
        rospy.loginfo("From " + info_from_prev_state["state_came_from"])
        rospy.loginfo("To " + info_from_prev_state["state_go_to"])

        # Shape and place the word on the screen
        # NOTE: need to handle text_shaper and screen_manager
        shaped_word = self.device_manager.text_shaper.shape_word(
            self.device_manager.word_manager
        )
        placed_word = self.device_manager.screen_manager.place_word(shaped_word)

        # Create a trajectory for the word
        traj = self.make_traj_msg(
            placed_word, float(self.dt) / self.DOWNSAMPLEFACTOR, log=True
        )

        # Downsample the trajectory for the robot arm motion
        downsampled_shaped_word = deepcopy(placed_word)
        downsampled_shaped_word.downsample(int(self.DOWNSAMPLEFACTOR))

        downsampled_traj = self.make_traj_msg(downsampled_shaped_word, self.dt)

        # TODO: Request the tablet to display the letters' and word's bounding boxes

        # Get the starting position of the trajectory
        if traj.poses:
            traj_start_position = traj.poses[0].pose.position  # type: ignore
        else:
            traj_start_position = None

        # Look at the tablet if the Nao is connected
        if self.nao_settings.nao_connected:
            self.nao_settings.look_at_tablet()

        # Publish the trajectories
        # NOTE: need to handle pub_traj
        self.publish_manager.pub_traj_downsampled.publish(downsampled_traj)
        self.publish_manager.pub_traj.publish(traj)

        # Transition to the next state
        next_state = "WAITING_FOR_LETTER_TO_FINISH"
        info_for_next_state = {
            "state_came_from": "PUBLISHING_WORD",
            "state_go_to": info_from_prev_state["state_go_to"],
            "centre": traj_start_position,
            "item_written": info_from_prev_state["item_written"],
        }

        return next_state, info_for_next_state

    def wait_for_shape_to_finish(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Manages the state transitions in the system while waiting for a
        shape to finish being drawn. This method takes into account the
        current state, the previous state, and other conditions such as
        stop requests.
        When a shape is finished, this method will handle the placement
        of reference bounding boxes and determine the appropriate next
        state to transition to.

        Args:
            info_from_prev_state (Dict[str, Any]):
                A dictionary containing information about the previous
                state and any additional details needed for state
                transitions.

        Returns:
            Tuple[str, Dict[str, Any]]:
                A tuple containing the next state as a string and a
                dictionary with information to be passed to the next
                state.
        """
        # First time into this state preparations
        # if info_from_prev_state['state_came_from'] != "WAITING_FOR_LETTER_TO_FINISH":
        #     rospy.loginfo("STATE: WAITING_FOR_LETTER_TO_FINISH")
        #     self.info_to_restore_wait_for_shape_to_finish = info_from_prev_state

        # info_for_next_state = {'state_came_from': 'WAITING_FOR_LETTER_TO_FINISH',
        #                        'state_go_to': info_from_prev_state['state_go_to']}
        next_state = None

        # Once shape is finished
        if self.subscriber_callbacks.shape_finished:
            # Draw the templates for the demonstrations
            ref_boundingboxes = self.device_manager.screen_manager.place_reference_boundingboxes(
                self.device_manager.word_manager.current_collection
            )
            for bb in ref_boundingboxes:
                self.publish_manager.pub_bounding_boxes.publish(
                    self.make_bounding_box_msg(bb, selected=False)
                )
                rospy.sleep(0.2)

            self.subscriber_callbacks.shape_finished = False
            # info_for_next_state = self.info_to_restore_wait_for_shape_to_finish  # type: ignore
            if info_from_prev_state["state_go_to"] is not None:
                next_state = info_from_prev_state["state_go_to"]
            else:
                rospy.loginfo("STATE: WAITING_FOR_LETTER_TO_FINISH")
                rospy.loginfo("WARNING: state_go_to not set")
                next_state = "WAITING_FOR_FEEDBACK"
            info_for_next_state = {
                "state_came_from": "WAITING_FOR_LETTER_TO_FINISH",
                "item_written": info_from_prev_state["item_written"],
            }

        # Check if next state should be stopping.
        next_state = self.check_stop_request_received(next_state)

        # If haven't received next state then go into waiting state
        if next_state is None:
            rospy.sleep(0.1)
            next_state = "WAITING_FOR_LETTER_TO_FINISH"
            info_for_next_state = {
                "state_go_to": info_from_prev_state["state_go_to"],
                "state_came_from": "WAITING_FOR_LETTER_TO_FINISH",
                "item_written": info_from_prev_state["item_written"],
            }
        else:
            info_for_next_state = info_from_prev_state

        return next_state, info_for_next_state

    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    # def respond_to_test_card(self, info_from_prev_state: Dict[str, Any]) -> Tuple[str, Dict[str, Any]]:
    #     """
    #     Handles the state when the system is responding to a test card. In this state, if the NAO robot is
    #     enabled to speak, it will say the test phrase. The system then transitions to the "WAITING_FOR_WORD"
    #     state.

    #     Args:
    #         info_from_prev_state (Dict[str, Any]): A dictionary containing information about the previous state
    #             and any additional details needed for state transitions.

    #     Returns:
    #         Tuple[str, Dict[str, Any]]: A tuple containing the next state as a string and a dictionary with
    #             information to be passed to the next state.
    #     """
    #     rospy.loginfo("STATE: RESPONDING_TO_TEST_CARD")
    #     if self.nao_settings.nao_speaking:
    #         self.nao_settings.nao_speak_and_log_phrase(
    #             self.nao_settings.phrase_manager.test_phrase[0])

    #     next_state = "WAITING_FOR_WORD"
    #     info_for_next_state = {'state_came_from': "RESPONDING_TO_TEST_CARD"}

    #     return next_state, info_for_next_state

    def stop_interaction(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, int]:
        """
        Stops the current interaction by having the NAO robot say the
        thank you phrase (if speaking is enabled), disabling effector
        control and resting the robot (if connected), and then
        transitioning to the "EXIT" state.

        Shuts down the ROS node upon completion.

        Args:
            info_from_prev_state (Dict[str, Any]):
                A dictionary containing information about the previous
                state and any additional details needed for state
                transitions.

        Returns:
            Tuple[str, int]:
                A tuple containing the next state as a string and an
                integer with information to be passed to the next
                state.
        """
        rospy.loginfo("STATE: STOPPING")

        # Factor two if statements to method for NaoSettings
        if self.nao_settings.nao_speaking:
            self.nao_settings.nao_speak_and_log_phrase(
                self.nao_settings.phrase_manager.thank_you_phrase[0]
            )

        # Set nao to rest
        self.nao_settings.nao_rest()

        next_state = "EXIT"
        info_for_next_state = 0
        rospy.signal_shutdown("Interaction exited")

        return next_state, info_for_next_state

    def start_interaction(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Starts the interaction with the person, including speaking,
        asking for feedback, and looking at the person. Determines the
        next state based on the stop request status.

        Args:
            info_from_prev_state (Dict[str, Any]):
                A dictionary containing information from the previous
                state.

        Returns:
            Tuple[str, Dict[str, Any]]:
                A tuple containing the next state and a dictionary with
                information for the next state.
        """
        rospy.loginfo("STATE: STARTING_INTERACTION")
        # If nao speaking say intro phrase
        if self.nao_settings.nao_speaking:
            self.nao_settings.handle_look_and_ask_for_feedback(
                self.nao_settings.phrase_manager.intro_phrase
            )

        next_state = "WAITING_FOR_WORD"
        info_for_next_state = {"state_came_from": "STARTING_INTERACTION"}

        next_state = self.check_stop_request_received(next_state)

        return next_state, info_for_next_state

    def wait_for_word(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Waits for a word from the user, publishes camera status and
        determines the next state based on the received word or stop
        request status.

        Args:
            info_from_prev_state (Dict[str, Any]):
                A dictionary containing information from the previous
                state.

        Returns:
            Tuple[str, Dict[str, Any]]:
                A tuple containing the next state and a dictionary with
                information for the next state.
        """
        next_state = None
        if info_from_prev_state["state_came_from"] != "WAITING_FOR_WORD":
            rospy.loginfo("STATE: WAITING_FOR_WORD")

            # broken for now (pub_camera_status in PublisherManager class)
            # self.publish_manager.pub_camera_status.publish(
            #     True)  # Turn camera on

        if info_from_prev_state["state_came_from"] == "STARTING_INTERACTION":
            pass

        info_for_next_state = {"state_came_from": "WAITING_FOR_WORD"}
        if self.subscriber_callbacks.word_received is None:
            next_state = "WAITING_FOR_WORD"
            rospy.sleep(0.1)  # Don't check again immediately
        else:
            # Check for received word and modify next_state if so
            next_state, info_for_next_state = self.handle_word_received(
                next_state, info_for_next_state
            )  # type: ignore
            # self.publish_manager.pub_camera_status.publish(
            #     False)  # Turn camera off

        if self.subscriber_callbacks.stop_request_received:
            next_state = "STOPPING"
            # self.publish_manager.pub_camera_status.publish(
            #     False)  # Turn camera off

        return next_state, info_for_next_state

    def wait_for_feedback(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Waits for user feedback, demonstration shapes, a new word, or a
        test request, and determines the next state based on the
        received data or stop request status.

        Args:
            info_from_prev_state (Dict[str, Any]):
                A dictionary containing information from the previous
                state.

        Returns:
            Tuple[str, Dict[str, Any]]:
                A tuple containing the next state and a dictionary with
                information for the next state.
        """
        # if info_from_prev_state['state_came_from'] != "WAITING_FOR_FEEDBACK":
        #     rospy.loginfo("STATE: WAITING_FOR_FEEDBACK")
        #     self.publish_manager.pub_camera_status.publish(
        #         True)  # turn camera on

        info_for_next_state: Dict[str, Any] = {
            "state_came_from": "WAITING_FOR_FEEDBACK"
        }
        next_state = None

        if self.subscriber_callbacks.feedback_received is not None:
            rospy.loginfo(self.subscriber_callbacks.feedback_received)
            rospy.loginfo("STATE: WAITING_FOR_FEEDBACK, got feedback")
            info_for_next_state[
                "feedback_received"
            ] = self.subscriber_callbacks.feedback_received
            self.subscriber_callbacks.feedback_received = None
            next_state = "WAITING_FOR_WORD"

            # Commented method/function out because not presently in use
            # TODO: reintegrate or remove
            # Ensure robot is connected before going to that state
            # info_for_next_state['state_go_to'] = [next_state]
            # next_state = 'WAITING_FOR_ROBOT_TO_CONNECT'

        if self.subscriber_callbacks.demo_shapes_received:
            rospy.loginfo(self.subscriber_callbacks.demo_shapes_received)
            rospy.loginfo("STATE: WAITING_FOR_FEEDBACK, got demo")
            info_for_next_state[
                "demo_shapes_received"
            ] = self.subscriber_callbacks.demo_shapes_received
            self.subscriber_callbacks.demo_shapes_received = []
            rospy.loginfo(self.subscriber_callbacks.demo_shapes_received)
            next_state = "RESPONDING_TO_DEMONSTRATION_FULL_WORD"

            # Commented method/function out because not presently in use
            # TODO: reintegrate or remove
            # Ensure robot is connected before going to that state
            # info_for_next_state['state_go_to'] = [next_state]
            # next_state = 'WAITING_FOR_ROBOT_TO_CONNECT'

        # Commented method/function out because not presently in use
        # TODO: reintegrate or remove
        # next_state, info_for_next_state = self.handle_word_received(
        #     next_state, info_for_next_state)  # type: ignore
        # if next_state is not None:
        #     Ensure robot is connected before going to that state
        #     info_for_next_state['state_go_to'] = [next_state]
        #     next_state = 'WAITING_FOR_ROBOT_TO_CONNECT'

        # if self.subscriber_callbacks.test_request_received:
        #     self.subscriber_callbacks.test_request_received = False
        #     next_state = "RESPONDING_TO_TEST_CARD"
        # Ensure robot is connected before going to that state
        # info_for_next_state['state_go_to'] = [next_state]
        # next_state = 'WAITING_FOR_ROBOT_TO_CONNECT'

        next_state = self.check_stop_request_received(next_state)

        # if next_state != 'WAITING_FOR_FEEDBACK':
        #     self.publish_manager.pub_camera_status.publish(
        #         False)  # Turn camera off

        if next_state is None:
            # Default behavior is to loop
            rospy.sleep(0.1)  # Don't check again immediately
            next_state = "WAITING_FOR_FEEDBACK"
            info_for_next_state = {"state_came_from": "WAITING_FOR_FEEDBACK"}

        return next_state, info_for_next_state

    def get_next_phrase(
        self, phrases: List[str], counter: int, variable: Optional[str] = None
    ) -> Tuple[str, int]:
        """
        Helper method to factor the repeating pattern of selecting the
        next phrase from a list of phrases, optionally formatting the
        phrase with a variable, and updating the counter.

        Args:
            phrases (List[str]): List of phrases to choose from.
            counter (int): Current position in the list of phrases.
            variable (Optional[str]):
                A variable to be used for formatting he phrase, if
                applicable.

        Returns:
            Tuple[str, int]:
                The chosen phrase, with the variable formatted if
                applicable, and the updated counter.
        """
        try:
            to_say = phrases[counter] % variable
        except TypeError:  # String wasn't meant to be formatted
            to_say = phrases[counter]

        counter += 1
        if counter == len(phrases):
            counter = 0

        return to_say, counter

    def respond_to_new_word(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Responds to a new word and starts learning it.

        Args:
            info_from_prev_state (Dict[str, Any]):
                A dictionary containing information from the previous
                state.

        Returns:
            Tuple[str, Dict[str, Any]]:
                A tuple containing the next state and a dictionary with
                information for the next state.
        """
        rospy.loginfo("STATE: RESPONDING_TO_NEW_WORD")
        word_to_learn = info_from_prev_state["word_received"].data
        word_seen_before = self.device_manager.word_manager.new_collection(
            word_to_learn
        )

        if self.nao_settings.nao_speaking:
            if word_seen_before:
                # word_again_response_phrases doesnt exist
                (
                    to_say,
                    self.nao_settings.phrase_manager.word_again_response_phrases_counter,
                ) = self.get_next_phrase(
                    self.nao_settings.phrase_manager.word_again_response_phrases,
                    self.nao_settings.phrase_manager.word_again_response_phrases_counter,
                    word_to_learn,
                )
            else:
                # word_response_phrases doesnt exist
                (
                    to_say,
                    self.nao_settings.phrase_manager.word_response_phrases_counter,
                ) = self.get_next_phrase(
                    self.nao_settings.phrase_manager.word_response_phrases,
                    self.nao_settings.phrase_manager.word_response_phrases_counter,
                    word_to_learn,
                )

            self.nao_settings.nao_speak_and_log_phrase(to_say)

        # Clear screen
        self.device_manager.screen_manager.clear()

        # Commented method/function out because not presently in use
        #        TODO: reintegrate or remove
        # currently the probulisher is not implimented, skipping publish
        # !!!!!!!! change later
        # self.publish_manager.pub_clear.publish(Empty())

        rospy.sleep(0.5)

        # Start learning
        shapes_to_publish = []
        for i in range(len(word_to_learn)):
            shape = self.device_manager.word_manager.start_next_shape_learner()
            shapes_to_publish.append(shape)

        next_state = "PUBLISHING_WORD"
        info_for_next_state = {
            "state_came_from": "RESPONDING_TO_NEW_WORD",
            "state_go_to": "ASKING_FOR_FEEDBACK",
            "shapes_to_publish": shapes_to_publish,
            "item_written": word_to_learn,
        }

        # Check if word received and respond appropriately
        next_state, info_for_next_state = self.handle_word_received(
            next_state, info_for_next_state
        )

        # Commented method/function out because not presently in use
        # TODO: reintegrate or remove
        # if self.subscriber_callbacks.test_request_received:
        #     self.subscriber_callbacks.test_request_received = False
        #     next_state = "RESPONDING_TO_TEST_CARD"

        next_state = self.check_stop_request_received(next_state)

        return next_state, info_for_next_state

    def ask_for_feedback(
        self, info_from_prev_state: Dict[str, Any]
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Ask for feedback after publishing a word or letter.

        Args:
            info_from_prev_state (Dict[str, Any]):
                A dictionary containing information from the previous
                state.

        Returns:
            Tuple[str, Dict[str, Any]]:
                A tuple containing the next state and information for
                the next state.
        """
        rospy.loginfo("STATE: ASKING_FOR_FEEDBACK")
        rospy.loginfo("From " + info_from_prev_state["state_came_from"])
        rospy.loginfo("item_written = " + info_from_prev_state["item_written"])

        item_written = info_from_prev_state["item_written"]
        if self.nao_settings.nao_speaking:
            (
                to_say,
                self.nao_settings.phrase_manager.asking_phrases_after_word_counter,
            ) = self.get_next_phrase(
                self.nao_settings.phrase_manager.asking_phrases_after_word,
                self.nao_settings.phrase_manager.asking_phrases_after_word_counter,
                item_written,
            )

            self.nao_settings.handle_look_and_ask_for_feedback(to_say)

            self.nao_settings.look_at_tablet()

        # Commented method/function out because not presently in use
        # TODO: reintegrate or remove
        # if info_from_prev_state['state_came_from'] == "PUBLISHING_WORD":
        #     word_written = info_from_prev_state['word_written']
        #     rospy.loginfo('Asking for feedback on word ' + word_written)

        #     if self.nao_settings.nao_speaking:
        #         # Get phrease based on published word
        #         to_say, self.nao_settings.phrase_manager.asking_phrases_after_word_counter = self.get_next_phrase(
        #             self.nao_settings.phrase_manager.asking_phrases_after_word,
        #             self.nao_settings.phrase_manager.asking_phrases_after_word_counter,
        #             word_written
        #         )

        #         self.nao_settings.handle_look_and_ask_for_feedback(to_say)

        #         self.nao_settings.look_at_tablet()

        # elif info_from_prev_state['state_came_from'] == "PUBLISHING_LETTER":
        #     shape_type = info_from_prev_state['shape_published']
        #     rospy.loginfo(f'Asking for feedback on letter {shape_type}')

        #     if self.nao_settings.nao_speaking:
        #         # Get phrase based on published letter/shape
        #         to_say, self.nao_settings.phrase_manager.asking_phrases_after_feedback_counter = self.get_next_phrase(
        #             self.nao_settings.phrase_manager.asking_phrases_after_feedback,
        #             self.nao_settings.phrase_manager.asking_phrases_after_feedback_counter,
        #             shape_type
        #         )

        #         self.nao_settings.handle_look_and_ask_for_feedback(to_say)

        #         self.nao_settings.look_at_tablet()

        next_state = "WAITING_FOR_FEEDBACK"
        info_for_next_state = {"state_came_from": "ASKING_FOR_FEEDBACK"}

        next_state, info_for_next_state = self.handle_word_received(
            next_state, info_for_next_state
        )

        next_state = self.check_stop_request_received(next_state)

        return next_state, info_for_next_state

    # --------------------------------------------- HELPER METHODS

    def downsample_shape(self, shape):
        """
        Downsamples a user-drawn shape to a size suitable for the
        ShapeModeler.

        This function reduces the number of points in the given shape
        to match the number of points required by the ShapeModeler. It
        uses linear interpolation to create a new, downsampled version
        of the shape with the same overall structure, but fewer points.
        The shape is then normalized and reshaped to a 2D array with
        only one column.

        Args:
            shape (numpy.ndarray):
                An array of shape points to be downsampled. The first
                half of the  array represents x coordinates and the
                second half represents y coordinates of the shape.

        Returns:
            numpy.ndarray:
                The downsampled shape. The shape is 2D, with one column
                and a number of rows equal to
                self.NUMPOINTS_SHAPEMODELER * 2. The first half of the
                rows represent x coordinates and the second half
                represent y coordinates of the downsampled shape.
        """
        # downsample user-drawn shape so appropriate size for shapeLearner
        num_points_in_shape = len(shape) // 2
        x_shape = shape[0:num_points_in_shape]
        y_shape = shape[num_points_in_shape:]

        # make shape have the same number of points as the shape_modeler
        t_current = np.linspace(0, 1, num_points_in_shape)
        t_desired = np.linspace(0, 1, self.NUMPOINTS_SHAPEMODELER)
        f = interpolate.interp1d(t_current, x_shape, kind="linear")
        x_shape = f(t_desired)
        f = interpolate.interp1d(t_current, y_shape, kind="linear")
        y_shape = f(t_desired)

        shape = []
        shape[0 : self.NUMPOINTS_SHAPEMODELER] = x_shape
        shape[self.NUMPOINTS_SHAPEMODELER :] = y_shape

        shape = ShapeModeler.normalise_shape_height(np.array(shape))
        # explicitly make it 2D array with only one column
        shape = np.reshape(shape, (-1, 1))

        return shape

    def make_bounding_box_msg(self, bbox, selected=False):
        """
        Constructs a bounding box message with coordinates from the
        bbox parameter.

        This function creates a Float64MultiArray message and
        populates it with bounding box information. The `selected`
        parameter is used to label the array dimension.

        Args:
            bbox (tuple):
                A tuple of four floats representing the minimum and
                maximum coordinates of the bounding box in the format
                (x_min, y_min, x_max, y_max).

            selected (bool, optional):
                A boolean value that represents whether the bounding
                box is selected. Defaults to False.

        Returns:
            message that contains the bounding box information. The
            array dimension label is 'bb' if not selected, else
            'select'.
        """

        bb = Float64MultiArray()
        bb.layout.data_offset = 0
        dim = MultiArrayDimension()
        # we use the label of the first dimension to carry the
        # selected/not selected infomation
        dim.label = "bb" if not selected else "select"
        bb.layout.dim = [dim]

        x_min, y_min, x_max, y_max = bbox
        bb.data = [x_min, y_min, x_max, y_max]

        return bb

    def make_traj_msg(self, shaped_word, delta_t, log=False):
        """
        Constructs a trajectory message based on the paths of the
        shaped_word.

        This function creates a `Path` message and populates it with
        the positions derived from the shaped_word's letters paths.
        It also sets the header of the trajectory and each point within
        it.

        Args:
            shaped_word (ShapedWord):
                The shaped word containing the letters' paths for the
                trajectory.

            delta_t (float):
                The time difference between points.

            log (bool, optional):
                If true, logs the generated paths. Defaults to False.

        Returns:
            traj:
                A ROS `Path` message that contains the trajectory info.
        """
        traj = Path()
        traj.header.frame_id = self.FRAME
        traj.header.stamp = rospy.Time.now() + rospy.Duration(
            int(self.delay_before_executing)
        )

        point_idx = 0
        paths = shaped_word.get_letters_paths()

        if log:
            self.generated_word_logger.info(
                "%s" % [[(x, -y) for x, y in path] for path in paths]
            )

        for path in paths:
            first = True
            for x, y in path:
                point = PoseStamped()

                point.pose.position.x = x
                point.pose.position.y = y
                point.header.frame_id = self.FRAME
                # @TODO allow for variable time between points for now
                point.header.stamp = rospy.Time(self.t0 + point_idx * delta_t)

                if first:
                    point.header.seq = 1
                    first = False

                traj.poses.append(deepcopy(point))  # type: ignore

                point_idx += 1

        return traj


# ------------------------------------------- MAIN
shapes_learnt = []
words_learnt = []
shape_learners = []
current_word = []
settings_shape_learners = []


if __name__ == "__main__":
    # Init node inside main to avoid running node if imported
    # rospy.init_node("learning_words_nao")
    rclpy.init()
    node = LearningWordsNao()

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

    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    # state_machine.add_state("WAITING_FOR_ROBOT_TO_CONNECT",
    #                         state_manager.wait_for_robot_to_connect)
    # publish_shape currently removed as we received it in a broken state
    # state_machine.add_state("PUBLISHING_LETTER", state_manager.publish_shape)
    # respond_to_feedback currently removed as it was not in use
    # state_machine.add_state("RESPONDING_TO_FEEDBACK",
    #                         state_manager.respond_to_feedback)
    # state_machine.add_state("RESPONDING_TO_DEMONSTRATION",
    #                         state_manager.respond_to_demonstration)
    # state_machine.add_state("RESPONDING_TO_TEST_CARD",
    #                         state_manager.respond_to_test_card)
    # stateMachine.add_state("RESPONDING_TO_TABLET_DISCONNECT", respondToTabletDisconnect)
    # state_machine.add_state("WAITING_FOR_TABLET_TO_CONNECT",
    #                         state_manager.wait_for_tablet_to_connect)

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
