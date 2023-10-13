#!/usr/bin/env python3
# coding: utf-8

import os.path
import logging
from time import sleep
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
from scipy import interpolate
from copy import deepcopy
import rclpy
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node

from requests import Session

import threading

from letter_learning_interaction.include.wrapper_class import (
    DeviceManager,
    PublisherManager,
    SubscriberTopics,
)

from letter_learning_interaction.include.phrase_manager import PhraseManagerGPT
from letter_learning_interaction.include.phrase_manager import PhraseManager
from letter_learning_interaction.include.gpt_word_generator import (
    GPT_Word_Generator,
)

from letter_learning_interaction.include.interaction_settings import (
    InteractionSettings,
)
from letter_learning_interaction.include.state_machine import StateMachine
from letter_learning_interaction.include.shape_modeler import ShapeModeler

from interface.msg import Shape as ShapeMsg  # type: ignore
from interface.srv import ClearAllShapes, GenerateWord  # type: ignore


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Empty, Float64MultiArray, MultiArrayDimension


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


class LearningWordsNao(Node):
    def __init__(
        self,
        state_machine: StateMachine,
        nao_settings,
        session: Session,
        phrase_manager: PhraseManager,
        generated_word_logger,
    ):
        super().__init__("learning_words_nao")
        self.gpt_word_generator = GPT_Word_Generator(phrase_manager)
        self.session = session
        self.declare_parameter("dataset_directory", "default")
        self.topics = SubscriberTopics(self)
        # listen for request to clear screen (from tablet)
        # clear_subscriber = self.create_subscription(
        #     Subscriberself.topics.CLEAR_SURFACE_TOPIC,
        #     Empty,
        #     subscriber_callbacks.on_clear_screen_received,
        # )

        self.device_manager = DeviceManager(self)
        self.publish_manager = PublisherManager(self)
        self.managerGPT = PhraseManagerGPT("English")

        self.session = session
        self.publish_manager.init_publishers()
        self.generated_word_logger = generated_word_logger

        self.nao_settings = nao_settings
        # self.managerGPT = managerGPT
        # self.device_manager = device_manager
        # self.publish_manager = publish_manager
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

        self.phrase_manager = phrase_manager

        self.topics = SubscriberTopics(self)

        self.create_service(
            GenerateWord,
            "generate_word_service",
            self.generate_word_callback,
        )

        self.create_subscription(
            String,
            self.topics.NEW_CHILD_TOPIC,
            self.on_new_child_received,
            10,
        )
        # listen for words to write
        self.create_subscription(
            String, self.topics.WORDS_TOPIC, self.on_word_received, 10
        )

        # listen for test time
        self.create_subscription(
            Empty,
            self.topics.TEST_TOPIC,
            self.on_test_request_received,
            10,
        )

        # listen for when to stop
        self.create_subscription(
            Empty,
            self.topics.STOP_TOPIC,
            self.on_stop_request_received,
            10,
        )

        # listen for user-drawn shapes
        self.create_subscription(
            ShapeMsg,
            self.topics.PROCESSED_USER_SHAPE_TOPIC,
            self.on_user_drawn_shape_received,
            10,
        )

        # listen for user-drawn finger gestures
        self.create_subscription(
            PointStamped,
            self.topics.GESTURE_TOPIC,
            self.on_set_active_shape_gesture,
            10,
        )

        self.create_subscription(
            String,
            self.topics.SHAPE_FINISHED_TOPIC,
            self.on_shape_finished,
            10,
        )

        # Commented method/function out because not presently in use
        # TODO: reintegrate or remove
        # listen for request to clear screen (from tablet)
        # clear_subscriber = self.create_subscription(
        #     Subscriberself.topics.CLEAR_SURFACE_TOPIC,
        #     Empty,
        #     self.on_clear_screen_received,
        # )

        TOPIC_GPT_INPUT = "chatgpt_input"
        self.create_subscription(
            String, TOPIC_GPT_INPUT, self.on_user_chat_received, 10
        )

        START_SENDING_VOICE = "speech_rec"
        self.create_subscription(
            String,
            START_SENDING_VOICE,
            self.on_feedback_received,
            10,
        )

        # initialise display manager for shapes (manages positioning of shapes)
        self.clear_all_shapes_service = self.create_client(
            ClearAllShapes, "clear_all_shapes"
        )
        self.get_logger().info(
            "Waiting for display manager services to become available"
        )
        self.clear_all_shapes_service.wait_for_service()

        sleep(2.0)  # Allow some time for the subscribers to do t heir thing,
        # or the first message will be missed
        # (eg. first traj on tablet, first clear request locally)

        # Trajectory publishing parameters
        (
            self.t0,
            self.dt,
            self.delay_before_executing,
        ) = InteractionSettings.get_trajectory_timings(
            self.nao_settings.get("nao_writing")
        )

        # Phrase params are in nao_settings.phrase_manager

        # text_to_speech, motion_proxy, effector - all need handled
        # Should these be handled by a class?
        # Number of params being passed here is stupid
        # Phrases could be handled by a class as well

        self.info_to_restore_wait_for_shape_to_finish = None
        self.info_to_restore_wait_for_robot_to_connect = None
        # self.info_to_restore_wait_for_tablet_to_connect = None

        self.drawing_letter_substates: List[str] = [
            "WAITING_FOR_ROBOT_TO_CONNECT",
            "WAITING_FOR_TABLET_TO_CONNECT",
            "PUBLISHING_LETTER",
        ]

        # shape params
        # Frame ID to publish points in
        # FRAME = rospy.get_param("~writing_surface_frame_id", "writing_surface")
        self.frame = "writing_surface"
        # self.ros_node.get_parameter(
        #     "writing_surface_frame_id"
        # ).value

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

    def clear_all_shapes(self, message):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service is not available, waiting...")

        future = self.client.call_async(message)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            # Process the service response
            pass
        else:
            self.get_logger().error("Service call failed.")

    def on_user_chat_received(self, msg: String) -> None:
        """
        if self.chat_enabled is enabled, send the word to Chatgpt,
        and let robot say() the output

        :in_chat: std_msgs.msg.String, input fot chat in .data
        """
        self.get_logger().info("input for GPT: " + msg.data)
        if self.chatGPT_enabled:
            self.response = self.managerGPT.get_gpt_response(msg.data)
            if self.chatGPT_to_say_enabled:
                self.session.post(
                    "http://localhost:5000/say", json={"phrase": self.response}
                )

    def on_user_drawn_shape_received(self, shape: ShapeMsg) -> None:
        """
        Processes a user-drawn shape and identifies the corresponding
        letter.

        :param shape: The received shape message from the user.
        """
        self.get_logger().info("Received processed shape")
        self.get_logger().info(self.state_machine.get_state())
        if (
            self.state_machine.get_state() == "WAITING_FOR_FEEDBACK"
            or self.state_machine.get_state() == "ASKING_FOR_FEEDBACK"
        ):
            self.get_logger().info("processing received shape")
            nbpts = int(len(shape.path) / 2)
            # Create a path from the shape by combining x and y coordinates into tuples
            path = list(
                zip(shape.path[:nbpts], [-y for y in shape.path[nbpts:]])
            )
            self.get_logger().info(str(path))
            # self.get_logger().info(shape.path)

            # Split the path into segments based on a template
            demo_from_template = (
                self.device_manager.screen_manager.split_path_from_template(
                    path
                )
            )  # type: ignore
            self.get_logger().info(str(demo_from_template))
            # If the demo_from_template is not empty
            if demo_from_template:
                self.get_logger().info(
                    f"Received template demonstration for letters {demo_from_template.keys()}"  # noqa: E501
                )

                # Iterate through the path segments
                for name, path in demo_from_template.items():
                    # If the path segment corresponds to a multi-stroke letter,
                    # ignore it
                    if name in ["i", "j", "t"]:
                        self.get_logger().warn(
                            f"Received demonstration for multi-stroke letter {name}: ignoring it."  # noqa: E501
                        )
                        continue

                    # Flatten the path by combining x and y coordinates
                    # into a single list
                    # Note: Unclear why y-coordinate is being flipped
                    flatpath = [x for x, y in path]
                    flatpath.extend([-y for x, y in path])

                    # Append the flattened path as a ShapeMsg to
                    # the demo_shapes_received list
                    self.demo_shapes_received.append(
                        ShapeMsg(path=flatpath, shape_type=name)
                    )

            # If demo_from_template is empty
            else:
                # If an active letter exists
                if self.active_letter:
                    # Assign the shape type as the active letter
                    # and reset the active letter
                    shape.shape_type = self.active_letter
                    self.active_letter = None
                    self.get_logger().info(
                        f"Received demonstration for selected letter {shape.shape_type}"
                    )
                else:
                    self.get_logger().info(str(int(len(shape.path) / 2)))
                    # Find the letter corresponding to the shape path
                    letter, bb = self.device_manager.screen_manager.find_letter(
                        shape.path
                    )

                    # If a letter is found
                    if letter:
                        # Assign the shape type as the found letter
                        shape.shape_type = letter
                        self.get_logger().info(
                            f"Received demonstration for {shape.shape_type}"
                        )
                    else:
                        self.get_logger().warn(
                            "Received demonstration, but unable to find the letterthat was demonstrated! Ignoring it."  # noqa: E501
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
        if self.nao_settings.get("nao_writing"):
            if self.nao_settings.get("nao_standing"):
                self.session.post(
                    "http://localhost:5000/go_to_posture",
                    json={"posture": "StandInit", "speed": 0.3},
                )
            else:
                self.session.post("http://localhost:5000/rest")
                self.session.post(
                    "http://localhost:5000/set_stiffness",
                    json={
                        "joints": ["Head", "LArm", "RArm"],
                        "set_stiffness": 0.5,
                    },
                )
                self.session.post(
                    "http://localhost:5000/set_stiffness",
                    json={
                        "joints": [
                            "LHipYawPitch",
                            "LHipRoll",
                            "LHipPitch",
                            "RHipYawPitch",
                            "RHipRoll",
                            "RHipPitch",
                        ],
                        "set_stiffness": 0.8,
                    },
                )

        if self.nao_settings.get("nao_speaking"):
            if self.nao_settings.get("alternate_sides_looking_at"):
                self.session.post(
                    "http://localhost:5000/look_and_ask_for_feedback",
                    json={
                        "phrase": self.phrase_manager.intro_phrase,
                        "side": self.nao_settings.get("next_side_to_look_at"),
                    },
                )
            else:
                self.session.post(
                    "http://localhost:5000/look_and_ask_for_feedback",
                    json={
                        "phrase": self.phrase_manager.intro_phrase,
                        "side": self.nao_settings.get("person_side"),
                    },
                )
        # clear screen
        self.publish_manager.pub_clear.publish(Empty())
        sleep(2.0)

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
            self.get_logger().info(f"Received word: {self.word_received}")
        else:
            self.get_logger().info("no word received")
            self.word_received = None  # ignore

    def on_clear_screen_received(self, message) -> None:
        """
        Processes a clear screen request and updates the instance
        variables accordingly.

        :param message (Empty): The received clear screen request
        message.
        """
        self.get_logger().info("Clearing display")

        # NOTE: follow up this clear_all_shapes pass to service then call func
        # clear_all_shapes = rospy.ServiceProxy("clear_all_shapes", ClearAllShapes)
        self.clear_all_shapes(message)

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
        self.get_logger().info("input for GPT: " + in_chat.data)
        if self.chatGPT_enabled:
            self.response = self.managerGPT.get_gpt_response(in_chat.data)
            if (
                self.nao_settings.get("nao_connected")
                and self.chatGPT_to_say_enabled
            ):
                self.session.post(
                    "http://localhost:5000/say", json={"phrase": self.response}
                )

                # signal listening again after nao speaks
                self.publish_manager.pub_listening_signal.publish(
                    String(data="convo")
                )
        else:
            self.response = "feedback"
            self.session.post(
                "http://localhost:5000/say",
                json={"phrase": "Thanks for feedback"},
            )

        self.feedback_received = self.response

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
        if self.word_received is not None:
            info_for_next_state["word_received"] = self.word_received
            self.word_received = None
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
        if self.stop_request_received:
            next_state = "STOPPING"

        return next_state

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
        self.get_logger().info("STATE: RESPONDING_TO_DEMONSTRATION_FULL_WORD")

        demo_shapes_received = info_from_prev_state["demo_shapes_received"]

        letters = "".join([s.shape_type for s in demo_shapes_received])

        if self.nao_settings.get("nao_speaking"):
            (
                to_say,
                self.phrase_manager.demo_response_phrases_counter,
            ) = self.get_next_phrase(
                self.phrase_manager.demo_response_phrases,
                self.phrase_manager.demo_response_phrases_counter,
                letters,
            )

            self.session.post(
                "http://localhost:5000/nao_speak_and_log_phrase",
                json={"phrase": to_say},
            )

        # 1- Update the shape models with the incoming demos
        for shape in demo_shapes_received:
            glyph = shape.path
            shape_name = shape.shape_type

            self.get_logger().debug(f"Downsampling {shape_name}...")
            glyph = self.downsample_shape(glyph)
            self.get_logger().info(
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
        sleep(2.0)

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
        self.get_logger().info("STATE: PUBLISHING_WORD")
        self.get_logger().info(
            "From " + info_from_prev_state["state_came_from"]
        )
        self.get_logger().info("To " + info_from_prev_state["state_go_to"])

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
        if self.nao_settings.get("nao_connected"):
            self.session.post("http://localhost:5000/look_at_tablet")

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
        #     self.ros_node.get_logger().info("STATE: WAITING_FOR_LETTER_TO_FINISH")
        #     self.info_to_restore_wait_for_shape_to_finish = info_from_prev_state

        # info_for_next_state = {'state_came_from': 'WAITING_FOR_LETTER_TO_FINISH',
        #                        'state_go_to': info_from_prev_state['state_go_to']}
        next_state = None

        # Once shape is finished
        if self.shape_finished:
            # Draw the templates for the demonstrations
            ref_boundingboxes = self.device_manager.screen_manager.place_reference_boundingboxes(
                self.device_manager.word_manager.current_collection
            )
            for bb in ref_boundingboxes:
                self.publish_manager.pub_bounding_boxes.publish(
                    self.make_bounding_box_msg(bb, selected=False)
                )
                sleep(0.2)

            self.shape_finished = False
            # info_for_next_state = self.info_to_restore_wait_for_shape_to_finish
            if info_from_prev_state["state_go_to"] is not None:
                next_state = info_from_prev_state["state_go_to"]
            else:
                self.get_logger().info("STATE: WAITING_FOR_LETTER_TO_FINISH")
                self.get_logger().warn("WARNING: state_go_to not set")
                next_state = "WAITING_FOR_FEEDBACK"
            info_for_next_state = {
                "state_came_from": "WAITING_FOR_LETTER_TO_FINISH",
                "item_written": info_from_prev_state["item_written"],
            }

        # Check if next state should be stopping.
        next_state = self.check_stop_request_received(next_state)

        # If haven't received next state then go into waiting state
        if next_state is None:
            sleep(0.1)
            next_state = "WAITING_FOR_LETTER_TO_FINISH"
            info_for_next_state = {
                "state_go_to": info_from_prev_state["state_go_to"],
                "state_came_from": "WAITING_FOR_LETTER_TO_FINISH",
                "item_written": info_from_prev_state["item_written"],
            }
        else:
            info_for_next_state = info_from_prev_state

        return next_state, info_for_next_state

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
        self.get_logger().info("STATE: STOPPING")

        # Factor two if statements to method for NaoSettings
        if self.nao_settings.get("nao_speaking"):
            self.session.post(
                "http://localhost:5000/nao_speak_and_log_phrase",
                json={"phrase": self.phrase_manager.thank_you_phrase[0]},
            )

        # Set nao to rest
        self.session.post("http://localhost:5000/rest")

        next_state = "EXIT"
        info_for_next_state = 0
        self.get_logger().info("Interaction exited")
        rclpy.shutdown()

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
        self.get_logger().info("STATE: STARTING_INTERACTION")
        # If nao speaking say intro phrase
        if self.nao_settings.get("nao_speaking"):
            self.session.post(
                "http://localhost:5000/handle_look_and_ask_for_feedback",
                json={"phrase": self.phrase_manager.intro_phrase},
            )

        next_state = "WAITING_FOR_WORD"
        info_for_next_state = {"state_came_from": "STARTING_INTERACTION"}

        next_state = self.check_stop_request_received(next_state)

        return next_state, info_for_next_state

    def generate_word_callback(self, request, response) -> None:
        """
        Callback function that is triggered when a new word is
        generated.

        :param message: The generated word.
        """
        response.data = self.gpt_word_generator.generate_word()
        self.get_logger().info("Generated word: " + response.data)
        return response

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
            self.get_logger().info("STATE: WAITING_FOR_WORD")

            # broken for now (pub_camera_status in PublisherManager class)
            # self.publish_manager.pub_camera_status.publish(
            #     True)  # Turn camera on

        if info_from_prev_state["state_came_from"] == "STARTING_INTERACTION":
            # signal listening to start conversation
            self.publish_manager.pub_listening_signal.publish(
                String(data="convo")
            )

        info_for_next_state = {"state_came_from": "WAITING_FOR_WORD"}
        if self.word_received is None:
            next_state = "WAITING_FOR_WORD"
            sleep(0.1)  # Don't check again immediately
        else:
            # Check for received word and modify next_state if so
            next_state, info_for_next_state = self.handle_word_received(
                next_state, info_for_next_state
            )  # type: ignore
            # self.publish_manager.pub_camera_status.publish(
            #     False)  # Turn camera off

        if self.stop_request_received:
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
        #     self.ros_node.get_logger().info("STATE: WAITING_FOR_FEEDBACK")
        #     self.publish_manager.pub_camera_status.publish(
        #         True)  # turn camera on

        info_for_next_state: Dict[str, Any] = {
            "state_came_from": "WAITING_FOR_FEEDBACK"
        }
        next_state = None

        if self.feedback_received is not None:
            self.get_logger().info(self.feedback_received)
            self.get_logger().info("STATE: WAITING_FOR_FEEDBACK, got feedback")
            info_for_next_state["feedback_received"] = self.feedback_received
            self.feedback_received = None
            next_state = "WAITING_FOR_WORD"

            # Commented method/function out because not presently in use
            # TODO: reintegrate or remove
            # Ensure robot is connected before going to that state
            # info_for_next_state['state_go_to'] = [next_state]
            # next_state = 'WAITING_FOR_ROBOT_TO_CONNECT'

        if self.demo_shapes_received:
            # self.get_logger().info(
            #     self.demo_shapes_received
            # )
            self.get_logger().info("STATE: WAITING_FOR_FEEDBACK, got demo")
            info_for_next_state[
                "demo_shapes_received"
            ] = self.demo_shapes_received
            self.demo_shapes_received = []
            # self.get_logger().info(
            #     self.demo_shapes_received
            # )
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
            sleep(0.1)  # Don't check again immediately
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
        self.get_logger().info("STATE: RESPONDING_TO_NEW_WORD")
        word_to_learn = info_from_prev_state["word_received"].data
        word_seen_before = self.device_manager.word_manager.new_collection(
            word_to_learn
        )

        if self.nao_settings.get("nao_speaking"):
            if word_seen_before:
                # word_again_response_phrases doesnt exist
                (
                    to_say,
                    self.phrase_manager.word_again_response_phrases_counter,
                ) = self.get_next_phrase(
                    self.phrase_manager.word_again_response_phrases,
                    self.phrase_manager.word_again_response_phrases_counter,
                    word_to_learn,
                )
            else:
                # word_response_phrases doesnt exist
                (
                    to_say,
                    self.phrase_manager.word_response_phrases_counter,
                ) = self.get_next_phrase(
                    self.phrase_manager.word_response_phrases,
                    self.phrase_manager.word_response_phrases_counter,
                    word_to_learn,
                )

            self.session.post(
                "http://localhost:5000/nao_speak_and_log_phrase",
                json={"phrase": to_say},
            )

        # Clear screen
        self.device_manager.screen_manager.clear()

        # Commented method/function out because not presently in use
        #        TODO: reintegrate or remove
        # currently the probulisher is not implimented, skipping publish
        # !!!!!!!! change later
        # self.publish_manager.pub_clear.publish(Empty())

        sleep(0.5)

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
        self.get_logger().info("STATE: ASKING_FOR_FEEDBACK")
        self.get_logger().info(
            "From " + info_from_prev_state["state_came_from"]
        )
        self.get_logger().info(
            "item_written = " + info_from_prev_state["item_written"]
        )

        item_written = info_from_prev_state["item_written"]
        if self.nao_settings.get("nao_speaking"):
            (
                to_say,
                self.phrase_manager.asking_phrases_after_word_counter,
            ) = self.get_next_phrase(
                self.phrase_manager.asking_phrases_after_word,
                self.phrase_manager.asking_phrases_after_word_counter,
                item_written,
            )

            self.session.post(
                "http://localhost:5000/handle_look_and_ask_for_feedback",
                json={"phrase": to_say},
            )
            self.session.post("http://localhost:5000/look_at_tablet")

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

        shape[0 : self.NUMPOINTS_SHAPEMODELER] = x_shape  # noqa: E203
        shape[self.NUMPOINTS_SHAPEMODELER :] = y_shape  # noqa: E203

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
        traj.header.frame_id = self.frame
        # traj.header.stamp = rospy.Time.now() + rospy.Duration(
        #     int(self.delay_before_executing)
        # )
        traj.header.stamp = (
            ROSClock().now()
            + Duration(seconds=int(self.delay_before_executing))
        ).to_msg()

        point_idx = 0
        paths = shaped_word.get_letters_paths()

        if log:
            self.generated_word_logger.info(
                "%s" % [[(x, -y) for x, y in path] for path in paths]
            )

        for path in paths:
            for x, y in path:
                point = PoseStamped()

                point.pose.position.x = x
                point.pose.position.y = y
                point.header.frame_id = self.frame
                # @TODO allow for variable time between points for now
                point.header.stamp = Time(
                    seconds=self.t0 + point_idx * delta_t
                ).to_msg()

                traj.poses.append(deepcopy(point))  # type: ignore

                point_idx += 1

        return traj


def get_nao_settings(session):
    return session.get("http://localhost:5000/get_settings").json()


def main(args=None):
    # Init node inside main to avoid running node if imported
    # rospy.init_node("learning_words_nao")
    session = Session()
    nao_settings = get_nao_settings(session)
    phrase_manager = PhraseManager(nao_settings.get("LANGUAGE"))

    rclpy.init(args=None)

    # init node
    # node = Node("learning_words_nao")

    session = session
    # topics = SubscriberTopics(node)

    # dataset_directory = rospy.get_param("~dataset_directory", "default")
    # node.declare_parameter("dataset_directory", "default")
    dataset_directory = "/home/nao/share/letter_model_datasets/alexis_set_for_children"  # noqa: E501
    # "default" #node.get_parameter("dataset_directory").value

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
        except:  # noqa: E722
            RuntimeError("Missing Dataset")

    # Init state machine
    state_machine = StateMachine()

    generated_word_logger = logging.getLogger("word_logger")
    # HACK: should properly configure the path from an option
    generated_word_logger = configure_logging(generated_word_logger)
    node = LearningWordsNao(
        state_machine,
        nao_settings,
        session,
        phrase_manager,
        generated_word_logger,
    )

    # Add interaction states to state machine
    state_machine.add_state("STARTING_INTERACTION", node.start_interaction)
    state_machine.add_state("WAITING_FOR_WORD", node.wait_for_word)
    state_machine.add_state("RESPONDING_TO_NEW_WORD", node.respond_to_new_word)
    state_machine.add_state("PUBLISHING_WORD", node.publish_word)
    state_machine.add_state(
        "WAITING_FOR_LETTER_TO_FINISH", node.wait_for_shape_to_finish
    )
    state_machine.add_state("ASKING_FOR_FEEDBACK", node.ask_for_feedback)
    state_machine.add_state("WAITING_FOR_FEEDBACK", node.wait_for_feedback)
    state_machine.add_state(
        "RESPONDING_TO_DEMONSTRATION_FULL_WORD",
        node.respond_to_demonstration_with_full_word,
    )
    state_machine.add_state("STOPPING", node.stop_interaction)
    state_machine.add_state("EXIT", None, end_state=True)
    state_machine.set_start("STARTING_INTERACTION")
    info_for_start_state = {"state_came_from": None}

    # Set nao up for interaction
    # nao_settings.set_nao_interaction()
    session.post("http://localhost:5000/set_interaction")

    # Init subscribers
    # listen for a new child signal

    # TODO: Make a ROS server so that *everyone* can access the connection statuses

    # robot_watchdog = Watchdog('watchdog_clear/robot', 0.8)

    node.get_logger().info(
        "Nao configuration: writing=%s, speaking=%s (%s), standing=%s, handedness=%s"
        % (
            nao_settings.get("nao_writing"),
            nao_settings.get("nao_speaking"),
            nao_settings.get("LANGUAGE"),
            nao_settings.get("nao_standing"),
            nao_settings.get("NAO_HANDEDNESS"),
        )
    )

    # initialise word manager (passes feedback to shape learners
    # and keeps history of words learnt)
    InteractionSettings.set_dataset_directory(dataset_directory)
    # path to a log file where all learning steps will be stored

    # Start the interaction
    # state_machine.run(info_for_start_state)
    thread = threading.Thread(
        target=lambda: state_machine.run(info_for_start_state)
    )
    thread.start()

    node.get_logger().info("Interaction started")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    node.device_manager.tablet_watchdog.stop()

    # robot_watchdog.stop()


if __name__ == "__main__":
    main()
