#!/usr/bin/env python
# coding: utf-8

"""
Nao learning words using the shape_learning package.
This node manages the state machine which maintains the interaction sequence,
receives interaction inputs e.g. which words to write and user demonstrations, 
passes these demonstrations to the learning algorithm, and publishes the 
resulting learned shapes for the robot and tablet to draw.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Empty, Bool, Float64MultiArray
from nav_msgs.msg import Path
from bluering_letter_learning.include.state_machine import StateMachine
from bluering_letter_learning.include.controllers.state_controller import (
    StateController,
)
from interface.msg import Shape as ShapeMsg
from interface.msg import ActionToDo as ActionToDoMsg
from interface.srv import ClearAllShapes


class InteractiveLearning(Node):
    """
    Interaction config parameters come from launch file
    """

    def __init__(self):
        super().__init__("interactive_learning")

        self.get_logger.info("[interactive_learning] Starting...")

        # ? Nao parameters
        self.NAO_IP = self.declare_parameter(
            "~nao_ip", "127.0.0.1"
        ).value  # ? default behaviour is to connect to simulator locally
        self.NAO_SPEAKING = self.declare_parameter(
            "~nao_speaking", True
        ).value  # ? whether or not the robot should speak
        self.NAO_WRITING = self.declare_parameter(
            "~nao_writing", True
        ).value  # ? whether or not the robot should move its arms
        self.NAO_STANDING = self.declare_parameter(
            "~nao_standing", True
        ).value  # ? whether or not the robot should stand or rest on its knies
        self.NAO_CONNECTED = self.declare_parameter(
            "~use_robot_in_interaction", True
        ).value  # ? whether or not the robot is being used for the interaction (looking, etc.)
        self.NAO_WRITING = (
            self.NAO_WRITING and self.NAO_CONNECTED
        )  # ? use NAO_CONNECTED var as the stronger property
        self.NAO_SPEAKING = self.NAO_SPEAKING and self.NAO_CONNECTED
        self.NAO_STANDING = self.NAO_STANDING and self.NAO_CONNECTED
        self.LANGUAGE = self.declare_parameter(
            "~language", "english"
        ).value  # ? language to speak
        self.NAO_HANDEDNESS = self.declare_parameter(
            "~nao_handedness", "right"
        ).value
        if self.NAO_HANDEDNESS.lower() == "right":
            self.effector = "RArm"
        elif self.NAO_HANDEDNESS.lower() == "left":
            self.effector = "LArm"
        else:
            print("error in handedness param")
        self.isFrontInteraction = True

        # ? shape params
        self.FRAME = self.declare_parameter(
            "~writing_surface_frame_id", "writing_surface"
        ).value  # ? Frame ID to publish points in
        self.FEEDBACK_TOPIC = self.declare_parameter(
            "~shape_feedback_topic", "shape_feedback"
        ).value  # ? Name of topic to receive feedback on
        BOUNDING_BOXES_TOPIC = self.declare_parameter(
            "~bounding_boxes_topic", "/boxes_to_draw"
        ).value  # ? Name of topic to publish bounding boxes of letters to
        SHAPE_TOPIC = self.declare_parameter(
            "~trajectory_output_topic", "/write_traj"
        ).value  # ? Name of topic to publish shapes to
        SHAPE_TOPIC_DOWNSAMPLED = self.declare_parameter(
            "~trajectory_output_nao_topic", "/write_traj_downsampled"
        ).value  # ? Name of topic to publish shapes to

        self.SHAPE_LOGGING_PATH = self.declare_parameter(
            "~shape_log", ""
        ).value  # ? path to a log file where all learning steps will be stored
        self.DATASET_DIR = self.declare_parameter(
            "~dataset_directory", "default"
        ).value
        self.loadDataset()

        # ? tablet params
        self.CLEAR_SURFACE_TOPIC = self.declare_parameter(
            "~clear_writing_surface_topic", "clear_screen"
        ).value
        self.SHAPE_FINISHED_TOPIC = self.declare_parameter(
            "~shape_writing_finished_topic", "shape_finished"
        ).value
        # ? Name of topic to get gestures representing the active shape for demonstration
        self.GESTURE_TOPIC = self.declare_parameter(
            "~gesture_info_topic", "gesture_info"
        ).value

        # ? interaction params
        self.WORDS_TOPIC = self.declare_parameter(
            "~words_to_write_topic", "words_to_write"
        ).value
        self.PROCESSED_USER_SHAPE_TOPIC = self.declare_parameter(
            "~processed_user_shape_topic", "user_shapes_processed"
        ).value  # ? Listen for user shapes
        self.TEST_TOPIC = self.declare_parameter(
            "~test_request_topic", "test_learning"
        ).value  # ? Listen for when test card has been shown to the robot
        self.STOP_TOPIC = self.declare_parameter(
            "~stop_request_topic", "stop_learning"
        ).value  # ? Listen for when stop card has been shown to the robot
        self.NEW_CHILD_TOPIC = self.declare_parameter(
            "~new_teacher_topic", "new_child"
        ).value  # ? Welcome a new teacher but don't reset learning algorithm's 'memory'
        self.PERSON_SIDE = self.declare_parameter(
            "~person_side", self.NAO_HANDEDNESS.lower()
        ).value  # ? side where person is (left/right)
        self.PUBLISH_STATUS_TOPIC = self.declare_parameter(
            "~camera_publishing_status_topic", "camera_publishing_status"
        ).value  # ? Controls the camera based on the interaction state (turn it off for writing b/c CPU gets maxed)

        self.ACTION_TODO_TOPIC = self.declare_parameter(
            "~action_todo_topic", "action_todo"
        ).value

        self.alternateSidesLookingAt = False  # ? if true, nao will look to a different side each time. (not super tested)
        # nextSideToLookAt = 'Right'

        self.drawingLetterSubstates = [
            "WAITING_FOR_ROBOT_TO_CONNECT",
            "WAITING_FOR_TABLET_TO_CONNECT",
            "PUBLISHING_LETTER",
        ]

        """ publishers """
        self.pub_camera_status = self.create_publisher(
            Bool, self.PUBLISH_STATUS_TOPIC, 10
        )
        self.pub_traj = self.create_publisher(
            Path, SHAPE_TOPIC, 10
        )  # ? publish to traj topic for the trajectory_following to write
        self.pub_bounding_boxes = self.create_publisher(
            Float64MultiArray, BOUNDING_BOXES_TOPIC, 10
        )
        self.pub_traj_downsampled = self.create_publisher(
            Path, SHAPE_TOPIC_DOWNSAMPLED, 10
        )
        self.pub_clear = self.create_publisher(
            Empty, self.CLEAR_SURFACE_TOPIC, 10
        )
        LISTENING_SIGNAL_TOPIC = self.declare_parameter(
            "~listening_signal_topic", "listening_signal"
        ).value  # ? Name of topic to publish listening signal to
        self.pub_listening_signal = self.create_publisher(
            String, LISTENING_SIGNAL_TOPIC, 10
        )  # ? take in either `convo` or `mode``

        """ tablet watchdog """
        # TODO: Make a ROS server so that *everyone* can access the connection statuses
        from bluering_letter_learning.include.watchdog import Watchdog

        self.tabletWatchdog = Watchdog("watchdog_clear/tablet", 0.4)
        # robotWatchdog = Watchdog('watchdog_clear/robot', 0.8)

    def loadDataset(self):
        self.get_logger.info(f"[ ] DATASET_DIR : {self.DATASET_DIR}")
        if self.DATASET_DIR.lower() == "default":  # ? use default
            import inspect
            from shape_learning.shape_modeler import ShapeModeler

            # TODO: letters templates must be in share files of Allograph
            fileName = inspect.getsourcefile(ShapeModeler)
            installDirectory = fileName.split("/lib")[0]
            self.DATASET_DIR = (
                installDirectory
                + "/share/shape_learning/letter_model_datasets/uji_pen_chars2"
            )
        self.get_logger.info(f"[+] DATASET_DIR : {self.DATASET_DIR}")

    def initControllers(self):
        self.get_logger.info(
            "[interactive_learning] Initializing StateController"
        )

        self.get_logger.info(
            "Nao configuration: writing=%s, speaking=%s (%s), standing=%s, handedness=%s"
            % (
                self.NAO_WRITING,
                self.NAO_SPEAKING,
                self.LANGUAGE,
                self.NAO_STANDING,
                self.NAO_HANDEDNESS,
            )
        )

        """ state machine controller """
        self.stateMachine = StateMachine()
        self.controller = StateController(
            self.stateMachine,
            self.NAO_IP,
            9559,
            self.NAO_CONNECTED,
            self.effector,
            self.NAO_WRITING,
            self.NAO_SPEAKING,
            self.NAO_STANDING,
            self.LANGUAGE,
            self.isFrontInteraction,
            self.PERSON_SIDE,
            self.alternateSidesLookingAt,
            self.DATASET_DIR,
            self.SHAPE_LOGGING_PATH,
            self.FRAME,
            self.drawingLetterSubstates,
            self.pub_camera_status,
            self.pub_bounding_boxes,
            self.pub_clear,
            self.tabletWatchdog,
            self.pub_traj,
            self.pub_traj_downsampled,
            self.pub_listening_signal,
        )

        self.stateMachine.add_state(
            "STARTING_INTERACTION", self.controller.startInteraction
        )
        self.stateMachine.add_state(
            "WAITING_FOR_ROBOT_TO_CONNECT",
            self.controller.waitForRobotToConnect,
        )
        self.stateMachine.add_state(
            "WAITING_FOR_CHILD", self.controller.waitForChild
        )
        self.stateMachine.add_state(
            "WAITING_FOR_WORD", self.controller.waitForWord
        )
        self.stateMachine.add_state(
            "RESPONDING_TO_NEW_WORD", self.controller.respondToNewWord
        )
        self.stateMachine.add_state(
            "PUBLISHING_WORD", self.controller.writeWord
        )  # ? directly write the word instead of publishing to ros topics
        self.stateMachine.add_state(
            "PUBLISHING_LETTER", self.controller.publishShape
        )
        self.stateMachine.add_state(
            "WAITING_FOR_LETTER_TO_FINISH", self.controller.waitForShapeToFinish
        )
        self.stateMachine.add_state(
            "ASKING_FOR_FEEDBACK", self.controller.askForFeedback
        )
        self.stateMachine.add_state(
            "WAITING_FOR_FEEDBACK", self.controller.waitForFeedback
        )
        self.stateMachine.add_state(
            "RESPONDING_TO_FEEDBACK", self.controller.respondToFeedback
        )
        # self.stateMachine.add_state('RESPONDING_TO_DEMONSTRATION', self.controller.respondToDemonstration)
        self.stateMachine.add_state(
            "RESPONDING_TO_DEMONSTRATION_FULL_WORD",
            self.controller.respondToDemonstrationWithFullWord,
        )
        self.stateMachine.add_state(
            "RESPONDING_TO_TEST_CARD", self.controller.respondToTestCard
        )
        # self.stateMachine.add_state('RESPONDING_TO_TABLET_DISCONNECT', respondToTabletDisconnect)
        self.stateMachine.add_state(
            "WAITING_FOR_TABLET_TO_CONNECT",
            self.controller.waitForTabletToConnect,
        )
        self.stateMachine.add_state("STOPPING", self.controller.stopInteraction)
        self.stateMachine.add_state("EXIT", None, end_state=True)
        self.stateMachine.set_start(
            "WAITING_FOR_ROBOT_TO_CONNECT"
        )  # ? first state is to wait for robot to connect

    def subscribeEvents(self):
        """listen for events from ROS"""
        # ? listen for a new child signal
        self.create_subscription(
            String, self.NEW_CHILD_TOPIC, self.controller.onChildReceived
        )

        # ? listen for words to write
        self.create_subscription(
            String, self.WORDS_TOPIC, self.controller.onWordReceived
        )

        # ? listen for request to clear screen (from tablet)
        self.create_subscription(
            Empty,
            self.CLEAR_SURFACE_TOPIC,
            self.controller.onClearScreenReceived,
        )

        # ? listen for test time
        self.create_subscription(
            Empty, self.TEST_TOPIC, self.controller.onTestRequestReceived
        )

        # ? listen for when to stop
        self.create_subscription(
            Empty, self.STOP_TOPIC, self.controller.onStopRequestReceived
        )

        # ? listen for user-drawn shapes
        self.create_subscription(
            ShapeMsg,
            self.PROCESSED_USER_SHAPE_TOPIC,
            self.controller.onUserDrawnShapeReceived,
        )

        # ? listen for user-drawn finger gestures
        self.create_subscription(
            PointStamped,
            self.GESTURE_TOPIC,
            self.controller.onSetActiveShapeGesture,
        )

        # ? wait for finishing signal from where ?
        self.controller.infoToRestore_waitForShapeToFinish = (
            self.create_subscription(
                String,
                self.SHAPE_FINISHED_TOPIC,
                self.controller.onShapeFinished,
            )
        )

        # ? listen for what to act from diagram_manager
        self.create_subscription(
            ActionToDoMsg, self.ACTION_TODO_TOPIC, self.controller.onToAct
        )

    def spin(self):
        # ? initialise display manager for shapes (manages positioning of shapes)
        self.get_logger.info(
            "Waiting for display manager services to become available"
        )

        clear_all_shapes = self.create_client(
            ClearAllShapes, "clear_all_shapes"
        )
        while not clear_all_shapes.wait_for_service(timeout_sec=1.0):
            self.get_logger.info(
                "Display manager services not available, waiting..."
            )

        self.get_clock().sleep_for(
            Duration(seconds=2.0)
        )  # ? Allow some time for the subscribers to do their thing,
        # ? or the first message will be missed (eg. first traj on tablet, first clear request locally)

        """ start running """
        self.infoForStartState = {
            "state_goTo": ["STARTING_INTERACTION"],  # ? next state
            "state_cameFrom": None,  # ? previous state
        }
        self.stateMachine.run(
            self.infoForStartState
        )  # ? run the program with the initial state

        rclpy.spin(self)

        self.tabletWatchdog.stop()
        # robotWatchdog.stop()


def main(args=None):
    rclpy.init(args=args)
    iLearning = InteractiveLearning()
    iLearning.initControllers()
    iLearning.subscribeEvents()
    iLearning.spin()
    iLearning.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
