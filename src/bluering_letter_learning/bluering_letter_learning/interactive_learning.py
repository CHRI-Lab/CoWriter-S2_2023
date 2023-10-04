#!/usr/bin/env python
# coding: utf-8

"""
Nao learning words using the shape_learning package.
This node manages the state machine which maintains the interaction sequence,
receives interaction inputs e.g. which words to write and user demonstrations, 
passes these demonstrations to the learning algorithm, and publishes the 
resulting learned shapes for the robot and tablet to draw.
"""
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Empty, Bool, Float64MultiArray
from nav_msgs.msg import Path
from bluering_letter_learning.msg import Shape as ShapeMsg
from bluering_letter_learning.msg import ActionToDo as ActionToDoMsg
from bluering_letter_learning.state_machine import StateMachine
from bluering_letter_learning.controllers.state_controller import (
    StateController,
)
from bluering_letter_learning.srv import *

rospy.init_node("interactive_learning")
rospy.loginfo("[interactive_learning] Starting...")


class InteractiveLearning(object):
    """
    Interaction config parameters come from launch file
    """

    def __init__(self):
        # ? Nao parameters
        self.NAO_IP = rospy.get_param(
            "~nao_ip", "127.0.0.1"
        )  # ? default behaviour is to connect to simulator locally
        self.NAO_SPEAKING = rospy.get_param(
            "~nao_speaking", True
        )  # ? whether or not the robot should speak
        self.NAO_WRITING = rospy.get_param(
            "~nao_writing", True
        )  # ? whether or not the robot should move its arms
        self.NAO_STANDING = rospy.get_param(
            "~nao_standing", True
        )  # ? whether or not the robot should stand or rest on its knies
        self.NAO_CONNECTED = rospy.get_param(
            "~use_robot_in_interaction", True
        )  # ? whether or not the robot is being used for the interaction (looking, etc.)
        self.NAO_WRITING = (
            self.NAO_WRITING and self.NAO_CONNECTED
        )  # ? use NAO_CONNECTED var as the stronger property
        self.NAO_SPEAKING = self.NAO_SPEAKING and self.NAO_CONNECTED
        self.NAO_STANDING = self.NAO_STANDING and self.NAO_CONNECTED
        self.LANGUAGE = rospy.get_param(
            "~language", "english"
        )  # ? language to speak
        self.NAO_HANDEDNESS = rospy.get_param("~nao_handedness", "right")
        if self.NAO_HANDEDNESS.lower() == "right":
            self.effector = "RArm"
        elif self.NAO_HANDEDNESS.lower() == "left":
            self.effector = "LArm"
        else:
            print("error in handedness param")
        self.isFrontInteraction = True

        # ? shape params
        self.FRAME = rospy.get_param(
            "~writing_surface_frame_id", "writing_surface"
        )  # ? Frame ID to publish points in
        self.FEEDBACK_TOPIC = rospy.get_param(
            "~shape_feedback_topic", "shape_feedback"
        )  # ? Name of topic to receive feedback on
        BOUNDING_BOXES_TOPIC = rospy.get_param(
            "~bounding_boxes_topic", "/boxes_to_draw"
        )  # ? Name of topic to publish bounding boxes of letters to
        SHAPE_TOPIC = rospy.get_param(
            "~trajectory_output_topic", "/write_traj"
        )  # ? Name of topic to publish shapes to
        SHAPE_TOPIC_DOWNSAMPLED = rospy.get_param(
            "~trajectory_output_nao_topic", "/write_traj_downsampled"
        )  # ? Name of topic to publish shapes to

        self.SHAPE_LOGGING_PATH = rospy.get_param(
            "~shape_log", ""
        )  # ? path to a log file where all learning steps will be stored
        self.DATASET_DIR = rospy.get_param("~dataset_directory", "default")
        self.loadDataset()

        # ? tablet params
        self.CLEAR_SURFACE_TOPIC = rospy.get_param(
            "~clear_writing_surface_topic", "clear_screen"
        )
        self.SHAPE_FINISHED_TOPIC = rospy.get_param(
            "~shape_writing_finished_topic", "shape_finished"
        )
        # ? Name of topic to get gestures representing the active shape for demonstration
        self.GESTURE_TOPIC = rospy.get_param(
            "~gesture_info_topic", "gesture_info"
        )

        # ? interaction params
        self.WORDS_TOPIC = rospy.get_param(
            "~words_to_write_topic", "words_to_write"
        )
        self.PROCESSED_USER_SHAPE_TOPIC = rospy.get_param(
            "~processed_user_shape_topic", "user_shapes_processed"
        )  # ? Listen for user shapes
        self.TEST_TOPIC = rospy.get_param(
            "~test_request_topic", "test_learning"
        )  # ? Listen for when test card has been shown to the robot
        self.STOP_TOPIC = rospy.get_param(
            "~stop_request_topic", "stop_learning"
        )  # ? Listen for when stop card has been shown to the robot
        self.NEW_CHILD_TOPIC = rospy.get_param(
            "~new_teacher_topic", "new_child"
        )  # ? Welcome a new teacher but don't reset learning algorithm's 'memory'
        self.PERSON_SIDE = rospy.get_param(
            "~person_side", self.NAO_HANDEDNESS.lower()
        )  # ? side where person is (left/right)
        self.PUBLISH_STATUS_TOPIC = rospy.get_param(
            "~camera_publishing_status_topic", "camera_publishing_status"
        )  # ? Controls the camera based on the interaction state (turn it off for writing b/c CPU gets maxed)

        self.ACTION_TODO_TOPIC = rospy.get_param(
            "~action_todo_topic", "action_todo"
        )

        self.alternateSidesLookingAt = False  # ? if true, nao will look to a different side each time. (not super tested)
        # nextSideToLookAt = 'Right'

        self.drawingLetterSubstates = [
            "WAITING_FOR_ROBOT_TO_CONNECT",
            "WAITING_FOR_TABLET_TO_CONNECT",
            "PUBLISHING_LETTER",
        ]

        """ publishers """
        self.pub_camera_status = rospy.Publisher(
            self.PUBLISH_STATUS_TOPIC, Bool, queue_size=10
        )
        self.pub_traj = rospy.Publisher(
            SHAPE_TOPIC, Path, queue_size=10
        )  # ? publish to traj topic for the trajectory_following to write
        self.pub_bounding_boxes = rospy.Publisher(
            BOUNDING_BOXES_TOPIC, Float64MultiArray, queue_size=10
        )
        self.pub_traj_downsampled = rospy.Publisher(
            SHAPE_TOPIC_DOWNSAMPLED, Path, queue_size=10
        )
        self.pub_clear = rospy.Publisher(
            self.CLEAR_SURFACE_TOPIC, Empty, queue_size=10
        )
        LISTENING_SIGNAL_TOPIC = rospy.get_param(
            "~listening_signal_topic", "listening_signal"
        )  # ? Name of topic to publish listening signal to
        self.pub_listening_signal = rospy.Publisher(
            LISTENING_SIGNAL_TOPIC, String, queue_size=10
        )  # ? take in either `convo` or `mode``

        """ tablet watchdog """
        # TODO: Make a ROS server so that *everyone* can access the connection statuses
        from bluering_letter_learning.watchdog import Watchdog

        self.tabletWatchdog = Watchdog("watchdog_clear/tablet", 0.4)
        # robotWatchdog = Watchdog('watchdog_clear/robot', 0.8)

    def loadDataset(self):
        rospy.loginfo(f"[ ] DATASET_DIR : {self.DATASET_DIR}")
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
        rospy.loginfo(f"[+] DATASET_DIR : {self.DATASET_DIR}")

    def initControllers(self):
        rospy.loginfo("[interactive_learning] Initializing StateController")

        rospy.loginfo(
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
        new_child_subscriber = rospy.Subscriber(
            self.NEW_CHILD_TOPIC, String, self.controller.onChildReceived
        )

        # ? listen for words to write
        words_subscriber = rospy.Subscriber(
            self.WORDS_TOPIC, String, self.controller.onWordReceived
        )

        # ? listen for request to clear screen (from tablet)
        clear_subscriber = rospy.Subscriber(
            self.CLEAR_SURFACE_TOPIC,
            Empty,
            self.controller.onClearScreenReceived,
        )

        # ? listen for test time
        test_subscriber = rospy.Subscriber(
            self.TEST_TOPIC, Empty, self.controller.onTestRequestReceived
        )

        # ? listen for when to stop
        stop_subscriber = rospy.Subscriber(
            self.STOP_TOPIC, Empty, self.controller.onStopRequestReceived
        )

        # ? listen for user-drawn shapes
        shape_subscriber = rospy.Subscriber(
            self.PROCESSED_USER_SHAPE_TOPIC,
            ShapeMsg,
            self.controller.onUserDrawnShapeReceived,
        )

        # ? listen for user-drawn finger gestures
        gesture_subscriber = rospy.Subscriber(
            self.GESTURE_TOPIC,
            PointStamped,
            self.controller.onSetActiveShapeGesture,
        )

        # ? wait for finishing signal from where ?
        self.controller.infoToRestore_waitForShapeToFinish = rospy.Subscriber(
            self.SHAPE_FINISHED_TOPIC, String, self.controller.onShapeFinished
        )

        # ? listen for what to act from diagram_manager
        to_act_subscriber = rospy.Subscriber(
            self.ACTION_TODO_TOPIC, ActionToDoMsg, self.controller.onToAct
        )

    def spin(self):
        # ? initialise display manager for shapes (manages positioning of shapes)
        rospy.loginfo(
            "Waiting for display manager services to become available"
        )
        rospy.wait_for_service("clear_all_shapes")

        rospy.sleep(
            2.0
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

        rospy.spin()

        self.tabletWatchdog.stop()
        # robotWatchdog.stop()


if __name__ == "__main__":
    rospy.loginfo("[interactive_learning] Entering main")
    iLearning = InteractiveLearning()
    iLearning.initControllers()
    iLearning.subscribeEvents()
    iLearning.spin()
