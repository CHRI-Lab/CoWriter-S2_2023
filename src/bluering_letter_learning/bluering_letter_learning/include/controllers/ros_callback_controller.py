#!/usr/bin/env python
# coding: utf-8

import os
import rclpy
from rclpy.duration import Duration

from std_msgs.msg import Empty
from shape_learning.shape_learner_manager import ShapeLearnerManager

from bluering_letter_learning.include.controllers.robot_controller import (
    RobotController,
)
from bluering_letter_learning.include.interaction_settings import (
    InteractionSettings,
)
from bluering_letter_learning.include.utils.text_shaper import (
    TextShaper,
    ScreenManager,
)
from bluering_letter_learning.include.utils.shape_helper import ShapeHelper

from interface.msg import Shape as ShapeMsg
from interface.srv import ClearAllShapes

import logging

generatedWordLogger = logging.getLogger("word_logger")


def configure_logging(path="/tmp"):
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
    generatedWordLogger.addHandler(handler)
    generatedWordLogger.setLevel(logging.DEBUG)


# HACK: should properly configure the path from an option
configure_logging()


class ROSCbController:
    """
    All callbacks for when receiving new messages from ROS
    """

    def __init__(
        self,
        ros_node,
        stateMachine,
        naoIP,
        naoPort,
        naoConnected,
        effector,
        naoWriting,
        naoSpeaking,
        naoStanding,
        language,
        isFrontInteraction,
        personSide,
        alternateSidesLookingAt,
        datasetDirectory,
        shapeLoggingPath,
        shapeHelperFrame,
        pub_camera_status,
        pub_bounding_boxes,
        pub_clear,
        pub_traj,
        pub_traj_downsampled,
        pub_listening_signal,
    ) -> None:
        self.ros_node = ros_node
        self.ros_node.get_logger.info("[ROSCbController] init called")

        self.stateMachine = stateMachine

        self.stopRequestReceived = False
        self.wordReceived = None
        self.childReceived = None
        self.feedbackReceived = None
        self.testRequestReceived = False
        self.shapeFinished = False
        self.demoShapesReceived = []

        self.naoConnected = naoConnected
        self.naoSpeaking = naoSpeaking
        self.naoWriting = naoWriting
        self.naoStanding = naoStanding
        self.effector = effector

        self.personSide = personSide
        self.alternateSidesLookingAt = alternateSidesLookingAt

        """
        Publishers
        """
        self.pub_camera_status = pub_camera_status
        self.pub_bounding_boxes = pub_bounding_boxes
        self.pub_clear = pub_clear
        self.pub_traj = pub_traj
        self.pub_traj_downsampled = pub_traj_downsampled
        self.pub_listening_signal = pub_listening_signal

        """
        Connect to Nao Robot to control
        """
        self.robotCtl = None
        if naoConnected:
            self.ros_node.get_logger.info(
                "[ROSCbController] Initializing RobotController"
            )

            # TODO: change qi_url to use nao_ip and nao_port from arguments
            self.robotCtl = RobotController(
                naoIP,
                naoPort,
                effector,
                naoWriting,
                naoSpeaking,
                naoStanding,
                language,
                isFrontInteraction,
                alternateSidesLookingAt,
            )
            self.ros_node.get_logger.info(
                f"[ROSCbController] Connecting to {naoIP}:{naoPort}"
            )

        """
        Utilities for shape learning and writing
        """
        shapeLoggingDir = os.path.dirname(shapeLoggingPath)
        if not os.path.isdir(shapeLoggingDir):
            os.makedirs(shapeLoggingDir)
        self.shapeLoggingPath = shapeLoggingPath
        # ? initialise word manager (passes feedback to shape learners and keeps history of words learnt)
        InteractionSettings.setDatasetDirectory(datasetDirectory)
        self.wordManager = ShapeLearnerManager(
            InteractionSettings.generateSettings, self.shapeLoggingPath
        )
        self.ros_node.get_logger.info(
            f"[ROSCbController] SHAPE_LOGGING_PATH = {self.shapeLoggingPath}"
        )
        self.ros_node.get_logger.info(
            f"[ROSCbController] datasetDirectory = {datasetDirectory}"
        )

        self.textShaper = TextShaper()
        self.screenManager = ScreenManager(0.2, 0.1395)

        # ? trajectory publishing parameters
        (
            t0,
            self.dt,
            delayBeforeExecuting,
        ) = InteractionSettings.getTrajectoryTimings(naoWriting)
        NUMDESIREDSHAPEPOINTS = (
            5.0  # ? Number of points to downsample the length of shapes to
        )
        NUMPOINTS_SHAPEMODELER = 70  # ? Number of points used by ShapeModelers
        # TODO this could vary for each letter
        self.downSampleFactor = float(NUMPOINTS_SHAPEMODELER - 1) / float(
            NUMDESIREDSHAPEPOINTS - 1
        )
        # ? shapeHelper to generate trajectory
        self.shapeHelper = ShapeHelper(
            shapeHelperFrame,
            NUMDESIREDSHAPEPOINTS,
            NUMPOINTS_SHAPEMODELER,
            t0,
            delayBeforeExecuting,
            generatedWordLogger,
        )

        return

    def onChildReceived(self, message):
        """
        Callback when receiving new child subscribed from ros topic
        ---
        When a child registers to the system, send a message via this ros topic
        This program receives signal a child being logged in or registered and say hello
        Identify new or old child from message data
        ---
        param: message (string) (format: child_name|type)
            childName: string
            childType: string (`new` or `old`)
        """
        print(f"[onChildReceived] called. message = {message}")
        data = message.data
        childName = ""
        childType = ""
        if (
            "|" not in data
        ):  # ? if data passed not contain serialization delimiter, consider as new child
            childName = data
            childType = "new"
        else:
            childName = data.split("|")[0]
            childType = data.split("|")[1]

        old_childReceived = self.childReceived
        self.childReceived = childName
        if self.robotCtl is not None:
            if childType == "new":
                self.robotCtl.say(
                    f"Hello {childName}! I'm NAO. I'm looking forward to learning with you.",
                    self.personSide,
                )
                if (
                    old_childReceived is not None
                ):  # ? a child already been logged in, now seems like trying to log another child
                    if old_childReceived != self.childReceived:
                        print(
                            f"Log a new child in: {self.childReceived}"
                        )  # ? no need to say anything anymore.
                    else:
                        print(f"Already logged {self.childReceived} in")
                else:  # ? starting a completely new session with a new child
                    self.robotCtl.say(
                        "Why don't you choose a word for me to start? Give me one word.",
                        self.personSide,
                    )
            elif childType == "old":
                self.robotCtl.say(
                    f"Welcome back, {childName}! I'm happy to seeing you again. Shall we start?",
                    self.personSide,
                )

        # ? clear screen
        self.pub_clear.publish(Empty())
        self.ros_node.get_clock().sleep_for(Duration(seconds=0.5))
        self.pub_listening_signal.publish("word")

    def onWordReceived(self, message):
        """
        Callback when receiving a word from ros topic
        ---
        Control the robot to write the received word
        """
        print("[onWordReceived] called")

        if (
            self.stateMachine.get_state() == "WAITING_FOR_FEEDBACK"
            or self.stateMachine.get_state() == "WAITING_FOR_WORD"
            or self.stateMachine.get_state() == "ASKING_FOR_FEEDBACK"
            or self.stateMachine.get_state() == "STARTING_INTERACTION"
            or self.stateMachine.get_state() is None
        ):  # ? state machine hasn't started yet - word probably came from input arguments
            self.wordReceived = message.data
            self.ros_node.get_logger.info(
                f"[ROSCbController][onWordReceived] Received word: {self.wordReceived}"
            )
        else:
            self.wordReceived = None  # ? ignore

    def onClearScreenReceived(self, message):
        """
        Callback when receiving a signal to clear screen
        """
        self.ros_node.get_logger.info(
            "[ROSCbController][onClearScreenReceived] Clearing display"
        )
        clear_all_shapes = self.ros_node.create_client(
            ClearAllShapes, "clear_all_shapes"
        )
        while not clear_all_shapes.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().info(
                "service not available, waiting again..."
            )
        future = clear_all_shapes.call_async()
        rclpy.spin_until_future_complete(self.ros_node, future)

    def onStopRequestReceived(self, message):
        self.stopRequestReceived = True

    def onFeedbackReceived(self, message):
        if (
            self.stateMachine.get_state() == "ASKING_FOR_FEEDBACK"
            or self.stateMachine.get_state() == "WAITING_FOR_FEEDBACK"
            or self.stateMachine.get_state() == "WAITING_FOR_LETTER_TO_FINISH"
        ):
            self.feedbackReceived = (
                message  # ? replace any existing feedback with new
            )
            self.ros_node.get_logger.info(
                "[onFeedbackReceived] Received feedback"
            )
        elif self.stateMachine.get_state() == "RESPONDING_TO_FEEDBACK":
            self.feedbackReceived = None  # ? ignore feedback

    def onTestRequestReceived(self, message):
        # TODO don't respond to test card unless something has been learnt
        self.testRequestReceived = True

    def onShapeFinished(self, message):
        self.shapeFinished = True  # @TODO only register when appropriate

    def onUserDrawnShapeReceived(self, shape):
        """
        The main task here is to identify the letter(s) we got demos for
        """
        if (
            self.stateMachine.get_state() == "WAITING_FOR_FEEDBACK"
            or self.stateMachine.get_state() == "ASKING_FOR_FEEDBACK"
        ):
            # rospy.loginfo(f'[ROSCbController][onUserDrawnShapeReceived] shape = {shape} | len(self.demoShapesReceived) = {len(self.demoShapesReceived)}')

            nbpts = int(len(shape.path) / 2)
            _path = list(
                zip(
                    [x for x in shape.path[:nbpts]],
                    [-y for y in shape.path[nbpts:]],
                )
            )

            demo_from_template = self.screenManager.split_path_from_template(
                _path
            )
            # rospy.loginfo(f'[ROSCbController][onUserDrawnShapeReceived] demo_from_template = {demo_from_template}')

            if demo_from_template is not None:
                self.ros_node.get_logger.info(
                    f"[ROSCbController][onUserDrawnShapeReceived] Received template demonstration for letters {demo_from_template.keys()}"
                )

                for name, path in demo_from_template.items():
                    # HACK: do not learn multi-stroke letters for now
                    if name in ["i", "j", "t"]:
                        self.ros_node.get_logger.warn(
                            f"[ROSCbController][onUserDrawnShapeReceived] Received demonstration for multi-stroke letter <{name}>: ignoring it."
                        )
                        continue

                    flatpath = [x for x, y in path]
                    flatpath.extend([-y for x, y in path])

                    self.demoShapesReceived.append(
                        ShapeMsg(path=flatpath, shapeType=name)
                    )

            # else: #? dont know what this does ?
            #     rospy.loginfo(f'[ROSCbController][onUserDrawnShapeReceived][!] No demo_from_template{demo_from_template}')

            #     if self.activeLetter:
            #         shape.shapeType = self.activeLetter
            #         self.activeLetter = None
            #         rospy.loginfo(f'[ROSCbController][onUserDrawnShapeReceived] Received demonstration for selected letter {shape.shapeType}')
            #     else:
            #         letter, bb = self.screenManager.find_letter(shape.path)

            #         if letter:
            #             shape.shapeType = letter
            #             # self.pub_bounding_boxes.publish(self.shapeHelper.make_bounding_box_msg(bb, selected=True))
            #             rospy.loginfo(f'[ROSCbController][onUserDrawnShapeReceived] Received demonstration for {shape.shapeType}')
            #         else:
            #             rospy.logwarn(f'[ROSCbController][onUserDrawnShapeReceived] Received demonstration, but unable to find the letter that was demonstrated! Ignoring it.')
            #             return

            #     self.demoShapesReceived = [shape] #? replace any existing feedback with new

            # rospy.loginfo(f'[ROSCbController][onUserDrawnShapeReceived] self.demoShapesReceived = {self.demoShapesReceived}')

        else:
            pass  # ? ignore feedback

    def onSetActiveShapeGesture(self, message):
        self.activeLetter, bb = self.screenManager.closest_letter(
            message.point.x, message.point.y, strict=True
        )

        # if self.activeLetter:
        #    self.pub_bounding_boxes.publish(self.shapeHelper.make_bounding_box_msg(bb, selected=True))

    def onToSpeak(self, message):
        """
        Robot replies to something child has said
        """
        if self.robotCtl is not None:
            self.robotCtl.say(message, self.personSide)

    def onToAct(self, message):
        """
        Robot perform actions
        """
        if self.robotCtl is not None:
            if message.action_type == "speak":
                self.robotCtl.say(message.data, self.personSide)
                self.pub_listening_signal.publish(
                    "convo"
                )  # ? after finish speaking, tell the diagram_manager to capture audio again
            elif message.action_type == "long_word":
                self.robotCtl.say(message.data, self.personSide)
                # rospy.sleep(10)
                self.pub_listening_signal.publish(
                    "word"
                )  # ? keep waiting for word
            else:
                self.pub_listening_signal.publish(
                    "convo"
                )  # ? after receiving, tell the diagram_manager to capture audio again
