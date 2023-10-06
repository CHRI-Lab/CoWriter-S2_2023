from copy import deepcopy
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Empty

from bluering_letter_learning.include.interaction_settings import (
    InteractionSettings,
)
from bluering_letter_learning.include.controllers.ros_callback_controller import (
    ROSCbController,
)


class StateController(ROSCbController):
    """
    All callbacks for when stateMachine changes
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
        drawingLetterSubstates,
        pub_camera_status,
        pub_bounding_boxes,
        pub_clear,
        tabletWatchdog,
        pub_traj,
        pub_traj_downsampled,
        pub_listening_signal,
    ) -> None:
        super().__init__(
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
        )

        self.ros_node.get_logger().info("[StateController] init called")

        self.infoToRestore_waitForRobotToConnect = None
        self.infoToRestore_waitForTabletToConnect = None
        self.infoToRestore_waitForShapeToFinish = None

        self.drawingLetterSubstates = drawingLetterSubstates
        # ? initialise arrays of phrases to say at relevant times
        (
            introPhrase,
            self.demo_response_phrases,
            self.asking_phrases_after_feedback,
            self.asking_phrases_after_word,
            self.word_response_phrases,
            self.word_again_response_phrases,
            self.testPhrase,
            self.thankYouPhrase,
        ) = InteractionSettings.getPhrases(language)

        self.tabletWatchdog = tabletWatchdog

        self.activeLetter = None

        self.asking_phrases_after_word_counter = 0
        self.asking_phrases_after_feedback_counter = 0
        self.demo_response_phrases_counter = 0
        self.word_response_phrases_counter = 0
        self.word_again_response_phrases_counter = 0

        return

    # NOTE THAT THIS WAS FOR TOUCH-BASED FEEDBACK, WHICH ISN'T USED ANYMORE
    def respondToFeedback(self, infoFromPrevState):
        self.ros_node.get_logger().info(
            "[StateController][respondToFeedback] STATE: RESPONDING_TO_FEEDBACK"
        )

        stringReceived = infoFromPrevState["feedbackReceived"]

        nextState = "WAITING_FOR_FEEDBACK"
        infoForNextState = {"state_cameFrom": "ASKING_FOR_FEEDBACK"}

        # ? convert feedback string into settings
        feedback = stringReceived.data.split("_")
        processMessage = True
        try:
            shapeIndex_messageFor = int(feedback[0])
        except:
            self.ros_node.get_logger().error(
                f"[respondToFeedback] Shape type index must be an integer. Received : {feedback[0]}"
            )
            processMessage = False

        try:
            bestShape_index = int(feedback[1])
        except:
            self.ros_node.get_logger().error(
                f"[respondToFeedback] Best shape index must be an integer. Received : {feedback[0]}"
            )
            processMessage = False

        noNewShape = False  # ? usually make a new shape based on feedback
        if len(feedback) > 2:
            feedbackMessage = feedback[2]
            if feedbackMessage == "noNewShape":
                noNewShape = True
            else:
                processMessage = False
                self.ros_node.get_logger().error(
                    f"[respondToFeedback] Unknown message received in feedback string: {feedbackMessage}"
                )

        if processMessage:
            if noNewShape:  # ? just respond to feedback, don't make new shape
                if self.robotCtl is not None and self.naoSpeaking:
                    self.ros_node.get_logger().info(
                        f"[StateController][respondToFeedback] toSay: {toSay}"
                    )
                    toSay = "Ok, thanks for helping me"
                    self.robotCtl.speak(toSay)

                # ? pass feedback to shape manager
                response = self.wordManager.feedbackManager(
                    shapeIndex_messageFor, bestShape_index, noNewShape
                )
                if response == -1:
                    self.ros_node.get_logger().error(
                        "[respondToFeedback] Something's gone wrong in the feedback manager"
                    )

            else:
                if self.naoSpeaking:
                    self.ros_node.get_logger().info(
                        f"[StateController][respondToFeedback] NAO: {toSay}"
                    )
                    shape_messageFor = (
                        self.wordManager.shapeAtIndexInCurrentCollection(
                            shapeIndex_messageFor
                        )
                    )
                    toSay = f"Ok, I'll work on the {shape_messageFor}"
                    self.robotCtl.speak(toSay)

                [
                    numItersConverged,
                    newShape,
                ] = self.wordManager.feedbackManager(
                    shapeIndex_messageFor, bestShape_index, noNewShape
                )

                if numItersConverged == 0:
                    state_goTo = deepcopy(self.drawingLetterSubstates)
                    nextState = state_goTo.pop(0)
                    infoForNextState = {
                        "state_goTo": state_goTo,
                        "state_cameFrom": "RESPONDING_TO_FEEDBACK",
                        "shapesToPublish": [newShape],
                    }
                else:
                    pass  # TODO handle convergence

        if self.wordReceived is not None:
            infoForNextState["wordReceived"] = self.wordReceived
            self.wordReceived = None
            nextState = "RESPONDING_TO_NEW_WORD"
        if self.testRequestReceived:
            self.testRequestReceived = False
            nextState = "RESPONDING_TO_TEST_CARD"
        if self.stopRequestReceived:
            nextState = "STOPPING"
        return nextState, infoForNextState

    def startInteraction(self, infoFromPrevState):
        self.ros_node.get_logger().info(
            "[StateController][startInteraction] STATE: STARTING_INTERACTION"
        )
        # if self.naoSpeaking:
        #     if alternateSidesLookingAt:
        #         self.robotCtl.lookAndAskForFeedback(introPhrase, nextSideToLookAt)
        #     else:
        #         self.robotCtl.lookAndAskForFeedback(introPhrase, self.personSide)
        self.robotCtl.sayHello(self.personSide)

        nextState = "WAITING_FOR_CHILD"
        infoForNextState = {"state_cameFrom": "STARTING_INTERACTION"}
        if self.stopRequestReceived:
            nextState = "STOPPING"
        return nextState, infoForNextState

    def waitForChild(self, infoFromPrevState):
        if infoFromPrevState["state_cameFrom"] != "WAITING_FOR_CHILD":
            self.ros_node.get_logger().info(
                "[StateController][waitForChild] STATE: WAITING_FOR_CHILD"
            )
            self.pub_camera_status.publish(True)  # ? turn camera on
        if infoFromPrevState["state_cameFrom"] == "STARTING_INTERACTION":
            self.ros_node.get_clock().sleep_for(Duration(seconds=1))
            self.robotCtl.say(
                "Okay Minh. Please log or register a child so we can start new session",
                self.personSide,
            )
            self.pub_listening_signal.publish(
                "convo"
            )  # ? capturing audio input to listen for word to write
            pass
        elif infoFromPrevState["state_cameFrom"] != "WAITING_FOR_CHILD":
            self.pub_listening_signal.publish(
                "convo"
            )  # ? capturing audio input to listen for word to write

        if self.childReceived is None:
            nextState = "WAITING_FOR_CHILD"
            infoForNextState = {"state_cameFrom": "WAITING_FOR_CHILD"}
            self.ros_node.get_clock().sleep_for(
                Duration(seconde=0.1)
            )  # ? don't check again immediately
        else:
            nextState = "WAITING_FOR_WORD"
            infoForNextState = {"state_cameFrom": "STARTING_INTERACTION"}
            if self.stopRequestReceived:
                nextState = "STOPPING"

        return nextState, infoForNextState

    def waitForRobotToConnect(self, infoFromPrevState):
        # ? FORWARDER STATE
        if (
            infoFromPrevState["state_cameFrom"]
            != "WAITING_FOR_ROBOT_TO_CONNECT"
        ):
            self.ros_node.get_logger().info(
                "[StateController][waitForRobotToConnect] STATE: WAITING_FOR_ROBOT_TO_CONNECT"
            )
            self.infoToRestore_waitForRobotToConnect = infoFromPrevState

        nextState = "WAITING_FOR_ROBOT_TO_CONNECT"
        infoForNextState = {"state_cameFrom": "WAITING_FOR_ROBOT_TO_CONNECT"}

        # if robotWatchdog.isResponsive() or not naoConnected:
        if True:  # ? don't use watchdog for now
            infoForNextState = self.infoToRestore_waitForRobotToConnect
            nextState = infoForNextState["state_goTo"].pop(0)
        else:
            self.ros_node.get_clock().sleep_for(
                Duration(seconds=0.1)
            )  # ? don't check again immediately

        if self.stopRequestReceived:
            nextState = "STOPPING"
        return nextState, infoForNextState

    def waitForShapeToFinish(self, infoFromPrevState):
        # ? FORWARDER STATE
        # ? first time into this state preparations
        if (
            infoFromPrevState["state_cameFrom"]
            != "WAITING_FOR_LETTER_TO_FINISH"
        ):
            self.ros_node.get_logger().info(
                "[StateController][waitForShapeToFinish] STATE: WAITING_FOR_LETTER_TO_FINISH"
            )
            self.infoToRestore_waitForShapeToFinish = infoFromPrevState

        infoForNextState = {"state_cameFrom": "WAITING_FOR_LETTER_TO_FINISH"}
        nextState = None

        # rospy.loginfo(f'[StateController][waitForShapeToFinish] self.shapeFinished = {self.shapeFinished}')

        # ? once shape has finished
        if self.shapeFinished:
            # ? draw the templates for the demonstrations
            ref_boundingboxes = (
                self.screenManager.place_reference_boundingboxes(
                    self.wordManager.currentCollection
                )
            )
            for bb in ref_boundingboxes:
                boxesToPub = self.shapeHelper.make_bounding_box_msg(
                    bb, selected=False
                )
                self.ros_node.get_logger().info(
                    f"[StateController][waitForShapeToFinish] called make_bounding_box_msg to calc boxesToPub = {len(bb)}"
                )
                self.pub_bounding_boxes.publish(boxesToPub)
                self.ros_node.get_clock().sleep_for(
                    Duration(secondes=0.2)
                )  # ? leave some time for the tablet to process the bbs

            self.shapeFinished = False

            infoForNextState = self.infoToRestore_waitForShapeToFinish
            self.ros_node.get_logger().info(
                f"[StateController][waitForShapeToFinish] infoForNextState = {infoForNextState}"
            )
            try:
                if (
                    infoForNextState["state_goTo"] is not None
                    and len(infoForNextState["state_goTo"]) > 0
                ):
                    nextState = infoForNextState["state_goTo"].pop(
                        0
                    )  # ? go to the next state requested to and remove it from the list
                    # TODO make sure it actually gets executed before popping it...
            except:
                # ? nothing planned..
                nextState = "WAITING_FOR_FEEDBACK"

        if self.stopRequestReceived:
            nextState = "STOPPING"

        if nextState is None:
            # ? default behaviour is to keep waiting
            self.ros_node.get_clock().sleep_for(
                Duration(seconds=0.1)
            )  # ? don't check straight away
            nextState = "WAITING_FOR_LETTER_TO_FINISH"
            infoForNextState = {
                "state_cameFrom": "WAITING_FOR_LETTER_TO_FINISH"
            }

        return nextState, infoForNextState

    def respondToNewWord(self, infoFromPrevState):
        self.ros_node.get_logger().info(
            "[StateController][respondToNewWord] STATE: RESPONDING_TO_NEW_WORD"
        )
        wordToLearn = infoFromPrevState["wordReceived"]
        wordSeenBefore = self.wordManager.newCollection(wordToLearn)
        if self.naoSpeaking:
            if wordSeenBefore:
                try:
                    toSay = (
                        self.word_again_response_phrases[
                            self.word_again_response_phrases_counter
                        ]
                        % wordToLearn
                    )
                except TypeError:  # ? string wasn't meant to be formatted
                    toSay = self.word_again_response_phrases[
                        self.word_again_response_phrases_counter
                    ]
                self.word_again_response_phrases_counter += 1
                if self.word_again_response_phrases_counter == len(
                    self.word_again_response_phrases
                ):
                    self.word_again_response_phrases_counter = 0

            else:
                try:
                    toSay = (
                        self.word_response_phrases[
                            self.word_response_phrases_counter
                        ]
                        % wordToLearn
                    )
                except TypeError:  # ? string wasn't meant to be formatted
                    toSay = self.word_response_phrases[
                        self.word_response_phrases_counter
                    ]
                self.word_response_phrases_counter += 1
                if self.word_response_phrases_counter == len(
                    self.word_response_phrases
                ):
                    self.word_response_phrases_counter = 0

            self.ros_node.get_logger().info(
                f"[StateController][respondToNewWord] NAO: {toSay}"
            )
            self.robotCtl.speak(toSay)

        # ? word received. done speaking. now capturing audio input to maintain conversation
        self.pub_listening_signal.publish("convo")

        # ? clear screen
        self.screenManager.clear()
        self.pub_clear.publish(Empty())
        self.ros_node.get_clock().sleep_for(Duration(seconds=0.5))

        # ? start learning
        shapesToPublish = []
        for i in range(len(wordToLearn)):
            shape = self.wordManager.startNextShapeLearner()
            shapesToPublish.append(shape)

        nextState = "PUBLISHING_WORD"
        infoForNextState = {
            "state_cameFrom": "RESPONDING_TO_NEW_WORD",
            "shapesToPublish": shapesToPublish,
            "wordToWrite": wordToLearn,
        }

        if self.wordReceived is not None:
            infoForNextState["wordReceived"] = self.wordReceived
            self.wordReceived = None
            nextState = "RESPONDING_TO_NEW_WORD"
        if self.testRequestReceived:
            self.testRequestReceived = False
            nextState = "RESPONDING_TO_TEST_CARD"
        if self.stopRequestReceived:
            nextState = "STOPPING"
        return nextState, infoForNextState

    def askForFeedback(self, infoFromPrevState):
        self.ros_node.get_logger().info(
            "[StateController][askForFeedback] STATE: ASKING_FOR_FEEDBACK"
        )
        centre = infoFromPrevState["centre"]
        self.ros_node.get_logger().info(
            "[StateController][askForFeedback] state_camefrom = "
            + infoFromPrevState["state_cameFrom"]
        )

        if infoFromPrevState["state_cameFrom"] == "PUBLISHING_WORD":
            wordWritten = infoFromPrevState["wordWritten"]
            self.ros_node.get_logger().info(
                f"[StateController][askForFeedback] Asking for feedback on word {wordWritten}"
            )
            if self.naoSpeaking:
                try:
                    toSay = (
                        self.asking_phrases_after_word[
                            self.asking_phrases_after_word_counter
                        ]
                        % wordWritten
                    )
                except TypeError:  # ? string wasn't meant to be formatted
                    toSay = self.asking_phrases_after_word[
                        self.asking_phrases_after_word_counter
                    ]
                self.asking_phrases_after_word_counter += 1
                if self.asking_phrases_after_word_counter == len(
                    self.asking_phrases_after_word
                ):
                    self.asking_phrases_after_word_counter = 0

                if self.alternateSidesLookingAt:
                    self.robotCtl.lookAndAskForFeedback(
                        toSay, self.nextSideToLookAt
                    )
                    if self.nextSideToLookAt == "Left":
                        self.nextSideToLookAt = "Right"
                    else:
                        self.nextSideToLookAt = "Left"
                else:
                    self.robotCtl.lookAndAskForFeedback(toSay, self.personSide)

                self.robotCtl.lookAtTablet()

        elif infoFromPrevState["state_cameFrom"] == "PUBLISHING_LETTER":
            shapeType = infoFromPrevState["shapePublished"]
            self.ros_node.get_logger().info(
                f"[StateController][askForFeedback] Asking for feedback on letter {shapeType}"
            )

            if self.naoSpeaking:
                try:
                    toSay = (
                        self.asking_phrases_after_feedback[
                            self.asking_phrases_after_feedback_counter
                        ]
                        % shapeType
                    )
                except TypeError:  # ? string wasn't meant to be formatted
                    toSay = self.asking_phrases_after_feedback[
                        self.asking_phrases_after_feedback_counter
                    ]
                self.asking_phrases_after_feedback_counter += 1
                if self.asking_phrases_after_feedback_counter == len(
                    self.asking_phrases_after_feedback
                ):
                    self.asking_phrases_after_feedback_counter = 0

                if self.alternateSidesLookingAt:
                    self.robotCtl.lookAndAskForFeedback(
                        toSay, self.nextSideToLookAt
                    )
                    if self.nextSideToLookAt == "Left":
                        self.nextSideToLookAt = "Right"
                    else:
                        self.nextSideToLookAt = "Left"
                else:
                    self.robotCtl.lookAndAskForFeedback(toSay, self.personSide)

                # self.robotCtl.lookAtTablet()

        nextState = "WAITING_FOR_FEEDBACK"
        infoForNextState = {"state_cameFrom": "ASKING_FOR_FEEDBACK"}

        if self.wordReceived is not None:
            infoForNextState["wordReceived"] = self.wordReceived
            self.wordReceived = None
            nextState = "RESPONDING_TO_NEW_WORD"
        if self.wordReceived is not None:
            self.testRequestReceived = False
            nextState = "RESPONDING_TO_TEST_CARD"
        if self.stopRequestReceived:
            nextState = "STOPPING"
        return nextState, infoForNextState

    def respondToTestCard(self, infoFromPrevState):
        self.ros_node.get_logger().info(
            f"[StateController][respondToTestCard] STATE: RESPONDING_TO_TEST_CARD"
        )
        if self.naoSpeaking:
            self.robotCtl.speak(self.testPhrase)
            self.ros_node.get_logger().info(
                f"[StateController]NAO: {self.testPhrase}"
            )
        nextState = "WAITING_FOR_WORD"
        infoForNextState = {"state_cameFrom": "RESPONDING_TO_TEST_CARD"}
        return nextState, infoForNextState

    def stopInteraction(self, infoFromPrevState):
        self.ros_node.get_logger().info(
            "[StateController][stopInteraction] STATE: STOPPING"
        )
        if self.naoSpeaking:
            self.robotCtl.speak(self.thankYouPhrase)
        if self.naoConnected:
            self.robotCtl.motionProxy.wbEnableEffectorControl(
                self.effector, False
            )
            self.robotCtl.motionProxy.rest()
        nextState = "EXIT"
        infoForNextState = 0
        rclpy.shutdown()
        return nextState, infoForNextState

    def waitForWord(self, infoFromPrevState):
        if infoFromPrevState["state_cameFrom"] != "WAITING_FOR_WORD":
            self.ros_node.get_logger().info(
                "[StateController][waitForWord] STATE: WAITING_FOR_WORD"
            )
            self.pub_camera_status.publish(True)  # ? turn camera on
            # self.pub_listening_signal.publish('word') #? capturing audio input to listen for word to write
        if infoFromPrevState["state_cameFrom"] == "STARTING_INTERACTION":
            pass

        infoForNextState = {"state_cameFrom": "WAITING_FOR_WORD"}

        if self.wordReceived is None:
            nextState = "WAITING_FOR_WORD"
            self.ros_node.get_clock().sleep_for(
                Duration(seconds=0.1)
            )  # ? don't check again immediately
        else:
            infoForNextState["wordReceived"] = self.wordReceived
            self.wordReceived = None
            nextState = "RESPONDING_TO_NEW_WORD"
            self.pub_camera_status.publish(False)  # ? turn camera off

        if self.stopRequestReceived:
            nextState = "STOPPING"
            self.pub_camera_status.publish(False)  # ? turn camera off

        return nextState, infoForNextState

    def waitForFeedback(self, infoFromPrevState):
        if infoFromPrevState["state_cameFrom"] != "WAITING_FOR_FEEDBACK":
            self.ros_node.get_logger().info(
                "[StateController][waitForFeedback] STATE: WAITING_FOR_FEEDBACK"
            )
            self.pub_camera_status.publish(True)  # ? turn camera on

        # rospy.loginfo(f'[StateController][waitForFeedback] self.demoShapesReceived = {self.demoShapesReceived}')

        infoForNextState = {"state_cameFrom": "WAITING_FOR_FEEDBACK"}
        nextState = None

        if self.feedbackReceived is not None:
            infoForNextState["feedbackReceived"] = self.feedbackReceived
            self.feedbackReceived = None
            nextState = "RESPONDING_TO_FEEDBACK"
            infoForNextState["state_goTo"] = [nextState]
            nextState = "WAITING_FOR_ROBOT_TO_CONNECT"

        if self.demoShapesReceived and len(self.demoShapesReceived) > 0:
            infoForNextState["demoShapesReceived"] = self.demoShapesReceived
            # self.demoShapesReceived = [] #? this doesn't help
            nextState = "RESPONDING_TO_DEMONSTRATION_FULL_WORD"
            infoForNextState["state_goTo"] = [
                nextState
            ]  # ? ensure robot is connected before going to that state
            nextState = "WAITING_FOR_ROBOT_TO_CONNECT"

        if self.wordReceived is not None:
            infoForNextState["wordReceived"] = self.wordReceived
            self.wordReceived = None
            nextState = "RESPONDING_TO_NEW_WORD"
            infoForNextState["state_goTo"] = [
                nextState
            ]  # ? ensure robot is connected before going to that state
            nextState = "WAITING_FOR_ROBOT_TO_CONNECT"

        if self.testRequestReceived:
            self.testRequestReceived = False
            nextState = "RESPONDING_TO_TEST_CARD"
            infoForNextState["state_goTo"] = [
                nextState
            ]  # ? ensure robot is connected before going to that state
            nextState = "WAITING_FOR_ROBOT_TO_CONNECT"

        if self.stopRequestReceived:
            nextState = "STOPPING"

        if nextState != "WAITING_FOR_FEEDBACK":
            self.pub_camera_status.publish(False)  # ? turn camera off

        # ? debug
        if "state_goTo" in infoForNextState:
            state_goTo = infoForNextState["state_goTo"]
            self.ros_node.get_logger().info(
                f"[StateController][waitForFeedback] state_goTo = {state_goTo}"
            )

        if nextState is None:
            # ? default behaviour is to loop
            self.ros_node.get_clock().sleep_for(
                Duration(seconds=0.1)
            )  # ? don't check again immediately
            nextState = "WAITING_FOR_FEEDBACK"
            infoForNextState = {"state_cameFrom": "WAITING_FOR_FEEDBACK"}

        return nextState, infoForNextState

    def waitForTabletToConnect(self, infoFromPrevState):
        # ? FORWARDER STATE
        if (
            infoFromPrevState["state_cameFrom"]
            != "WAITING_FOR_TABLET_TO_CONNECT"
        ):
            # print('------------------------------------------ waiting_for_tablet_to_connect')
            self.ros_node.get_logger().info(
                "[StateController][waitForTabletToConnect] STATE: waiting_for_tablet_to_connect"
            )
            self.infoToRestore_waitForTabletToConnect = infoFromPrevState

        nextState = "WAITING_FOR_TABLET_TO_CONNECT"
        infoForNextState = {"state_cameFrom": "WAITING_FOR_TABLET_TO_CONNECT"}

        if (
            self.tabletWatchdog.isResponsive()
        ):  # ? reconnection - send message to wherever it was going
            infoForNextState = self.infoToRestore_waitForTabletToConnect
            nextState = infoForNextState["state_goTo"].pop(0)
        else:
            self.ros_node.get_clock().sleep_for(
                Duration(seconds=0.1)
            )  # ? don't check again immediately

        if self.stopRequestReceived:
            nextState = "STOPPING"
        return nextState, infoForNextState

    def publishShape(self, infoFromPrevState):
        # TODO: publishShape is currently broken. Needs to be updated to use the
        raise RuntimeError("publish shape is currently broken!!")

    def writeWord(self, infoFromPrevState):
        """
        Control the robot to follow trajectory of a word
        """
        self.ros_node.get_logger().info(
            "[StateController][writeWord] STATE: PUBLISHING_WORD"
        )
        self.ros_node.get_logger().info(
            f"[StateController][writeWord] len(self.demoShapesReceived) = {len(self.demoShapesReceived)}"
        )

        shapedWord = self.textShaper.shapeWord(self.wordManager)
        placedWord = self.screenManager.place_word(shapedWord)

        # rospy.loginfo(f'[StateController][writeWord] shapedWord : {shapedWord}')
        # rospy.loginfo(f'[StateController][writeWord] placedWord : {placedWord}')

        # rospy.loginfo(f'[StateController][writeWord] Calling make_traj_msg on placedWord')
        traj = self.shapeHelper.make_traj_msg(
            placedWord, float(self.dt) / self.downSampleFactor, log=True
        )

        # ? downsampled the trajectory for the robot arm motion
        downsampledShapedWord = deepcopy(placedWord)
        # rospy.loginfo(f'[StateController][writeWord] downsampledShapedWord for {downsampledShapedWord} with downSampleFactor {self.downSampleFactor}')
        downsampledShapedWord.downsample(self.downSampleFactor)

        # rospy.loginfo(f'[StateController][writeWord] Calling make_traj_msg on downsampledShapedWord')
        downsampledTraj = self.shapeHelper.make_traj_msg(
            downsampledShapedWord, self.dt, log=True
        )

        trajStartPosition = traj.poses[0].pose.position

        if self.naoConnected:
            self.robotCtl.lookAtTablet()

        # ? tell the robot to write directly instead of publishing messages to ros
        self.pub_traj_downsampled.publish(
            downsampledTraj
        )  # ? for the robot to write
        self.pub_traj.publish(traj)  # ? to visualize trajectory
        # rospy.loginfo(f'[StateController][writeWord] downsampledTraj: {downsampledTraj}')
        self.robotCtl.followTrajectory(
            downsampledTraj
        )  # ? control the robot to write
        # TODO: Visualize the trajectory

        nextState = "WAITING_FOR_LETTER_TO_FINISH"
        infoForNextState = {
            "state_cameFrom": "PUBLISHING_WORD",
            "state_goTo": ["ASKING_FOR_FEEDBACK"],
            "centre": trajStartPosition,
            "wordWritten": infoFromPrevState["wordToWrite"],
        }

        return nextState, infoForNextState

    # def respondToDemonstration(self, infoFromPrevState):
    #     rospy.loginfo(f'[StateController][respondToDemonstration] STATE: RESPONDING_TO_DEMONSTRATION')
    #     self.demoShapesReceived = infoFromPrevState['demoShapesReceived']

    #     #? update the shape models with the incoming demos
    #     new_shapes = []

    #     letters = ''.join([s.shapeType for s in self.demoShapesReceived])

    #     if self.naoSpeaking:
    #         try:
    #             toSay = self.demo_response_phrases[self.demo_response_phrases_counter] % letters
    #         except TypeError: #? string wasn't meant to be formatted
    #             toSay = self.demo_response_phrases[self.demo_response_phrases_counter]
    #         self.demo_response_phrases_counter += 1
    #         if self.demo_response_phrases_counter == len(self.demo_response_phrases):
    #             self.demo_response_phrases_counter = 0
    #         self.robotCtl.speak(toSay)
    #         rospy.loginfo(f'[StateController][respondToDemonstration] NAO: {toSay}')

    #     rospy.loginfo(f'[StateController][respondToDemonstration] self.demoShapesReceived = {self.demoShapesReceived}')
    #     for shape in self.demoShapesReceived:
    #         rospy.loginfo(f'[StateController][respondToDemonstration] shape = {shape}')
    #         rospy.loginfo(f'[StateController][respondToDemonstration] shape.path = {shape.path}')
    #         glyph = shape.path
    #         shapeName = shape.shapeType

    #         glyph = self.shapeHelper.downsampleShape(glyph)

    #         rospy.loginfo(f'[StateController] Received demo for ' + shapeName)
    #         shapeIndex = self.wordManager.currentCollection.index(shapeName)
    #         shape = self.wordManager.respondToDemonstration(shapeIndex, glyph)

    #         new_shapes.append(shape)

    #     state_goTo = deepcopy(self.drawingLetterSubstates)
    #     nextState = state_goTo.pop(0)
    #     infoForNextState = {'state_goTo': state_goTo, 'state_cameFrom': 'RESPONDING_TO_DEMONSTRATION','shapesToPublish': new_shapes}
    #     return nextState, infoForNextState

    def respondToDemonstrationWithFullWord(self, infoFromPrevState):
        self.ros_node.get_logger().info(
            "[StateController][respondToDemonstrationWithFullWord] STATE: RESPONDING_TO_DEMONSTRATION_FULL_WORD"
        )
        # rospy.loginfo(f'[StateController][respondToDemonstrationWithFullWord] infoFromPrevState = {infoFromPrevState}')
        self.demoShapesReceived = infoFromPrevState["demoShapesReceived"]

        letters = "".join([s.shapeType for s in self.demoShapesReceived])

        if self.naoSpeaking:
            try:
                toSay = (
                    self.demo_response_phrases[
                        self.demo_response_phrases_counter
                    ]
                    % letters
                )
            except TypeError:  # ? string wasn't meant to be formatted
                toSay = self.demo_response_phrases[
                    self.demo_response_phrases_counter
                ]
            self.demo_response_phrases_counter += 1
            if self.demo_response_phrases_counter == len(
                self.demo_response_phrases
            ):
                self.demo_response_phrases_counter = 0
            self.robotCtl.speak(toSay)
            self.ros_node.get_logger().info(
                f"[StateController][respondToDemonstrationWithFullWord] NAO: {toSay}"
            )

        """ 1- update the shape models with the incoming demos """
        for shape in self.demoShapesReceived:
            glyph = shape.path
            shapeName = shape.shapeType

            self.ros_node.get_logger().debug(
                f"[StateController][respondToDemonstrationWithFullWord] Downsampling %s..."
                % shapeName
            )
            glyph = self.shapeHelper.downsampleShape(glyph)
            self.ros_node.get_logger().info(
                f"[StateController][respondToDemonstrationWithFullWord] Downsampling of %s done. Demo received for %s"
                % (shapeName, shapeName)
            )
            self.ros_node.get_logger().info(
                f"[StateController][respondToDemonstrationWithFullWord] self.wordManager.currentCollection: {self.wordManager.currentCollection} | shapeName : {shapeName} | self.wordManager.shapesLearnt : {self.wordManager.shapesLearnt}"
            )
            shapeIndex = None
            shapeIndex = self.wordManager.currentCollection.index(shapeName)
            self.ros_node.get_logger().info(
                f"[StateController][respondToDemonstrationWithFullWord] >>>  shapeIndex: {shapeIndex} "
            )
            self.wordManager.respondToDemonstration(shapeIndex, glyph)

        """ 2- display the update word """
        # ? clear screen
        self.screenManager.clear()
        self.pub_clear.publish(Empty())
        self.ros_node.get_clock().sleep_for(Duration(secondes=0.5))

        shapesToPublish = self.wordManager.shapesOfCurrentCollection()

        # ? clear feedback shape
        self.demoShapesReceived = []
        self.ros_node.get_logger().info(
            f"[StateController][respondToDemonstrationWithFullWord] self.demoShapesReceived reset! self.demoShapesReceived = {self.demoShapesReceived}"
        )

        nextState = "PUBLISHING_WORD"
        infoForNextState = {
            "state_cameFrom": "RESPONDING_TO_DEMONSTRATION_FULL_WORD",
            "shapesToPublish": shapesToPublish,
            "wordToWrite": self.wordManager.currentCollection,
        }

        return nextState, infoForNextState
