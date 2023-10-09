from os.path import expanduser
from std_msgs.msg import String, Float32, Empty

from rclpy.node import Node

from choose_adaptive_words.audio_processor import AudioProcessor
from choose_adaptive_words.parameters import *  # noqa: F403
from choose_adaptive_words.ai_image import AI_IMAGE


TOPIC_WORDS_TO_WRITE = "words_to_write"
TOPIC_MANAGER_ERASE = "manager_erase"
TOPIC_LEARNING_PACE = "simple_learning_pace"
TOPIC_GPT_INPUT = "chatgpt_input"
TOPIC_SHAPE_FINISHED = "shape_finished"


class ManagerUIBridge(Node):
    def __init__(self):
        super().__init__("manager_ui_bridge")
        self.ap = AudioProcessor("english", self)
        self.publish_word_to_write = self.create_publisher(
            String, TOPIC_WORDS_TO_WRITE, 10
        )
        self.publish_simple_learning_pace = self.create_publisher(
            Float32, TOPIC_LEARNING_PACE, 10
        )
        self.publish_manager_erase = self.create_publisher(
            String, TOPIC_MANAGER_ERASE, 10
        )
        self.publish_chatgpt_input = self.create_publisher(String, TOPIC_GPT_INPUT, 10)
        self.publish_shape_finished = self.create_publisher(
            String, TOPIC_SHAPE_FINISHED, 10
        )
        self.transcription_publisher = self.create_publisher(String, "speech_rec", 10)
        self.stop_publisher = self.create_publisher(Empty, "stop_learning", 10)
        self.ai_image = AI_IMAGE()

        self.gui.buttonStop.clicked.connect(self.buttonStopCliked)
        self.gui.buttonPredict.clicked.connect(self.buttonPredictClicked)
        self.gui.buttonSendRobot.clicked.connect(self.buttonSendRobotClicked)
        self.gui.buttonErase.clicked.connect(self.buttonEraseClicked)
        self.gui.buttonProfile.clicked.connect(self.buttonProfileClicked)
        self.gui.buttonPathDialog.clicked.connect(self.buttonPathDialogClicked)
        self.gui.buttonWordToWrite.clicked.connect(self.buttonWordToWriteClicked)
        self.gui.buttonGptText.clicked.connect(self.buttonGptTextClicked)
        self.gui.buttonRobotFinished.clicked.connect(self.buttonRobotFinishedClicked)
        self.gui.sliderLearningPace.sliderReleased.connect(
            self.sliderLearningPaceUpdated
        )
        self.gui.buttonTalkToMe.clicked.connect(self.buttonTalkToMeCliked)

    def buttonStopCliked(self):
        self.stop_publisher.publish(Empty())

    def buttonEraseClicked(self):
        self.get_logger().info("erasing child")
        self.publish_manager_erase.publish(String(data="erased"))

    def buttonTalkToMeCliked(self):
        self.ap.run()
        self.transcription_publisher.publish(String(data=self.ap.transcription))

    def buttonWordToWriteClicked(self):
        self.get_logger().info(
            "published " + self.gui.wordText.text().lower() + " to /words_to_write"
        )
        self.publish_word_to_write.publish(
            String(data=self.gui.wordText.text().lower())
        )
        self.ai_image.generate_image(self.gui.wordText.text().lower())

    def buttonGptTextClicked(self):
        self.get_logger().info(
            "published " + self.gui.gptText.text() + " to /chatgpt_input"
        )
        self.publish_chatgpt_input.publish(String(data=self.gui.gptText.text()))

    def sliderLearningPaceUpdated(self):
        self.publish_simple_learning_pace.publish(
            Float32(data=float(self.gui.sliderLearningPace.value() / 100))
        )
        self.gui.labelLeariningPace.setText(str(self.gui.sliderLearningPace.value()))

    def buttonProfileClicked(self):
        pass
        # self.activity.childProfile = ChildProfile(self.activity)
        # self.activity.childProfile.signal_profileCompleted.connect(self.callback_profileCompleted)

    def buttonPredictClicked(self):
        print("prediction currently disabled")
        # #get letters from boxes
        # trace = self.activity.tactileSurface.getData()
        # boxes = self.activity.tactileSurface.boxesToDraw
        # letters = self.activity.wt.separateWordsToLetters(trace, boxes, self.activity.tactileSurface.height(), self.activity.tactileSurface.convert_pix_meter)

        # #compute score of all letters
        # for index in letters:
        #     d_score = self.activity.predictor.predict(self.activity.childProfile.rightHanded,
        #     self.activity.childProfile.male,
        #     self.activity.childProfile.dateBirth.daysTo(QDate().currentDate())/30.5,
        #     self.activity.childProfile.section,
        #     letters[index],
        #     self.activity.lettersToWrite[index])

        #     self.activity.skills[self.activity.lettersToWrite[index]].dScore.append(d_score)

        # #save data
        # self.activity.saveData(letters)

        # #update knowledge about dico
        # self.activity.wt.updateWords(self.skills)

        # #choose next word
        # self.activity.algo()

        # #clear screen
        # self.activity.tactileSurface.eraseRobotTrace()
        # self.activity.tactileSurface.erasePixmap()

    def buttonSendRobotClicked(self):
        pass
        # data = self.activity.tactileSurface.data
        # boxesCoordinates = self.activity.tactileSurface.boxesToDraw

        # # create message containing path of word
        # words_drawn = Path()

        # for d in data:

        #     pose = PoseStamped()

        #     pose.pose.position.x = d.x*self.activity.tactileSurface.convert_pix_meter
        #     pose.pose.position.y = -d.y*self.activity.tactileSurface.convert_pix_meter + self.activity.tactileSurface.height()*self.activity.tactileSurface.convert_pix_meter# - boxesCoordinates[0][1]
        #     pose.header.seq = self.activity.seqWord
        #     words_drawn.poses.append(pose)

        #     self.activity.seqWord += 1

        # words_drawn.header.stamp = rospy.get_rostime()

        # # publish in topic
        # self.activity.publish_word_written.publish(words_drawn)

        # words_drawn = Path()
        # words_drawn.header.stamp = rospy.get_rostime()
        # self.activity.publish_word_written.publish(words_drawn)

        # # clear screen
        # self.activity.tactileSurface.eraseRobotTrace()
        # self.activity.tactileSurface.erasePixmap()

    def buttonPathDialogClicked(self):
        """
        select path for data base, write result in text widget
        """
        input_dir = QFileDialog.getExistingDirectory(
            None, "Select a folder:", expanduser("~")
        )
        if len(input_dir) > 0:
            self.gui.pathText.setText(input_dir)

    def buttonRobotFinishedClicked(self):
        self.publish_shape_finished.publish(String(data="finished"))
