from PyQt5 import uic, QtWidgets
import PyQt5.QtCore as QtCore
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtCore import QObject, QRect, Qt, QSize, QDate
from PyQt5.QtGui import QIcon
from datetime import datetime
from os.path import expanduser
from nav_msgs.msg import Path
from std_msgs.msg import (
    String,
    UInt8,
    Int32MultiArray,
    Float32,
    MultiArrayDimension,
    MultiArrayLayout,
    Empty,
)
from geometry_msgs.msg import PoseStamped
import numpy as np
import os

import sys

import rclpy
from rclpy.node import Node

from choose_adaptive_words.audio_processor import AudioProcessor
from choose_adaptive_words.ChildProfile import ChildProfile
from choose_adaptive_words.parameters import *

TOPIC_WORDS_TO_WRITE = "words_to_write"
TOPIC_MANAGER_ERASE = "manager_erase"
TOPIC_LEARNING_PACE = "simple_learning_pace"
TOPIC_GPT_INPUT = "chatgpt_input"
TOPIC_SHAPE_FINISHED = "shape_finished"


class Manager_UI(QtWidgets.QDialog, Node):
    def __init__(self):
        self.node = rclpy.create_node("manager_ui")
        self.choose_adaptive_words_path = os.path.dirname(
            os.path.dirname(os.path.realpath(__file__))
        )

        super(Manager_UI, self).__init__()
        uic.loadUi(
            self.choose_adaptive_words_path + "/design/manager_view.ui", self
        )
        self.show()
        # define QtWidgets
        self.buttonPredict.clicked.connect(self.buttonPredictClicked)
        self.buttonSendRobot.clicked.connect(self.buttonSendRobotClicked)
        self.buttonErase.clicked.connect(self.buttonEraseClicked)
        self.buttonProfile.clicked.connect(self.buttonProfileClicked)
        self.buttonPathDialog.clicked.connect(self.buttonPathDialogClicked)
        self.buttonWordToWrite.clicked.connect(self.buttonWordToWriteClicked)
        self.buttonGptText.clicked.connect(self.buttonGptTextClicked)
        self.buttonRobotFinished.clicked.connect(
            self.buttonRobotFinishedClicked
        )
        self.sliderLearningPace.sliderReleased.connect(
            self.sliderLearningPaceUpdated
        )
        self.buttonTalkToMe.clicked.connect(self.buttonTalkToMeCliked)
        self.buttonStop.clicked.connect(self.buttonStopCliked)

        self.labelLeariningPace.setText(str(self.sliderLearningPace.value()))
        self.ap = AudioProcessor("english", self.node)

        ## init publisher
        # self.publish_word_to_write = rospy.Publisher(TOPIC_WORDS_TO_WRITE, String, queue_size=10)
        self.publish_word_to_write = self.node.create_publisher(
            String, TOPIC_WORDS_TO_WRITE, 10
        )
        self.publish_simple_learning_pace = self.node.create_publisher(
            Float32, TOPIC_LEARNING_PACE, 10
        )
        self.publish_manager_erase = self.node.create_publisher(
            String, TOPIC_MANAGER_ERASE, 10
        )
        self.publish_chatgpt_input = self.node.create_publisher(
            String, TOPIC_GPT_INPUT, 10
        )
        self.publish_shape_finished = self.node.create_publisher(
            String, TOPIC_SHAPE_FINISHED, 10
        )
        self.transcription_publisher = self.node.create_publisher(
            String, "speech_rec", 10
        )
        self.stop_publisher = self.node.create_publisher(
            Empty, "stop_learning", 10
        )

        ## init path
        self.pathText.setText(PATH_DB)

    def buttonStopCliked(self):
        self.stop_publisher.publish()

    def buttonEraseClicked(self):
        self.publish_manager_erase.publish("erase")

    def buttonTalkToMeCliked(self):
        self.ap.run()
        self.transcription_publisher.publish(self.ap.transcription)

    def buttonWordToWriteClicked(self):
        print(
            "published " + self.wordText.text().lower() + " to /words_to_write"
        )
        self.publish_word_to_write.publish(self.wordText.text().lower())

    def buttonGptTextClicked(self):
        print("published " + self.gptText.text() + " to /chatgpt_input")
        self.publish_chatgpt_input.publish(self.gptText.text())

    def sliderLearningPaceUpdated(self):
        self.publish_simple_learning_pace.publish(
            np.uint8(self.sliderLearningPace.value() / 100)
        )
        self.labelLeariningPace.setText(str(self.sliderLearningPace.value()))

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
            self.pathText.setText(input_dir)

    def buttonRobotFinishedClicked(self):
        self.publish_shape_finished.publish("finish")


def main(args=None):
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    window = Manager_UI()
    window.show()
    rclpy.spin(window.node)
    rclpy.shutdown()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
