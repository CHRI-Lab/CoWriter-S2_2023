from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QRect, Qt, QSize, QDate
from PyQt5.QtGui import QIcon
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import sys
import os
import math
from datetime import datetime
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import String, UInt8, Float64MultiArray
import numpy as np

from TactileSurfaceArea import TactileSurfaceArea
from ChildProfile import ChildProfile
from Predictor import Predictor
from WordTool import WordTool
from Glyph import Glyph
from parameters import *
from manager_view import Manager

yBeginningTactile = 100
TOPIC_WORDS_COMING = "write_traj"
TOPIC_WORDS_TO_WRITE = "words_to_write"
TOPIC_WORDS_WRITTEN = "user_drawn_shapes"
TOPIC_WORDS_WRITTEN_FINISHED = "shape_finished"
TOPIC_BOXES = "boxes_to_draw"

class Activity(QtWidgets.QDialog):

    def __init__(self):
        self.tactileSurface = None
        self.childProfile = None
        self.seqWord = 0
        # self.pathWriter = None
        self.pathWriter = '/home/ubuntu/NaoGPT'
        #self.predictor = Predictor()
        self.lettersToWrite = []
        self.iteration = 0
        self.skills = {}
        self.initSkills()
        self.wt = WordTool()

        super(Activity, self).__init__()
        uic.loadUi('../design/activity_words.ui', self)
        self.show()

        #? subscribe to topics
        rospy.Subscriber(TOPIC_WORDS_COMING, Path, self.callback_words_coming)
        rospy.Subscriber(TOPIC_BOXES, Float64MultiArray, self.callback_boxes)
        rospy.Subscriber(TOPIC_WORDS_TO_WRITE, String, self.callback_words_to_write)

        self.publish_word_written = rospy.Publisher(TOPIC_WORDS_WRITTEN, Path, queue_size=10)
        self.publish_word_written_finished = rospy.Publisher(TOPIC_WORDS_WRITTEN_FINISHED, String, queue_size=10)
        self.publish_word_to_write = rospy.Publisher(TOPIC_WORDS_TO_WRITE, String, queue_size=10)

        #? add tactile surface
        self.tactileSurface = TactileSurfaceArea(self)
        self.tactileSurface.setGeometry(QRect(0, yBeginningTactile, self.frameGeometry().width(), self.frameGeometry().height() - yBeginningTactile))
        self.tactileSurface.show()

        #? add slots
        self.buttonErase.clicked.connect(self.buttonEraseClicked)
        self.tactileSurface.signalRobotFinishWriting.connect(self.callback_RobotFinishWriting)
        #self.pathWriter = self.pathText.text


    def initSkills(self):
        for glyph in "abcdefghijklmnopqrstuvwxyz":
            self.skills[glyph] = Glyph(glyph)

    def resizeEvent(self, event):
        if self.tactileSurface != None:
            self.tactileSurface.setGeometry(QRect(0, yBeginningTactile, self.frameGeometry().width(), self.frameGeometry().height() - yBeginningTactile))

        self.buttonErase.move(self.width() - self.buttonErase.width() - 10, self.buttonErase.y())
        #self.buttonProfile.move(self.width() - 2*self.buttonProfile.width() - 2*10, self.buttonProfile.y())

    def buttonEraseClicked(self):
        self.tactileSurface.erasePixmap()

    def callback_words_coming(self, data):
        #? clear screen and reset boxes
        self.tactileSurface.eraseRobotTrace()
        self.tactileSurface.erasePixmap()
        self.tactileSurface.boxesToDraw = []
        #? draw word
        self.tactileSurface.drawWord(data)

    def callback_boxes(self, data):
        self.tactileSurface.boxesToDraw.append(data.data)

    def callback_RobotFinishWriting(self):
        self.publish_word_written_finished.publish("true")
        rospy.sleep(3.0)
        self.tactileSurface.drawBoxes()

    def callback_words_to_write(self, data):
        self.lettersToWrite = [d for d in data.data]



    def algo(self):

        cost = {}
        motivation = 0.8

        alpha = 5
        beta = 5
        gamma = 5

        low = 0.2

        #? compute cost of each words
        for word in self.wt.words:
            temp = alpha*(self.wt.words[word]["mastery"] - motivation)*(1 - math.exp(-0.5*self.iteration))
            temp += (beta/motivation)*(low + (1 - low)*math.exp(-0.5*self.iteration) - self.wt.words[word]["uncertainty"])
            temp += gamma*(1 - self.wt.words[word]["pertinence"])

            cost[word] = temp

        #? choose word with least cost
        min_val = min(cost.itervalues())
        newWord = [k for k, v in cost.iteritems() if v == min_val][0]
        print(newWord)

        #? update letters under investigation
        for l in newWord.lower():
            self.skills[l].iterationOccurence.append(self.iteration)
            self.skills[l].nbTime += 1

        #? publish word
        self.iteration += 1
        self.publish_word_to_write.publish(newWord.lower())


    def chooseNextWord(self):

        proba = []
        letters = []

        # assign proba to every letters
        for index in self.skills:
            mastery = self.skills[index].getMastery()
            uncertainty = self.skills[index].getUncertainty()
            progress = self.skills[index].getProgress()
            frequency = self.skills[index].nbTime/float(self.iteration + 1)

            p = P_EXPLOITATION*((1. - mastery) + (1. - uncertainty)) + P_EXPLORATION*(1. - frequency) + P_PROGRESS*(progress)

            print(self.skills[index].glyph, self.skills[index].dScore, mastery, uncertainty, progress, frequency, "--- ", p)


            # 2 letters in a word -> manage that
            if p < 0:
                p = 0

            proba.append(p)
            letters.append(self.skills[index].glyph)


        # normalize -- sum(proba) should be = 1
        proba = [p/sum(proba) for p in proba]
        for i in range(len(proba)):
            print(letters[i], proba[i])


        # choose word in function of these proba
        newWord = None
        while newWord == None:
            # choose letters that should compose the word
            letters = np.random.choice([l for l in self.skills], 3, p=proba, replace=False)

            # choose new word in child dictionnary
            let = ""
            for l in letters:
                let += l
            newWord = self.wt.findWordWithLetters(let)

            # no words found in child's dico, try adult one
            if newWord == None:
                print("check in adult dico")
                newWord = self.wt.findWordWithLetters(let, childMode=False)

            print("newWord: ", newWord, " containing letters: ", let)

        # update letters under investigation
        for l in newWord.lower():
            self.skills[l].iterationOccurence.append(self.iteration)
            self.skills[l].nbTime += 1

        # publish word
        self.iteration += 1
        self.publish_word_to_write.publish(newWord.lower())

    def saveData(self, letters):

        pathInfo = self.pathWriter + "/info.txt"

        if os.path.isdir(PATH_DB):
            if not os.path.isdir(self.pathWriter):
                os.mkdir(self.pathWriter)
            if not os.path.isfile(pathInfo):
                file = open(pathInfo, "w")

                file.write("firstName: " + self.childProfile.firstName)
                file.write("\nlastName: " + self.childProfile.lastName)
                file.write("\nisRightHanded: " + str(self.childProfile.rightHanded))
                file.write("\nisMale: " + str(self.childProfile.male))
                file.write("\nbirthday: " + str(self.childProfile.dateBirth.toString('dd_MM_yy')))
                file.write("\nsection: " + str(self.childProfile.section))
                file.write("\ntestDate: " + str(datetime.now()))

                file.close()

        else:
            print("DB path doesn't exists, change it in parameters.py")

        nbSameLetter = 0
        for i, letter in enumerate(self.lettersToWrite):
            pathLetter = self.pathWriter + "/" + str(letter) + "_" + str(self.iteration) + ".csv"

            if os.path.isfile(pathLetter):
                pathLetter = self.pathWriter + "/" + str(letter) + "_" + str(self.iteration) + "_" + str(nbSameLetter) + ".csv"
                nbSameLetter += 1

            file = open(pathLetter, "w")
            file.write("time,x,y,x_tilt,y_tilt,pressure\n")

            for rec in letters[i]:
                file.write(str(rec.time))
                file.write(str(","))
                file.write(str(rec.x))
                file.write(str(","))
                file.write(str(rec.y))
                file.write(str(","))
                file.write(str(rec.x_tilt))
                file.write(str(","))
                file.write(str(rec.y_tilt))
                file.write(str(","))
                file.write(str(rec.pressure))
                file.write(str("\n"))

            file.close()

    def closeEvent(self, event):

        pathResults = self.pathWriter + "/results"
        if not os.path.isdir(pathResults):
            os.mkdirs(pathResults)

        for glyph in self.skills:
            file = open(pathResults + "/" + glyph + ".csv", "w")
            # dscore
            file.write("dScore")
            for d in self.skills[glyph].dScore:
                file.write(",")
                file.write(str(d))
            file.write("\n")

            # nb time writing
            file.write("nbTime," + str(self.skills[glyph].nbTime) + "\n")

            # iteration occurence
            file.write("occurence")
            for d in self.skills[glyph].iterationOccurence:
                file.write(",")
                file.write(str(d))
            file.write("\n")

            file.close()





if __name__ == '__main__':
    # init node
    rospy.init_node("choose_adaptive_words")


    app = QtWidgets.QApplication(sys.argv)
    #app.setOverrideCursor(Qt.BlankCursor)
    window = Activity()
    manager = Manager(window)
    # drawing_window = DrawingCanvas()


    sys.exit(app.exec_())

    rospy.spin()
