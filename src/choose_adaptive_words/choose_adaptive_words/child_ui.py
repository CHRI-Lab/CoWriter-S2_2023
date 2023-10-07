from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt, QSize, QLine, QTimer
from PyQt5 import QtGui
import sys
import os
from std_msgs.msg import (
    String,
    Int32MultiArray,
    MultiArrayDimension,
    MultiArrayLayout,
)

from threading import Thread
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
import pkg_resources
from choose_adaptive_words.strugg_letter import identify_strugg_letter
from choose_adaptive_words.manager_ui import word
from PIL import Image

WINDOW_DEFAULT_SIZE = (960, 540)
MIN_SIZE = (400, 300)
DRAWING_Y_OFFSET = 100
ERASR_SIZE = (100, 100)
ERASR_OFFSET = 10

TOPIC_SHAPES_TO_DRAW = "shapes_to_draw"
TOPIC_MANAGER_ERASE = "manager_erase"
TOPIC_LEARNING_PACE = "simple_learning_pace"
TOPIC_USER_DRAWN_SHAPES = "user_drawn_shapes"


class Child_UI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # Node.__init__(self, "child_ui")
        working_dr = os.getcwd()
        self.pathWriter = working_dr + "/ui_database"
        # self.ros_node = ros_node
        # choose_adaptive_words_path = os.path.dirname(
        #     os.path.dirname(os.path.realpath(__file__))
        # )
        self.choose_adaptive_words_path = pkg_resources.resource_filename(
            __name__, "design"
        )

        # set window
        self.setWindowTitle("Child UI")
        self.setGeometry(0, 0, WINDOW_DEFAULT_SIZE[0], WINDOW_DEFAULT_SIZE[1])
        self.setMinimumSize(MIN_SIZE[0], MIN_SIZE[1])

        # set drawing area
        self.drawing = QtWidgets.QLabel(self)
        self.drawing.setStyleSheet(
            "QLabel" "{" "border : 2px solid gray;" "background : white;" "}"
        )

        # set erase button
        self.button_erase = QtWidgets.QPushButton(self)
        self.button_erase.setIconSize(
            QSize(ERASR_SIZE[0] - ERASR_OFFSET, ERASR_SIZE[1] - ERASR_OFFSET)
        )
        self.button_erase.setIcon(
            QtGui.QIcon(self.choose_adaptive_words_path + "/assets/gomme.png")
        )
        self.button_erase.clicked.connect(self.button_erase_clicked)

        # set send feedback button
        self.button_feedback = QtWidgets.QPushButton(self)
        self.button_feedback.setIconSize(
            QSize(ERASR_SIZE[0] - ERASR_OFFSET, ERASR_SIZE[1] - ERASR_OFFSET)
        )
        self.button_feedback.setIcon(
            QtGui.QIcon(self.choose_adaptive_words_path + "/assets/send.svg")
        )

        # set strugg button
        self.button_strugg = QtWidgets.QPushButton(self)
        self.button_strugg.setIconSize(
            QSize(ERASR_SIZE[0] - ERASR_OFFSET, ERASR_SIZE[1] - ERASR_OFFSET)
        )
        self.button_strugg.setIcon(
            QtGui.QIcon(self.choose_adaptive_words_path + "/assets/predict.png")
        )
        self.button_strugg.clicked.connect(self.button_strugg_clicked)

        # insert thr AI-Generated image
        self.image = QtWidgets.QLabel(self)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(1000)

        self.updateImage()

        # init pos recording
        self.last_x, self.last_y = None, None

        # init child pen
        self.child_pen = QtGui.QPen()
        self.child_pen.setWidth(10)

        # init manager pen
        self.manager_pen = QtGui.QPen()
        self.manager_pen.setWidth(4)
        self.manager_pen.setColor(QtGui.QColor(255, 31, 31))

        # list of drawings
        self.child_point_list = list()
        self.child_point_lists = list()
        self.manager_point_lists = list()
        # self.manager_point_lists = [[(77, 50), (76, 49), (75, 49), (74, 49), (73, 49), (71, 48), (69, 47), (67, 47), (65, 46), (64, 46), (62, 46), (61, 45), (60, 45), (58, 45), (57, 45), (56, 45), (55, 45), (54, 45), (53, 45), (53, 46), (53, 47), (52, 48), (51, 49), (51, 50), (50, 51), (49, 53), (49, 54), (48, 55), (48, 56), (47, 57), (47, 59), (46, 60), (45, 61), (45, 62), (45, 63), (45, 64), (45, 65), (45, 66), (45, 67), (45, 69), (45, 71), (45, 72), (45, 73), (45, 74), (45, 75), (45, 76), (45, 77), (45, 78), (45, 80), (45, 81), (45, 82), (46, 83), (46, 84), (46, 85), (47, 86), (48, 87), (49, 87), (49, 88), (50, 88), (51, 88), (52, 89), (53, 90), (55, 90), (57, 90), (58, 91), (60, 92), (61, 92), (62, 92), (64, 92), (65, 92), (66, 92), (67, 92), (68, 92), (69, 92), (70, 91), (71, 91), (72, 91), (74, 90), (74, 89), (75, 89), (75, 88), (77, 87), (77, 86), (78, 86), (78, 85), (80, 84), (81, 83), (81, 82)],
        #                             [(31, 31), (32, 29), (33, 29), (34, 27), (35, 26), (35, 25), (36, 25), (37, 25), (37, 24), (38, 23), (39, 23), (39, 22), (40, 22), (40, 21), (41, 21), (42, 20), (44, 20), (44, 19), (45, 19), (46, 19), (47, 19), (47, 20), (48, 20), (49, 21), (51, 21), (52, 22), (54, 22), (56, 22), (59, 23), (61, 23), (62, 23), (62, 24), (62, 25), (63, 25), (63, 26), (63, 27), (64, 27), (64, 28), (64, 29), (65, 29), (65, 30), (65, 31), (65, 32), (65, 33), (66, 34), (66, 35), (66, 36), (67, 38), (67, 40), (67, 41), (67, 42), (67, 43), (67, 45), (67, 47), (67, 48), (67, 50), (67, 51), (67, 52), (67, 53), (67, 54), (67, 55), (67, 56), (67, 57), (67, 58), (66, 59), (66, 60), (66, 61), (66, 62), (66, 63), (64, 64), (63, 66), (62, 67), (61, 68), (60, 70), (58, 72), (58, 73), (56, 74), (55, 75), (55, 76), (54, 76), (53, 77), (51, 78), (50, 78), (50, 79), (49, 79), (48, 80), (46, 81), (45, 81), (44, 81), (43, 81), (42, 81), (41, 81), (39, 81), (37, 81), (35, 81), (32, 81), (30, 81), (28, 81), (27, 81), (26, 80), (26, 79), (26, 78), (25, 78), (25, 77), (24, 76), (24, 75), (23, 74), (23, 73), (22, 72), (22, 71), (22, 70), (22, 69), (22, 68), (22, 67), (22, 66), (22, 65), (22, 64), (22, 63), (22, 62), (23, 62), (23, 61), (24, 60), (25, 60), (25, 59), (25, 58), (25, 57), (26, 56), (26, 55), (28, 54), (29, 54), (30, 53), (31, 53), (32, 52), (33, 52), (34, 51), (36, 50), (37, 49), (37, 48), (38, 48), (39, 48), (40, 48), (41, 47), (42, 47), (43, 47), (44, 47), (45, 47), (46, 47), (47, 47), (48, 47), (50, 47), (51, 47), (53, 46), (54, 46), (56, 46), (57, 46), (58, 46), (59, 46), (60, 46), (60, 47), (61, 47), (62, 47), (63, 49), (64, 50), (64, 51), (65, 51), (65, 52), (65, 53), (66, 53), (66, 54), (66, 55), (66, 56), (67, 56), (67, 57), (67, 58), (67, 59), (67, 60), (67, 61), (67, 62), (67, 63), (67, 64), (67, 65), (67, 66), (68, 67), (68, 68), (68, 69), (69, 70), (69, 71), (69, 72), (70, 73), (70, 74), (71, 74), (71, 75), (71, 76), (72, 77), (73, 77), (73, 78), (74, 78), (75, 79), (77, 80), (78, 80), (79, 80), (80, 80), (81, 80), (82, 80), (82, 81), (81, 81)],
        #                             [(44, 39), (45, 39), (46, 39), (46, 38), (47, 37), (47, 38), (47, 39), (47, 40), (48, 40), (48, 41), (48, 42), (48, 43), (48, 44), (48, 46), (48, 47), (48, 48), (48, 49), (48, 50), (48, 51), (48, 52), (47, 52), (47, 53), (47, 54), (46, 54), (46, 55), (46, 56), (46, 57), (46, 58), (46, 59), (45, 59), (45, 60), (44, 60), (44, 61), (44, 62), (44, 63), (43, 63), (43, 64), (43, 65), (43, 66), (43, 67), (43, 68), (42, 70), (42, 71), (41, 71), (41, 72), (41, 73), (41, 74), (41, 75), (41, 76), (41, 77), (41, 78), (41, 79), (41, 80), (41, 81), (41, 82), (41, 83), (41, 84), (41, 85), (41, 86), (42, 86), (43, 87), (44, 87), (45, 87), (46, 87), (47, 87), (48, 87), (49, 87), (50, 87), (52, 87), (53, 87), (54, 87), (55, 87), (55, 86), (56, 86), (56, 85), (57, 84), (57, 83), (58, 82), (59, 81), (59, 80), (60, 79), (60, 78), (60, 77), (60, 76)], [(36, 52), (39, 51), (41, 51), (43, 51), (46, 50), (47, 50), (48, 50), (49, 50), (50, 50), (51, 50), (52, 50), (53, 50), (54, 50), (55, 50), (56, 50), (57, 50), (59, 50), (60, 50), (61, 50), (62, 50), (61, 49)]]
        # a = []
        # a.append(self.manager_point_lists[0])
        # b = []
        # for a in self.manager_point_lists[1]:
        #     b.append((a[0]+40,a[1]))

        # a.append(b)
        # b=[]
        # for a in self.manager_point_lists[2]:
        #     b.append((a[0]+80,a[1]))
        # a.append(b)

        # self.manager_point_lists = a

            

    def updateImage(self):
        self.pixmap = QtGui.QPixmap(
            self.choose_adaptive_words_path + "/assets/ai_image.png"
        )
        scale_pixmap = self.pixmap.scaled(
            QSize(ERASR_SIZE[0] - ERASR_OFFSET, ERASR_SIZE[1] - ERASR_OFFSET)
        )
        self.image.setPixmap(scale_pixmap)

    def update_drawings(self):
        # reset canvas
        canvas = QtGui.QPixmap(self.drawing.width(), self.drawing.height())
        canvas.fill(Qt.white)
        self.drawing.setPixmap(canvas)
        p = QtGui.QPainter(self.drawing.pixmap())

        # draw child lines
        child_line_list = list()
        for ps in self.child_point_lists + [self.child_point_list]:
            if len(ps) > 1:
                for i in range(1, len(ps)):
                    line_seg = QLine(
                        ps[i - 1][0], ps[i - 1][1], ps[i][0], ps[i][1]
                    )
                    child_line_list.append(line_seg)

        p.setPen(self.child_pen)
        for l in child_line_list:
            p.drawLine(l)

        # draw manager lines
        manager_line_list = list()
        for ps in self.manager_point_lists:
            if len(ps) > 1:
                for i in range(1, len(ps)):
                    line_seg = QLine(
                        ps[i - 1][0], ps[i - 1][1], ps[i][0], ps[i][1]
                    )
                    manager_line_list.append(line_seg)

        p.setPen(self.manager_pen)
        for l in manager_line_list:
            p.drawLine(l)

        p.end()

        # update
        self.update()

    def button_strugg_clicked(self):
        self.drawing.pixmap().save("draw.png")

        child_x_axle = []
        child_y_axle = []
        if len(self.child_point_lists) > 1:
            for i in range (len(self.child_point_lists)):
                for x in self.child_point_lists[i]:
                    child_x_axle.append(x[0])
                    child_y_axle.append(x[1])
        else:
            for x in self.child_point_lists[0]:
                child_x_axle.append(x[0])
                child_y_axle.append(x[1])
        # Crop image of child writing
        image = Image.open('draw.png')
        image.crop((min(child_x_axle)-20,min(child_y_axle),max(child_x_axle)+20,max(child_y_axle))).save("draw.png")

        file = open('feedback.txt','w')
        if word != None:
            text = word.strip()
            test_result = identify_strugg_letter('draw.png')
            a = test_result['value']
            result = a.strip()
            print("Identidy your strugged letter(s)")
            file.write("Your feedback: \n")
            if len(result) == len(text):
                
                for i in range(len(result)):
                    if text[i] == result[i]:
                        print ("-------------------------------------------")
                        file.write("-------------------------------------------\n")
                        print ("Your wrote letter: ", text[i], " seems good! Keep going!" )
                        file.write("Your wrote letter: "+ text[i]+ " seems good! Keep going! \n")
                    else:
                        print ("-------------------------------------------")
                        file.write("-------------------------------------------\n")
                        print ("Your wrote letter: ", text[i], " looks like: ", result[i], ". Please practice more" )
                        file.write("Your wrote letter: "+ text[i]+ " looks like: ", result[i], ". Please practice more\n")
            else:
                file.write("You witing looks like: " + result+ ", so it does match your input word. \n")
            print("Feedback Finished")
            file.write("-------------------------------------------\n")
        else:
            file.write("You did not input any word\n")
            

    def button_erase_clicked(self):
        self.child_point_lists = list()
        self.update_drawings()

    def pack_writing_pts(self, pts):
        """
        capture mouse move, draw line when clicked
        """
        unpacked_pts = []
        for p in pts:
            unpacked_pts.append(p[0])
            unpacked_pts.append(p[1])

        dim1 = MultiArrayDimension()
        dim1.label = "pts"
        dim1.size = int(len(unpacked_pts) / 2)
        dim1.stride = len(unpacked_pts)

        dim2 = MultiArrayDimension()
        dim2.label = "pt"
        dim2.size = 2
        dim2.stride = 2

        layout = MultiArrayLayout()
        layout.dim = [dim1, dim2]
        layout.data_offset = 0

        pt_msg = Int32MultiArray()
        pt_msg.layout = layout
        pt_msg.data = unpacked_pts

        return pt_msg

    def resizeEvent(self, event):
        if hasattr(self, "button_erase"):
            self.button_erase.setGeometry(
                ERASR_OFFSET,
                ERASR_OFFSET,
                ERASR_SIZE[0],
                ERASR_SIZE[1],
            )
        if hasattr(self, "button_feedback"):
            self.button_feedback.setGeometry(
                self.width() - ERASR_SIZE[0] - ERASR_OFFSET,
                ERASR_OFFSET,
                ERASR_SIZE[0],
                ERASR_SIZE[1],
            )
        if hasattr(self, "image"):
            self.image.setGeometry(
                self.width() // 3,
                ERASR_OFFSET,
                ERASR_SIZE[0],
                ERASR_SIZE[1],
            )

        if hasattr(self, "button_strugg"):
            self.button_strugg.setGeometry(
                self.width() *2 // 3 ,
                ERASR_OFFSET,
                ERASR_SIZE[0],
                ERASR_SIZE[1],
            )

        self.update_drawings()
        if hasattr(self, "drawing"):
            self.drawing.setGeometry(
                0,
                ERASR_OFFSET * 2 + ERASR_SIZE[1],
                self.width(),
                self.height() - DRAWING_Y_OFFSET,
            )
        self.update_drawings()

    def mouseMoveEvent(self, e):
        """
        capture mouse move, draw line when clicked
        """
        mouse_x, mouse_y = e.x(), e.y() - ERASR_OFFSET * 2 - ERASR_SIZE[1]

        # first time calling
        if self.last_x is None:
            self.last_x = mouse_x
            self.last_y = mouse_y
            return

        self.child_point_list.append((mouse_x, mouse_y))
        self.update_drawings()

    def mouseReleaseEvent(self, e):
        self.last_x = None
        self.last_y = None
        self.child_point_lists.append(self.child_point_list)
        self.child_point_list = []


class ChildGUINode(Node):
    def __init__(self, gui: Child_UI):
        super().__init__("child_ui")
        self.gui = gui
        # init subscribers
        self.create_subscription(
            Int32MultiArray,
            TOPIC_SHAPES_TO_DRAW,
            self.callback_words_to_write,
            10,
        )
        self.sub_eraser = self.create_subscription(
            String, TOPIC_MANAGER_ERASE, self.callback_manager_erase, 10
        )

        # init publisher
        self.publish_user_drawn_shapes = self.create_publisher(
            Int32MultiArray, TOPIC_USER_DRAWN_SHAPES, 10
        )
        self.gui.button_feedback.clicked.connect(self.feedback_clicked)

        # self.subscription = self.create_subscription(
        #     String,
        #     'topic',
        #     self.listener_callback,
        #     10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def callback_words_to_write(self, data):
        pts = [
            (data.data[i * 2], data.data[i * 2 + 1])
            for i in range(int(len(data.data) / 2))
        ]
        self.gui.manager_point_lists.append(pts)
        self.gui.update_drawings()

    def callback_manager_erase(self, data):
        self.get_logger().info('I heard: "%s"' % data.data)
        self.get_logger().info("erasing")
        self.gui.manager_point_lists = list()
        self.gui.update_drawings()

    def feedback_clicked(self):
        """
        publish feedback on click
        """
        total_list = []
        i = 0

        for l in self.gui.child_point_lists:
            total_list.append((i, len(l)))
            total_list += l
            i += 1
        print(total_list)
        self.publish_user_drawn_shapes.publish(
            self.gui.pack_writing_pts(total_list)
        )


def main(args=None):
    rclpy.init()

    app = QtWidgets.QApplication(sys.argv)
    window = Child_UI()

    node = ChildGUINode(window)
    # node.create_subscription(
    #         String,
    #         'topic',
    #         node.listener_callback,
    #         10)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = Thread(target=executor.spin)
    thread.start()
    node.get_logger().info("node spin")

    try:
        window.show()
        sys.exit(app.exec_())

    finally:
        node.get_logger().info("Shutting down ROS2 Node . . .")
        node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
