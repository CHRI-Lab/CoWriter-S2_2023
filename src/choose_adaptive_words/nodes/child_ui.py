from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt, QSize, QLine
from PyQt5 import QtGui
import sys
import os
from std_msgs.msg import (
    String,
    Int32MultiArray,
    MultiArrayDimension,
    MultiArrayLayout,
)

import rclpy
from rclpy.node import Node

WINDOW_DEFAULT_SIZE = (960, 540)
MIN_SIZE = (400, 300)
DRAWING_Y_OFFSET = 100
ERASR_SIZE = (100, 100)
ERASR_OFFSET = 10

TOPIC_SHAPES_TO_DRAW = "shapes_to_draw"
TOPIC_MANAGER_ERASE = "manager_erase"
TOPIC_LEARNING_PACE = "simple_learning_pace"
TOPIC_USER_DRAWN_SHAPES = "user_drawn_shapes"


class Child_UI(QtWidgets.QMainWindow, Node):
    def __init__(self):
        Node.__init__(self, "child_ui")
        super().__init__()
        working_dr = os.getcwd()
        self.pathWriter = working_dr + "/ui_database"

        choose_adaptive_words_path = os.path.dirname(
            os.path.dirname(os.path.realpath(__file__))
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
            QtGui.QIcon(choose_adaptive_words_path + "/design/assets/gomme.png")
        )
        self.button_erase.clicked.connect(self.button_erase_clicked)

        # set send feedback button
        self.button_feedback = QtWidgets.QPushButton(self)
        self.button_feedback.setIconSize(
            QSize(ERASR_SIZE[0] - ERASR_OFFSET, ERASR_SIZE[1] - ERASR_OFFSET)
        )
        self.button_feedback.setIcon(
            QtGui.QIcon(choose_adaptive_words_path + "/design/assets/send.svg")
        )
        self.button_feedback.clicked.connect(self.feedback_clicked)

        # init pos recording
        self.last_x, self.last_y = None, None

        # init child pen
        self.child_pen = QtGui.QPen()
        self.child_pen.setWidth(4)

        # init manager pen
        self.manager_pen = QtGui.QPen()
        self.manager_pen.setWidth(2)
        self.manager_pen.setColor(QtGui.QColor(255, 31, 31))

        # list of drawings
        self.child_point_list = list()
        self.child_point_lists = list()
        self.manager_point_lists = list()

        # init subscribers
        self.create_subscription(
            Int32MultiArray,
            TOPIC_SHAPES_TO_DRAW,
            self.callback_words_to_write,
            10,
        )
        self.create_subscription(
            String, TOPIC_MANAGER_ERASE, self.callback_manager_erase, 10
        )

        # init publisher
        self.publish_user_drawn_shapes = self.create_publisher(
            Int32MultiArray, TOPIC_USER_DRAWN_SHAPES, 10
        )

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

    def feedback_clicked(self):
        """
        publish feedback on click
        """
        total_list = []
        i = 0

        for l in self.child_point_lists:
            total_list.append((i, len(l)))
            total_list += l
            i += 1
        print(total_list)
        self.publish_user_drawn_shapes.publish(
            self.pack_writing_pts(total_list)
        )

    def resizeEvent(self, event):
        if hasattr(self, "button_erase"):
            self.button_erase.setGeometry(
                self.width() - ERASR_SIZE[0] - ERASR_OFFSET,
                ERASR_OFFSET,
                ERASR_SIZE[0],
                ERASR_SIZE[1],
            )
        if hasattr(self, "button_feedback"):
            self.button_feedback.setGeometry(
                self.width() - 2 * ERASR_SIZE[0] - 2 * ERASR_OFFSET,
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

    def callback_words_to_write(self, data):
        pts = [
            (data.data[i * 2], data.data[i * 2 + 1])
            for i in range(int(len(data.data) / 2))
        ]
        self.manager_point_lists.append(pts)
        self.update_drawings()

    def callback_manager_erase(self, data):
        self.manager_point_lists = list()
        self.update_drawings()


if __name__ == "__main__":
    rclpy.init()

    app = QtWidgets.QApplication(sys.argv)

    window = Child_UI()
    window.show()

    rclpy.spin(window)

    window.destroy_node()
    rclpy.shutdown()

    sys.exit(app.exec_())
