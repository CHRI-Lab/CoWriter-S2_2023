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


WINDOW_DEFAULT_SIZE = (960, 540)
MIN_SIZE = (400, 300)
DRAWING_Y_OFFSET = 100
ERASR_SIZE = (100, 100)
ERASR_OFFSET = 10

TOPIC_SHAPES_TO_DRAW = "shapes_to_draw"
TOPIC_MANAGER_ERASE = "manager_erase"
TOPIC_LEARNING_PACE = "simple_learning_pace"
TOPIC_USER_DRAWN_SHAPES = "user_drawn_shapes"


class ChildUIBridge(Node):
    def __init__(self):
        super().__init__("child_ui_bridge")
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
        self.publish_user_drawn_shapes.publish(self.gui.pack_writing_pts(total_list))
