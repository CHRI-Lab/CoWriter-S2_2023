from std_msgs.msg import (
    String,
    Int32MultiArray,
    MultiArrayDimension,
    MultiArrayLayout,
)

import rclpy
from rclpy.node import Node

from interface.msg import Strokes, Stroke
from interface.srv import GetDemo, TextToImage
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

# from choose_adaptive_words.input_interpreter import InputInterpreter

WINDOW_DEFAULT_SIZE = (960, 540)
MIN_SIZE = (400, 300)
DRAWING_Y_OFFSET = 100
ERASR_SIZE = (100, 100)
ERASR_OFFSET = 10

TOPIC_SHAPES_TO_DRAW = "shapes_to_draw"
TOPIC_MANAGER_ERASE = "manager_erase"
TOPIC_LEARNING_PACE = "simple_learning_pace"
TOPIC_USER_DRAWN_SHAPES = "user_drawn_shapes"
TOPIC_WORDS_TO_WRITE = "words_to_write"
TOPIC_IMAGE_URL = "image_url"
SHAPE_TOPIC = "write_traj"

LETTER_FILES_READY_TOPIC = "files_ready"


# TODO: Difan add your code here for the child UI


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

        self.sub_word_to_write = self.create_subscription(
            String, TOPIC_WORDS_TO_WRITE, self.callback_text, 10
        )

        self.image_url = self.create_subscription(
            String, TOPIC_IMAGE_URL, self.callback_image_url, 10
        )

        self.publisher_ = self.create_publisher(Strokes, "strokesMessage", 10)

        self.publish_letter_files_ready = self.create_publisher(
            String, LETTER_FILES_READY_TOPIC, 10
        )

        self.image_url = ""
        self.generate_canvas = False
        self.inputText = ""
        self.traj = []
        self.erased = ""

    def callback_words_to_write(self, data):
        pts = [
            (data.data[i * 2], data.data[i * 2 + 1])
            for i in range(int(len(data.data) / 2))
        ]
        self.traj = pts

    def callback_manager_erase(self, data):
        self.get_logger().info('I heard: "%s"' % data.data)
        self.get_logger().info("erasing")
        # self.gui.manager_point_lists = list()
        # self.gui.update_drawings()
        self.erased = data.data

    def erase(self):
        if self.erased == "erased":
            self.erased = ""
            return "erased"
        else:
            return ""

    def canvas_status(self, status: bool):
        # self.inputText = ""
        self.generate_canvas = status

    def callback_text(self, data):
        self.get_logger().info('I heard: "%s"' % data.data)
        if len(data.data) != 0:
            self.inputText = data.data
            self.canvas_status(True)
        else:
            self.canvas_status(False)

    def callback_image_url(self, data):
        self.get_logger().info('I heard image url: "%s"' % data.data)
        self.image_url = data.data

    def update_image(self):
        return self.ai_image.generate_image(self.inputText)

    # Interpreter
    def publish_strokesMessage(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info(f"InputInterpreter publish: {msg.shape_type}")

    def shapes_to_dict(self, shapes):
        shapes_dict = {}

        for index, shape in enumerate(shapes.shapes):
            shapes_dict[shapes.word_to_learn[index]] = shape.path.tolist()

        return shapes_dict

    # create strokes msg for publishing
    def process_user_input(self, user_inputs):
        # print(user_input, type(user_input))
        # strokes = Strokes()
        strokes_value = []
        for k, user_input in enumerate(user_inputs):
            stroke_value = []
            for coordinates in user_input["strokes"][0]:
                x, y = coordinates.values()
                stroke_value.append((int(x), int(y)))
            min_y = min([y for _, y in stroke_value])
            max_y = max([y for _, y in stroke_value])
            middle_y = (min_y + max_y) // 2
            corrected_stroke_value = [
                (x, middle_y + (middle_y - y)) for x, y in stroke_value
            ]
            x_coords, y_coords = zip(*corrected_stroke_value)
            # plt.figure(figsize=(6, 6))
            plt.plot(x_coords, y_coords, "-", linewidth=10, color="black")
            plt.axis("off")
            plt.savefig(
                f"/home/nao/strugg_letter_data/{k}.png",
                format="png",
                dpi=300,
                bbox_inches="tight",
                pad_inches=0,
            )
            plt.close()
            strokes_value += stroke_value
        self.publish_letter_files_ready.publish(String(data=self.inputText))

        return self.pack_writing_pts(strokes_value)

    def pack_writing_pts(self, strokes_value):
        # capture mouse move, draw line when clicked
        unpacked_pts = []
        for p in strokes_value:
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
