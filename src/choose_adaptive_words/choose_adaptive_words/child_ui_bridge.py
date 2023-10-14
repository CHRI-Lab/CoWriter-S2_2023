from std_msgs.msg import String, Int32MultiArray

import rclpy
from rclpy.node import Node

from interface.msg import Strokes, Stroke
from interface.srv import GetDemo, TextToImage

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
TOPIC_HANDWRITING_STORKES = "handwriting_strokes"
TOPIC_WORDS_TO_WRITE = "words_to_write"
TOPIC_IMAGE_URL = "image_url"


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

        self.publish_handwriting = self.create_publisher(
            Int32MultiArray, TOPIC_HANDWRITING_STORKES, 10
        )

        self.sub_word_to_write = self.create_subscription(
            String, TOPIC_WORDS_TO_WRITE, self.callback_text, 10
        )

        self.image_url = self.create_subscription(
            String, TOPIC_IMAGE_URL, self.callback_image_url, 10
        )
        # self.text_to_url = self.create_client(TextToImage, TOPIC_IMAGE_URL)
        # while not self.text_to_url.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("service not available, waiting again...")

        # self.request = TextToImage.Request()

        self.publisher_ = self.create_publisher(Strokes, "strokesMessage", 10)

        # define service - get demo characters
        # self.cli = self.create_client(GetDemo, "g")
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(
        #         "GetDemo service not available, waiting again..."
        #     )
        # self.get_demo_req = GetDemo.Request()

        # self.get_demo_response = None

        self.image_url = ""
        self.generate_canvas = False
        self.inputText = ""

    def callback_words_to_write(self, data):
        pts = [
            (data.data[i * 2], data.data[i * 2 + 1])
            for i in range(int(len(data.data) / 2))
        ]

    def callback_manager_erase(self, data):
        self.get_logger().info('I heard: "%s"' % data.data)
        self.get_logger().info("erasing")
        # self.gui.manager_point_lists = list()
        # self.gui.update_drawings()

    # def feedback_clicked(self):
    #     """
    #     publish feedback on click
    #     """
    #     total_list = []
    #     i = 0

    #     for point_list in self.gui.child_point_lists:
    #         total_list.append((i, len(point_list)))
    #         total_list += point_list
    #         i += 1
    #     print(total_list)
    #     self.publish_user_drawn_shapes.publish(
    #         self.gui.pack_writing_pts(total_list)
    #     )

    # def send_strokes(self, strokes):
    #     self.get_logger().info("published " + strokes + " to /handwriting_strokes")
    #     self.publish_handwriting.publish(String(data=strokes))

    # def generate_image(self, text: str):
    #     self.get_logger().info("published " + text + " to /image_url")
    #     self.input_word.publish(String(data=text))

    def update_canvas(self, status: bool):
        self.inputText = ""
        self.generate_canvas = status

    def callback_text(self, data):
        self.get_logger().info('I heard: "%s"' % data.data)
        self.inputText = data.data
        self.update_canvas(True)

    def callback_image_url(self, data):
        self.get_logger().info('I heard image url: "%s"' % data.data)
        self.image_url = data.data

    def call_generate_image(self, text):
        # self.request.text = text
        # self.future = self.text_to_url.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result().img_url

        return self.ai_image.generate_image(self.inputText)

    def update_image(self):
        return self.ai_image.generate_image(self.inputText)

    # Interpreter
    def publish_strokesMessage(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info(f"InputInterpreter publish: {msg.shape_type}")

    def send_request(self, word):
        self.get_logger().info(
            f"InputInterpreter request demo for word: {word}"
        )
        self.get_demo_req.word = word
        self.future = self.cli.call_async(self.get_demo_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def shapes_to_dict(self, shapes):
        shapes_dict = {}

        for index, shape in enumerate(shapes.shapes):
            shapes_dict[shapes.word_to_learn[index]] = shape.path.tolist()

        return shapes_dict

    # create strokes msg for publishing
    def process_user_input(self, user_input):
        # print(user_input, type(user_input))
        strokes = Strokes()
        strokes_value = []
        for s in user_input["strokes"]:
            stroke = Stroke()
            stroke_value = []
            for coordinates in s:
                x, y = coordinates.values()
                stroke_value.extend([float(x), float(y)])
            stroke.stroke = stroke_value
        strokes_value.append(stroke)
        strokes.strokes = strokes_value

        if user_input["shape_id"] is not None:
            strokes.shape_id = user_input["shape_id"]
        if user_input["shape_type"] is not None:
            strokes.shape_type = user_input["shape_type"]
        if user_input["shapetype_code"] is not None:
            strokes.shapetype_code = user_input["shapetype_code"]
        if user_input["params_to_vary"] is not None:
            strokes.params_to_vary = user_input["params_to_vary"]
        if user_input["param_values"] is not None:
            strokes.param_values = user_input["param_values"]
        return strokes
