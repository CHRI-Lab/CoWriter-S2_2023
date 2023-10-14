import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from interface.msg import Strokes, Stroke
from interface.srv import GetDemo

from flask import Flask, request, jsonify, current_app
from typing import List
import threading
from flask_cors import CORS


# input_interpreter node
from cowriter_letter_learning.Redback_Learning.shape_learner_manager import (
    ShapeLearnerManager,
    generateSettings,
)


class InputInterpreter(Node, threading.Thread):
    def __init__(self):
        # define topic - strokeMessage
        self.publisher_ = self.create_publisher(Strokes, "strokesMessage", 10)

        # define service - get demo characters
        self.cli = self.create_client(GetDemo, "get_demo")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "GetDemo service not available, waiting again..."
            )
        self.get_demo_req = GetDemo.Request()

        self.get_demo_response = None

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

    def run(self):
        rclpy.spin(self)
        self.get_logger().info("Publisher InputInterpreter is running")

    @classmethod
    def shutdown_ros2(cls):
        if rclpy.ok():
            current_app.input_interpreter.destroy_node()
            rclpy.shutdown()


def shapes_to_dict(shapes):
    shapes_dict = {}

    for index, shape in enumerate(shapes.shapes):
        shapes_dict[shapes.word_to_learn[index]] = shape.path.tolist()

    return shapes_dict


# create strokes msg for publishing
def process_user_input(user_input):
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
