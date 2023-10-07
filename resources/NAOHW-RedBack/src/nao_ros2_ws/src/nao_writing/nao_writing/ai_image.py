import os
import openai
import urllib.request
import pkg_resources
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from interface.msg import *
from interface.srv import *

from flask import Flask, request, jsonify, current_app
from typing import List
import threading
from flask_cors import CORS


api_key = "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"
openai.api_key = api_key


class AiImage(Node, threading.Thread):
    def __init__(self):
        Node.__init__(self, "ai_image")
        threading.Thread.__init__(self)

        # define topic - strokeMessage
        self.publisher_ = self.create_publisher(String, "TextMessage", 10)

        self.user_prompt = "Child friendly cartoon image about"
        self.size = "256x256"
        self.file_name = "ai_image.png"

    def generate_image(self, text):
        # choose_adaptive_words_path = pkg_resources.resource_filename(
        #     __name__, "design"
        # )

        response = openai.Image.create(
            prompt=self.user_prompt + text, n=1, size=self.size
        )
        image_url = response["data"][0]["url"]
        return image_url
        # urllib.request.urlretrieve(
        #     image_url, choose_adaptive_words_path + "/assets/ai_image.png"
        # )


# flask backend services
class CustomFlask(Flask):
    def __init__(self, interpreter, *args, **kwargs):
        super().__init__(__name__, *args, **kwargs)
        self.input_interpreter = interpreter


app = CustomFlask(None)
cors_config = {
    "origins": ["*"],
    "methods": ["GET", "POST", "PUT", "DELETE"],
    "allow_headers": ["Content-Type", "Authorization"],
}

# Overlook CORS error when sending msg back to browser
CORS(app, resources={r"*": cors_config})


# flask api method
@app.route("/send_text", methods=["POST"])
def send_strokes():
    user_inputs = request.json
    inputText = user_inputs.get("inputText")
    image_url = current_app.ai_image.generate_image(inputText)

    return jsonify({"image_url": image_url})


def main():
    rclpy.init()

    ai_image = AiImage()
    app.ai_image = ai_image

    ai_image.start()

    app.run(host="0.0.0.0", port=5000)

    AiImage.shutdown_ros2()


if __name__ == "__main__":
    main()
