# import os
# import openai
# import urllib.request
# import pkg_resources

# openai.api_key = os.getenv("OPENAI_API_KEY")


# class AI_IMAGE:
#     def __init__(self):
#         self.user_prompt = "A cartoon"
#         self.size = "256x256"
#         self.file_name = "ai_image.png"

#     def generate_image(self, text):
#         choose_adaptive_words_path = pkg_resources.resource_filename(
#             __name__, "design"
#         )

#         response = openai.Image.create(
#             prompt=self.user_prompt + text, n=1, size=self.size
#         )
#         image_url = response["data"][0]["url"]
#         urllib.request.urlretrieve(
#             image_url, choose_adaptive_words_path + "/assets/ai_image.png"
#         )

import os
import openai
import urllib.request
import pkg_resources
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from interface.msg import *
from interface.srv import TextToImage

from typing import List
import threading
from flask_cors import CORS

TOPIC_IMAGE_URL = "image_url"

api_key = os.getenv("OPENAI_API_KEY")
openai.api_key = api_key


class AiImage(Node):
    def __init__(self):
        Node.__init__(self, "ai_image")

        self.image_url = self.create_service(
            TextToImage, TOPIC_IMAGE_URL, self.generate_image
        )

        self.user_prompt = (
            "which is child-freindly and safe for children to see."
        )
        self.size = "256x256"

    # def generate_image(self, request, response):
    #     result = openai.Image.create(
    #         prompt="A image about" + self.user_prompt + request.text,
    #         n=1,
    #         size=self.size,
    #     )
    #     response.image_url = result["data"][0]["url"]
    #     return response

    def generate_image(self, text):
        result = openai.Image.create(
            prompt="A image about" + self.user_prompt + text,
            n=1,
            size=self.size,
        )
        image_url = result["data"][0]["url"]
        return image_url
