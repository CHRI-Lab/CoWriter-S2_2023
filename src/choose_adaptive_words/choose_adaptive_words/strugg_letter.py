from std_msgs.msg import (
    String,
    Int32MultiArray,
    MultiArrayDimension,
    MultiArrayLayout,
)
import requests
from glob import glob
import rclpy
from rclpy.node import Node
import os
from interface.msg import Strokes, Stroke
from interface.srv import GetDemo, TextToImage
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

LETTER_FILES_READY_TOPIC = "files_ready"
FEEDBACK_READY_TOPIC = "feedback_ready"


class StruggLetterNode(Node):
    def __init__(self):
        super().__init__("strugg_letter")

        self.create_subscription(
            String,
            LETTER_FILES_READY_TOPIC,
            self.compute_strugg_feedback,
            10,
        )

        self.feedback_publisher = self.create_publisher(
            String, FEEDBACK_READY_TOPIC, 10
        )

    def compute_strugg_feedback(self, msg):
        word = str(msg.data)
        images_dir = "/home/nao/strugg_letter_data/"
        image_files = glob(os.path.join(images_dir, "*.png"))
        if word is not None:
            for k, image_file in enumerate(image_files):
                text = word[k]
                test_result = identify_strugg_letter(image_file)
                a = test_result["value"]
                result = a.strip()

                self.get_logger().info(result + "!!!!!!!!!!")

                self.get_logger().info("Identidy your strugged letter(s)")
                if len(result) == len(text):
                    for i in range(len(result)):
                        if text[i] == result[i]:
                            self.get_logger().info(
                                "-------------------------------------------"
                            )

                            self.get_logger().info(
                                "Your wrote letter: "
                                + text[i]
                                + " seems good! Keep going!"
                            )
                        else:
                            self.get_logger().info(
                                "-------------------------------------------"
                            )

                            self.get_logger().info(
                                "Your wrote letter: "
                                + text[i]
                                + " looks like: "
                                + result[i]
                                + ". Please practice more"
                            )
                else:
                    self.get_logger().info(
                        "You witing looks like: "
                        + result
                        + ", so it does match your input word."
                    )

                self.get_logger().info("Feedback Finished")
                self.get_logger().info(
                    "-------------------------------------------"
                )
            else:
                self.get_logger().info("You did not input any word")


def identify_strugg_letter(filename):
    url = "https://pen-to-print-handwriting-ocr.p.rapidapi.com/recognize/"

    files = {"srcImg": open(filename, "rb")}
    payload = {"Session": "string"}
    headers = {
        "X-RapidAPI-Key": "c97960a792mshf676e38f49cd3e6p12b5abjsnb145d15144af",
        "X-RapidAPI-Host": "pen-to-print-handwriting-ocr.p.rapidapi.com",
    }

    response = requests.post(url, data=payload, files=files, headers=headers)

    return response.json()
