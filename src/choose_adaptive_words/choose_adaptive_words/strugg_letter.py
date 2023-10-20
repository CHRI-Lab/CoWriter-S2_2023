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
from PIL import Image

LETTER_FILES_READY_TOPIC = "files_ready"
FEEDBACK_READY_TOPIC = "feedback_ready"
TOPIC_USER_DRAWN_SHAPES = "user_drawn_shapes"


class StruggLetterNode(Node):
    def __init__(self):
        super().__init__("strugg_letter")

        self.create_subscription(
            String,
            LETTER_FILES_READY_TOPIC,
            self.compute_strugg_feedback,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            TOPIC_USER_DRAWN_SHAPES,
            self.pentrack,
            10,
        )

        self.feedback_publisher = self.create_publisher(
            String, FEEDBACK_READY_TOPIC, 10
        )
        self.track = None
        self.feedback = []

    def pentrack(self, msg):
        self.track = msg.data

    def compute_strugg_feedback(self, msg):
        self.feedback = []
        word = str(msg.data)
        # images_dir = "/home/nao/strugg_letter_data/"
        # image_files = glob(os.path.join(images_dir, "*.png"))

        image_filenames = []

        # Specify the directory where your images are located
        image_directory = "/home/nao/strugg_letter_data/"
        cache_folders = [x[0] for x in os.walk(image_directory)]
        cache_folders.sort(key=os.path.getmtime)
        last_folder = cache_folders[-1]

        # Iterate through the files in the directory
        for filename in os.listdir(last_folder):
            if filename.endswith(
                ".png"
            ):  # You can adjust the file format as needed
                image_filenames.append(os.path.join(last_folder, filename))

        # Open and resize the images, assuming they have the same size
        images = [
            Image.open(filename).resize((255, 255))
            for filename in sorted(image_filenames)
        ]

        # Calculate the size of the combined image
        total_width = 255 * (len(images) + 2)
        combined_image = Image.new(
            "RGB", (total_width, 270), color=(255, 255, 255)
        )

        # Paste the images side by side
        x_offset = 255
        for image in images:
            combined_image.paste(image, (x_offset, 15))
            x_offset += 255

        # Save the combined image
        combined_image.save(os.path.join(last_folder, "combined_image.png"))

        if word is not None:
            text = word.strip()
            test_result = self.identify_strugg_letter(
                os.path.join(last_folder, "combined_image.png")
            )
            a = test_result
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
                        self.feedback.append(
                            "Your wrote letter: "
                            + text[i]
                            + " seems good! Keep going!"
                        )

                    elif text[i] == result[i].lower():
                        self.get_logger().info(
                            "-------------------------------------------"
                        )

                        self.get_logger().info(
                            "Your wrote letter: "
                            + text[i]
                            + " seems good! Keep going!"
                        )
                        self.feedback.append(
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
                        self.feedback.append(
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

                self.feedback.append(
                    "You witing looks like: "
                    + result
                    + ", so it does match your input word."
                )

            self.get_logger().info("Feedback Finished")
            self.feedback.append("Feedback Finished")
            self.get_logger().info(
                "-------------------------------------------"
            )
        else:
            self.get_logger().info("You did not input any word")
            self.feedback.append("You did not input any word")

        # self.get_logger().info(word + "!!!!!!!!!!")
        # if word is not None:
        #     for k, image_file in enumerate(image_files):
        #         text = word[k]
        #         test_result = self.identify_strugg_letter(image_file)
        #         a = test_result
        #         result = a.strip()
        #         self.get_logger().info("Identidy your strugged letter(s)")
        #         if len(result) == len(text):
        #             if text == result:
        #                 self.get_logger().info(
        #                     "-------------------------------------------"
        #                 )

        #                 self.get_logger().info(
        #                     "Your wrote letter: "
        #                     + text
        #                     + " seems good! Keep going!"
        #                 )
        #             else:
        #                 self.get_logger().info(
        #                     "-------------------------------------------"
        #                 )

        #                 self.get_logger().info(
        #                     "Your wrote letter: "
        #                     + text
        #                     + " looks like: "
        #                     + result
        #                     + ". Please practice more"
        #                 )
        #         else:
        #             self.get_logger().info(
        #                 "You writing looks like: "
        #                 + result
        #                 + ", so it does match your input word."
        #             )

        #     self.get_logger().info("Feedback Finished")
        #     self.get_logger().info(
        #         "-------------------------------------------"
        #     )
        # else:
        #     self.get_logger().info("You did not input any word")

    def identify_strugg_letter(self, filename):
        url = "https://pen-to-print-handwriting-ocr.p.rapidapi.com/recognize/"

        files = {"srcImg": open(filename, "rb")}
        payload = {"Session": "string"}
        headers = {
            "X-RapidAPI-Key": os.getenv("RAPIDAPI_KEY"),
            "X-RapidAPI-Host": "pen-to-print-handwriting-ocr.p.rapidapi.com",
        }

        response = requests.post(
            url, data=payload, files=files, headers=headers
        )
        self.get_logger().info("response " + str(response.json()))
        result1 = response.json()
        result1 = result1["value"]

        return result1
