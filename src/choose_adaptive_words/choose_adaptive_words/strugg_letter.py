# from std_msgs.msg import (
#     String,
#     Int32MultiArray,
#     MultiArrayDimension,
#     MultiArrayLayout,
# )
# import requests
# from glob import glob
# import rclpy
# from rclpy.node import Node
# import os
# from interface.msg import Strokes, Stroke
# from interface.srv import GetDemo, TextToImage
# from nav_msgs.msg import Path
# import matplotlib.pyplot as plt
# import pytesseract
# from PIL import Image

# LETTER_FILES_READY_TOPIC = "files_ready"
# FEEDBACK_READY_TOPIC = "feedback_ready"
# TOPIC_USER_DRAWN_SHAPES = "user_drawn_shapes"


# class StruggLetterNode(Node):
#     def __init__(self):
#         super().__init__("strugg_letter")

#         self.create_subscription(
#             String,
#             LETTER_FILES_READY_TOPIC,
#             self.compute_strugg_feedback,
#             10,
#         )
#         self.create_subscription(
#             Int32MultiArray,
#             TOPIC_USER_DRAWN_SHAPES,
#             self.pentrack,
#             10,
#         )

#         self.feedback_publisher = self.create_publisher(
#             String, FEEDBACK_READY_TOPIC, 10
#         )
#         self.track = None

#     def pentrack(self, msg):
#         self.track = msg.data

#     def compute_strugg_feedback(self, msg):
#         word = str(msg.data)
#         images_dir = "/home/nao/strugg_letter_data/"
#         image_files = glob(os.path.join(images_dir, "*.png"))
#         self.get_logger().info(word + "!!!!!!!!!!")
#         if word is not None:
#             for k, image_file in enumerate(image_files):
#                 text = word[k]
#                 ocr, result1, test_result = self.identify_strugg_letter(
#                     image_file, text
#                 )
#                 self.get_logger().info("test_result " + str(test_result))
#                 a = test_result
#                 result = a.strip()
#                 self.get_logger().info("Identidy your strugged letter(s)")
#                 if len(result) == len(text):
#                     if text == result:
#                         self.get_logger().info(
#                             "-------------------------------------------"
#                         )

#                         self.get_logger().info(
#                             "Your wrote letter: "
#                             + text
#                             + " seems good! Keep going!"
#                         )
#                     else:
#                         self.get_logger().info(
#                             "-------------------------------------------"
#                         )

#                         self.get_logger().info(
#                             "Your wrote letter: "
#                             + text
#                             + " looks like: "
#                             + result
#                             + ". Please practice more"
#                         )
#                 else:
#                     self.get_logger().info(
#                         "You writing looks like: "
#                         + result
#                         + ", so it does match your input word."
#                     )

#             self.get_logger().info("Feedback Finished")
#             self.get_logger().info(
#                 "-------------------------------------------"
#             )
#         else:
#             self.get_logger().info("You did not input any word")

#     def identify_strugg_letter(self, filename, letter):
#         url = "https://pen-to-print-handwriting-ocr.p.rapidapi.com/recognize/"

#         files = {"srcImg": open(filename, "rb")}
#         payload = {"Session": "string"}
#         headers = {
#             "X-RapidAPI-Key": "c97960a792mshf676e38f49cd3e6p12b5abjsnb145d15144af",
#             "X-RapidAPI-Host": "pen-to-print-handwriting-ocr.p.rapidapi.com",
#         }

#         response = requests.post(
#             url, data=payload, files=files, headers=headers
#         )
#         self.get_logger().info("response " + str(response.json()))
#         result1 = response.json()
#         result1 = result1["value"]

#         img = Image.open(filename)
#         # Perform OCR using Tesseract
#         custom_config = r"--oem 3 --psm 6"
#         text = pytesseract.image_to_string(
#             img, lang="eng", config=custom_config
#         )
#         result2 = text.strip()

#         OCR = 0
#         if result1 == letter or result2 == letter:
#             OCR = 0.5

#         return OCR, result1, result2
