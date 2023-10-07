import json
import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import openai
import re
import speech_recognition as sr
import time
import os
import sys

api_key = "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"
openai.api_key = api_key


class ChatbotNode(Node):
    def __init__(self):
        super().__init__("chatbot_node")
        self.answers = ""
        self.recognizer = sr.Recognizer()
        self.audio = None
        self.stop_listening = None
        self.stop_button = None
        # add a socket object to the node
        self.__host = "127.0.0.1"
        self.__port = 12345
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP
        self.connect_socket()

    def connect_socket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP
        self.socket.connect((self.__host, self.__port))

    def start_listen(self):
        self.stop_listening = self.recognizer.listen_in_background(
            sr.Microphone(), self.callback
        )

    def stop_listen_and_recognize(self, entry):
        if self.stop_listening:
            self.stop_listening(wait_for_stop=False)
        if self.audio:
            try:
                text = self.recognizer.recognize_google(self.audio)
                entry.insert(0, text)
            except sr.UnknownValueError:
                self.get_logger().info("Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().info(
                    "Could not request results; {0}".format(e)
                )
        self.audio = None
        self.stop_button.config(state="disabled")

    def callback(self, recognizer, audio):
        self.audio = audio
        self.stop_button.after_idle(
            lambda: self.stop_button.config(state="normal")
        )

    def input_dialog(self):
        def on_enter(event=None):
            user_input = entry.get()
            user_input = user_input.lower()

            if "let's practise" in user_input or "let's practice" in user_input:
                words = user_input.replace("let's practice", "")

                ask = f"List me five English words related to {words}, I want to learn some English words related to {words} to expand my English vocabulary."
                if user_input:
                    response = self.completion(ask)
                    self.get_logger().info(response)

                    let_user_choose = "\n Please choose a word and let's practice it on the drawing board."
                    self.get_logger().info(let_user_choose)

                    self.answers = (
                        re.sub("\n\n", "", response) + let_user_choose
                    )

                    message = json.dumps({"TALK": self.answers.strip()}).encode(
                        "utf-8"
                    )
                    message_length = struct.pack(">I", len(message))
                    try:
                        self.get_logger().info("trying to make suggestion....")

                        self.socket.sendall(message_length)
                        self.socket.sendall(message)
                        # self.get_logger().info(self.answers + '-'*30)
                    except (ConnectionResetError, BrokenPipeError):
                        self.connect_socket()
                        self.socket.sendall(message_length)
                        self.socket.sendall(message)

                    entry.delete(0, tk.END)
                    self.answers = ""

            else:
                if user_input:
                    response = self.completion(user_input)
                    self.get_logger().info(response)

                    self.answers = re.sub("\n\n", "", response)
                    # print(self.answers)

                    message = json.dumps({"TALK": self.answers.strip()}).encode(
                        "utf-8"
                    )
                    message_length = struct.pack(">I", len(message))
                    try:
                        self.get_logger().info("trying to make suggestion....")

                        self.socket.sendall(message_length)
                        self.socket.sendall(message)
                        # self.get_logger().info(self.answers + '-'*30)
                    except (ConnectionResetError, BrokenPipeError):
                        self.connect_socket()
                        self.socket.sendall(message_length)
                        self.socket.sendall(message)

                    entry.delete(0, tk.END)
                    self.answers = ""
                    # if self.answers == '':
                    #     print('empty')

        root = tk.Tk()
        root.title("Let's talk")
        prompt_label = tk.Label(
            root, text="Glad to see you!!!!! Please enter your words here:"
        )
        prompt_label.pack()
        entry = tk.Entry(root, width=50)
        entry.pack()

        record_button = tk.Button(
            root, text="Record", command=self.start_listen
        )
        record_button.pack()

        self.stop_button = tk.Button(
            root,
            text="Stop",
            command=lambda: self.stop_listen_and_recognize(entry),
        )
        self.stop_button.pack()
        self.stop_button.config(state="disabled")

        clear_button = tk.Button(
            root, text="Clear", command=lambda: entry.delete(0, "end")
        )
        clear_button.pack()

        submit_button = tk.Button(root, text="Submit", command=on_enter)
        submit_button.pack()

        root.protocol("WM_DELETE_WINDOW", root.quit)
        root.mainloop()

    @staticmethod
    def completion(prompt):
        response = openai.Completion.create(
            model="text-davinci-003",
            prompt=prompt,
            temperature=0.5,
            max_tokens=1024,
            n=1,
            stop=None,
        )
        message = response.choices[0].text
        return message


def main(args=None):
    rclpy.init(args=args)
    chatbot_node = ChatbotNode()

    try:
        chatbot_node.input_dialog()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
