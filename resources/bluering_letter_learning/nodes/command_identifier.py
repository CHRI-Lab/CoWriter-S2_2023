#!/usr/bin/env python

import re
import rospy
from std_msgs.msg import String

CHANG_WORD = "change word"
LOGIN = "log"
REGISTER = "register"


def detect_command(text):
    commands = [CHANG_WORD, LOGIN, REGISTER]
    pattern = "|".join(re.escape(cmd) for cmd in commands)
    matches = re.findall(pattern, text, re.IGNORECASE)
    return matches


def detect_change_word(text):
    pattern = r'(?=.*\bchange\b)(?=.*\bword\b)'
    matches = re.search(pattern, text)
    return matches


def generate_new_word():
    word = "sheep"
    return word


def command_publisher(text):
    # Initialize the node and the publisher
    rospy.init_node('command_publisher', anonymous=True)

    word = generate_new_word();
    WORDS_TOPIC = rospy.get_param('~words_to_write_topic', 'words_to_write')
    change_word_pub = rospy.Publisher(WORDS_TOPIC, String, queue_size=10)

    # commands = detect_command(text)
    commands = detect_change_word(text)

    if commands:
        # Wait for subscriber to be ready to receive messages
        while change_word_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        rospy.loginfo(f"Publishing command: {CHANG_WORD} to {word}")
        change_word_pub.publish(word)
    else:
        rospy.loginfo("No commands detected.")


if __name__ == '__main__':
    try:
        text = "I've completely mastered writing this particular word. Please change another word."
        command_publisher(text)
    except rospy.ROSInterruptException:
        pass
