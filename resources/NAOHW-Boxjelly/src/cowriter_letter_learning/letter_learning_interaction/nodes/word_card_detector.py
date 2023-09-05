#!/usr/bin/env python3
"""
word_detector.py

This script detects and publishes words based on fiducial markers
(chilitags) detected in the camera frame. It listens to the transform
information of tags, processes them to extract words, and publishes
these words to specific topics. It also handles special_tags, which
have specific meanings or actions associated with them.

The script utilizes the tf.TransformListener to listen to the tag's
position and orientation in the camera frame. It sorts detected letters
based on their x-coordinate to form words, which are then published.
"""


import rospy
import tf
from std_msgs.msg import String, Empty
# import operator
from functools import cmp_to_key
from typing import Dict, Set, Tuple

special_tags: Dict[str, str] = {
    'tag_17': 'test',
    'tag_302': 'stop',
    'tag_300': 'next',
    'tag_301': 'prev',
    'tag_430': 'help',
    'tag_341': 'go'
}

tags_words_mapping: Dict[str, str] = {
    'tag_5': 'cow',
    'tag_6': 'son',
    'tag_7': 'cue',
    'tag_8': 'new',
    'tag_9': 'use',
    'tag_10': 'cou',
    'tag_11': 'son',
    'tag_12': 'ces',
    'tag_13': 'une',  # no tag_14: too many false positives!
    'tag_15': 'nos',
    'tag_16': 'ose',
    'tag_18': 'eau'
}

tags_letters_mapping: Dict[str, str] = {}

# Add individual letters: tag IDs are the ASCII code of the letter
for char in range(ord('a'), ord('z') + 1):
    tags_letters_mapping["tag_%d" % char] = chr(char)


def compare(l1: Tuple[str, float], l2: Tuple[str, float]) -> int:
    '''
    Compare two tuples (label, value) based on their values.

    Note in the code below this is going to be used to sort
    words based on the x-coordinate of their translation.

    Args:
    l1: A tuple with label and value (str, int)
    l2: A tuple with label and value (str, int)

    Returns:
    1 if the value of l1 is less than l2, else -1.
    '''
    _, x1 = l1
    _, x2 = l2
    return 1 if x1 < x2 else -1


def last_seen_since(frame: str, tf_listener: tf.TransformListener, camera_frame: str) -> float:
    '''
    Calculate the time since the given frame was last seen.

    Args:
    frame: The frame label as a string.

    Returns:
    The time elapsed since the frame was last seen in seconds.
    If the frame has never been seen, returns 10000.
    '''
    try:
        return (rospy.Time().now() - tf_listener.getLatestCommonTime(camera_frame, frame)).to_sec()
    except tf.Exception:
        # card *never seen*
        return 10000


if __name__ == "__main__":
    rospy.init_node("word_detector")

    # Get parameters or set defaults for different topics and frames
    words_topic: str = rospy.get_param(
        '~detected_words_topic', 'words_to_write')
    special_topic: str = rospy.get_param(
        '~special_cards_topic', 'special_symbols')
    stop_topic: str = rospy.get_param(
        '~stop_card_detected_topic', 'stop_learning')
    test_topic: str = rospy.get_param(
        '~test_card_detected_topic', 'test_learning')
    camera_frame: str = rospy.get_param('~detector_frame_id', 'camera_frame')
    camera_frame = 'v4l_frame'

    language: str = rospy.get_param('~language', 'english')

    # Create publishers for different topics
    pub_words = rospy.Publisher(words_topic, String, queue_size=10)
    pub_special = rospy.Publisher(special_topic, String, queue_size=10)
    pub_stop = rospy.Publisher(stop_topic, Empty, queue_size=10)
    pub_test = rospy.Publisher(test_topic, Empty, queue_size=10)

    # Initialize the TransformListener
    tf_listener = tf.TransformListener(True, rospy.Duration(nsecs=500000000))
    rospy.sleep(0.5)
    rate = rospy.Rate(10)
    prev_word: str = ''
    word_to_publish: str = ''
    go_card_last_seen: float = 0.0  # in seconds

    while not rospy.is_shutdown():

        # Wait for the go card to appear
        while not rospy.is_shutdown():
            go_card_last_seen = last_seen_since(
                "tag_341", tf_listener, camera_frame)
            if go_card_last_seen < 0.1:
                break

        if rospy.is_shutdown():
            break

        rospy.loginfo("Got a 'GO' card! preparing a word to publish")

        # Keep track of detected letters
        letters_detected: Set[Tuple[str, float]] = set()

        # Iterate through the tags_letters_mapping dictionary
        for tag, letter in tags_letters_mapping.items():

            try:
                # Get the latest common time for the tag and camera_frame
                common_time = tf_listener.getLatestCommonTime(
                    tag, camera_frame)

                # Check if the time difference is less than 0.3 seconds
                if (rospy.Time().now() - common_time).to_sec() < 0.3:

                    try:
                        # Get the transform (translation and rotation) between the tag and camera_frame
                        translation, rotation = tf_listener.lookupTransform(
                            tag, camera_frame, common_time)

                        # Check if the tag is facing the camera (based on rotation values)
                        if rotation[2] - rotation[3] > 0:
                            # The tag is not facing the camera, so skip to the next iteration
                            continue

                    except tf.ExtrapolationException:
                        # If an extrapolation exception occurs, continue to the next iteration
                        continue

                    # Add the detected letter and its x-coordinate in the camera frame to the letters_detected set
                    letters_detected.add((letter, translation[0]))

            except tf.Exception:
                # If the tag has not been seen yet, continue to the next iteration
                continue

        # If no letters were detected, log a warning message
        if not letters_detected:
            rospy.logwarn("Got a 'GO' card, but unable to find any letter!")
        else:
            # Sort the detected letters based on their x-coordinate in the camera frame
            sorted_letters = sorted(letters_detected, key=cmp_to_key(compare))

            # Create the word to publish by joining the sorted letters
            word_to_publish = ''.join([l for l, _ in sorted_letters])

            # Check if the word to publish is the same as the previous word
            if word_to_publish in prev_word:
                rospy.loginfo(
                    f"I'm not publishing '{word_to_publish}' since it is still the same word (or part thereof).")
            else:
                rospy.loginfo(f'Publishing word: {word_to_publish}')

                # Create a String message and publish the word
                message = String()
                message.data = word_to_publish
                pub_words.publish(message)

                # Reset the tags_detected set
                tags_detected: Set[str] = set()

                # Update the previous word
                prev_word = word_to_publish

            # Log a debug message and sleep for 1 second to clear the tf cache
            rospy.logdebug("Sleeping a bit to clear the tf cache...")
            rospy.sleep(1)  # Wait till the tag times out (in the use context
            # it's not likely to get two words within 5s)
            rospy.logdebug("Ok, waiting for a new word")

        rate.sleep()
