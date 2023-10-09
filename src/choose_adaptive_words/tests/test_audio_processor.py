#!/usr/bin/env python3


import os
import rospy
import rostest
import sys
import threading
import unittest

from choose_adaptive_words.audio_processor import (
    AudioProcessor,
    SpeechTranscriber,
)


class TestSpeechTranscriber(unittest.TestCase):
    """
    Class to test SpeechTranscriber.
    """

    def setUp(self):
        self.transcriber = SpeechTranscriber("english")

    def tearDown(self):
        del self.transcriber

    def test_init(self):
        self.assertEqual(self.transcriber.language, "english")


class TestAudioProcessor(unittest.TestCase):
    """
    Class to test AudioProcessor class.
    """

    def setUp(self):
        self.audio = AudioProcessor("english")
        self.audio.record_file = os.getcwd() + "/record.wav"

        # Remove the file if it exists
        if os.path.exists(self.audio.record_file):
            os.remove(self.audio.record_file)

    def tearDown(self):
        self.audio.stop()
        del self.audio

    def test_audio_recording(self):
        """
        Test that the audio recording is saved to a file.
        """
        # Run the audio processing in a separate thread
        thread = threading.Thread(target=self.audio.run)
        thread.start()

        # Sleep for 10s to ensure recording has time to start and stop
        rospy.sleep(10)

        # Stop the audio processing
        self.audio.stop()

        # Join the thread
        thread.join()

        # Assert the file exists
        self.assertTrue(os.path.exists(self.audio.record_file))

        # Remove the file after assertion
        if os.path.exists(self.audio.record_file):
            os.remove(self.audio.record_file)


if __name__ == "__main__":
    rospy.init_node("test_audio_processor")
    rostest.rosrun(
        "letter_learning_interaction",
        "test_audio_processor",
        TestSpeechTranscriber,
        sys.argv,
    )
    rostest.rosrun(
        "letter_learning_interaction",
        "test_audio_processor",
        TestAudioProcessor,
        sys.argv,
    )
