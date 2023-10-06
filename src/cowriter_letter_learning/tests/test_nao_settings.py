#!/usr/bin/env python3
# coding: utf-8

"""
This file contains unit tests for the NaoSettings and PhraseManager 
classes found in the nao_settings module.

The NaoSettings class manages Nao's settings, such as language, 
handedness, and phrases, while the PhraseManager class manages 
Nao's phrases in various languages.

In some of the test functions, the @patch decorator is used to 
replace the rospy.get_param function with a MagicMock object, 
_get_param_mock. When the MagicMock object is used in place of the 
rospy.get_param function, it returns a predefined value ('english' or 
'right') instead of accessing the actual ROS parameters. This prevents 
any side effects from calling the actual rospy.get_param function 
during testing and ensures that the tests are  isolated from the ROS 
parameter server.
"""

import os
import rospy
import rostest
import sys
import unittest
from unittest.mock import MagicMock, patch
sys.path.insert(
    0, 
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from nao_settings import PhraseManager, PhraseManagerGPT, NaoSettings


class TestPhraseManager(unittest.TestCase):
    """
    Test suite for the PhraseManager class, which manages Nao's phrases
    in various languages.
    
    Only testing for English phrases for now.
    """

    def setUp(self):
        """
        Set up new PhraseManager for each test.
        """
        self.phrase_manager = PhraseManager('english')

    def tearDown(self):
        """
        Delete PhraseManager instance after each test.
        """
        del self.phrase_manager
    
    def test_phrase_manager_init(self):
        """
        Test the initialization of the PhraseManager class and verify
        that the phrases are set correctly.
        """
        self.assertEqual(self.phrase_manager.intro_phrase, 
                         "Hello. I'm Nao. Please show me a word to practice.")
        self.assertEqual(self.phrase_manager.test_phrase, 
                         "Ok, test time. I'll try my best.")
        self.assertEqual(self.phrase_manager.thank_you_phrase, 
                         'Thank you for your help.')
        
    def test_response_counters(self):
        """
        Test the response counters in the PhraseManager class to ensure
        they are initialized correctly.
        """
        self.assertEqual(self.phrase_manager.demo_response_phrases_counter,
                         self.phrase_manager.word_response_phrases_counter)
        self.assertEqual(self.phrase_manager.demo_response_phrases_counter, 0)
        self.assertEqual(len(self.phrase_manager.demo_response_phrases), 6)
        
    def test_phrase_lists(self):
        """
        Test the phrase lists in the PhraseManager class to ensure they
        contain the correct phrases.
        """
        self.assertEqual(self.phrase_manager.demo_response_phrases[
            self.phrase_manager.demo_response_phrases_counter], 
                         "Okay, I'll try it like you")
        
class TestPhraseManagerGPT(unittest.TestCase):
    """
    Class to test PhraseManagerGPT class
    """
    def setUp(self):
        self.gpt = PhraseManagerGPT('english')

    def tearDown(self):
        del self.gpt

    def test_init(self):
        initial_message = self.gpt.messages[0]['content']
        self.assertEqual(initial_message, 
            "You are a handwriting student of a child.")

    def test_get_gpt_response(self):
        response = self.gpt.get_gpt_response('Hi chatgpt,' +
            'say something philosophical, begin with "Hi Aristotle"')
        self.assertIsInstance(response, str)
        response = self.gpt.get_gpt_response('Hi chatgpt,' +
            'say something philosophical, begin with "Hi Aristotle"')
        self.assertIsInstance(response, str)
        response = self.gpt.get_gpt_response('Hi chatgpt,' +
            'say something philosophical, begin with "Hi Aristotle"')
        self.assertIsInstance(response, str)
        response = self.gpt.get_gpt_response('Hi chatgpt,' +
            'say something philosophical, begin with "Hi Aristotle"')
        self.assertIsInstance(response, str)

class TestSpeechTranscriber(unittest.TestCase):
    """
    Class to test SpeechTranscriber.
    """
    def setUp(self):
        self.transcriber = SpeechTranscriber('english')

    def tearDown(self):
        del self.transcriber

    def test_init(self):
        self.assertEqual(self.transcriber.language, 'english')

class TestAudioProcessor(unittest.TestCase):
    """
    Class to test AudioProcessor class.
    """
    def setUp(self):
        self.audio = AudioProcessor('english')

    def tearDown(self):
        del self.audio

    def test_audio_processor_thread(self):
        """
        Test thread will start and stop.
        """
        # Start the thread
        self.audio.start()

        # Allow some time for the thread to start
        time.sleep(2)

        # Check that the thread is running
        self.assertTrue(self.audio.is_alive())

        # Stop the thread and allow some time to stop
        self.audio.stop()
        time.sleep(2)

        # Check that the thread has stopped
        self.assertFalse(self.audio.is_alive())

class TestNaoSettings(unittest.TestCase):
    """
    Test suite for the NaoSettings class, which manages Nao's settings,
    such as language, handedness, and phrases.
    """
    
    def get_param_mock(param_name, default=None):
        """
        Function to use in patch to return correct mock values.
        """
        if default == None:
            default = True
        if param_name == '~language':
            return 'english'
        elif param_name == '~handedness':
            return 'right'
        return default
        
    @patch('nao_settings.rospy.get_param', side_effect=get_param_mock)
    def setUp(self, _get_param_mock):
        """
        Generate NaoSettings instance for each test.
        """
        self.nao_settings = NaoSettings()

    def tearDown(self):
        """
        Delete NaoSettings instance after each test.
        """
        del self.nao_settings
    
    def test_nao_settings_init(self):
        """
        Test the initialization of the NaoSettings class and verify
        that the settings are set correctly.
        """
        self.assertEqual(self.nao_settings.LANGUAGE, 'english')
        self.assertEqual(self.nao_settings.nao_connected, True)
        self.assertEqual(self.nao_settings.nao_speaking, True)
        self.assertEqual(self.nao_settings.nao_writing, True)

    def test_set_effector(self):
        """
        Test the set_effector method of the NaoSettings class, which
        sets Nao's writing hand.
        """
        # set_effector called in __init__, no need to call explicitly
        self.assertEqual(self.nao_settings.effector, 'RArm')

        self.nao_settings.NAO_HANDEDNESS = 'left'
        # Need to call explicitly this time because handedness changed
        self.nao_settings.set_effector()
        self.assertEqual(self.nao_settings.effector, 'LArm')

    def test_set_phrase_manager(self):
        """
        Test the set_phrase_manager method of the NaoSettings class,
        which initialises a PhraseManager instance.
        """
        # Method called in __init__, no need to call explicitly
        self.assertIsInstance(self.nao_settings.phrase_manager, PhraseManager)

    def test_set_orientation_params(self):
        """
        Test the set_orientation_params method of the NaoSettings
        class.
        """
        # Method called in __init__, no need to call explicitly
        self.assertEqual(self.nao_settings.person_side, 'right')
        self.assertEqual(self.nao_settings.alternate_sides_looking_at, False)
        self.assertEqual(self.nao_settings.next_side_to_look_at, 'Right')
        self.assertEqual(self.nao_settings.head_angles_look_at_tablet_down,
                         (-0.01538, 0.512))


if __name__ == '__main__':
    rospy.init_node('test_nao_settings')
    rostest.rosrun('letter_learning_interaction', 'test_nao_settings',
                   TestPhraseManager, sys.argv)
    rostest.rosrun('letter_learning_interaction', 'test_nao_settings',
                   TestPhraseManagerGPT, sys.argv)
    rostest.rosrun('letter_learning_interaction', 'test_nao_settings',
                   TestNaoSettings, sys.argv)

