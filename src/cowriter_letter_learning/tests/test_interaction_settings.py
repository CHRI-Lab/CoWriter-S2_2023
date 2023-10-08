#!/usr/bin/env python3
"""
This module contains unit tests for the InteractionSettings class in 
the interaction_settings.py. It tests the various methods of the 
InteractionSettings class, including generate_settings(), 
set_dataset_directory(), get_trajectory_timings(), get_head_angles(), 
and get_phrases().
"""

import os
import rospy
import rostest
import sys
import unittest
from unittest.mock import patch
sys.path.insert(
    0, 
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from interaction_settings import InteractionSettings, LearningModes


class TestLearningModes(unittest.TestCase):
    """
    Unit tests for the LearningModes class in the interaction_settings.py.
    """

    def test_enum_values(self):
        """
        Tests that the LearningModes enumeration contains the correct 
        values.
        """
        self.assertEqual(LearningModes.starts_good.value, 0)
        self.assertEqual(LearningModes.starts_bad.value, 1)
        self.assertEqual(LearningModes.starts_random.value, 2)


class TestInteractionSettings(unittest.TestCase):
    """
    Unit tests for the InteractionSettings class in the 
    interaction_settings.py.
    """

    def test_generate_settings(self):
        """
        Tests the generate_settings() method of the InteractionSettings
        class.
        """
        # Test that RuntimeError raised when dataset_directory is None
        with self.assertRaises(RuntimeError):
            InteractionSettings.generate_settings('c')

        # Mock the dataset_directory so that dataset files can be found
        with patch('interaction_settings.InteractionSettings.dataset_directory',
                   '/path/to/dataset'):
            # Test RuntimeError raised if no dataset for shape
            with self.assertRaises(RuntimeError):
                InteractionSettings.generate_settings('unknown_shape')

    def test_set_dataset_directory(self):
        """
        Tests the set_dataset_directory() method of the 
        InteractionSettings class.
        """
        # Test that dataset_directory is None by default
        self.assertIsNone(InteractionSettings.dataset_directory)

        # Test that dataset_directory can be set correctly
        InteractionSettings.set_dataset_directory('/path/to/dataset')
        self.assertEqual(InteractionSettings.dataset_directory, 
                         '/path/to/dataset')

    def test_get_trajectory_timings(self):
        """
        Tests the get_trajectory_timings() method of the 
        InteractionSettings class.
        """
        # Test function returns correct timings for nao_writing=True
        t0, dt, delay_before_executing = \
            InteractionSettings.get_trajectory_timings(True)
        self.assertEqual(t0, 3.0)
        self.assertEqual(dt, 0.25)
        self.assertEqual(delay_before_executing, 3.0)

        # Test function returns correct timings for nao_writing=False
        t0, dt, delay_before_executing = \
            InteractionSettings.get_trajectory_timings(False)
        self.assertEqual(t0, 0.01)
        self.assertEqual(dt, 0.1)
        self.assertEqual(delay_before_executing, 2.5)

    def test_get_head_angles(self):
        """
        Tests the get_head_angles() method of the InteractionSettings 
        class.
        """
        # Test that the function returns the correct head angles
        head_angles = InteractionSettings.get_head_angles()
        self.assertEqual(head_angles[0], (-0.01538, 0.512))
        self.assertEqual(head_angles[1], (-0.2, 0.08125996589660645))
        self.assertEqual(head_angles[2], (0.2, 0.08125996589660645))
        self.assertEqual(head_angles[3], (-0.0123, 0.1825))
        self.assertEqual(head_angles[4], (-0.9639739513397217, 
                                          0.08125996589660645))
        self.assertEqual(head_angles[5], (0.9639739513397217, 
                                          0.08125996589660645))

    def test_get_phrases(self):
        """
        Tests the get_phrases function of the InteractionSettings 
        class.
        """
        # Test the phrases for English
        (intro_phrase, demo_response_phrases, asking_phrases_after_feedback,
         asking_phrases_after_word, word_response_phrases, 
         word_again_response_phrases, test_phrase, 
         thank_you_phrase) = InteractionSettings.get_phrases(
            'english')
         
        self.assertIsInstance(intro_phrase, str)
        self.assertIsInstance(test_phrase, str)
        self.assertIsInstance(thank_you_phrase, str)
        self.assertIsInstance(demo_response_phrases, list)
        self.assertIsInstance(asking_phrases_after_feedback, list)
        self.assertIsInstance(asking_phrases_after_word, list)
        self.assertIsInstance(word_response_phrases, list)
        self.assertIsInstance(word_again_response_phrases, list)

        self.assertEqual(intro_phrase, 
                         "Hello. I'm Nao. Please show me a word to practice.")
        self.assertEqual(test_phrase, "Ok, test time. I'll try my best.")
        self.assertEqual(thank_you_phrase, "Thank you for your help.")
        self.assertIn("Okay, I'll try it like you", demo_response_phrases)
        self.assertIn("Any better?", asking_phrases_after_feedback)
        self.assertIn("Okay, what do you think?", asking_phrases_after_word) 

        # Test the phrases for French
        (intro_phrase, demo_response_phrases, asking_phrases_after_feedback, 
         asking_phrases_after_word, word_response_phrases, 
         word_again_response_phrases, test_phrase, 
         thank_you_phrase) = InteractionSettings.get_phrases(
            'french')

        self.assertIsInstance(intro_phrase, str)
        self.assertIsInstance(test_phrase, str)
        self.assertIsInstance(thank_you_phrase, str)
        self.assertIsInstance(demo_response_phrases, list)
        self.assertIsInstance(asking_phrases_after_feedback, list)
        self.assertIsInstance(asking_phrases_after_word, list)
        self.assertIsInstance(word_response_phrases, list)
        self.assertIsInstance(word_again_response_phrases, list)

        self.assertEqual(intro_phrase, "Allez, on écrit des mots")
        self.assertIn("D'accord, j'essaye comme ça", demo_response_phrases)
        self.assertIn("C'est mieux ?", asking_phrases_after_feedback)
        self.assertIn("Bon, qu'est ce que tu en penses ?", 
                      asking_phrases_after_word)
        self.assertIn("D'accord pour %s", word_response_phrases)


if __name__ == '__main__':
    rospy.init_node('test_interaction_settings')
    rostest.rosrun('letter_learning_interaction', 'test_interaction_settings',
                   TestLearningModes, sys.argv)
    rostest.rosrun('letter_learning_interaction', 'test_interaction_settings',
                   TestInteractionSettings, sys.argv)
