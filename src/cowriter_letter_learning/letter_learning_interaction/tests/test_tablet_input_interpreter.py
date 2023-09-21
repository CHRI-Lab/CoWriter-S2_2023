#!/usr/bin/env python3

"""
test_tablet_input_interpreter.py

This script contains test cases for the TabletInputInterpreter class methods in the
tablet_input_interpreter ROS node. It tests the following methods:
- make_shape_message
- process_shape_longest_stroke
- process_shape_merge_strokes
- process_shape_first_stroke
"""

import os
import rospy
import rostest
import sys
import unittest
import numpy as np

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from letter_learning_interaction.msg import Shape as ShapeMsg

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../nodes'))
from tablet_input_interpreter import TabletInputInterpreter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from shape_learner_manager import ShapeLearnerManager, Shape


class TestTabletInputInterpreter(unittest.TestCase):
    """
    Test cases for TabletInputInterpreter class methods.
    """

    def setUp(self):
        """
        Set up the test case by creating a TabletInputInterpreter 
        instance.
        """
        self.interpreter = TabletInputInterpreter(
            rospy.Publisher("dummy_topic", ShapeMsg, queue_size=10))

    def test_make_shape_message(self):
        """
        Test the make_shape_message method by checking if the output 
        message is equal to the input object.
        """
        shape_object = Shape()
        shape_object.path = [0.0, 0.0, 1.0, 1.0]
        shape_object.shape_id = 1
        shape_object.shape_type = 'A'
        shape_object.shape_type_code = 0
        shape_object.params_to_vary = [0, 1]
        shape_object.param_values = [0.5, 0.5]
        rospy.loginfo('shape: %s', shape_object)

        shape_msg = TabletInputInterpreter.make_shape_message(shape_object)

        self.assertEqual(shape_msg.path, shape_object.path)
        self.assertEqual(shape_msg.shapeID, shape_object.shape_id)
        self.assertEqual(shape_msg.shape_type, shape_object.shape_type)
        self.assertEqual(shape_msg.shape_type_code,
                         shape_object.shape_type_code)
        self.assertEqual(shape_msg.params_to_vary, 
                         shape_object.params_to_vary)
        self.assertEqual(shape_msg.param_values, shape_object.param_values)

    def test_process_shape_longest_stroke(self):
        """
        Test the process_shape_longest_stroke method by checking if it
        returns the longest stroke.
        """
        self.interpreter.strokes = [
            np.array([[0, 0], [1, 1]]), np.array([[0, 0], [1, 1], [2, 2]])]
        longest_stroke = self.interpreter.process_shape_longest_stroke()
        np.testing.assert_array_equal(
            longest_stroke, np.array([[0, 0], [1, 1], [2, 2]]))

    def test_process_shape_first_stroke(self):
        """
        Test the process_shape_first_stroke method by checking if it
        returns the first stroke.
        """
        self.interpreter.strokes = [
            np.array([[0, 0], [1, 1]]), np.array([[0, 0], [1, 1], [2, 2]])]
        first_stroke = self.interpreter.process_shape_first_stroke()
        np.testing.assert_array_equal(first_stroke, 
                                      np.array([[0, 0], [1, 1]]))


if __name__ == '__main__':
    rostest.rosrun('letter_learning_interaction', 
                   'test_tablet_input_interpreter', 
                   TestTabletInputInterpreter, 
                   sys.argv)
