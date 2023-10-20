#!/usr/bin/env python3
"""
test_shape_counter.py

This script contains a set of unit tests for the ShapeCounter class in 
the shape_counter.py script. It tests the on_word, on_user_traj, and 
on_robot_traj methods.
"""

import unittest
import csv
import io
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped, Point
import os
import sys
import rospy
import rostest

sys.path.insert(
    0, 
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '../scripts/'))
from count_shapes import ShapeCounter


class TestShapeCounter(unittest.TestCase):
    """
    A test class for the ShapeCounter class in count_shapes.py
    """

    def setUp(self):
        """
        Set up for each test case
        """
        # Create csv file buffers
        self.csvfile = io.StringIO()
        self.csvfile_letters = io.StringIO()

        # Create csv writers
        writer = csv.writer(self.csvfile, delimiter=' ',
                            quoting=csv.QUOTE_MINIMAL)
        writer_letters = csv.writer(
            self.csvfile_letters, delimiter=' ', quoting=csv.QUOTE_MINIMAL)

        # Create ShapeCounter instance
        self.shape_counter = ShapeCounter(writer, writer_letters)

    def tearDown(self):
        """
        Tear down for each test case
        """
        # Close csv file buffers
        self.csvfile.close()
        self.csvfile_letters.close()

    def test_on_word(self):
        """
        Test case for ShapeCounter.on_word method
        """
        # Record initial number of words
        initial_number_of_words = self.shape_counter.number_of_words

        # Send two String messages to ShapeCounter instance
        self.shape_counter.on_word(String(data='hello'))
        self.assertEqual(self.shape_counter.number_of_words,
                         initial_number_of_words + 1)

        self.shape_counter.on_word(String(data='end'))
        self.assertEqual(self.shape_counter.number_of_words,
                         initial_number_of_words + 1)

    def create_pose(self, x, y):
        """Helper method for creating PoseStamped instances"""
        pose = PoseStamped()
        pose.pose.position = Point(x, y, 0)
        return pose

    def test_on_user_traj(self):
        """Test case for ShapeCounter.on_user_traj method"""
        # Record initial number of user shapes
        initial_user_shapes = self.shape_counter.number_of_user_shapes

        # Create a trajectory message with two points
        traj = Path()
        traj.poses = [self.create_pose(0, 0), self.create_pose(1, 1)]
        self.shape_counter.on_user_traj(traj)

        # Number of user shapes should not change
        self.assertEqual(
            self.shape_counter.number_of_user_shapes, initial_user_shapes)

        # Send an empty trajectory message
        empty_traj = Path()
        self.shape_counter.on_user_traj(empty_traj)

        # Number of user shapes should increase by 1
        self.assertEqual(
            self.shape_counter.number_of_user_shapes, initial_user_shapes + 1)

    def test_on_robot_traj(self):
        """
        Test case for ShapeCounter.on_robot_traj method
        This test method checks whether the ShapeCounter.on_robot_traj
        method correctly increments the number of robot shapes counted 
        by the ShapeCounter instance when a new robot trajectory is 
        received.
        """
        # Get the initial count of robot shapes and corrections
        initial_robot_shapes = self.shape_counter.number_robot_shapes
        initial_corrections = \
            self.shape_counter.number_corrections_responded_to_this_word

        # Create an empty trajectory message
        traj = Path()
        self.shape_counter.on_robot_traj(traj)

        # Check whether the number of robot shapes has been incremented
        self.assertEqual(self.shape_counter.number_robot_shapes,
                         initial_robot_shapes + 1)
                         
        # Check whether corrections has been incremented
        self.assertEqual(
            self.shape_counter.number_corrections_responded_to_this_word,
            initial_corrections + 1)
            
    def test_on_test_request_received(self):
        """
        Test on_test_request_received method.
        """
        # Change number of words before test
        self.shape_counter.number_of_words_before_test = 1
        
        # Pass method Empty test request message
        msg = Empty()
        self.shape_counter.on_test_request_received(msg)
        
        # Check number words before test reset to 0
        # (0 will be number for number_of_words)
        self.assertEquals(self.shape_counter.number_of_words_before_test, 0)


if __name__ == '__main__':
    rospy.init_node('test_shape_counter')
    rostest.rosrun('letter_learning_interaction', 'test_shape_counter',
                   TestShapeCounter, sys.argv)
