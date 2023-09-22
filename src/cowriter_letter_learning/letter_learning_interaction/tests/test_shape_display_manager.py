#!/usr/bin/env python3
"""
test_display_manager.py

This script contains a set of unit tests for the ShapeDisplayManager 
class found in the shape_display_manager.py module. The tests cover 
various functionalities provided by the ShapeDisplayManager class, 
ensuring correct behavior and performance.
"""


import numpy as np
import os
import rospy
import rostest
import sys
import unittest
sys.path.insert(
    0, 
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from shape_display_manager import ShapeDisplayManager


class TestShapeDisplayManager(unittest.TestCase):
    """
    Test class for ShapeDisplayManager, containing tests for various
    functionalities provided by the ShapeDisplayManager class.
    """

    def setUp(self):
        """
        Set up method that runs before each test case, creating an 
        instance of ShapeDisplayManager to be used in the tests.
        """
        self.manager = ShapeDisplayManager()
        
    def tearDown(self):
        """
        Tear down method that runs after each test case, cleaning up
        the ShapeDisplayManager instance.
        """
        del self.manager

    def test_clear_all_shapes(self):
        """
        Test method to check if the clear_all_shapes() method in
        ShapeDisplayManager works as expected, clearing all displayed 
        shapes.
        """
        self.manager.display_new_shape(0)
        self.manager.clear_all_shapes()
        self.assertTrue(np.all(np.isnan(self.manager.shapes_drawn)))

    def test_display_new_shape(self):
        """
        Test method to check if the display_new_shape() method in
        ShapeDisplayManager works as expected, displaying a new shape 
        and returning the position of the new shape.

        Coordinates of shape are not known a priori, so just checks 
        that it returns a tuple of two floats.
        """
        position = self.manager.display_new_shape(0)
        self.assertIsInstance(position, list)
        self.assertEqual(len(position), 2)
        self.assertIsInstance(position[0], float)
        self.assertIsInstance(position[1], float)

    def test_is_possible_to_display_new_shape(self):
        """
        Test method to check if the is_possible_to_display_new_shape()
        method in ShapeDisplayManager works as expected, determining if
        it is possible to display a new shape given the current state 
        of displayed shapes.
        """
        self.assertTrue(self.manager.is_possible_to_display_new_shape(0))
        self.manager.shapes_drawn.fill(0)
        self.assertFalse(self.manager.is_possible_to_display_new_shape(0))

    def test_index_of_location(self):
        """
        Test method to check if the index_of_location() method in
        ShapeDisplayManager works as expected, returning the row and 
        column indices corresponding to a given location.

        Indices of position are not known a priori, so just checks that
        a pair of ints are returned.
        """
        position = [0.02, 0.093]
        row, column = self.manager.index_of_location(position)
        self.assertIsInstance(row, int)
        self.assertIsInstance(column, int)

    def test_shape_at_location(self):
        """
        Test method to check if the shape_at_location() method in
        ShapeDisplayManager works as expected, returning the shape type
        code and shape ID for the shape at a given location.
        """
        shape_type_code, shape_id = self.manager.shape_at_location([
                                        0.02, 0.093])
        self.assertEqual(shape_type_code, 0)
        self.assertEqual(shape_id, -1)

    def test_closest_shapes_to_location(self):
        """
        Test method to check if the closest_shapes_to_location() method
        in ShapeDisplayManager works as expected, returning the shape 
        type codes and shape IDs for the closest shapes to a given 
        location.
        """
        shape_type_codes, shape_ids = \
            self.manager.closest_shapes_to_location([0.02, 0.093])
        self.assertEqual(shape_type_codes, [0])
        self.assertEqual(shape_ids, [-1])

    def test_display_shape_at_location(self):
        """
        Test method to check if the display_shape_at_location() method 
        in ShapeDisplayManager works as expected, displaying a shape at
        a specified location and returning whether the operation was 
        successful.
        """
        success = self.manager.display_shape_at_location(0, [0.02, 0.093])
        self.assertTrue(success)


if __name__ == '__main__':
    rospy.init_node('test_shape_display_manager')
    rostest.rosrun('letter_learning_interaction', 'test_shape_display_manager',
                   TestShapeDisplayManager, sys.argv)
