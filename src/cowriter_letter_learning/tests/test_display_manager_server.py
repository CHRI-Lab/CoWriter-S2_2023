#!/usr/bin/env python3

import os
import rospy
import rostest
import sys
import unittest

from letter_learning_interaction.srv import *
from geometry_msgs.msg import Point

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../nodes"))

from display_manager_server import *


class TestDisplayManager(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_display_manager", anonymous=True)

    def test_handle_clear_all_shapes(self):
        """
        Test handle_clear_all_shapes function.

        """
        request = ClearAllShapesRequest()
        response = handle_clear_all_shapes(request)
        # Check correct response type received and success registered
        self.assertIsInstance(response, ClearAllShapesResponse)
        self.assertTrue(response.success.data)

    def test_handle_display_new_shape(self):
        """
        Test handle_display_new_shape function
        """
        request = DisplayNewShapeRequest()
        request.shape_type_code = 0
        response = handle_display_new_shape(request)
        # Check response type is correct and point returned
        self.assertIsInstance(response, DisplayNewShapeResponse)
        self.assertIsInstance(response.location, Point)

    def test_handle_index_of_location(self):
        request = IndexOfLocationRequest(location=Point(x=1, y=2))
        response = handle_index_of_location(request)
        self.assertIsInstance(response, IndexOfLocationResponse)
        self.assertIsInstance(response.row, int)
        self.assertIsInstance(response.column, int)

    def test_handle_shape_at_location(self):
        request = ShapeAtLocationRequest(location=Point(0, 0, 0))
        response = handle_shape_at_location(request)
        self.assertIsInstance(response, ShapeAtLocationResponse)
        self.assertIsInstance(response.shape_type_code, int)
        self.assertIsInstance(response.shape_id, int)

    def test_handle_closest_shapes_to_location(self):
        request = ClosestShapesToLocationRequest(location=Point(1, 1, 1))
        response = handle_closest_shapes_to_location(request)
        self.assertIsInstance(response, ClosestShapesToLocationResponse)
        self.assertIsInstance(response.shape_type_code[0], int)
        self.assertIsInstance(response.shape_id[0], int)

    def test_handle_possible_to_display(self):
        request = IsPossibleToDisplayNewShapeRequest()
        request.shape_type_code = 0
        response = handle_possible_to_display(request)
        self.assertIsInstance(response, IsPossibleToDisplayNewShapeResponse)
        self.assertTrue(response.is_possible.data)

    def test_handle_display_shape_at_location(self):
        request = DisplayShapeAtLocationRequest(
            shape_type_code=0, location=Point(x=1, y=1)
        )
        response = handle_display_shape_at_location(request)
        self.assertIsInstance(response, DisplayShapeAtLocationResponse)
        self.assertIsInstance(response.success.data, bool)


if __name__ == "__main__":
    rostest.rosrun(
        "letter_learning_interaction",
        "test_display_manager_server",
        TestDisplayManager,
        sys.argv,
    )
