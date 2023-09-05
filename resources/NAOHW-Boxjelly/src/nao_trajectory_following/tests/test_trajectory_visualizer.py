#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Test script for the trajectory listener
# ROS package: trajectory_visualizer_test.py
#
# Created by Chang Shen on 26/04/2023
#
# This script tests the following functionalities:
# - test_visualize_traj_single_shape
# - test_visualize_traj_multiple_shapes
#
# To run this test, type:
# $ python trajectory_visualizer_test.py
import unittest
import rostest
from unittest.mock import Mock
import sys
import time
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point
import os 
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../nodes'))
from trajectory_visualizer import visualize_traj, on_traj

class TestVisualizeTraj(unittest.TestCase):
    def test_visualize_traj_single_shape(self):
        # Arrange
        points = [(1, 2, 3), (4, 5, 6), (7, 8, 9)]
        FRAME = "frame"
        WRITE_MULTIPLE_SHAPES = False
        shapeCount = 0
        # Act
        pub_markers = Mock()
        visualize_traj(points, FRAME, WRITE_MULTIPLE_SHAPES, shapeCount, pub_markers)

        # Assert
        pub_markers.publish.assert_called_once()
        traj = pub_markers.publish.call_args[0][0]
        self.assertEqual(traj.header.frame_id, FRAME)
        self.assertEqual(traj.ns, "writing_traj")
        self.assertEqual(traj.action, Marker.ADD)
        self.assertEqual(traj.pose.orientation.w, 1.0)
        self.assertEqual(traj.type, Marker.LINE_STRIP)
        self.assertEqual(traj.scale.x, 0.01)
        self.assertEqual(traj.color.r, 1.0)
        self.assertEqual(traj.color.b, 0.0)
        self.assertEqual(traj.color.a, 1.0)
        self.assertEqual(traj.id, 0)
        self.assertEqual(traj.lifetime.secs, 1)
        self.assertEqual(list(traj.points), points)

    def test_visualize_traj_multiple_shapes(self):
        # Arrange
        points1 = [(1, 2, 3), (4, 5, 6), (7, 8, 9)]
        points2 = [(11, 12, 13), (14, 15, 16), (17, 18, 19)]
        FRAME = "frame"
        WRITE_MULTIPLE_SHAPES = True
        shapeCount = 0
        # Act
        pub_markers = Mock()
        visualize_traj(points1, FRAME, WRITE_MULTIPLE_SHAPES, shapeCount, pub_markers)
        traj1 = pub_markers.publish.call_args[0][0]

        shapeCount += 1
        visualize_traj(points2, FRAME, WRITE_MULTIPLE_SHAPES, shapeCount, pub_markers)
        traj2 = pub_markers.publish.call_args[0][0]

        # Assert
        self.assertEqual(traj1.id, 0)
        self.assertEqual(list(traj1.points), points1)
        self.assertEqual(traj2.id, 1)
        self.assertEqual(list(traj2.points), points2)


if __name__ == '__main__':
    rospy.init_node("trajectory_visualizer");

    CLEAR_TOPIC = rospy.get_param('~clear_surface_topic','clear_screen')
    SHAPE_TOPIC = rospy.get_param('~trajectory_nao_topic','write_traj')
    MARKER_TOPIC = rospy.get_param('~visualization_output_topic','visualization_markers')
    FRAME = rospy.get_param('~writing_surface_frame_id','writing_surface')
    rostest.run('trajectory_visualizer', 'trajectory_visualizer_test', TestVisualizeTraj, sys.argv)

