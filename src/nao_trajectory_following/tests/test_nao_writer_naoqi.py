#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Test script for the trajectory listener
# ROS package: nao_writer_naoqi.py
#
# Created by Chang Shen on 26/04/2023
#
# This script tests the following functionalities:
# - traj_to_path
#
# To run this test, type:
# $ python nao_writer_naoqi_test.py
import rostest
import math
import unittest
from unittest.mock import Mock
import rospy
import os
import tf
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_msgs.msg import Header

import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../nodes'))

class TestTrajectoryListener(unittest.TestCase):

    def traj_to_path(self, points):
        arm_len = 0.323
        converted_pts = []
        corner_min = [0.12, 0.03, 0.32]
        corner_max = [0.12, -0.18, 0.47]
        #limit the coordinate range for writing
        for point in points:#Convert the writing trajectory of the X and Y axes into a writing trajectory perpendicular to the ground
            y = point[0]
            z = point[1]
            converted_pts.append([0, y, z, point[3], point[4], point[5]])
        
        pt_min = [0, min([point[1] for point in converted_pts]), min([point[2] for point in converted_pts])]
        pt_max = [0, max([point[1] for point in converted_pts]), max([point[2] for point in converted_pts])]

        normal_factor = [0, 
                         (corner_max[1] - corner_min[1]) / (pt_max[1] - pt_min[1]),
                         (corner_max[2] - corner_min[2]) / (pt_max[2] - pt_min[2])]
        
        for i, point in enumerate(converted_pts):
            y = (point[1] - pt_min[1]) * normal_factor[1] + corner_min[1]
            z = (point[2] - pt_min[2]) * normal_factor[2] + corner_min[2]
            #The length of the robot's arm is limited, and it needs to adjust the X-axis coordinate size according to the writing position
            x = math.sqrt(max([0.12 - (y + 0.07) * (y + 0.07) - z * z, 0.1])) * 1.2 - 0.21
            converted_pts[i] = [x, y, z, point[3], point[4], point[5]]

        return converted_pts
    
    def test_traj_to_path(self):
        points = [
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            [2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            [3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        ]

        expect_points = [[0.1694733192202055, 0.03, 0.32, 4.0, 5.0, 6.0],
                         [0.1694733192202055, -0.075, 0.395, 5.0, 6.0, 7.0],
                         [0.1694733192202055, -0.18, 0.47, 6.0, 7.0, 8.0]]
        actual_result = self.traj_to_path(points)
        self.assertEqual(actual_result, expect_points)


if __name__ == '__main__':
    rospy.init_node('test')
    rostest.run('nao_writer_naoqi', 'nao_writer_naoqi_test', TestTrajectoryListener, sys.argv)
