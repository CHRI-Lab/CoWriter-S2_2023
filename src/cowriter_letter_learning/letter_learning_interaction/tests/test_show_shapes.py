#!/usr/bin/env python3

"""
test_show_shapes.py

This script contains a set of unit tests for the show_shapes function in the
show_shapes.py script. It tests the function's ability to display shapes stored
in a CSV file as matplotlib plots.
"""

import csv
import io
import os
import rospy
import rostest
import sys
import unittest
from matplotlib import pyplot as plt

sys.path.insert(
    0, 
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '../scripts/'))
from show_shapes import show_shapes

class TestShowShapes(unittest.TestCase):
    """
    This class defines a set of unit tests for the show_shapes function
    in the show_shapes.py script. It tests the function's ability to 
    display shapes stored in a CSV file as matplotlib plots.
    """

    def setUp(self):
        """
        Sets up the test case by creating a csv file to be used as test
        data.
        """

        self.csvfile = io.StringIO()

        writer = csv.writer(self.csvfile, delimiter=',',
                            quoting=csv.QUOTE_MINIMAL)

        writer.writerow(['', '', '0.1', '0.1', '0.2',
                        '0.2', '0.3', '0.3', '0.4', '0.4'])
        writer.writerow(['', '', '0.5', '0.5', '0.6', '0.6'])

        with open('test_csv_file.csv', 'w') as f:
            f.write(self.csvfile.getvalue())

        self.input_filename = 'test_csv_file.csv'

    def tearDown(self):
        """
        Cleans up the test case by closing the csv file.
        """

        self.csvfile.close()
        os.remove(self.input_filename)

    def test_show_shapes(self):
        """
        Tests the show_shapes function by verifying that the plot is 
        shown and that the correct number of lines have been plotted.
        """

        show_shapes(self.input_filename, no_clear=True)

        # Check if plot has been shown
        self.assertIsNotNone(plt.gca())

        # Check if shape lines have been plotted
        self.assertEqual(len(plt.gca().lines), 2)


if __name__ == '__main__':
    rospy.init_node('test_show_shapes')
    rostest.rosrun('letter_learning_interaction', 'test_show_shapes',
                   TestShowShapes, sys.argv)
