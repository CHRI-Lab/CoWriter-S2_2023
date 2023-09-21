#!/usr/bin/env python3
"""
This module contains the TestWatchdogInteraction test class for testing
the interaction between the Watchdog and WatchdogClearer classes from 
your_module.
"""


import os
import rospy
import rostest
import sys
import time
import unittest
from threading import Event
sys.path.insert(
    0, 
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from watchdog import Watchdog, WatchdogClearer


class TestWatchdogInteraction(unittest.TestCase):
    """
    A test class for testing the interaction between the Watchdog and 
    WatchdogClearer classes.
    """
    
    def setUp(self):
        """
        Set up watchdog and clearer for tests.
        """
        self.clear_topic = "/watchdog_test/clear"
        self.timeout_sec = 1.0
        self.time_between_clears_sec = 0.5
        self.watchdog = Watchdog(self.clear_topic, self.timeout_sec)
        self.clearer = WatchdogClearer(self.clear_topic, 
                                       self.time_between_clears_sec)

    def tearDown(self):
        """
        Destroy test objects after test.
        """
        if self.watchdog:
            self.watchdog.stop()
        if self.clearer:
            self.clearer.stop()
            
    def test_watchdog_functionality(self):
        """
        Tests the basic functionality of the Watchdog class by checking
        if the default handler is called when the timer overflows.
        """
        self.clearer.stop()  # Stop the clearer before the test
        self.assertTrue(self.watchdog.is_responsive(), 
                         "Watchdog should be responsive initially.")
        time.sleep(self.timeout_sec * 1.5)
        self.assertFalse(self.watchdog.is_responsive(), 
                        "Watchdog should not be responsive after timeout.")
    
    def test_interaction(self):
        """
        Tests the interaction between the Watchdog and WatchdogClearer 
        classes by checking if the default handler is not called when
        a manual clear message is sent just before the Watchdog's 
        timeout.
        """
        self.clearer.stop()
    
        time.sleep(self.timeout_sec * 0.9)
        self.clearer.clear_watchdog()
        time.sleep(self.timeout_sec)
    
        self.assertTrue(
            self.watchdog.is_responsive(),
            "Watchdog should be responsive after the manual clear message.")



if __name__ == "__main__":
    rospy.init_node('test_watchdog', anonymous=True)
    rostest.rosrun('letter_learning_interaction', 'test_watchdog',
                   TestWatchdogInteraction, sys.argv)
