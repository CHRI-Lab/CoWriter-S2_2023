#!/usr/bin/env python3

"""
This module provides two classes for managing watchdog timers over ROS 
topics: Watchdog and WatchdogClearer.

Watchdog:
    Listens for watchdog timer clears over a topic and raises a warning
    if the timer overflows, meaning the device responsible for clearing
    the timer has not performed as expected. When a clear message is 
    received from a WatchdogClearer instance, it resets the timer and 
    updates the responsive status.

WatchdogClearer:
    Publishes watchdog timer clears over a topic at specified 
    intervals. Sends clear messages to a Watchdog instance 
    listening on the same topic, allowing the Watchdog to reset its 
    timer and maintain its responsive status.
"""


import rospy
from std_msgs.msg import Empty
from threading import Timer
from typing import Callable, Optional


class Watchdog:
    """
    Listens for watchdog timer clears over a topic and if the timer
    overflows, raises a warning that the device responsible for
    clearing the timer has not performed as expected.
    """

    def __init__(self, clear_topic: str, timeout_sec: float, 
                 user_handler: Optional[Callable]=None):
        self.clear_topic: str = clear_topic
        self.timeout_sec: float = timeout_sec
        if user_handler is None:
            self.handler: Callable = self.default_handler
        else:
            self.handler: Callable = user_handler
        self.subscriber = rospy.Subscriber(self.clear_topic, Empty, 
                                           self.on_clear)

        rospy.loginfo('Starting new timer')
        self.responsive: bool = True
        self.running: bool = True
        self.timer: Timer = Timer(self.timeout_sec, self.handler)
        self.timer.start()

    def on_clear(self, message: Empty) -> None:
        """
        Called when a clear message is received. Resets the timer and
        updates the responsive status.
        """
        self.timer.cancel()  # Clear timer
        if self.running:
            if not self.responsive:
                rospy.loginfo('Re-connection')
            self.responsive = True
            # Schedule new timeout
            self.timer = Timer(self.timeout_sec, self.handler)  
            self.timer.start()

    def stop(self) -> None:
        """
        Stops the watchdog timer.
        """
        self.timer.cancel() 
        self.running = False
        rospy.loginfo('Stopped')

    def restart(self) -> None:
        """
        Restarts the watchdog timer.
        """
        self.responsive = True
        self.running = True
        self.timer = Timer(self.timeout_sec, self.handler)
        self.timer.start()

    def default_handler(self) -> None:
        """
        The default handler to be executed when the timer overflows.
        Logs a message indicating the watchdog has not received a clear
        message.
        """
        rospy.loginfo(
            f"Haven't received a clear on '{self.clear_topic}' topic")
        self.responsive = False

    def is_responsive(self) -> bool:
        """
        Returns True if the watchdog is responsive, otherwise raises a
        RuntimeError.
        """
        if self.running:
            return self.responsive
        else:
            raise RuntimeError(
                "Responsiveness cannot be determined while the watchdog is" +
                " not running")

    def is_running(self) -> bool:
        """
        Returns True if the watchdog is running, otherwise False.
        """
        return self.running


class WatchdogClearer:
    """
    Publishes watchdog timer clears over a topic at specified 
    intervals.
    """

    def __init__(self, clear_topic: str, time_between_clears_sec: float):
        self.clear_topic: str = clear_topic
        self.time_between_clears_sec: float = time_between_clears_sec
        self.publisher = rospy.Publisher(self.clear_topic, Empty, queue_size=10)
        self.timer: Timer = Timer(self.time_between_clears_sec, 
                                  self.clear_watchdog)
        self.timer.start()
    
    def clear_watchdog(self) -> None:
        """
        Publishes a clear message and schedules the next clear message.
        """
        self.timer.cancel()
        self.publisher.publish(Empty())  # Clear watchdog
        # Schedule next clear
        self.timer = Timer(self.time_between_clears_sec, self.clear_watchdog)
        self.timer.start()

    def stop(self) -> None:
        """
        Stops publishing clear messages.
        """
        self.timer.cancel()
        rospy.loginfo('Stopping publishing clears')

    def restart(self) -> None:
        """
        Restarts publishing clear messages.
        """
        rospy.loginfo('Restarting publishing clears')
        self.timer = Timer(self.time_between_clears_sec, self.clear_watchdog)
        self.timer.start()

