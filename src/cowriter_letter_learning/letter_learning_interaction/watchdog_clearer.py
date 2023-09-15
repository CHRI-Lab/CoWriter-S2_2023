#!/usr/bin/env python3
"""
Publish watchdog clears on the /watchdog_clear/device_name topic every
time_between_clears seconds.
"""

import argparse


from .include.watchdog import WatchdogClearer

import rclpy
from rclpy.node import Node

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Publish watchdog clears \
    on the /watchdog_clear/device_name topic."
    )

    parser.add_argument(
        "device_name",
        action="store",
        type=str,
        help="the name of the device which the watchdog is monitoring",
    )

    parser.add_argument(
        "time_between_clears",
        action="store",
        type=float,
        help="the time interval between watchdog clears in seconds",
    )

    args = parser.parse_args()

    node_name = args.device_name + "_watchdog_clearer"
    # rospy.init_node(node_name)
    rclpy.init()
    node = Node(node_name)

    # Init a WatchdogClearer to publish device clearing status.
    watchdog_clearer = WatchdogClearer(
        f"watchdog_clear/{args.device_name}", args.time_between_clears
    )

    node.get_logger().info(
        f"Starting new watchdog_clearer for {args.device_name}"
    )

    # Spin node until shutdown
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # When node is shutdown, stop watchdog_clearer
    watchdog_clearer.stop()
