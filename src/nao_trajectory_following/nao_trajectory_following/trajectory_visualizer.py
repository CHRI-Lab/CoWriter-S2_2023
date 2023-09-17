#!/usr/bin/env python3

"""
Listens on a topic for a trajectory message, and publishes the corresponding
trajectory with markers as an animation suitable for RViz.
"""

import logging

logger = logging.getLogger("write." + __name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("[%(levelname)s] %(name)s -> %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import Empty


potentialShapesMissed = 20  # perhaps a clear request is received for shapes
# displayed before the node started, so shapeCount
# won't be accurate. This is the number of
# (potential) shapes to delete in such a case.


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("trajectory_visualizer")

        CLEAR_TOPIC = self.declare_parameter(
            "~clear_surface_topic", "clear_screen"
        ).value
        SHAPE_TOPIC = self.declare_parameter(
            "~trajectory_nao_topic", "write_traj"
        ).value
        MARKER_TOPIC = self.declare_parameter(
            "~visualization_output_topic", "visualization_markers"
        ).value

        self.FRAME = self.declare_parameter(
            "~writing_surface_frame_id", "writing_surface"
        ).value
        self.WRITE_MULTIPLE_SHAPES = (
            True  # if True, modify the marker ID so as to not
        )
        # overwrite the previous shape(s)

        self.pub_markers = self.create_publisher(Marker, MARKER_TOPIC, 5)
        self.shapeCount = 0
        # when we get a trajectory, start publishing the animation
        traj_subscriber = self.create_subscription(
            Path, SHAPE_TOPIC, self.on_traj, rclpy.qos.QoSProfile()
        )
        # when we get a clear request, delete previously drawn shapes
        clear_subscriber = self.create_subscription(
            Empty, CLEAR_TOPIC, self.on_clear, rclpy.qos.QoSProfile()
        )

    def visualize_traj(self, points):
        """
        Visualize a trajectory by publishing a marker representing the trajectory.

        Parameters:
        - points (list): A list of points representing the trajectory.

        Returns:
        - None
        """
        traj = Marker()
        traj.header.frame_id = self.FRAME
        traj.header.stamp = self.get_clock().now()
        traj.ns = "writing_traj"
        traj.action = Marker.ADD
        traj.pose.orientation.w = 1.0
        traj.type = Marker.LINE_STRIP
        traj.scale.x = 0.01  # line width
        traj.color.r = 1.0
        traj.color.b = 0.0
        traj.color.a = 1.0

        if self.WRITE_MULTIPLE_SHAPES:
            traj.id = self.shapeCount
        else:
            traj.id = 0  # overwrite any existing shapes
            traj.lifetime.secs = 1  # timeout for display

        traj.points = list(points)

        self.pub_markers.publish(traj)

    def on_traj(self, requested_traj):
        written_points = []

        # wait until time instructed to start executing
        # rospy.sleep(requested_traj.header.stamp-rospy.Time.now())

        # wait for robot to get to starting point
        # rospy.sleep(requested_traj.poses[0].header.stamp.to_sec())

        # add points to the display one at a time, like an animation
        for i in range(len(requested_traj.poses) - 1):
            p = requested_traj.poses[i].pose.position
            written_points.append(p)
            self.visualize_traj(written_points)
            duration = (
                requested_traj.poses[i + 1].header.stamp
                - requested_traj.poses[i].header.stamp
            )
            self.get_clock().sleep_for(
                Duration(seconds=duration)
            )  # wait until it's time to show the next point

        # show final point (no sleep afterwards, but it may have a "lifetime" set
        # in visualize_traj)
        p = requested_traj.poses[len(requested_traj.poses) - 1].pose.position
        written_points.append(p)
        self.visualize_traj(written_points)

        self.shapeCount += 1

    def on_clear(self, message):
        # clear each of the trajectories displayed
        for i in range(max(self.shapeCount, potentialShapesMissed)):
            traj = Marker()
            traj.header.frame_id = self.FRAME
            traj.header.stamp = self.get_clock().now()
            traj.ns = "writing_traj"
            traj.action = Marker.DELETE
            traj.id = self.shapeCount
            self.pub_markers.publish(traj)
        self.shapeCount = 0


def main():
    rclpy.init()
    traj_visualizer = TrajectoryVisualizer()
    rclpy.spin(traj_visualizer)

    traj_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
