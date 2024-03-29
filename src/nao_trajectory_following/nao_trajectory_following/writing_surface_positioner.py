#!/usr/bin/env python3
"""
Publish marker and frame representing a writing surface, either with an
interactive marker or wherever a fiducial marker has been detected (e.g. a
chilitag).
"""

# import roslib; roslib.load_manifest("interactive_markers")
import rclpy
from rclpy.duration import Duration

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
import math
from visualization_msgs.msg import Marker, InteractiveMarkerControl


from interactive_markers.interactive_marker_server import (
    InteractiveMarker,
    InteractiveMarkerServer,
)

# server = None
global frame_pose
frame_pose = None


def processFeedback(feedback):
    """
    Callback function for processing feedback from the interactive marker.

    Parameters:
    - feedback (InteractiveMarkerFeedback): The feedback object containing the pose
    of the interactive marker.

    Returns:
    - None
    """
    p = feedback.pose.position  # noqa: F841
    o = feedback.pose.orientation  # noqa: F841
    frame_pose = feedback.pose  # noqa: F841


def writing_surface(surface_width=0.217, surface_height=0.136):
    """
    Create a marker representing the writing surface.

    Returns:
    - surface (Marker): The marker representing the writing surface.
    """
    surface = Marker()
    surface.pose.orientation.w = 1.0
    surface.pose.position.z = -0.0005
    surface.pose.position.x = surface_width / 2
    surface.pose.position.y = surface_height / 2
    surface.id = 99
    surface.type = Marker.CUBE
    surface.scale.x = surface_width
    surface.scale.y = surface_height
    surface.scale.z = 0.0005
    surface.color.b = 1.0
    surface.color.g = 1.0
    surface.color.r = 1.0
    surface.color.a = 1.0

    return surface


def make6DofMarker(pose, frame_id, surface_width, surface_height, server, fixed=False):
    """
    Create a 6-DOF interactive marker.

    Parameters:
    - fixed (bool): Flag indicating whether the marker has fixed orientation.

    Returns:
    - None
    """
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/map"
    int_marker.scale = 0.05

    int_marker.pose.orientation.x = pose.orientation.x
    int_marker.pose.orientation.y = pose.orientation.y
    int_marker.pose.orientation.z = pose.orientation.z
    int_marker.pose.orientation.w = pose.orientation.w
    int_marker.pose.position.x = pose.position.x
    int_marker.pose.position.y = pose.position.y
    int_marker.pose.position.z = pose.position.z

    int_marker.name = frame_id
    int_marker.description = "Place the writing surface"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = surface_width
    box_marker.scale.y = surface_height
    box_marker.scale.z = 0.001
    box_marker.color.r = 1.0
    box_marker.color.g = 1.0
    box_marker.color.b = 1.0
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(writing_surface(surface_width, surface_height))

    # add the control to the interactive marker
    int_marker.controls.append(box_control)

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)


def main(args=None):
    rclpy.init()
    node = rclpy.create_node("writing_surface_positioner")
    tf_broadcaster = tf2_ros.TransformBroadcaster(node)

    # method used for positioning the writing surface frame
    POSITIONING_METHOD = node.declare_parameter(
        "positioning_method", "interactive_marker"
    ).value

    # name of frame to publish as writing surface origin
    # (at bottom left, with x horizontal and y vertical)
    FRAME_ID = node.declare_parameter(
        "writing_surface_frame_id", "writing_surface"
    ).value

    # size of marker to be displayed (default values for the galaxy note 10.1
    # in landscape orientation
    SURFACE_WIDTH = node.declare_parameter("surface_width", 0.217).value
    SURFACE_HEIGHT = node.declare_parameter("surface_height", 0.136).value

    if POSITIONING_METHOD.lower() == "fiducial_marker_detection":
        TAG_FRAME = node.declare_parameter(
            "tag_frame_id", "tag_1"
        ).value  # name of frame to
        # detect writing surface with
        ROTATE_TAG_FRAME = node.declare_parameter(
            "rotate_tag_frame", True
        ).value  # chilitag
        # frame has y horizontal and x vertical (graphics coordinate system) and
        # needs to be changed to 'robotics' coordinate system

        tf_listener = tf2_ros.transform_listener.TransformListener(
            Buffer(), node
        )  # True, Duration(seconds=10))
        node.get_clock().sleep_for(Duration(seconds=0.5))
        rate = node.create_rate(50)
        while rclpy.ok():
            # tf_broadcaster.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"v4l_frame","gaze")
            # manually "attach" the webcam to the robot's frame (for testing)
            try:
                tf_listener.waitForTransform(
                    "map",
                    TAG_FRAME,
                    node.get_clock().now(),
                    Duration(seconds=5),
                )
                t = tf_listener.getLatestCommonTime("map", TAG_FRAME)
                (trans, rot) = tf_listener.lookupTransform("map", TAG_FRAME, t)
            except (
                tf2_ros.Exception,
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                continue

            # rotate coordinate system of tag to match the desired one for the tablet
            surfacePose = PoseStamped()
            surfacePose.header.frame_id = TAG_FRAME

            if ROTATE_TAG_FRAME:
                # convert 'x up, y to the right' of chilitag from
                # to 'y up, x to the right' for the writing surface
                orientation = tf_transformations.quaternion_from_euler(
                    3.14159, 0, 3.14159 / 2
                )

            surfacePose.pose.orientation.x = orientation[0]
            surfacePose.pose.orientation.y = orientation[1]
            surfacePose.pose.orientation.z = orientation[2]
            surfacePose.pose.orientation.w = orientation[3]
            surfacePose = tf_listener.transformPose("map", surfacePose)
            o = surfacePose.pose.orientation
            # publish writing surface's frame w.r.t. map
            tf_broadcaster.sendTransform(
                trans,
                (o.x, o.y, o.z, o.w),
                node.get_clock().now(),
                FRAME_ID,
                "map",
            )

            single_transform = TransformStamped()
            single_transform.header.stamp = node.get_clock().now().to_msg()
            single_transform.header.frame_id = "parent_frame"
            single_transform.child_frame_id = "child_frame"
            single_transform.transform.translation.x = 1.0
            single_transform.transform.translation.y = 2.0
            single_transform.transform.translation.z = 3.0
            single_transform.transform.rotation.x = 0.0
            single_transform.transform.rotation.y = 0.0
            single_transform.transform.rotation.z = 0.0
            single_transform.transform.rotation.w = 1.0

            tf_broadcaster.sendTransform(single_transform)

            # pub_markers.publish(
            #     writing_surface(SURFACE_WIDTH, SURFACE_HEIGHT)
            # )  # show writing surface

        rate.sleep()

    # we are going here !!!
    elif POSITIONING_METHOD.lower() == "interactive_marker":
        NAO_HANDEDNESS = node.declare_parameter("nao_handedness", "right").value

        # assign default values of pose
        frame_pose = Pose()
        # use values from 'rosrun tf tf_echo map writing_surface'
        # with interactive marker in desired position
        if NAO_HANDEDNESS.lower() == "right":
            frame_pose.orientation.x = -0.4
            frame_pose.orientation.y = 0.5
            frame_pose.orientation.z = 0.6
            frame_pose.orientation.w = -0.5
            frame_pose.position.x = 0.225
            frame_pose.position.y = 0.02
            frame_pose.position.z = 0.27
        elif NAO_HANDEDNESS.lower() == "left":
            frame_pose.orientation.x = 0.4
            frame_pose.orientation.y = -0.4
            frame_pose.orientation.z = -0.5
            frame_pose.orientation.w = 0.6
            frame_pose.position.x = 0.16
            frame_pose.position.y = 0.032 + SURFACE_WIDTH
            frame_pose.position.z = 0.27
        else:
            node.get_logger().error("error in handedness input")

        server = InteractiveMarkerServer(node, "writing_surface_positioner")
        make6DofMarker(
            frame_pose,
            FRAME_ID,
            SURFACE_WIDTH,
            SURFACE_HEIGHT,
            server,
            fixed=True,
        )

        # 'commit' changes and send to all clients
        server.applyChanges()

        rate = node.create_rate(10.0)
        while rclpy.ok():
            if frame_pose:
                p = frame_pose.position
                o = frame_pose.orientation
                tot = math.sqrt((o.x**2 + o.y**2 + o.z**2 + o.w**2))
                o.x /= tot
                o.y /= tot
                o.z /= tot
                o.w /= tot
                # tf_broadcaster.sendTransform(
                #     (p.x, p.y, p.z),
                #     (o.x, o.y, o.z, o.w),
                #     node.get_clock().now(),
                #     FRAME_ID,
                #     "map",
                # )
                transform = TransformStamped()
                transform.transform.translation.x = p.x
                transform.transform.translation.y = p.y
                transform.transform.translation.z = p.z
                transform.transform.rotation.x = o.x
                transform.transform.rotation.y = o.y
                transform.transform.rotation.z = o.z
                transform.transform.rotation.w = o.w
                transform.header.stamp = node.get_clock().now().to_msg()
                transform.header.frame_id = FRAME_ID
                transform.child_frame_id = "map"

                # Send the transform
                tf_broadcaster.sendTransform(transform)
            rate.sleep()


if __name__ == "__main__":
    main()
