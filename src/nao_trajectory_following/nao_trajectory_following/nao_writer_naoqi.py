#!/usr/bin/env python3
"""
Listens for a trajectory to write and sends it to the nao via naoqi SDK.

Requires a running robot/simulation with ALNetwork proxies.

"""
import json
# from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Empty, Header
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from requests import Session

# import motion
import math
import time

class nao_writer_naoqi(Node):
    def __init__(
        self,
        session: Session,
        effector="RArm",
        naoWriting=True,
        naoSpeaking=True,
        naoStanding=True,
        language="english",
        isFrontInteraction=True,
        alternateSidesLookingAt=False,
    ) -> None:
        """
        Initialize the RobotController object.

        Parameters:
        - effector (str, optional): The effector to use for writing. Defaults to 'RArm'.
        - naoWriting (bool, optional): Flag indicating whether the NAO robot is capable of writing. Defaults to True.
        - naoSpeaking (bool, optional): Flag indicating whether the NAO robot is capable of speaking. Defaults to True.
        - naoStanding (bool, optional): Flag indicating whether the NAO robot should stand initially. Defaults to True.
        - language (str, optional): The language to use for speech. Defaults to 'english'.
        - isFrontInteraction (bool, optional): Flag indicating whether the interaction is in front of the robot. Defaults to True.
        - alternateSidesLookingAt (bool, optional): Flag indicating whether the robot should alternate sides when looking at objects. Defaults to False.
        """
        super().__init__("nao_writer")
        self.session = session
        SHAPE_TOPIC = self.declare_parameter(
            "~trajectory_nao_topic", "write_traj"
        ).value
        self.nao_settings = session.get(
            "http://localhost:5000/get_settings"
        ).json()
        TRAJ_TOPIC = "write_traj_nao"  # rospy.get_param('~trajectory_nao_input_topic','/write_traj_nao')
        # NAO_IP = self.declare_parameter("~nao_ip", "127.0.0.1").value
        # PORT = int(self.declare_parameter("~nao_port", "9559").value)
        # NAO_HANDEDNESS = self.declare_parameter(
        #     "~nao_handedness", "right"
        # ).value

        if self.nao_settings.get("nao_handedness") == "right":
            self.effector = "RArm"
        elif self.nao_settings.get("nao_handedness") == "left":
            self.effector = "LArm"
        else:
            self.get_logger().info("error in handedness param")

        # self.qi_url = f"tcp://{NAO_IP}:{PORT}"
        # self.get_logger().info(
        #     f"[RobotController] Connecting to qi_url={self.qi_url}"
        # )
        # self.app = qi.Application(url=self.qi_url)
        # self.app.start()
        # # app.run()
        # self.get_logger().info(f"[RobotController] app started")
        # self.session = self.app.session
        # self.motionProxy = self.session.service("ALMotion")
        # self.memoryProxy = self.session.service("ALMemory")
        # self.postureProxy = self.session.service("ALRobotPosture")
        # self.ttsProxy = self.session.service("ALTextToSpeech")
        self.tl = TransformListener(Buffer(), self)#, True, Duration(seconds=10))
        self.space = 2  # {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}
        self.isAbsolute = True

        self.create_subscription(
            Path, SHAPE_TOPIC, self.on_traj, 10
        )

    def point_dist(self, p1, p2):
        return math.sqrt(
            (p1[0] - p2[0]) * (p1[0] - p2[0])
            + (p1[1] - p2[1]) * (p1[1] - p2[1])
            + (p1[2] - p2[2]) * (p1[2] - p2[2])
        )

    def on_traj(self, traj):
        """
        Callback function to handle trajectory messages.

        Parameters:
        - traj (Path): The trajectory message containing the desired poses.

        Note: The parameters `AXIS_MASK_X`, `AXIS_MASK_Y`, `AXIS_MASK_Z`, and `AXIS_MASK_WX` are assumed to be defined outside this method.

        Returns:
        None
        """
        self.get_logger().info("got traj at " + str(self.get_clock().now()))
        AXIS_MASK_X = 1
        AXIS_MASK_Y = 2
        AXIS_MASK_Z = 4
        AXIS_MASK_WX = 8

        axisMask = [AXIS_MASK_X + AXIS_MASK_Y + AXIS_MASK_Z + AXIS_MASK_WX]

        if self.effector == "LArm":
            self.session.post(
                "http://localhost:5000/open_hand", json={"hand": "LHand"}
            )
            self.session.post(
                "http://localhost:5000/close_hand", json={"hand": "LHand"}
            )

            roll = (
                -1.7
            )  # rotate wrist to the left (about the x axis, w.r.t. robot frame)
        else:
            self.session.post(
                "http://localhost:5000/open_hand", json={"hand": "RHand"}
            )
            self.session.post(
                "http://localhost:5000/close_hand", json={"hand": "RHand"}
            )
            roll = 1.7  # rotate wrist to the right (about the x axis, w.r.t. robot frame)

        target = PoseStamped()

        target_frame = traj.header.frame_id
        target.header.frame_id = target_frame

        """
        #go to first point then wait
        path = []  
        times = [] 
        trajStartPosition = traj.poses[0].pose.position 
        traj.poses[0].pose.position.z = 0.05
        target.pose.position = deepcopy(traj.poses[0].pose.position)
        target.pose.orientation = deepcopy(traj.poses[0].pose.orientation)
        trajStartPosition_robot = tl.transformPose("base_footprint",target)
        point = [trajStartPosition_robot.pose.position.x,trajStartPosition_robot.pose.position.y,trajStartPosition_robot.pose.position.z,roll,0,0] 
        
        path.append(point) 
        timeToStartPosition = traj.poses[0].header.stamp.to_sec() 
        times.append(timeToStartPosition) 
        motionProxy.setPosition(effector,space,point,0.5,axisMask) #,times,isAbsolute) 
        """
        path = []
        times = []
        timer = 1.0
        timer_step = 0.01
        last_point = None
        for trajp in traj.poses:
            target.pose.position = deepcopy(trajp.pose.position)
            target.pose.orientation = deepcopy(trajp.pose.orientation)
            target.header.frame_id = "base_footprint"
            target_robot = self.tl.buffer.transform(target, "base_footprint", rclpy.duration.Duration(seconds=5.0))

            point = [
                float(target_robot.pose.position.x),
                float(target_robot.pose.position.y),
                float(0.0),
                roll,
                0.0,
                0.0,
            ]  # roll,pitch,yaw]
            # point = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

            times.append(timer)
            if not last_point is None:
                point_dist = self.point_dist(last_point, point)
                timer += point_dist * 80
            else:
                timer += 0.4
            # to keep time increasing
            timer += timer_step
            path.append(point)
            # times.append(trajp.header.stamp.to_sec() )#- timeToStartPosition)
            last_point = point
        # wait until time instructed to start executing
        # self.get_clock().sleep_for(
        #     traj.header.stamp - self.get_clock().now()
        # )  # +rospy.Duration(timeToStartPosition))

        time.sleep(3)
        self.get_logger().info(
            "executing rest of traj at " + str(self.get_clock().now())
        )
        startTime = self.get_clock().now()

        self.session.post(
            "http://localhost:5000/position_interpolation",
            json={
                "effector": self.effector,
                "space": self.space,
                "path": self.traj_to_path(path),
                "axisMask": axisMask,
                "times": times,
                "isAbsolute": self.isAbsolute,
            },
        )
        self.get_logger().info(
            "Time taken for rest of trajectory: "
            #+ str((self.get_clock().now() - startTime).seconds())
        )

    # normalize a path for nao robot so it can write comfortablely
    # lot of random numbers due to artistic freedom
    def traj_to_path(self, points):
        """
        Convert a list of points representing a trajectory into a path suitable for writing.

        Parameters:
        - points (list): A list of points representing the trajectory, where each point is a list with six elements [x, y, z, roll, pitch, yaw].

        Returns:
        - converted_pts (list): A list of converted points representing the writing path, where each point is a list with six elements [x, y, z, roll, pitch, yaw].
        """
        arm_len = 0.323
        converted_pts = []
        corner_min = [0.12, 0.03, 0.32]
        corner_max = [0.12, -0.18, 0.47]
        # limit the coordinate range for writing
        for (
            point
        ) in (
            points
        ):  # Convert the writing trajectory of the X and Y axes into a writing trajectory perpendicular to the ground
            y = point[0]
            z = point[1]
            converted_pts.append([0, y, z, point[3], point[4], point[5]])

        pt_min = [
            0,
            min([point[1] for point in converted_pts]),
            min([point[2] for point in converted_pts]),
        ]
        pt_max = [
            0,
            max([point[1] for point in converted_pts]),
            max([point[2] for point in converted_pts]),
        ]

        normal_factor = [
            0,
            (corner_max[1] - corner_min[1]) / (pt_max[1] - pt_min[1]),
            (corner_max[2] - corner_min[2]) / (pt_max[2] - pt_min[2]),
        ]

        for i, point in enumerate(converted_pts):
            y = (point[1] - pt_min[1]) * normal_factor[1] + corner_min[1]
            z = (point[2] - pt_min[2]) * normal_factor[2] + corner_min[2]
            # The length of the robot's arm is limited, and it needs to adjust the X-axis coordinate size according to the writing position
            x = (
                math.sqrt(max([0.12 - (y + 0.07) * (y + 0.07) - z * z, 0.1]))
                * 1.2
                - 0.21
            )
            converted_pts[i] = [x, y, z, point[3], point[4], point[5]]

        return converted_pts


def main():
    rclpy.init()
    session = Session()
    writer = nao_writer_naoqi(session)
    rclpy.spin(writer)

    writer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
