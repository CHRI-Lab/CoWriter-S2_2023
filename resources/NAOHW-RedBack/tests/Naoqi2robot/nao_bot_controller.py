import json
import struct
import sys
import socket
import threading
from threading import Thread

from naoqi import ALProxy
import time
import math
import motion
import almath

upper_arm_length = 0.0575
lower_arm_length = 0.055

ACTIONS = {'DRAW', 'TALK'}

coordinates = [
    (0.01, 0.01),
    (0.015, 0.015),
    (0.02, 0.02),
    (0.03, 0.03)
]


def to_radians(angles):
    return [math.radians(angle) for angle in angles]


def calculate_joint_angles(x, y, upper_arm_length, lower_arm_length):
    d = math.sqrt(x ** 2 + y ** 2)
    if d > upper_arm_length + lower_arm_length:
        print("Target position is unreachable.")
        return None

    angle_a = math.acos((x ** 2 + y ** 2 + upper_arm_length ** 2 - lower_arm_length ** 2) / (2 * upper_arm_length * d))
    angle_b = math.acos(
        (x ** 2 + y ** 2 - upper_arm_length ** 2 - lower_arm_length ** 2) / (2 * upper_arm_length * lower_arm_length))

    return [angle_a, math.pi - angle_b]


def input_path_process(path):
    scale_down_factor = 30

    mid = len(path) // 2
    x = path[:mid]
    y = path[mid:]
    coordinates = list(map(lambda (x,y):(x/scale_down_factor, y/scale_down_factor), zip(x, y)))
    # Apply 90 degree rotation clockwise
    coordinates = [(-y, -x) for x, y in coordinates]

    delta = [coordinates[0]]
    sum_x = 0
    sum_y = 0

    for i in range(1, len(coordinates)):
        x2, y2 = coordinates[i]
        x1, y1 = coordinates[i-1]
        delta.append((x2-x1, y2-y1))
        sum_x += x2-x1
        sum_y += y2-y1

    # delta.reverse()

    # delta.append((-sum_x, -sum_y))
    return  delta


class Robot(Thread):
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.motion = ALProxy("ALMotion", ip, port)
        self.posture = ALProxy("ALRobotPosture", ip, port)
        self.speaker = ALProxy("ALTextToSpeech", ip, port)

        self.host = '0.0.0.0'
        self.port = 12345
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def read_message(self, s):
        # First read the length of the message
        length_prefix = s.recv(4)
        if length_prefix:
            length = struct.unpack('>I', length_prefix)[0]
            msg = ''
            # Then read the message itself
            while len(msg) < length:
                chunk = s.recv(length - len(msg))  # only receive what's left
                if chunk:
                    msg += chunk
                else:
                    # No more data is received from the client. Break the loop.
                    break
            return msg

    def handle_client(self, client_socket, addr):
        try:
            print 'Received connection from:', addr
            while True:
                # Receive and print the entire message
                msg = self.read_message(client_socket)
                # msg = client_socket.recv(1024)

                if msg is not None:
                    msg = json.loads(msg)
                    for action, content in msg.items():
                        if action not in ACTIONS:
                            continue

                        if action == 'DRAW':
                            print(type(content))
                            delta = input_path_process(content)
                            print(delta)
                            # self.move_hand_to_coordinates(input_path_process(content))
                            # self.motion.rest()
                            self.motion.wakeUp()
                            self.stand_straight()
                            # self.reset_arm_position()
                            self.get_ready()
                            self.moveRightArm(0.02, 0.1, 0.15, dt=3)

                            # move hand into position
                            dx, dy = delta[0]
                            self.moveRightArm(dx, dy, 0, dt=1)

                            for dx, dy in delta[1:]:
                                self.moveRightArm(dx, dy, 0, dt=0.005)

                            print('---------')

                        if action == "TALK":
                            print(type(content))
                            self.speaker.say(content.encode('utf-8'))

                    # Optionally, send a response back to the client
                    response = 'Received your message!'
                    client_socket.send(response)  # response must be encoded before sending
        finally:
            client_socket.close()

    def start_listening(self, n):

        self.server_socket.bind((self.host, self.port))
        # Listen for incoming connections
        self.server_socket.listen(n)
        print 'Server socket is listening on port 12345...'



        while True:
            # Wait for a client to connect
            client_socket, addr = self.server_socket.accept()

            client_handler = threading.Thread(target=self.handle_client, args=(client_socket,addr))
            client_handler.start()

    def stand_straight(self):
        self.posture.goToPosture("StandInit", 1)

    def move_hand_to_coordinates(self, coordinates):
        for coord in coordinates:
            joint_angles = calculate_joint_angles(coord[0], coord[1], upper_arm_length, lower_arm_length)
            if joint_angles:
                angles = [0, joint_angles[0], 0, joint_angles[1], 0, 0]
                self.motion.angleInterpolationWithSpeed("RArm", angles, 0.2)
                time.sleep(1)

    def reset_arm_position(self):
        # self.posture.goToPosture("Stand", 0.5)
        angles = [math.radians(angle) for angle in [90, 0, 0, 0, 0, 0]]
        self.motion.angleInterpolationWithSpeed("RArm", angles, 0.2)
        self.motion.angleInterpolationWithSpeed("LArm", angles, 0.2)

    def run(self, coordinates):
        # self.start_listening()
        self.motion.rest()
        self.stand_straight()
        self.move_hand_to_coordinates(coordinates)
        hand_name = "RHand"  # Replace with the name of the hand you want to close
        # self.motion.openHand(hand_name)
        # Close the hand
        hand_angle = 0.0  # Replace with the desired angle to close the hand (0.0 is fully closed)
        hand_speed = 0.5  # Replace with the desired speed of hand movement (0.0 to 1.0)
        self.motion.setAngles(hand_name, hand_angle, hand_speed)
        # self.motion.killAll()
        self.reset_arm_position()
        # self.stand_straight()

    def lift_arm(self):
        motion_service = self.motion
        posture_service = self.posture

        # Wake up robot
        motion_service.rest()
        motion_service.wakeUp()

        self.stand_straight()

        # Set stiffness
        names = "LArm"
        stiffnessLists = 0
        timeLists = 1.0
        motion_service.stiffnessInterpolation(names, stiffnessLists, timeLists)

        for i in range(0, 90, 10):
            # desired arm position (in radians)
            # this is an example position, you should adjust these values to your needs
            shoulderPitchAngle = almath.TO_RAD * 0  # 90 degrees vertical
            shoulderRollAngle = almath.TO_RAD * -i  # 0 degrees forward/backward
            elbowYawAngle = almath.TO_RAD * 0  # 0 degrees twist
            elbowRollAngle = almath.TO_RAD * 0  # 0 degrees bend
            wristYawAngle = almath.TO_RAD * 90  # 0 degrees twist

            # apply the desired arm position
            motion_service.setAngles('RShoulderPitch', shoulderPitchAngle, 0.3)
            motion_service.setAngles('RShoulderRoll', shoulderRollAngle, 0.3)
            motion_service.setAngles('RElbowYaw', elbowYawAngle, 0.3)
            motion_service.setAngles('RElbowRoll', elbowRollAngle, 0.3)
            motion_service.setAngles('RWristYaw', wristYawAngle, 0.3)

        # isEnabled = True
        # motion_service.wbEnable(isEnabled)


    def get_ready(self):
        motion_service = self.motion
        posture_service = self.posture
        shoulderPitchAngle = almath.TO_RAD * 35  # 90 degrees vertical
        shoulderRollAngle = almath.TO_RAD * 0  # 0 degrees forward/backward
        elbowYawAngle = almath.TO_RAD * 0  # 0 degrees twist
        elbowRollAngle = almath.TO_RAD * 0  # 0 degrees bend
        wristYawAngle = almath.TO_RAD * 90  # 0 degrees twist

        # apply the desired arm position
        motion_service.setAngles('RShoulderPitch', shoulderPitchAngle, 0.3)
        motion_service.setAngles('RShoulderRoll', shoulderRollAngle, 0.3)
        motion_service.setAngles('RElbowYaw', elbowYawAngle, 0.3)
        motion_service.setAngles('RElbowRoll', elbowRollAngle, 0.3)
        motion_service.setAngles('RWristYaw', wristYawAngle, 0.3)

    def test(self):
        # Get the services ALMotion & ALRobotPosture.
        self.stand_straight()

        motion_service = self.motion
        posture_service = self.posture

        motion_service.rest()

        # Wake up robot
        motion_service.wakeUp()


        shoulderPitchAngle = almath.TO_RAD * 35  # 90 degrees vertical
        shoulderRollAngle = almath.TO_RAD * 0  # 0 degrees forward/backward
        elbowYawAngle = almath.TO_RAD * 0  # 0 degrees twist
        elbowRollAngle = almath.TO_RAD * 0  # 0 degrees bend
        wristYawAngle = almath.TO_RAD * 90  # 0 degrees twist

        # apply the desired arm position
        motion_service.setAngles('RShoulderPitch', shoulderPitchAngle, 0.3)
        motion_service.setAngles('RShoulderRoll', shoulderRollAngle, 0.3)
        motion_service.setAngles('RElbowYaw', elbowYawAngle, 0.3)
        motion_service.setAngles('RElbowRoll', elbowRollAngle, 0.3)
        motion_service.setAngles('RWristYaw', wristYawAngle, 0.3)


        self.moveRightArm(0.02, 0.1, 0.25, dt=3)
        width = 0.05
        self.moveRightArm(width, 0.0, 0)

        self.moveRightArm(0.00, width, 0)
        self.moveRightArm(-width, 0.0, 0)
        self.moveRightArm(0.0, -width, 0)

    def moveRightArm(self, dx, dy, dz, dt=1.0):
        motion_service = self.motion

        effectorList = []
        pathList = []
        axisMaskList = []
        timeList = []
        frame = motion.FRAME_ROBOT
        useSensorValues = False

        effectorList.append("RArm")
        currentPos = motion_service.getPosition("RArm", frame, useSensorValues)
        targetPos = almath.Position6D(currentPos)

        targetPos.z += dz
        targetPos.x += dx
        targetPos.y += dy
        pathList.append(list(targetPos.toVector()))

        axisMaskList.append(motion.AXIS_MASK_VEL)
        timeList.append([dt])

        motion_service.positionInterpolations(effectorList, frame, pathList,
                                              axisMaskList, timeList)

        # self.motion.rest()


if __name__ == "__main__":
    # robot_ip = '192.168.56.1'  # choreograph simulation
    # robot_ip = '10.21.167.40'

    # robot_ip = '192.168.0.9'
    robot_ip = '127.0.0.1'
    # robot_ip = '10.21.170.41'
    # robot_ip = '169.254.16.139'
    robot_port = 9559

    robot = Robot(robot_ip, robot_port)
    # robot.run(coordinates)
    robot.start_listening(10)
    # robot.lift_arm()
    # robot.draw()
    # robot.test()
