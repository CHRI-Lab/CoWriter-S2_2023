#!/usr/bin/env python3
# coding: utf-8

# from .interaction_settings import InteractionSettings
from typing import Tuple

# from std_msgs.msg import String
# import sounddevice as sd
# from scipy.io.wavfile import write


class NaoSettings:
    """
    NaoSettings is a class that handles the configuration and
    interaction settings for a NAO robot, including its handedness,
    language, speaking, writing, and connection status. It also
    provides methods for controlling the robot's behavior, such as
    speaking, looking at a tablet, and asking for feedback.

    The class uses ROS parameters to get the values for various
    settings. Some parameters are treated as constants and stored as
    class variables, while others are fetched during the instance
    initialization.

    Constant attributes:
        NAO_IP (str): The IP address of the NAO robot.
        FRONT_INTERACTION (bool): Flag indicating if the interaction is
                                  from the front of the robot.
        NAO_HANDEDNESS (str): The handedness of the NAO robot (either
                              'right' or 'left').
    """

    # Following parameters treated as constants so for now they are not
    # passed in as arguments, but treated as class variables.

    # Default behaviour is to connect to simulator locally
    NAO_IP: str =  "127.0.0.1"
    NAO_PORT = 9559
    FRONT_INTERACTION: bool = True
    NAO_HANDEDNESS: str = "right" 

    def __init__(self) -> None:
        # Nao parameters
        self.LANGUAGE: str =  "english" 
        # whether or not the robot should stand or rest on its knees
        self.nao_standing: bool = True 
        # whether or not the robot is being used for the interaction
        self.nao_connected: bool = True 
        

        # speaking and writing conjunct with conn as stronger property
        # whether or not the robot should speak
        self.nao_speaking: bool = True
       
        self.nao_writing: bool = True
        
        # Set effector based on handedness
        self.set_effector()

        # self.set_phrase_manager()
        self.set_orientation_params()

    def get_settings(self) -> dict:
        return {
            "nao_writing": self.nao_writing,
            "nao_speaking": self.nao_speaking,
            "nao_standing": self.nao_standing,
            "nao_connected": self.nao_connected,
            "nao_handedness": self.NAO_HANDEDNESS,
            "alternate_sides_looking_at": self.alternate_sides_looking_at,
            "LANGUAGE": self.LANGUAGE,
            "person_side": self.person_side,
        }
    def set_effector(self) -> None:
        """
        Sets the effector for the nao settings class based on nao
        handedness.
        """
        if self.NAO_HANDEDNESS.lower() == "right":
            self.effector: str = "RArm"
        elif self.NAO_HANDEDNESS.lower() == "left":
            self.effector: str = "LArm"
        else:
            print("error in handedness param")

    # def set_phrase_manager(self) -> None:
    #     """
    #     Sets the phrase manager for the nao settings based on nao
    #     language.
    #     """
    #     self.phrase_manager: PhraseManager = PhraseManager(self.LANGUAGE)

    def set_orientation_params(self):
        """
        Set parameters for head angles, side person is one, whether to
        alternate which side nao is looking at, and the next side to
        look at if alternating.
        """

        # Get appropriate angles for looking at things
        (
            self.head_angles_look_at_tablet_down,
            self.head_angles_look_at_tablet_right,
            self.head_angles_look_at_tablet_left,
            self.head_angles_look_at_person_front,
            self.head_angles_look_at_person_right,
            self.head_angles_look_at_person_left,
        ) = get_head_angles()

        # Get the side of where the person is (left/right)
        self.person_side = self.NAO_HANDEDNESS.lower()

        # Using default values for params below, as in original code
        # if alternate sides, nao looks to a different side each time.
        # Not well tested
        self.alternate_sides_looking_at: bool = False
        self.next_side_to_look_at: str = "Right"

    def set_nao_interaction(self):
        """
        Sets the parameters for interaction with the robot.

        Must initialise an instance of NaoSettings and call this method
        in main for the interaction to work.
        """

        if self.nao_connected:
            import qi

            qi_url = f"tcp://{self.NAO_IP}:{self.NAO_PORT}"
            print(f"[RobotController] Connecting to qi_url={qi_url}")
            self.session = qi.Session()
            self.session.connect(qi_url)
            print(f"[RobotController] app started")

            self.nextSideToLookAt = "Right"

            # ? all services needed
            self.motion_proxy = self.session.service("ALMotion")
            # return
            self.posture_proxy = self.session.service("ALRobotPosture")
            self.text_to_speech = self.session.service("ALTextToSpeech")
            self.text_to_speech.setLanguage(self.LANGUAGE.capitalize())
            # textToSpeech.setVolume(1.0)
            if self.nao_writing:
                if self.nao_standing:
                    self.posture_proxy.goToPosture("StandInit", 0.2)
                    self.motion_proxy.wbEnableEffectorControl(
                        self.effector, True
                    )  # turn whole body motion control on
                else:
                    self.motion_proxy.rest()
                    self.motion_proxy.setStiffnesses(
                        ["Head", "LArm", "RArm"], 0.5
                    )
                    self.motion_proxy.setStiffnesses(
                        [
                            "LHipYawPitch",
                            "LHipRoll",
                            "LHipPitch",
                            "RHipYawPitch",
                            "RHipRoll",
                            "RHipPitch",
                        ],
                        0.8,
                    )
                    self.motion_proxy.wbEnableEffectorControl(
                        self.effector, False
                    )  # turn whole body motion control off

                self.arm_joints_stand_init = self.motion_proxy.getAngles(
                    self.effector, True
                )

    def nao_speak_and_log_phrase(self, phrase: str) -> None:
        """
        Makes NAO speak the phrase, and logs the phrase.

        Args:
            phrase (str): The phrase for NAO to speak and log.
        """
        # rospy.loginfo(f"NAO: {phrase}")
        self.text_to_speech.say(phrase)

    def look_at_tablet(self):
        """
        Makes the NAO robot look at the tablet by adjusting its head
        angles.

        The method considers the interaction orientation (front or side)
        and the handedness of the robot to determine the direction in
        which the robot should look.

        The head angles are set using the 'motion_proxy.setAngles'
        method.
        """
        if self.FRONT_INTERACTION:
            self.motion_proxy.setAngles(
                ["HeadYaw", "HeadPitch"],
                self.head_angles_look_at_tablet_down,
                0.2,
            )

        else:
            if self.effector == "RArm":  # Tablet will be on our right
                self.motion_proxy.setAngles(
                    ["HeadYaw", "HeadPitch"],
                    self.head_angles_look_at_tablet_right,
                    0.2,
                )
            else:
                self.motion_proxy.setAngles(
                    ["HeadYaw", "HeadPitch"],
                    self.head_angles_look_at_tablet_left,
                    0.2,
                )

    def look_and_ask_for_feedback(self, to_say: str, side: str) -> None:
        """
        Makes the NAO robot look at a person and ask for feedback by
        adjusting its head angles and speaking the provided phrase.

        The method considers the interaction orientation
        (front or side), the provided side (right or left),
        and the robot's speaking and writing states.

        Args:
            to_say (str): The phrase for the NAO robot to speak when
                          asking for feedback.
            side (str): The side the person is on, either "right" or
                        "left".
        """
        if self.nao_connected:
            if self.nao_writing:
                # Put arm down
                self.motion_proxy.angleInterpolationWithSpeed(
                    self.effector, self.arm_joints_stand_init, 0.3
                )

            if self.FRONT_INTERACTION:
                self.motion_proxy.setAngles(
                    ["HeadYaw", "HeadPitch"],
                    self.head_angles_look_at_person_front,
                    0.2,
                )
            else:
                if side == "right":  # Person will be on our right
                    self.motion_proxy.setAngles(
                        ["HeadYaw", "HeadPitch"],
                        self.head_angles_look_at_person_right,
                        0.2,
                    )
                else:  # Person will be on our left
                    self.motion_proxy.setAngles(
                        ["HeadYaw", "HeadPitch"],
                        self.head_angles_look_at_person_left,
                        0.2,
                    )

            if self.nao_speaking:
                self.nao_speak_and_log_phrase(to_say)

    def handle_look_and_ask_for_feedback(self, phrase: str) -> None:
        """
        Helper method that calls look_and_ask_forfeedback either
        alternating sides or using a fixed side, depending on value of
        alternate_sides_looking_at and next_side_to_look_at.

        Args:
            phrase (str): The phrase for NAO to speak and ask for
                          feedback.
        """
        if self.alternate_sides_looking_at:
            self.look_and_ask_for_feedback(phrase, self.next_side_to_look_at)

            # Since alternating, change next side to look at
            if self.next_side_to_look_at == "Left":
                self.next_side_to_look_at = "Right"
            else:
                self.next_side_to_look_at = "Left"
        else:
            self.look_and_ask_for_feedback(phrase, self.person_side)

    def nao_rest(self):
        """
        Set NAO robot to the resting state by disabling effector
        control and invoking the 'rest' command.

        This method first checks if the NAO robot is connected by
        verifying the 'nao_connected' attribute. If connected, it
        disables the whole body effector control for the specified
        effector (stored in 'self.effector') by calling
        'wbEnableEffectorControl' with 'False' as the second argument.
        Finally, the 'rest' command is sent to the robot using the
        'motion_proxy.rest()' method, putting the NAO robot into a
        resting state.
        """
        if self.nao_connected:
            self.motion_proxy.wbEnableEffectorControl(self.effector, False)
            self.motion_proxy.rest()


def get_head_angles() -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        """
        Get the head angles for NAO to look at different positions.

        Returns:
            tuple: A tuple containing the head angles for NAO to look
                   at tablet down, tablet right, tablet left, person
                   front, person right, and person left.
        """
        head_angles_look_at_tablet_down: Tuple[float,
                                               float] = (-0.01538, 0.512)
        head_angles_look_at_tablet_right: Tuple[float,
                                                float] = (-0.2, 0.08125996589660645)
        head_angles_look_at_tablet_left: Tuple[float, float] = (
            0.2, 0.08125996589660645)
        head_angles_look_at_person_front: Tuple[float,
                                                float] = (-0.0123, 0.1825)
        head_angles_look_at_person_right: Tuple[float,
                                                float] = (-0.9639739513397217, 0.08125996589660645)
        head_angles_look_at_person_left: Tuple[float, float] = (
            0.9639739513397217, 0.08125996589660645)

        return head_angles_look_at_tablet_down, head_angles_look_at_tablet_right, head_angles_look_at_tablet_left, head_angles_look_at_person_front, head_angles_look_at_person_right, head_angles_look_at_person_left
