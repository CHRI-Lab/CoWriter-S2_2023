#!/usr/bin/env python
# coding: utf-8

"""
Control robot
Get data from robot
"""
import qi
from bluering_letter_learning.interaction_settings import InteractionSettings
#? for trajectory_following
from datetime import datetime
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import rospy
import tf
from bluering_letter_learning.utils import motion
from bluering_letter_learning.utils.writing_helper import *
import time


class RobotController:
    def __init__(self, naoIP, naoPort, effector='RArm', naoWriting=True, naoSpeaking=True, naoStanding=True, language='english', isFrontInteraction=True, alternateSidesLookingAt=False) -> None:
        qi_url = f'tcp://{naoIP}:{naoPort}'
        rospy.loginfo(f'[RobotController][RobotController] Connecting to qi_url={qi_url}')
        app = qi.Application(url=qi_url)
        app.start()
        # app.run()
        rospy.loginfo(f'[RobotController][RobotController] app started')

        self.session = app.session

        self.naoWriting = naoWriting #? whether the robot should write
        self.naoSpeaking = naoSpeaking #? whether the robot should speak
        self.naoStanding = naoStanding #? whether the robot should stand
        self.effector = effector
        self.isFrontInteraction = isFrontInteraction

        self.alternateSidesLookingAt = alternateSidesLookingAt
        self.nextSideToLookAt = 'Right'

        #? all services needed
        self.motionProxy = self.session.service('ALMotion')
        # return
        self.postureProxy = self.session.service('ALRobotPosture')
        self.ttsProxy = self.session.service('ALTextToSpeech')
        

        #? init state for robot
        self.init()

        #? set language for the robot
        self.setLanguage(language)

        #? load angles for each position, these configs are used to control robot
        self.__head_configs__ = InteractionSettings.getHeadAngles()


        """ params for trajectory_following """
        self.hasFallen = False #? to keep track if the robot has fallen. if robot has fallen it will have a hard time getting up if the effector is still trying to be kept in a particular position
        self.trackerProxy = self.session.service('ALTracker')
        self.tl = tf.TransformListener()

        self.space = motion.FRAME_ROBOT
        self.effectorList = ['RArm']
        self.axisMaskList = [motion.AXIS_MASK_X + motion.AXIS_MASK_Y + motion.AXIS_MASK_Z + motion.AXIS_MASK_WX]
        self.isAbsolute = True

        pass


    def setLanguage(self, language) -> None:
        """
        Set language for NAO to speak
        """
        self.language = language
        #? load default phrases to speak
        self.introPhrase, demo_response_phrases, asking_phrases_after_feedback, asking_phrases_after_word, word_response_phrases, word_again_response_phrases, testPhrase, thankYouPhrase = InteractionSettings.getPhrases(self.language)
        rospy.loginfo(f'[RobotController][setLanguage] setting language')
        self.ttsProxy.setLanguage(language.capitalize())


    def init(self) -> None:
        rospy.loginfo(f'[RobotController][init] called')

        #? These two lines are init state in trajectory_following code
        self.motionProxy.wbEnableEffectorControl(self.effector, False) #? if robot has fallen it will have a hard time getting up if the effector is still trying to be kept in a particular position. So, at init state, set effector to False and make the robot stand
        self.postureProxy.goToPosture('Stand', 0.2)

        #? belows are init state in letter_learning code
        if self.naoWriting: #? if we want robot to write
            if self.naoStanding: #? if we want robot to stand
                rospy.loginfo(f'[RobotController][init] removed goToPosture StandInit')
                # self.postureProxy.goToPosture('StandInit', 0.2) #? make the robot stand
                rospy.loginfo(f'[RobotController][init] wbEnableEffectorControl True')
                self.motionProxy.wbEnableEffectorControl(self.effector, True) #? turn whole body motion control on

            else: #? if we dont need the robot to stand
                rospy.loginfo(f'[RobotController][init] rest')
                self.motionProxy.rest() #? goes to a relaxed and safe position and sets Motor off. For Nao6, goes to the Crouch posture and sets the Stiffness off.

                #? the below dont know what's for
                rospy.loginfo(f'[RobotController][init] setStiffnesses Head LArm RArm')
                self.motionProxy.setStiffnesses(['Head', 'LArm', 'RArm'], 0.5)
                rospy.loginfo(f'[RobotController][init] setStiffnesses LHipYawPitch LHipRoll LHipPitch')
                self.motionProxy.setStiffnesses(['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch'], 0.8)
                rospy.loginfo(f'[RobotController][init] wbEnableEffectorControl False')
                self.motionProxy.wbEnableEffectorControl(self.effector, False) #? turn whole body motion control off

            self.armJoints_standInit = self.motionProxy.getAngles(self.effector, True)
            rospy.loginfo(f'[RobotController][init] self.armJoints_standInit {self.armJoints_standInit}')


    def speak(self, toSay) -> None:
        if self.naoSpeaking: #? we want the robot to speak
            self.ttsProxy.say(toSay) #? say the string we tell the robot to say
            rospy.loginfo(f'[RobotController][lookAndAskForFeedback] toSay: {toSay}')


    def lookAndAskForFeedback(self, toSay, side) -> None:
        """
        Tell the robot to stop doing whatever it's doing, looking the child, and asking for feedback
        """
        if self.naoWriting: #? if the robot can write
            #? make sure the robot put arm down first
            self.motionProxy.angleInterpolationWithSpeed(self.effector, self.armJoints_standInit, 0.3)

        if self.isFrontInteraction:
            self.motionProxy.setAngles(['HeadYaw', 'HeadPitch'],self.__head_configs__['headAngles_lookAtPerson_front'], 0.2)
        else:
            if side == 'Right': #? the person is on our right, then tell the robot to look right
                self.motionProxy.setAngles(['HeadYaw', 'HeadPitch'], self.__head_configs__['headAngles_lookAtPerson_right'], 0.2)
            else: #? the person is on our left, then tell the robot to look left
                self.motionProxy.setAngles(['HeadYaw', 'HeadPitch'], self.__head_configs__['headAngles_lookAtPerson_left'], 0.2)
        self.speak(toSay)


    def lookAtTablet(self) -> None:
        """
        Tell the robot to look at tablet
        """
        rospy.loginfo(f'[RobotController][lookAtTablet] called')
        if self.isFrontInteraction:
            rospy.loginfo(f'[RobotController][lookAtTablet] setAngles headAngles_lookAtTablet_down')
            self.motionProxy.setAngles(['HeadYaw', 'HeadPitch'], self.__head_configs__['headAngles_lookAtTablet_down'], 0.2)

        else:
            if self.effector=='RArm': #? tablet will be on our right
                rospy.loginfo(f'[RobotController][lookAtTablet] setAngles headAngles_lookAtTablet_right')
                self.motionProxy.setAngles(['HeadYaw', 'HeadPitch'], self.__head_configs__['headAngles_lookAtTablet_right'], 0.2)
            else: 
                rospy.loginfo(f'[RobotController][lookAtTablet] setAngles headAngles_lookAtTablet_left')
                self.motionProxy.setAngles(['HeadYaw', 'HeadPitch'], self.__head_configs__['headAngles_lookAtTablet_left'], 0.2)


    def sayHello(self, personSide) -> None:
        self.say(self.introPhrase, personSide)


    def say(self, phrase, personSide) -> None:
        if self.naoWriting:
            if self.naoStanding:
                rospy.loginfo(f'[RobotController][say] removed goToPosture(StandInit, 0.3)')
                # self.postureProxy.goToPosture('StandInit', 0.3)
            else:
                self.motionProxy.rest()
                self.motionProxy.setStiffnesses(['Head', 'LArm', 'RArm'], 0.5)
                self.motionProxy.setStiffnesses(['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch'], 0.8)

        if self.naoSpeaking:
            if self.alternateSidesLookingAt:
                self.lookAndAskForFeedback(phrase, self.nextSideToLookAt)
            else:
                self.lookAndAskForFeedback(phrase, personSide)


    def followTrajectory(self, traj) -> None:
        rospy.loginfo(f'[RobotController][followTrajectory] got traj at {datetime.now()}') 

        if self.hasFallen == False: #? no harm in executing trajectory
            if self.effector == 'LArm':
                # self.motionProxy.openHand('LHand')
                self.motionProxy.closeHand('LHand')
                # roll = -1.7 #? rotate wrist to the left (about the x axis, w.r.t. robot frame)
                roll = 1.7
            else:
                # self.motionProxy.openHand('RHand')
                self.motionProxy.closeHand('RHand')
                roll = 1.7 #? rotate wrist to the right (about the x axis, w.r.t. robot frame)

            target = PoseStamped()
            target_frame = traj.header.frame_id
            target.header.frame_id = target_frame
            
            points = []
            timeList = []
            # stime = 2.0
            stime = 1.0
            rospy.loginfo(f'[RobotController][followTrajectory] traj.poses len : {len(traj.poses)}')
            for i, trajp in enumerate(traj.poses):
                trajp.pose.position.z = 0.01
                target.pose.position = deepcopy(trajp.pose.position)
                target.pose.orientation = deepcopy(trajp.pose.orientation)

                # rospy.loginfo(f'[RobotController][followTrajectory]   [ ] traij i = {i}')
                # rospy.loginfo(f'[RobotController][followTrajectory]      [ ] target.pose.position : \n{target.pose.position}')
                target_robot = self.tl.transformPose('base_footprint', target)

                #? ------ add time --------
                stime += 0.2
                if points: #? what does this check? points is true or points not null ?
                    # rospy.loginfo(f'[RobotController]      [+] <if points> reached')
                    #? if more than 3mm between points, it means letter separation, give more time to reach this point
                    if gapBetweenPoints([points[-1][1], points[-1][2]], [target_robot.pose.position.y, target_robot.pose.position.z]) > 0.003:
                        stime += 0.5 #traj.poses[i+1].header.stamp - trajp.header.stamp


                #? ------- add position ----------
                points.append([0, target_robot.pose.position.y, target_robot.pose.position.z, roll, 0, 0])
                timeList.append(stime)
                rospy.loginfo(f'[RobotController][followTrajectory]      [ ] append to points : [0, {target_robot.pose.position.y}, {target_robot.pose.position.z}, {roll}, 0, 0]')
                
            
            #? wait until time instructed to start executing
            # now_stamp = rospy.Time.now()
            # sleepTimeSec = (traj.header.stamp - now_stamp).to_sec()
            # rospy.loginfo(f'[RobotController][followTrajectory] traj.header.stamp : {traj.header.stamp}  |  now_stamp : {now_stamp}  |  sleepTimeSec : {sleepTimeSec}')

            # # time.sleep(sleepTimeSec)
            # rospy.loginfo(f'[RobotController][followTrajectory] executing rest of traj for {sleepTimeSec} at {now_stamp} and call positionInterpolations')

            # startTime = datetime.now()

            rospy.sleep(traj.header.stamp-rospy.Time.now())
            startTime = rospy.Time.now()

            #? send position to proxy
            points_scaled_flipped = scaleAndFlipPoints(points)
            points_scaled_flipped = [[float(v) for v in item] for item in points_scaled_flipped] #? convert list of numpy.float64 to list of float

            # rospy.loginfo(f'[RobotController][followTrajectory]    points : {points} | type = {type(points)} | type = {type(points_scaled_flipped[0][1])}')
            # rospy.loginfo(f'[RobotController][followTrajectory]    self.effectorList : {self.effectorList} | type = {type(self.effectorList)}')
            # rospy.loginfo(f'[RobotController][followTrajectory]    self.space : {self.space} | type = {type(self.space)}')
            rospy.loginfo(f'[RobotController][followTrajectory]    points_scaled_flipped : {points_scaled_flipped} | type = {type(points_scaled_flipped)} | type = {type(points_scaled_flipped[0][1])}')
            # rospy.loginfo(f'[RobotController][followTrajectory]    self.axisMaskList : {self.axisMaskList} | type = {type(self.axisMaskList)} | type = {type(self.axisMaskList[0])}')
            rospy.loginfo(f'[RobotController][followTrajectory]    timeList : {timeList} | type = {type(timeList)} | type = {type(timeList[0])}')

            # self.motionProxy.wbEnableBalanceConstraint(True, 'Legs')
            # self.motionProxy.wbFootState('Fixed', 'Legs')
            self.motionProxy.positionInterpolations(self.effectorList, self.space, points_scaled_flipped, self.axisMaskList, timeList, True)

            # rospy.loginfo(f'[RobotController][followTrajectory] Time taken for rest of trajectory: {(datetime.now() - startTime)}')
            rospy.loginfo(f'[RobotController][followTrajectory] Time taken for rest of trajectory: {(rospy.Time.now() - startTime)}')

            self.postureProxy.goToPosture('Stand', 0.2) #? finish writing, back to standing position

        else:
            rospy.loginfo(f'[RobotController][followTrajectory] Got traj but not allowed to execute it because I\'ve fallen')

