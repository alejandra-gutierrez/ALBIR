# -*- coding: UTF-8 -*-
"""
@Project : ALBIR 
@File    : laneFollower_2.py
@IDE     : PyCharm 
@Author  : Peter
@Date    : 19/03/2022 15:03 
@Brief   : 
"""

## library for lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2021

from time import time

from PIDcontroller import PID_controller
from pixyBot import pixyBot
from pixyCam import pixyCam


# laneFollower class: has a bunch of convinient functions for navigation
#
# input arguments
#   bot                         - (optional) pixyBot object
#   cam                         - (optional) pixyCam object
#
# attributes - feel free to add more!
#   bot                         - pixyBot object
#   cam                         - pixyCam object
#   IDs                         - signature ID number for various colour blocks
#   frameTimes                  - list of frameTimes for blockSize and blockAngle
#   blockSize                   - list of angular size of queried block over frameTimes in ([deg])
#   blockAngle                  - list of angular error of queried block over frameTimes in ([deg])
#   biasControl                 - bias controller for lane following
#
# methods
#   drive                       - differential drive function that takes bias for right left wheel speed
#   getBlockParams              - get and store the size and anglular error of a selected block
#   visTrack                    - move camera to point at selected block
#   follow                      - move bot along a lane, following markers
class laneFollower(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
        self.bot = bot

        self.bot = bot
        self.cam = cam
        self.biasControl = PID_controller(0.015, 0, 0.001)

        self.centerLineID = 1
        self.leftLineID = 2
        self.rightLineID = 3
        self.obstacleID = 5

        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]

    # output            - none
    # drive             - desired general bot speed (-1~1)
    # bias              - ratio of drive speed that is used to turn right (-1~1)
    def drive(self, drive, bias):  # Differential drive function
        if bias > 1:
            bias = 1
        if bias < -1:
            bias = -1

        maxDrive = 1  # set safety limit for the motors

        totalDrive = drive * maxDrive  # the throttle of the car
        diffDrive = bias * totalDrive  # set how much throttle goes to steering
        straightDrive = totalDrive - abs(diffDrive)  # the rest for driving forward (or backward)

        lDrive = straightDrive + diffDrive
        rDrive = straightDrive - diffDrive
        self.bot.setMotorSpeeds(lDrive, rDrive)

    # output            - -1 if error
    # blockIdx          - index of block in block list that you want to save parameters for
    def getBlockParams(self, blockIdx):
        if (self.cam.newCount - 1) < blockIdx or blockIdx < 0:  # do nothing when block doesn't exist
            return -1
        else:
            pixeleftSize = self.cam.newBlocks[blockIdx].m_width
            angleSize = pixeleftSize / self.cam.pixyMaxX * self.cam.pixyX_FoV  # get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX
            angleError = pixelError / self.cam.pixyMaxX * self.cam.pixyX_FoV  # get angular error of block relative to front

            # save params
            self.blockSize.append(angleSize)
            self.blockAngle.append(angleError)
            self.frameTimes.append(time())

            # remove oldest params
            self.blockSize.pop(0)
            self.blockAngle.pop(0)
            self.frameTimes.pop(0)

    # output            - tracking error in ([deg]) and new camera angle in ([deg]) (tuple), or -1 if error
    # blockIdx          - index of block in block list that you want to track with the camera
    def visTrack(self, blockIdx):  # Get pixycam to rotate to track an object
        if blockIdx < 0:  # do nothing when block doesn't exist
            self.bot.setServoPosition(0)
            return -1
        else:
            pixelError = self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX  # error in pixels
            visAngularError = -(pixelError / self.cam.pixyMaxX * self.cam.pixyX_FoV)  # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(
                visAngularError)  # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition

    def mapf(self, angle_error, min_val, max_val, preferred_min, preferred_max):
        return (angle_error - min_val) / (max_val - min_val) * (preferred_max - preferred_min) + preferred_min

    # output            - none
    # speed             - general speed of bot
    def follow(self, speed):

        self.bot.setServoPosition(0)  # set servo to centre
        print("Centred")
        # self.bot.setMotorSpeeds(-0.1, 0)
        self.bot.setMotorSpeeds(0, 0)

        while True:
            lost = False
            self.cam.getLatestBlocks()
            # Check the line block
            centerLineBlock = self.cam.isInView(self.centerLineID)
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)

            flag = 1  # The marker to check the error

            turning_rate = 0
            weight_shift = 0.2

            mapped_leftAngle = 0
            mapped_rightAngle = 0
            mapped_centerAngle = 0

            if centerLineBlock >= 0:
                flag = 0
                self.getBlockParams(centerLineBlock)
                self.visTrack(centerLineBlock)  # only use red line to modify the servo
                centerAngle = self.blockAngle[-1]
                # map the center angle to [-1, 1]
                mapped_centerAngle = self.mapf(angle_error=centerAngle, min_val=-25, max_val=25, preferred_min=-1,
                                               preferred_max=1)
                print("Center line -- Center angle: ", centerAngle, "Mapped centerAngle: ", mapped_centerAngle)


            else:
                flag = 1  # we need to count the left and right error

                if leftLineBlock >= 0:
                    # we detect the left line
                    leftAngle = self.blockAngle[-1]
                    self.visTrack(leftLineBlock)
                    mapped_leftAngle = self.mapf(leftAngle, min_val=-25, max_val=25, preferred_min=0, preferred_max=1)
                    print("Left line -- Left angle: ", leftAngle, "Mapped leftAngle: ", mapped_leftAngle)
                if rightLineBlock >= 0:
                    rightAngle = self.blockAngle[-1]
                    self.visTrack(rightLineBlock)
                    mapped_rightAngle = self.mapf(rightAngle, min_val=-25, max_val=25, preferred_min=-1,
                                                  preferred_max=0)
                    print("Right line -- Right angle: ", rightAngle, "Mapped rightAngle: ", mapped_rightAngle)

            turning_rate = mapped_centerAngle + flag * (weight_shift * (mapped_leftAngle + mapped_rightAngle) -
                                                        weight_shift * mapped_centerAngle)

            print("Turning rate", turning_rate)
            if turning_rate != 0:
                # update the servo
                # self.bot.gimbal.update(turning_rate)

                self.drive(speed, turning_rate)
            else:
                print("lost")
                # go back and focus on the

                self.drive(0, 0)

        return

    def symmetricTrun(self, speed):
        # Here we define that if speed is positive, it will turn right,
        self.bot.setMotorSpeeds(speed, -speed)