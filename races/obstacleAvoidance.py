## library for obstacle avoidance and lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2020

import math
from math import pi
from time import time

from pixyBot import pixyBot
from pixyCam import pixyCam


# obstacleAvoidance class: has a bunch of convinient functions for navigation
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
#
# methods
#   drive                       - differential drive function that takes bias for the right wheel speed
#   getBlockParams              - get and store the size and anglular error of a selected block
#   visTrack                    - move camera to point at selected block
#   stopAtStationaryObstacles   - move bot along a lane until it encounters an obstacle
#   avoidStationaryObstacles    - move bot along a lane and move to avoid stationary obstacles
class obstacleAvoidance(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
        self.bot = bot

        self.bot = bot
        self.cam = cam

        self.centerLineID = 1  # red
        self.leftLineID = 2  # green
        self.rightLineID = 3  # blue, 4 is purple
        self.obstacleID = 5  # yellow

        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]
        self.blockAngleCenter = [float('nan') for i in range(nObservations)]
        self.blockDistance = [float('nan') for i in range(nObservations)]
        self.focalLength = 2.8

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
            pixelSizeH = self.cam.newBlocks[blockIdx].m_height
            pixelSizeW = self.cam.newBlocks[blockIdx].m_width
            distanceH = (self.focalLength * 140) / pixelSizeH
            distanceW = 2 * 2 * 300 / pixelSizeW

            bottomEdge = self.cam.newBlocks[blockIdx].m_y + pixelSizeH / 2
            alfa = 90 - (bottomEdge / self.cam.pixyMaxY) * self.cam.pixyY_FoV + self.cam.pixyY_FoV / 2 - 20
            # fixme: the angle calculation
            #alfa = 90 - abs((bottomEdge / self.cam.pixyMaxY) * self.cam.pixyY_FoV - self.cam.pixyY_FoV / 2)

            distanceBottomEdge = (self.cam.height * 100 * math.tan(alfa * pi / 180)) * 1.0098 + 2.193

            angleSize = pixelSizeW / self.cam.pixyMaxX * self.cam.pixyX_FoV  # get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX
            angleError = pixelError / self.cam.pixyMaxX * self.cam.pixyX_FoV  # get angular error of block relative to front

            # save params
            self.blockSize.append(angleSize)
            self.blockAngle.append(angleError)
            self.frameTimes.append(time())
            self.blockDistance.append(distanceBottomEdge)
            # remove oldest params
            self.blockSize.pop(0)
            self.blockAngle.pop(0)
            self.frameTimes.pop(0)

    def getCenterBlockParams(self, blockIdx):
        if (self.cam.newCount - 1) < blockIdx or blockIdx < 0:  # do nothing when block doesn't exist
            return -1
        else:
            pixelSize = self.cam.newBlocks[blockIdx].m_width
            # print(pixelSize)
            distance = (self.focalLength * 65) / pixelSize
            angleSize = pixelSize / self.cam.pixyMaxX * self.cam.pixyX_FoV  # get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX
            angleError = pixelError / self.cam.pixyMaxX * self.cam.pixyX_FoV  # get angular error of block relative to front

            self.blockAngleCenter.append(angleError)
            self.blockAngleCenter.pop(0)

    # output            - tracking error in ([deg]) and new camera angle in ([deg]) (tuple), or -1 if error
    # blockIdx          - index of block in block list that you want to track with the camera
    def visTrack(self, blockIdx):  # Get pixycam to rotate to track an object
        if blockIdx < 0:  # do nothing when block doesn't exist
            self.bot.setServoPosition(0)
            return 0, 0
        else:
            pixelError = self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX  # error in pixels
            visAngularError = -(pixelError / self.cam.pixyMaxX * self.cam.pixyX_FoV)  # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(
                visAngularError)  # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition

    # output            - none
    # speed             - general speed of bot
    def stopAtStationaryObstacles(self, speed):
        targetDistance = 20  # cm
        targetAngle = -50  # degrees
        kp = 0.015
        kd = 0
        ki = 0
        IntegralError = 0
        PreviousError = 0
        self.bot.setServoPosition(0)  # set servo to centre
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID)
            servoError, servoPos = self.visTrack(obstacleBlock)
            close = False
            finish = False
            if obstacleBlock >= 0:
                self.getBlockParams(obstacleBlock)
                print("Block Distance: ", self.blockDistance[-1])
                if self.blockDistance[-1] <= targetDistance:
                    close = True
                    print("Is pbstacle close -- ", close)
                    self.drive(0, 0)
                    while True:
                        self.cam.getLatestBlocks()
                        obstacleBlock = self.cam.isInView(self.obstacleID)
                        servoError, servoPos = self.visTrack(obstacleBlock)
                        # print([servoError,servoPos])
                        bias = 0.003 * (targetAngle - servoPos)
                        self.bot.setMotorSpeeds(bias, -bias)
                        print("Bias -- ", bias)
                        #if abs(bias) <= 0.06:
                        if abs(bias) <= 0.06:
                            finish = True
                            print('Done')
                            break

            if finish:
                break

            if not close:
                centerLineBlock = self.cam.isInView(self.centerLineID)
                if centerLineBlock >= 0:
                    self.getCenterBlockParams(centerLineBlock)
                    servo_error, servo_position = self.visTrack(centerLineBlock)
                    P = servo_position
                    I = IntegralError + servo_position
                    D = (servo_position - PreviousError)
                    bias = -(P * kp + I * ki + D * kd)
                    PreviousError = servo_position
                    IntegralError = IntegralError + servo_position

                    self.drive(speed, bias)

        return

    # output            - none
    # speed             - general speed of bot
    def avoidStationaryObstacles(self, speed):

        kp = 0.015
        kd = 0
        ki = 0
        IntegralError = 0
        PreviousError = 0

        self.bot.setServoPosition(0)  # set servo to centre
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID)
            speed = 0.4
            close = False
            finish = False
            if obstacleBlock >= 0:
                self.getBlockParams(obstacleBlock)
                print("Block distance: ", self.blockDistance[-1])
                #if self.blockDistance[-1] <= 0.07:
                if self.blockDistance[-1] <= 10:
                    print('yay ---- ', "Too close")
                    close = True
                    while True:
                        servoError, servoPos = self.visTrack(obstacleBlock)
                        print("Servo position: ", servoPos)
                        if servoPos > 0:
                            # servo truns left, the bot should turn right
                            value = 1
                            print("Servo turns left")
                            self.drive(0.6, 0.4)
                        else:
                            print("Servo turns right")
                            value = 2
                            self.drive(0.6, -0.4)
                        #if servoPos > 30 or servoPos < -30:
                        if abs(servoPos) > 30:
                            close = False
                            print("Turn a large angle for servo.")
                            # finish = True
                            if value == 1:
                                self.bot.setMotorSpeeds(0.3, 0.6, 0.3)
                                self.bot.setServoPosition(servoPos - 30)
                            elif value == 2:
                                self.bot.setMotorSpeeds(0.5, 0.2, 0.3)
                                self.bot.setServoPosition(servoPos + 30)
                            break

            if finish:
                break

            if not close:
                self.getCenterBlockParams(centerLineBlock)
                centerLineBlock = self.cam.isInView(self.centerLineID)
                if centerLineBlock >= 0:
                    self.getCenterBlockParams(centerLineBlock)
                    servo_error, servo_position = self.visTrack(centerLineBlock)
                    bias = -servo_position * 0.01
                    self.drive(speed, bias)
                    print("Follow the center line!")
                else:
                    print("Cannot detect center line!")
                    self.drive(0, 0)

        return
