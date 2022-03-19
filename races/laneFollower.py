## library for lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2021

from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
import math
from PIDcontroller import PID_controller

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

        self.centerLineID   = 1
        self.leftLineID     = 4  #
        self.rightLineID    = 3  #
        self.obstacleID     = 5
        self.biasController = PID_controller(0.01, 0, 0)

        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]

    # output            - none
    # drive             - desired general bot speed (-1~1)
    # bias              - ratio of drive speed that is used to turn right (-1~1)
    def drive(self, drive, bias): # Differential drive function
        if bias > 1:
            bias = 1
        if bias < -1:
            bias = -1
            
        maxDrive = 1 # set safety limit for the motors

        totalDrive = drive * maxDrive # the throttle of the car
        diffDrive = bias * totalDrive # set how much throttle goes to steering
        straightDrive = totalDrive - abs(diffDrive) # the rest for driving forward (or backward)

        lDrive = straightDrive + diffDrive
        rDrive = straightDrive - diffDrive
        self.bot.setMotorSpeeds(lDrive, rDrive)

    # output            - -1 if error
    # blockIdx          - index of block in block list that you want to save parameters for
    def getBlockParams(self, blockIdx):
        if (self.cam.newCount-1) < blockIdx or blockIdx < 0: # do nothing when block doesn't exist
            return -1
        else:
            pixelSize = self.cam.newBlocks[blockIdx].m_width
            angleSize = pixelSize/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x -  self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular error of block relative to front

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
    def visTrack(self, blockIdx): # Get pixycam to rotate to track an object
        if blockIdx < 0: # do nothing when block doesn't exist
            self.bot.setServoPosition(0)
            return -1
        else:
            pixelError =  self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX # error in pixels
            visAngularError = -(pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV) # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition

    # output            - none
    # speed             - general speed of bot
    def follow(self, speed):

        self.bot.setServoPosition(0)  # set servo to centre
        print("Centred")
        self.drive(0, 0)  # set racer to stop

        while True:

            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)  # try to find centreline
            # leftLineBlock = self.cam.isInView(self.leftLineID)
            # rightLineBlock = self.cam.isInView(self.rightLineID)
            # obstacleBlock = self.cam.isInView(self.obstacleID)
            
            if centerLineBlock >= 0: # drive while we see a line
            # if leftLineBlock >= 0:
                print("Detect red line!")
                ###Level 1### Please insert code here to compute the center line angular error as derived from the pixel error, then use this value
                ### to come up with a steering command to send to self.drive(speed, steering) function. Remember the steering takes values between -1 and 1.
                visAngError, newServoPos = self.visTrack(centerLineBlock)
                self.visTrack(centerLineBlock)
                bias = - self.biasController.update(newServoPos)
                print('Bias -- ', bias)
                self.drive(speed, bias)

            else:
                print("Cannot detect center line!")

                # rightLineBlock = - 1
                # while True:
                #     # try to detect the blue line,
                #     rightLineBlock = self.cam.isInView(self.rightLineID)
                #     if rightLineBlock >= 0:
                #         print("Detect the right line!")
                #         visAngError, newServoPos = self.visTrack(rightLineBlock)
                #         bias = - self.biasController.update(newServoPos)
                #         self.drive(speed, bias)
                #         print('Bias -- ', bias)
                #         break
                #     else:
                #         self.bot.setServoPosition(10)
                #         self.bot.setMotorSpeeds(0.4, 0.4, 0.2)

                leftLineBlock = self.cam.isInView(self.leftLineID)
                rightLineBlock = self.cam.isInView(self.rightLineID)

                # right line detection -- find the blue line
                # We need to make the servo turn right a little
                if leftLineBlock >= 0:
                    print("Detect the left line!")
                    visAngError, newServoPos = self.visTrack(leftLineBlock)
                    bias = - self.biasController.update(newServoPos)

                    print('Bias -- ', bias)
                    self.drive(speed, bias)
                    continue

                if rightLineBlock >= 0:
                    print("Detect the right line!")
                    visAngError, newServoPos = self.visTrack(rightLineBlock)
                    bias = - self.biasController.update(newServoPos)
                    self.drive(speed, bias)
                    print('Bias -- ', bias)
                    continue

                else:
                    print("Cannot detect any line!")
                    targetSpeed = 0
                    bias = 0
                    self.drive(targetSpeed, bias)
                    print('Bias -- ', bias)
                    continue

        return