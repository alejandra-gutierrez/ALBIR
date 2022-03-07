## library for obstacle avoidance and lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2020

from curses import KEY_PPAGE
from pixyBot import pixyBot
from pixyCam import pixyCam
from laneFollower import laneFollower
from PIDcontroller import PID_controller
from time import time
from numpy import mean
import math

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

        self.centerLineID   = 1
        self.leftLineID     = 2
        self.rightLineID    = 3
        self.obstacleID     = 4

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
            pixelSize = self.cam.newBlocks[blockIdx].m_height;
            distance = (self.focalLength*140)/pixelSize
            #distance = 2*2*300/pixelSize
            angleSize = pixelSize/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x -  self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular error of block relative to front

            # save params
            self.blockSize.append(angleSize)
            self.blockAngle.append(angleError)
            self.frameTimes.append(time())
            self.blockDistance.append(distance)
            # remove oldest params
            self.blockSize.pop(0)
            self.blockAngle.pop(0)
            self.frameTimes.pop(0)
    
    def getCenterBlockParams(self, blockIdx):
        if (self.cam.newCount-1) < blockIdx or blockIdx < 0: # do nothing when block doesn't exist
            return -1
        else:
            pixelSize = self.cam.newBlocks[blockIdx].m_width;
            #print(pixelSize)
            distance = (self.focalLength*65)/pixelSize
            angleSize = pixelSize/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x -  self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular error of block relative to front

            
            self.blockAngleCenter.append(angleError)
            self.blockAngleCenter.pop(0)

    # output            - tracking error in ([deg]) and new camera angle in ([deg]) (tuple), or -1 if error
    # blockIdx          - index of block in block list that you want to track with the camera
    def visTrack(self, blockIdx): # Get pixycam to rotate to track an object
        if blockIdx < 0: # do nothing when block doesn't exist
            self.bot.setServoPosition(0)
            return 0, 0
        else:
            pixelError =  self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX # error in pixels
            visAngularError = -(pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV) # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition

    # output            - none
    # speed             - general speed of bot
    def stopAtStationaryObstacles(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID) 
            speed = 0.4
            close = False
            finish = False
            if obstacleBlock >= 0:
                self.getBlockParams(obstacleBlock)
                print(self.blockDistance[-1])
                if self.blockDistance[-1] <= 2.0:
                    close = True
                    while True:
                        servoError,servoPos = self.visTrack(obstacleBlock)
                        print(servoPos)
                        if servoPos > 0:
                            self.drive(0.6, 0.4)
                        else:
                            self.drive(0.4, -0.4)
                        if servoPos > 40 or servoPos < -50: 
                            finish = True
                            break    
                    
            if finish:
                break
                
            if not close:
                centerLineBlock = self.cam.isInView(self.centerLineID)
                if centerLineBlock >= 0:
                    self.getCenterBlockParams(centerLineBlock)
                    servo_error, servo_position  =  self.visTrack(centerLineBlock)
                    bias = -servo_position*0.01
                    self.drive(speed, bias)


        return

    # output            - none
    # speed             - general speed of bot
    def avoidStationaryObstacles(self, speed):

        #self.bot.setServoPosition(-10)

        #while True:
        #    self.cam.getLatestBlocks()
        #    self.getBlockParams(self.cam.isInView(self.obstacleID))

            ###Level 4### Please insert code here to derive non-zero obstacleSteering and keep the robot running
            ### You need to first identify the obstacle and decide whether you want to visaully track it. Then you use what you learn from Level 1,2 and 3 to implement detection and steering.
            ### Hint: self.cam.newBlocks and self.cam.oldBlocks are sorted by decreasing pixel size. Do you always want to be focused on the largest object?

            #lineSteering = 0
            #obstacleSteering = 0

            #steering = obstacleSteering + lineSteering # we set the final steering as a linear combination of the obstacle steering and center line steering - but it's all up to you!
            #self.drive(targetSpeed, steering)
            ###
        #count = 0
        #while True:
        #    self.cam.getLatestBlocks()
        #    centerLineBlock = self.cam.isInView(self.centerLineID)
        #    obstacleBlock = self.cam.isInView(self.obstacleID) 
            #targetSpeed = 0.2
        #    speed = 0.4
        #    if obstacleBlock >= 0:
        #        self.getBlockParams(obstacleBlock)
        #        if count == 0 and self.blockDistance[-1] > 3.2:
        #            stop = 2
        #        elif count == 0 and self.blockDistance[-1] <= 3.2:
        #            stop = 1.5
        #        print(self.blockDistance[-1])
        #        count += 1
        #        if mean(self.blockDistance[-6:-1]) <= stop:
        #            servoError,servoPos = self.visTrack(obstacleBlock)
        #            print(servoPos)
        #            if servoPos +10.3 >= 0: #obstacle is on the left
        #                self.bot.setMotorSpeeds(0.4, 0.2, 0.8)
        #                self.bot.setMotorSpeeds(0.1, 0.4, 0.5)
        #                self.bot.setMotorSpeeds(0.2, 0.2, 0.6)
                        #print('done')
        #                self.centreTrack(0.2)
        #                continue
        #            elif servoPos - 5 < 0: #obstacle is on the right
        #                self.bot.setMotorSpeeds(0.2, 0.4, 1)
        #                self.bot.setMotorSpeeds(0.3, 0.1, 0.5)
        #                self.bot.setMotorSpeeds(0.2, 0.2, 0.6)
        #                self.centreTrack(0.2)
        #                continue

        #    self.centreTrack(speed)
            #self.drive(speed, 0)
        
        self.bot.setServoPosition(0) # set servo to centre
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID) 
            speed = 0.4
            close = False
            finish = False
            if obstacleBlock >= 0:
                self.getBlockParams(obstacleBlock)
                print(self.blockDistance[-1])
                if self.blockDistance[-1] <= 2.0:
                    print('yay')
                    close = True
                    while True:
                        servoError,servoPos = self.visTrack(obstacleBlock)
                        print(servoPos)
                        if servoPos > 0:
                            value = 1
                            self.drive(0.6, 0.4)
                        else:
                            value = 2
                            self.drive(0.4, -0.4)
                        if servoPos > 40 or servoPos < -50: 
                            if value == 1:
                                self.bot.setMotorSpeeds(0.4, 0.6, 0.8)
                            elif value == 2:
                                self.bot.setMotorSpeeds(0.6, 0.4, 0.8)
                            self.getCenterBlockParams(centerLineBlock)
                            servo_error, servo_position  =  self.visTrack(centerLineBlock)
                            close = False
                            break 
                        
                
            if not close:
                centerLineBlock = self.cam.isInView(self.centerLineID)
                if centerLineBlock >= 0:
                    self.getCenterBlockParams(centerLineBlock)
                    servo_error, servo_position  =  self.visTrack(centerLineBlock)
                    bias = -servo_position*0.01
                    self.drive(speed, bias)

        return
