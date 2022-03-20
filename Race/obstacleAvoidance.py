## library for obstacle avoidance and lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2020

from curses import KEY_PPAGE
from turtle import left
from pixyBot import pixyBot
from pixyCam import pixyCam
from laneFollower import laneFollower
from PIDcontroller import PID_controller
from time import time
import numpy as np
from numpy import mean
import math
from math import pi
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
        self.rightLineID    = 4
        self.obstacleID     = 5

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
    def drive(self, drive, bias, time=0): # Differential drive function
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
        if time == 0:
            self.bot.setMotorSpeeds(lDrive, rDrive)
        else:
            self.bot.setMotorSpeeds(lDrive, rDrive, time)

    # output            - -1 if error
    # blockIdx          - index of block in block list that you want to save parameters for
    def getBlockParams(self, blockIdx):
        if (self.cam.newCount-1) < blockIdx or blockIdx < 0: # do nothing when block doesn't exist
            return -1
        else:
            pixelSizeH = self.cam.newBlocks[blockIdx].m_height;
            pixelSizeW = self.cam.newBlocks[blockIdx].m_width;
            distanceH = (self.focalLength*140)/pixelSizeH
            distanceW = 2*2*300/pixelSizeW
            
            bottomEdge = self.cam.newBlocks[blockIdx].m_y + pixelSizeH/2
            alfa = 90 - (bottomEdge/self.cam.pixyMaxY) * self.cam.pixyY_FoV + self.cam.pixyY_FoV/2 - 20
            distanceBottomEdge = self.cam.height * math.tan(alfa*pi/180)
            
            angleSize = pixelSizeW/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x -  self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular error of block relative to front

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
    
    def get_largest_block(self,center_id, left_id, right_id):
        # If there is no block of either color, set that id really high - e.g 50
        if center_id == -1:
            center_id = 50
        if left_id == -1:
            left_id = 50
        if right_id == -1:
            right_id = 50

        # group all the marker ids together
        array = [center_id, left_id, right_id]
        # set initial id to -1, this is the value returned then we know that the camera cannot see anything.
        largest_block = -1

        # If color markers are recognised (i.e. min number in array is smaller than 50)
        if array[array.index(min(array))] < 50:     # find the id of the largest block
            largest_block = array.index(min(array))

        # return the id of the largest block: 0-center, 1-left, 2-right, -1- no marker recognised
        return  largest_block

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
                if self.blockDistance[-1] <= 0.1:
                    close = True
                    self.cam.getLatestBlocks()
                    obstacleBlock = self.cam.isInView(self.obstacleID) 
                    while True:
                        servoError,servoPos = self.visTrack(obstacleBlock)
                        print(servoPos)
                        if servoPos > 0:
                            self.drive(0.4, 0.4)
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

       
        kp = 0.015
        kd = 0
        ki = 0
        IntegralError = 0
        PreviousError = 0
        self.bot.setServoPosition(0) # set servo to centre
        speed = 0.4
        finish = False
        line = 1 
        while True:
            self.cam.getLatestBlocks()
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID) 
            if obstacleBlock >= 0:
                self.getBlockParams(obstacleBlock)
                print(self.blockDistance[-1])
                if self.blockDistance[-1] <= 0.1: 
                    if line == 0: #if one the left turn to face the right 
                        line = 1
                        self.drive(0.5, 0.3, 0.5)
                        print('right turn')
                    else:
                        line = 0
                        self.drive(0.5, -0.4, 0.5)
                        print('left turn')
       
            if finish:
                break    
        
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            if centerLineBlock >= 0:
                print('seeing red')
                break #go to race code
            if line == 0:
                block = leftLineBlock
                id = self.leftLineID
            else:
                block = rightLineBlock
                id = self.rightLineID
            if block >= 0:
                self.getCenterBlockParams(block)
                servo_error, servo_position  =  self.visTrack(block)
                CL_angular_error = self.blockAngle[-1]  
                # Account for the rotation of the camera
                camera_rotation = -(servo_position/50) * 25
                angle = CL_angular_error + camera_rotation
                lineSteering = angle * 0.02
                self.drive(speed,lineSteering)
            else: #look for the line
                for i in range(-35, 35):
                    self.bot.setMotorSpeeds(0,0)
                    self.bot.setServoPosition(i)
                    self.cam.getLatestBlocks()
                    seen = self.cam.isInView(id)
                    if seen >= 0:
                        break
                

        return
