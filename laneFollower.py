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
        self.biasControl = PID_controller(0.015, 0, 0)    

        self.centerLineID   = 1
        self.leftLineID     = 4
        self.rightLineID    = 2
        self.obstacleID     = 5

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
            pixeleftSize = self.cam.newBlocks[blockIdx].m_width;
            angleSize = pixeleftSize/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
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

        self.bot.setServoPosition(0) # set servo to centre
        print("Centred")
        self.bot.setMotorSpeeds(-0.1, 0)
     

        while True:
            lost = False
            self.cam.getLatestBlocks()                              #Get all current blocks etc
            centerLineBlock = self.cam.isInView(self.centerLineID) 
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID)

            centerightAngleProp = 0         #Centre angle proportion component
            rightAngleProp = 0         #Right angle proportion component
            leftAngleProp = 0         #Left angle proportion component
            
            rightSizeProp = 0         #Right size proportion component
            leftSizeProp = 0         #Left size proportion component
            
            rightAngle = 0
            leftAngle = 0
            centerAngle = 0
            centerightSize = 0
            rightSize = 0
            leftSize = 0
            
            
            if centerLineBlock >= 0:
                self.getBlockParams(centerLineBlock)
                centerAngle = self.blockAngle[-1]
                centerightSize = self.blockSize[-1]
                centerightAngleProp = 0.6
                
                
            if rightLineBlock >= 0:
                self.getBlockParams(rightLineBlock)
                rightAngle = self.blockAngle[-1]
                rightSize = self.blockSize[-1]
                rightAngleProp = 0.1
                #rightSizeProp = 0.05 #Not confident in block size scale factor yet

                
            if leftLineBlock >= 0:
                self.getBlockParams(leftLineBlock)
                leftAngle = self.blockAngle[-1]
                leftSize = self.blockSize[-1]
                leftAngleProp = 0.1
                #leftSizeProp = 0.05 #Not confident in block size scale factor yet
                
                
            if obstacleBlock >= 0:                      #Obstacle routine
                print('Obstacle')
                
            if (centerightAngleProp + rightAngleProp + leftAngleProp + rightSizeProp + leftSizeProp) == 0:      #Pan and find track 
                print('lost')
                lost = True
                self.drive(0,0)
            
            ##Find weights
            #Normalized by assumed max input (max angle or max block size etc)
            #Given proportions (as listed above)
            #Also given P number (below) to easily add or subtract weight (oposite of changing norms)
            
            sumK = centerightAngleProp + rightAngleProp + leftAngleProp + rightSizeProp + leftSizeProp 
            CaP = 1 #Center angle P
            RaP = 1 #Right angle P
            LaP = 1 #Left angle P
            RsP = 1 #Right size P 
            LsP = 1 #Left size P
            if not lost:
                waC = (centerightAngleProp/sumK) * (CaP/25)
                waR = (rightAngleProp/sumK) * (RaP/25)
                waL = (leftAngleProp/sumK) * (LaP/25)
                wsR = (rightSizeProp/sumK) * (RsP/1)
                wsL = (leftSizeProp/sumK) * (LsP/1)
                #print(waR, waL)
                
                thetaDot = -25*( waC*centerAngle + waR*(rightAngle-25) + waL*(leftAngle+25) + wsR*(-rightSize) + wsL*(leftSize) ) #at max error, should be scaled to max angle size
                print(waR*(rightAngle-25), waL*(leftAngle+25), thetaDot)
                
                ##Point cam in weighted direction
                visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(thetaDot)
                newServoPosition = self.bot.setServoPosition(visTargetAngle)     
                
                ##Drive robot in direction of cam 
                bias = -self.biasControl.update(newServoPosition) #PID for this at top of script
                #print(bias)
                self.drive(speed, bias)
            
 
        return