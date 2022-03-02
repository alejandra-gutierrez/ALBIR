## library for lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2021

from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
import math

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
        self.leftLineID     = 2
        self.rightLineID    = 3

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
            pixelSize = self.cam.newBlocks[blockIdx].m_width;
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

    def follow(self, speed):
        # todo: It seems that the speed has not been involved in the code.

        self.bot.setServoPosition(0)
        self.drive(0, 0)

        kp = 0.015 #Proportional
        kd = 0 #Derivative
        ki = 0 #Integral
        integral = 0
        last_error = 0

        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            correction = 0
            servo_pos = 0
            line_markers = [centerLineBlock, leftLineBlock, rightLineBlock]
            self.getBlockParams(line_markers[0])
            largest_block = self.get_largest_block(centerLineBlock, leftLineBlock, rightLineBlock)
            print("the largest block is: ", largest_block) #  0-center, 1-left, 2-right, -1 no marker recognised
            if centerLineBlock >= 0: # drive while we see a line
            ###Level 1### Please insert code here to compute the center line angular error as derived from the pixel error, then use this value
            ### to come up with a steering command to send to self.drive(speed, steering) function. Remember the steering takes values between -1 and 1.
                CL_angular_error = self.blockAngle[-1] + correction

                # Account for the rotation of the camera ? To be discussed
                camera_rotation = -(servo_pos/50) * 25
                heading_error = CL_angular_error + camera_rotation
                derivative = heading_error - last_error
                integral = integral + heading_error
                lineSteering = heading_error * kp + derivative * kd + integral * ki
                last_error = heading_error
                ###Level 2### Please insert code here to follow the lane when the red line is obstructed. How would you make sure the pixyBot still stays on the road?
                ### Come up with a steering command to send to self.drive(speed, steering) function

            else: # stop the racer and wait for new blocks
                self.drive(0, 0)
        return
