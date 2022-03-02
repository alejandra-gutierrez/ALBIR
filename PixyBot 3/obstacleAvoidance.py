## library for obstacle avoidance and lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2020

from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
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
    def __init__(self, bot: pixyBot = pixyBot(0), cam: pixyCam = pixyCam()):
        self.bot = bot

        self.bot = bot
        self.cam = cam

        self.centerLineID = 1
        self.leftLineID = 2
        self.rightLineID = 3
        self.obstacleID = 4

        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]

    # output            - none
    # drive             - desired general bot speed (-1~1)
    # bias              - ratio of drive speed that is used to turn right (-1~1)
    def drive(self, drive: int, bias: int) -> None:  # Differential drive function
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
    def getBlockParams(self, blockIdx: int):
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

            return None

    # output            - tracking error in ([deg]) and new camera angle in ([deg]) (tuple), or -1 if error
    # blockIdx          - index of block in block list that you want to track with the camera
    def visTrack(self, blockIdx):  # Get pixycam to rotate to track an object
        if blockIdx < 0:  # do nothing when block doesn't exist
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
        """We need to stop the vehicle at the given distance (15 - 50 cm) and angular position (-50 and 50). """

        self.bot.setServoPosition(0)  # set servo to centre
        self.drive(0, 0)  # set racer to stop

        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            if centerLineBlock != 1:
                print("Cannot detect the center red line!")

            # Try to find the obstacle: If the obstacle is detected, return 4
            obstacleBlock = self.cam.isInView(self.obstacleID)
            getBlockParamsSuccessMark = self.getBlockParams(obstacleBlock)  # try to find obstacle
            if not getBlockParamsSuccessMark:
                print("Cannot get the obstacle information!")

            #  Level 1 Set up an if statement to set targetSpeed = 0 when you detect an obstacle,
            #  and move targetSpeed = speed otherwise
            targetSpeed = speed
            if obstacleBlock == 4:
                # Level 2 Modify your if statement to set targetSpeed = 0 when you detect a obstacle at a specific
                #  distance.
                # Level 3 -  Modify your if statement to set targetSpeed = 0 when you detect a obstacle at within a
                #  distance and a frontal angular space.
                distance = self.__cal_distance__(block_idx=obstacleBlock)
                angle = self.__cal_angle__(block_idx=obstacleBlock)
                if 15 <= distance <= 50 and -50 <= angle <= 50:
                    # Set the drive speed to zero and no bias for the right wheel
                    targetSpeed = 0
                    self.drive(targetSpeed, 0)
                elif distance < 15:
                    print("Warning! The distance is too small.")
                else:
                    print("The distance is too long!")
            else:
                # todo: How could we import the lane follower?
                targetSpeed = speed

            # todo: How could we specify the `steering`?
            self.drive(targetSpeed, steering)
        return

    def __cal_distance__(self, block_idx: int) -> float:
        """Calculate the distance between the obstacle and the robot.

        Here we are going to use focal length to calculate the distance.
        1. Using F = (P * D) / W, where P is the apparent width in pixels, D is the known distance, W is the known
        width.
        2. Using D = (W * F) / P to gain the distance.
        """
        F = 0.28                                    # [pixel] focal length
        W = 4                                       # [cm], the width of the obstacle
        new_block = self.cam.newBlocks[block_idx]
        D_width = (W * F) / new_block.m_width       # [cm]

        return D_width

    def __cal_angle__(self, block_idx: int) -> float:
        """Calculate the angle.

        Note: The camera is ever-changing or not? or should we correct the camera when running?

        """
        theta1_pixel = self.cam.newBlocks[block_idx].m_x - self.cam.pixyCenterX     # error in pixels
        theta_1 = -(theta1_pixel / self.cam.pixyMaxX * self.cam.pixyX_FoV)          # error converted to angle
        theta = self.bot.servo.lastPosition

        return theta - theta_1

    # output            - none
    # speed             - general speed of bot
    def avoidStationaryObstacles(self, speed):

        self.bot.setServoPosition(0)

        while True:
            self.cam.getLatestBlocks()
            self.getBlockParams(self.cam.isInView(self.obstacleID))

            #todo: Level 4### Please insert code here to derive non-zero obstacleSteering and keep the robot running
            # You need to first identify the obstacle and decide whether you want to visaully track it. Then you use what you learn from Level 1,2 and 3 to implement detection and steering.
            # Hint: self.cam.newBlocks and self.cam.oldBlocks are sorted by decreasing pixel size. Do you always want to be focused on the largest object?

            lineSteering = 0
            obstacleSteering = 0

            steering = obstacleSteering + lineSteering # we set the final steering as a linear combination of the obstacle steering and center line steering - but it's all up to you!
            self.drive(targetSpeed, steering)
            ###

        return
