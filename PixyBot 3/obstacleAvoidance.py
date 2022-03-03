## library for obstacle avoidance and lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2020

from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
import typing as t
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
    def drive(self, drive: float, bias: float) -> None:  # Differential drive function
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
            steering = 0  # todo: should we consider to move the bot?
            if obstacleBlock == 4:
                # Level 2 Modify your if statement to set targetSpeed = 0 when you detect a obstacle at a specific
                #  distance.
                # Level 3 -  Modify your if statement to set targetSpeed = 0 when you detect a obstacle at within a
                #  distance and a frontal angular space.
                distance = self.__cal_distance__(block_idx=obstacleBlock)
                _, angle = self.__cal_angle__(block_idx=obstacleBlock)
                if 15 <= distance <= 50 and -50 <= angle <= 50:
                    # Set the drive speed to zero and no bias for the right wheel
                    targetSpeed = 0
                    steering = 0
                    self.drive(targetSpeed, steering)
                elif distance < 15:
                    print("Warning! The distance is too small.")
                else:
                    print("The distance is too long!")
            else:
                # todo: Paste the code in `follow` in laneFollower.py
                self.central_line_follow(speed)

            # todo: How could we specify the `steering`?
            # self.drive(targetSpeed, steering)
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
        new_block = self.cam.newBlocks[block_idx]   # block index can be wrong here
        D_width = (W * F) / new_block.m_width       # [cm]

        return D_width

    def __cal_angle__(self, block_idx: int) -> t.Tuple[bool, float]:
        """Calculate the angle.

        Note: The camera is ever-changing or not? or should we correct the camera when running?

        returns
            theta1_pixel is used to justify the direction of the obstacle relative to the camera center. If the value
                is negative, the obstacle lied on the left side, otherwise it is on the right side.
            return true means the obstacle is on the left side, otherwise it is on the right side.
        """
        theta1_pixel = self.cam.newBlocks[block_idx].m_x - self.cam.pixyCenterX     # error in pixels
        theta_1 = -(theta1_pixel / self.cam.pixyMaxX * self.cam.pixyX_FoV)          # error converted to angle
        theta = self.bot.servo.lastPosition

        return theta1_pixel < 0, theta - theta_1

    # output            - none
    # speed             - general speed of bot
    def avoidStationaryObstacles(self, speed):

        self.bot.setServoPosition(0)

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

            targetSpeed = speed
            lineSteering = 0
            obstacleSteering = 0  # negative value means turn left

            if obstacleBlock == 4:
                distance = self.__cal_distance__(block_idx=obstacleBlock)
                position_mark, angle = self.__cal_angle__(block_idx=obstacleBlock)

                if distance <= 20:
                    # we try to turn the bot if the distance less than 20 cm
                    if (self.bot.servo.lastPosition > 0) and position_mark == True:
                        # Now the obstacle is on the right side of the central line, turn the bot to the left
                        obstacleSteering = -0.2  # negative value means turn left
                    else:
                        # it is lies on the left side
                        obstacleSteering = 0.2  # negative value means turn left
                    steering = obstacleSteering + lineSteering
                    self.drive(targetSpeed, steering)
            else:
                # try to follow the central line. Turn the cam and try to make it focus on the red line
                if centerLineBlock == 1:
                    angle_err, new_servo_pos = self.visTrack(blockIdx=centerLineBlock)

            # todo: Level 4 Please insert code here to derive non-zero obstacleSteering and keep the robot running
            #  You need to first identify the obstacle and decide whether you want to visaully track it. Then you use
            #  what you learn from Level 1,2 and 3 to implement detection and steering.
            #  Hint: self.cam.newBlocks and self.cam.oldBlocks are sorted by decreasing pixel size. Do you always
            #  want to be focused on the largest object?

            # we set the final steering as a linear combination of the obstacle steering and center line steering -
            # but it's all up to you!

            self.drive(targetSpeed, steering)
            ###

        return

    def followTarget(self, speed):
        correction = 0
        servo_pos = 0

        CL_angular_error = self.blockAngle[-1] + correction
        camera_rotation = -(servo_pos / 50) * 25  # Account for the rotation of the camera
        angle = CL_angular_error + camera_rotation
        lineSteering = angle * 0.015
        self.drive(speed, lineSteering)

    def get_largest_block(self, center_id, left_id, right_id):
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
        if array[array.index(min(array))] < 50:  # find the id of the largest block
            largest_block = array.index(min(array))

        # return the id of the largest block: 0-center, 1-left, 2-right, -1- no marker recognised
        return largest_block

    def central_line_follow(self, speed):
        self.bot.setServoPosition(0)  # set servo to centre
        self.drive(0, 0)  # set racer to stop
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)  # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            correction = 0
            servo_pos = 0
            line_markers = [centerLineBlock, leftLineBlock, rightLineBlock]
            self.getBlockParams(line_markers[0])
            largest_block = self.get_largest_block(centerLineBlock, leftLineBlock, rightLineBlock)
            print("the largest block is: ", largest_block)
            line_markers = [centerLineBlock, leftLineBlock, rightLineBlock]
            print("line_markers: ", line_markers)
            self.getBlockParams(line_markers[0])

            if centerLineBlock >= 0:  # drive while we see a line
                self.getBlockParams(line_markers[0])
                self.followTarget(speed)

            elif leftLineBlock >= 0:
                self.getBlockParams(line_markers[1])
                self.followTarget(speed)

            elif rightLineBlock >= 0:
                self.getBlockParams(line_markers[2])
                self.followTarget(speed)

            else:  # stop the racer and wait for new blocks
                self.drive(0, 0)
