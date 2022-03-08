## library for lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2021

from time import time

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

        self.centerLineID = 1
        self.leftLineID = 2
        self.rightLineID = 3

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
            pixelSize = self.cam.newBlocks[blockIdx].m_width
            # get angular size of block
            angleSize = pixelSize / self.cam.pixyMaxX * self.cam.pixyX_FoV
            pixelError = self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX
            # get angular error of block relative to front
            angleError = pixelError / self.cam.pixyMaxX * self.cam.pixyX_FoV

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
            """
            If we want to check the largest block of the ID 
            """
            pixelError = self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX  # error in pixels
            visAngularError = -(pixelError / self.cam.pixyMaxX * self.cam.pixyX_FoV)  # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(
                visAngularError)  # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition, pixelError

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

    def map_error(self, mapVal, minIn = -158, maxIn = 158, minOut = -1, maxOut = 1):
        """This is going to be used in mapping the pixel error to non-dimensional error.

        Args:
            mapVal (_type_): _description_
            minIn (int, optional): _description_. Defaults to -158.
            maxIn (int, optional): _description_. Defaults to 158.
            minOut (int, optional): _description_. Defaults to -1.
            maxOut (int, optional): _description_. Defaults to 1.

        Returns:
            _type_: float. 
        """
        true_error =(((mapVal - minIn)*(maxOut - minOut))/(maxIn - minIn)) + minOut
        return true_error


    def follow(self, speed):
        """
        1. Use function `self.cam.isInView(ID_marker)` to gain the corresponding blockIdx (i.e., the index of in the
        newBlocks in PixyCam).
        2. Use function `self.getBlockParams(blockIdx)` to update the block parameters, including blockSize (
        angleSize, i.e., the angle that can cover all target),
        blockAngle(angle_error, i.e., the target position relative to the center of the camera) and frameTimes().
        3. Use function `self.visTrack(blockIdx=blockIdx)` to gain the error and new position of the servo.
        Meanwhile, these two values are for previous ID_marker.
        """

        self.bot.setServoPosition(0)  # set servo to centre
        self.drive(0, 0)  # set racer to stop

        self.kp = 0.015
        self.kd = 0
        self.ki = 0.005
        integral_error = 0
        pre_error = 0

        i += 0
        while True:
            # update the cam vie
            self.cam.getLatestBlocks()

            # Try to find the corresponding block
            centerLineBlock = self.cam.isInView(self.centerLineID)
            # if centerLineBlock == -1:
            #     print("Cannot detect the center red line ---- ", i)
            leftLineBlock = self.cam.isInView(self.leftLineID)
            # if leftLineBlock == -1:
            #     print("Cannot detect the left line --- ", i)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            # if rightLineBlock == -1:
            #     print("Cannot detect the right line --- ", i)

            # todo: why do we need update the blockParams here?
            line_markers = [centerLineBlock, leftLineBlock, rightLineBlock]

            if centerLineBlock >= 0:
                # If we detect the red line, we would follow it
                print("Detect the center red line ---- ", i)

                self.getBlockParams(line_markers[0])
                servo_error, servo_position, piexel_error = self.visTrack(blockIdx=centerLineBlock)

                # mapping the error, i.e., -25 to 25
                cur_error = self.map_error(piexel_error)
                bias, integral_error = self.predict_pid(cur_error, integral_error)
                print("Non-dimensional error: ", cur_error)

                self.drive(speed, bias)

            elif leftLineBlock >= 0:
                print("Detect the left line --- ", i)

                self.getBlockParams(line_markers[1])
                servo_error, servo_position, piexel_error = self.visTrack(blockIdx=leftLineBlock)
                cur_error = self.map_error(piexel_error)
                bias, integral_error = self.predict_pid(cur_error, integral_error)
                print("Non-dimensional error: ", cur_error)

                self.drive(speed, bias)
                

            elif rightLineBlock >= 0:
                print("Detect the right line --- ", i)

                self.getBlockParams(line_markers[2])
                servo_error, servo_position, piexel_error = self.visTrack(blockIdx=rightLineBlock)
                cur_error = self.map_error(piexel_error)
                bias, integral_error = self.predict_pid(cur_error, integral_error)
                print("Non-dimensional error: ", cur_error)

                self.drive(speed, bias)
                
                
            else:  # stop the racer and wait for new blocks
                self.drive(0, 0)

            # update index
            i += 1

        return

    def predict_pid(self, servo_error, integral_error):
        pre_error = servo_error
        integral_error = integral_error + pre_error

        derivative_error = servo_error

        bias = - (pre_error * self.kp + integral_error * self.ki + derivative_error * self.kd)

        return bias, integral_error