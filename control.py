# Library for the final robot race - Animal locomotion and Bioinspired Robots module
## Author: Jon Skerlj (Using the environment provided by the course)

# Import necessary libraries
from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time,sleep
import math
from math import pi


class robotRace(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
        self.bot = bot
      
        self.bot = bot
        self.cam = cam
      
        self.centerLineID   = 1
        self.leftLineID     = 2
        self.rightLineID    = 4
        self.obstacleID     = 5
        self.entranceID     = 7
        
        self.obstacleCourse = False
        
        self.count = 0
        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]
        self.pixelSize = [float('nan') for i in range(nObservations)]
        self.blockAngleCenter = [float('nan') for i in range(nObservations)]
        self.blockDistance = [float('nan') for i in range(nObservations)]
        self.focalLength = 2.8
        
        self.redAngles = [float('nan') for i in range(nObservations)]
        self.leftAngles = [float('nan') for i in range(nObservations)]
        self.rightAngles = [float('nan') for i in range(nObservations)]
        
        self.width_hist = [0 for i in range(30)]
        self.obst_dist_hist = [0 for i in range(30)]
        self.prev_steering = 0
        self.steeringHistory = [0 for i in range(60)]
        self.speedHistory = [0 for i in range(30)]
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
        self.bot.setMotorSpeeds(lDrive, rDrive)
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
            pixelSize = self.cam.newBlocks[blockIdx].m_width;
            angleSize = pixelSize/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x -  self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular error of block relative to front

            # save params
            self.pixelSize.append(pixelSize)
            self.blockSize.append(angleSize)
            self.blockAngle.append(angleError)
            self.frameTimes.append(time())

            # remove oldest params
            self.pixelSize.pop(0)
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
         
    def steering_weights(self, angle, distance=40):
        
        if distance > 39:
            
            turn_weight = abs(angle)/30
            line_weight = 1 - turn_weight
            obstacle_weight = 0
        ########## Weight for obstacle needs to be added
        else:
            alpha = self.obstacle_steering_ratio(distance)
            obstacle_weight = alpha
            line_weight = 1 - obstacle_weight
            turn_weight = 0
            
        return line_weight, turn_weight, obstacle_weight
 
    
    def obstacle_steering_ratio(self, distance):
      distance_th = 25
      if distance > distance_th:
        ratio = 0
      else:
        ratio = 0.5 - distance/(distance_th/0.5)
      return ratio
      
    def largest_block(self,center_id, left_id, right_id):
        # If there is no block of either color, set that id really high - e.g 50
        if center_id == -1:
            center_id = 50
        if left_id == -1:
            left_id = 50
        if right_id == -1:
            right_id = 50
        
        # group all the marker ids together
        array = [center_id, left_id, right_id]  
        
        # set initial id to -1 (can be used later to determine if the camera sees anything or not)
        thicc_boi = -1
        
        # If color markers are recognised (i.e. min number in array is smaller than 50)        
        if array[array.index(min(array))] < 50:
            # find the id of the largest block
            thicc_boi = array.index(min(array))
        
        # return the id of the largest block: 0-center, 1-left, 2-right, -1- no marker recognised
        return  thicc_boi
    
    def obstacle_course(self, obstacle_block, line, line_markers):
        print('Folowing: ', line)
        new_idx = line
        self.bot.setServoPosition(0)
        # Get the parameters of the largest block
        block = self.cam.isInView(line)
        self.getBlockParams(block)
        # Calculate the error corresponding to the offset (offset ~ 22)
        CL_angular_error = self.blockAngle[-1]
        #print(CL_angular_error)
        # Account for the rotation of the camera
        servo_pos = 0
        camera_rotation = -(servo_pos/50) * 25
        #camera_rotation = -(servo_pos)
        angle = CL_angular_error + camera_rotation
        
        steering = angle/(self.cam.pixyY_FoV)# + 0.01*(angle-old_error)
        
        
        
        ## Obstacle stuff
        #print('Obstacle block id: ', obstacle_block)
        if obstacle_block >=0:
            self.getBlockParams(obstacle_block)
            self.getBlockParams(obstacle_block)
            R = 2
            F = 300
            obstacle_size = self.pixelSize[-1]
            distance = 2*R*F/obstacle_size
            if distance <= 24: 
                if line ==2: 
                    print('Turning right')
                    self.drive(0.5, 0.4, 0.3)
                    self.drive(0.5, 0, 0.2)
                    new_idx = 4
                    print('done')
                    
                else:
                    print('Turning left')
                    self.drive(0.5, -0.2, 0.3)
                    self.drive(0.5, 0, 0.2)
                    new_idx = 2
                    print('done')
        else:
            print('No obstacle')
            
            
                
        
        return new_idx, steering
       
    # main function that will be running during the robot race
    # goal: Given the recorded input parameters from the camera, finish the specific race track as quickly as possible without making any mistakes (e.g. going of track, stopping ...)    
    def race(self, speed):
        # Initialise the important variables
        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop
        obstacleSteering = 0
        lineSteering = 0
        servo_pos = 0
        old_servo_pos = servo_pos
        scanPos = 0
        scanPosFreq = 40
        i = 0
        scan = True
        reverse = True
        line = self.leftLineID
        names = ['center', 'left', 'right']
        try: 
            while True:
                # get the recognised block IDs
                self.cam.getLatestBlocks()
                centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
                leftLineBlock = self.cam.isInView(self.leftLineID)
                rightLineBlock = self.cam.isInView(self.rightLineID)
                obstacleBlock =  self.cam.isInView(self.obstacleID)
                obstacleStartBlock = self.cam.isInView(self.entranceID)
                
                color_ids = [self.centerLineID, self.leftLineID, self.rightLineID] 
                line_markers = [centerLineBlock, leftLineBlock, rightLineBlock]
                
                # Find which color box is the biggest
                largest_idx = self.largest_block(centerLineBlock, leftLineBlock, rightLineBlock)
                
                # There are no markers if largest_idx = -1, otherwise largest_idx = 0 or 1 or 2
               # if obstacleStartBlock >= 0:
               #     self.bot.setServoPosition(0)
                    # Get the parameters of the largest block
               #     self.getBlockParams(obstacleStartBlock)
                    # Calculate the error corresponding to the offset (offset ~ 22)
               #     CL_angular_error = self.blockAngle[-1]
                    
                    # Account for the rotation of the camera
                #    camera_rotation = -(servo_pos/50) * 25
                #    angle = CL_angular_error + camera_rotation
                #    lineSteering = angle * 0.02
                #    lineSteerng = lineSteering
                #    speed_input = speed
                #    self.obstacleCourse = True
                
                if largest_idx >= 0:
                    correction = 0
                    
                    # Find the offset for the side markers (needs to be measured)
                    if largest_idx == 0:
                        correction = 0
                        speed_input = speed
                    # Left marker
                    elif largest_idx == 1:
                        correction = 23
                        speed_input = 1*speed
                    # Right  marker
                    else:
                        correction = -23
                        speed_input = 1*speed
                    
                    # Get the parameters of the largest block
                    self.getBlockParams(line_markers[largest_idx])
                    # Calculate the error corresponding to the offset (offset ~ 22)
                    CL_angular_error = self.blockAngle[-1] + correction
                    
                    # Account for the rotation of the camera
                    camera_rotation = -(servo_pos/50) * 25
                    angle = CL_angular_error + camera_rotation
                    lineSteering = angle * 0.02
                    
                    # Follow the red marker with the camera if it is present and is big enough
                    if centerLineBlock >= 0 and (self.cam.newBlocks[centerLineBlock].m_width * self.cam.newBlocks[centerLineBlock].m_height)>1000:
                        ang_err, servo_pos = self.visTrack(centerLineBlock)
                    else:
                        servo_pos = self.bot.setServoPosition(0)
                    
                    width = self.cam.newBlocks[line_markers[largest_idx]].m_width
                    
                    # Speed modulation
                    if width > 100:
                        print('Max slowing down')
                        speed_input = 0.4*speed
                        #lineSteering = lineSteering *1.02
                    elif width > 90:
                        speed_input = 0.5*speed
                    elif width > 70:
                        speed_input = 0.6*speed
                    elif width > 60:
                        speed_input = 0.8*speed
                        
                    else:
                        servo_pos = self.bot.setServoPosition(0)
                    
                    reverse = True

                #elif self.obstacleCourse == True:
                #    print('yay')
                    # Do obstacle stuffs
                #    new_idx, lineSteering = self.obstacle_course(obstacleBlock, line, line_markers)
                #    line = new_idx
                #    speed_input = speed
                #    if centerLineBlock >= 0:
                #        self.obstacleCourse = False
                else:
                    print("cannot see anything")
                    speed_input,lineSteering = 0,0
                    self.bot.setServoPosition(scanPos)
                    if scan == True :
                        scanPos = scanPos + 10
                       # self.drive(0, 0, 0.1)
                    elif scan == False:
                        scanPos = scanPos - 10
                       # self.drive(0, 0, 0.1)
                    if scanPos > 60:
                        scan = False
                    elif scanPos < -60:
                        scan = True
                    i += 1
                    #if i > 200 :
                    #    i=0
                    #    self.drive(0.5, 0, 0.3)
                    #    print("drive forward")
                    print(i)

                self.drive(speed_input, lineSteering)

        except KeyboardInterrupt:
            print("ended")
            self.bot.setServoPosition(0)
            self.drive(0,0)
    
    def turn_cam(self, speed):
        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0)
        
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            
            if centerLineBlock >= 0:
                vis_err, servo_pos = self.visTrack(centerLineBlock)
                lineSteering = -servo_pos/25
                
            self.drive(speed, lineSteering)   
    
    
    # Denoising the camera recordings
    # function that averages the last n elements of the data input
    def avg(self, data, n):
        # average the last 5 elements
        avg = sum(data[-n:])/len(data[-n:])
        return
