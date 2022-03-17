## script that starts the race
## made by Jon Skerlj


from pixyBot import pixyBot
from pixyCam import pixyCam
from control import robotRace
from PIDcontroller import PID_controller

# exercises
def main():
    ### IMPORTANT
    servoCorrection = 0 # put the servo correction for your robot here
    ###

    r = pixyBot(servoCorrection, PID_controller(0.08, 0, 0.1))
    p = pixyCam()
    rR = robotRace(r, p)

    rR.race(0.5)
    #rR.test2angles()
main()