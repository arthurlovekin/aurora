# Import libraries
import math
import time
from simple_pid.PID import PID
from servo_controller.ServoController import ServoController
from servo_controller.GimbalToServo import *

#xServo = ServoController(12, gimbalFunction=upperServoAngle)
yServo = ServoController(11, gimbalFunction=lowerServoAngle)

time.sleep(1)
#xServo.update()
yServo.update()

while True:
    for i in range(10):
        #xServo.update()
        yServo.update()
        angle = i * math.pi / 5
        #xServo.setAngle(5.0*math.cos(angle))
        yServo.setAngle(5.0*math.sin(angle))
        time.sleep(0.1)