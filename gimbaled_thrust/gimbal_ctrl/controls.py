# Import libraries
import time
from simple_pid.PID import PID
from servo_controller.ServoController import ServoController

pid = PID(1, 0.1, 0.05, output_limits=(0, 5))
servo = ServoController(11)

time.sleep(1)
servo.update()

while True:
    angleFromVertical = 5 # = someIMUFunction()

    #control = pid(angleFromVertical)

    servo.update()
    servo.setAngle(60)