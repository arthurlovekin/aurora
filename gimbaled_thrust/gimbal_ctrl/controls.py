# Import libraries
import time
from simple_pid import PID
from servo_controller import ServoController

pid = PID(1, 0.1, 0.05, output_limits=(0, 5))
servo = ServoController(11)

while True:
    angleFromVertical = 5 # = someIMUFunction()

    control = pid(angleFromVertical)

    servo.setAngle(control)