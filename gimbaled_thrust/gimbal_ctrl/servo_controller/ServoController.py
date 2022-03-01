import RPi.GPIO as GPIO
import time
import warnings
import math

try:
    # Get monotonic time to ensure that time deltas are always positive
    _current_time = time.monotonic
except AttributeError:
    # time.monotonic() not available (using python < 3.3), fallback to time.time()
    _current_time = time.time
    warnings.warn('time.monotonic() not available in python < 3.3, using time.time() as fallback')

class ServoController(object):
    """A simple servo controller."""

    def __init__(
        self,
        pin=11,
        hertz=50,
        initAngle=0,
        gimbalFunction=None
    ):
        self.pin = pin
        self.hertz = hertz

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.pin, self.hertz)
        self.servo.start(0)
        
        self.angle = 0
        self.setAngle(self, initAngle)

    def setAngle(self, angle):
        angle = self.gimbalFunction(angle)
        # if in angle change progress
        currentTime = _current_time()
        if(self.angle != self.toAngle):
            if(self.angleStartTime + (abs(self.angleRotation) * 0.1 / 60) <= currentTime):
                self.angle = math.floor(self.angle + self.angleRotation * (currentTime - self.angleStartTime) / (abs(self.angleRotation) * 0.1 / 60)) % 360
            else:
                self.angle = self.toAngle

        self.toAngle = angle
        self.angleRotation = (self.toAngle - self.angle + 180) % 360 - 180
        self.angleStartTime = currentTime

        self.servo.ChangeDutyCycle(2 + self.toAngle / 18)

    def update(self):
        if(self.angle != self.toAngle):
            currentTime = _current_time()
            if(self.angleStartTime + (abs(self.angleRotation) * 0.2 / 60) <= currentTime):
                self.angle = self.toAngle

                self.servo.ChangeDutyCycle(0)

    def close(self):
        self.servo.stop()