import RPi.GPIO as GPIO
import time

class ServoController(object):
    """A simple servo controller."""

    def __init__(
        self,
        pin=11,
        hertz=50,
        initAngle=0
    ):
        self.pin = pin
        self.hertz = hertz

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.pin, self.hertz)
        self.servo.start(0)
        
        self.setAngle(self, initAngle)

    def setAngle(self, angle):
        self.servo.ChangeDutyCycle(2 + angle / 18)

        # TODO: figure out how to do this on a thread
        # will remove jitter
        # time.sleep(0.1 * angle / 60)
        # wait time using angular velocity of 60 deg / 0.1 s
        # self.servo.ChangeDutyCycle(0)

        self.angle = angle

    def close(self):
        self.servo.stop()