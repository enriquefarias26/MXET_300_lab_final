import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setup(16,GPIO.IN)

int x
def gotShot():
    if GPIO.input(16):
        x = 1
    else:
        x = 0
    return x
