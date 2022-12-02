import RPi.GPIO as GPIO
import L1_motor as motor
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setup(16,GPIO.IN)

int x
def gotShot():
    if GPIO.input(16):
        motor.sendright(0.8)
        motor.sendleft('0.8)
        sleep(0.8)
        motor.sendright(0)
        motor.sendleft(0)
        x=1
    else:
        x=0
