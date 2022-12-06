import RPi.GPIO as GPIO
import L1_motor as motor
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setup(16,GPIO.IN)
x=0

def gotShot():
    if GPIO.input(16):
        x = x+1
        if x == 9:
            motor.sendRight(0)
            motor.sendLeft(0)
            sleep(30)
        else:
            motor.sendRight(0)
            motor.sendLeft(0)
            sleep(0.8)
            motor.sendRight(0.8)
            motor.sendLeft(0.8)
            sleep(0.8)
            motor.sendRight(0)
            motor.sendLeft(0)
       
       