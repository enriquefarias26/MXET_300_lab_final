import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setup(16,GPIO.IN)
GPIO.setup(26, GPIO.OUT)

try:
    while True:
        if GPIO.input(16):
            print ("got shot")
            GPIO.output(26, 1)
        else:
            print ("not shot")
            GPIO.output(26,0)
        sleep(0.1)
        
finally:
    GPIO.cleanup()