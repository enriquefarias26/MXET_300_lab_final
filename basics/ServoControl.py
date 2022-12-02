from gpiozero import Servo
from time import sleep

servo = Servo(25) #servo1 for shooting
val = 0

servo2 = Servo(26) #servo2 for reloading

def shoot():
	servo.value = val
	sleep (1)
	servo.value = val + 1
	sleep(1)
def reload():
	servo2.value = val
	sleep (1)
	servo2.value = val + 1
	sleep(1)
