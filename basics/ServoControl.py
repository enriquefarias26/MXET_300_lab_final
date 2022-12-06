from gpiozero import Servo
from time import sleep

shots = 0
servo1 = Servo(25) #servo1 for shooting
val = -1

servo2 = Servo(24) #servo2 for reloading

def shoot():
	servo1.value = val
	sleep (1)
	servo1.value = val + 2
	sleep(1)
def reload():
	servo2.value = val
	sleep (1)
	servo2.value = val + 2
	sleep(1)

while True:
	shoot()
	shots += 1
	if (shots == 12):
		reload()
		shots = 0
	sleep(1)

	