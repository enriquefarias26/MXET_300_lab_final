from gpiozero import Servo
from time import sleep

servo = Servo(25)
val = 0

try:
	while True:
	   servo.value = val
	   sleep(1)
	   servo.value = val + 1
	   sleep(1)

except KeyboardInterrupt:
	print("Program stopped")