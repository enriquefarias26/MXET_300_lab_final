#creating program  to do object avoidance
import numpy as np
import L1_lidar as lidar
import L1_motor as motor
from time import sleep
import cv2              # For image capture and processing
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import netifaces as ni
from math import radians, pi
from gpiozero import Servo
from time import sleep

import SignalReceiver as Sig


servo1 = Servo(24) #servo1 for shooting
val = -1

servo2 = Servo(25) #servo2 for reloading

def shoot():
	servo1.value = val
	sleep (0.6)
	servo1.value = val + 2
	sleep(0.6)
def gunreload():
	servo2.value = val
	sleep(0.6)
	servo2.value = val + 2
	sleep(0.6)

np.set_printoptions(precision=3)                    # after math operations, don't print long values

def getValid(scan):                                 # remove the rows which have invalid distances
    dist = scan[:, 0]                               # store just first column
    angles = scan[:, 1]                             # store just 2nd column
    valid = np.where(dist > 0.016)                  # find values 16mm
    myNums = dist[valid]                            # get valid distances
    myAng = angles[valid]                           # get corresponding valid angles
    output = np.vstack((myNums, myAng))             # recombine columns
    n = output.T                                    # transpose the matrix
    return n

def nearest(scan):                                  # find the nearest point in the scan
    dist = scan[:, 0]                               # store just first column
    column_mins = np.argmin(dist, axis=0)           # get index of min values along 0th axis (columns)
    row_index = column_mins                         # index of the smallest distance
    vec = scan[row_index, :]                        # return the distance and angle of the nearest object in scan
    return vec                                      # contains [r, alpha]

def getNearest():                                   # combine multiple functions into one.  Call to get nearest obstacle.
    scan = lidar.polarScan()                        # get a reading in meters and degrees
    valids = getValid(scan)                         # remove the bad readings
    vec = nearest(valids)                           # find the nearest
    return vec                                      # pass the closest valid vector [m, deg]

#ColorTracking
# Gets IP to grab MJPG stream
def getIp():
    for interface in ni.interfaces()[1:]:   #For interfaces eth0 and wlan0
    
        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip
            
        except KeyError:                    #We get a KeyError if the interface does not have the info
            continue                        #Try the next interface since this one has no IPv4
        
    return 0
    
#    Camera
stream_ip = getIp()
if not stream_ip: 
    print("Failed to get IP for camera stream")
    exit()

camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

size_w  = 240   # Resized image width. This is the image width in pixels.
size_h = 160	# Resized image height. This is the image height in pixels.

fov = 1         # Camera field of view in rad (estimate)

#    Color Range, described in HSV
v1_min = 145      # Minimum H value
v2_min = 85     # Minimum S value
v3_min = 50      # Minimum V value

v1_max = 175     # Maximum H value
v2_max = 195    # Maximum S value
v3_max = 255    # Maximum V value

target_width = 100      # Target pixel width of tracked object
angle_margin = 0.2      # Radians object can be from image center to be considered "centered"
width_margin = 10       # Minimum width error to drive forward/back



def main():
    # Try opening camera with default method
    shots = 0
    y=2
    try:
        camera = cv2.VideoCapture(0)    
    except: pass

    # Try opening camera stream if default method failed
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)    

    camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    camera.set(4, size_h)                       # Set height of images that will be retrived from camera

    try:
        while True:
            sleep(.05)                                          

            ret, image = camera.read()  # Get image from camera

            # Make sure image was grabbed
            if not ret:
                print("Failed to retrieve image!")
                break

            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)              # Convert image to HSV

            height, width, channels = image.shape                       # Get shape of image

            thresh = cv2.inRange(image, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))   # Find all pixels in color range

            kernel = np.ones((5,5),np.uint8)                            # Set kernel size
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)     # Open morph: removes noise w/ erode followed by dilate
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)      # Close morph: fills openings w/ dilate followed by erode
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
            
            Sig.gotShot()
            
            if len(cnts) and len(cnts) < 3:                             # If more than 0 and less than 3 closed shapes exist
                if y==1:
                    gunreload()
                y=0
                motor.sendRight(0)
                motor.sendLeft(0)
                shoot()
                shots += 1
                if shots >= 12:
                    gunreload()
                    shots = 0
                
                
                sleep(0.1)
                continue

            else:
                y=1
                myVector = getNearest()                                                 # call the function which utilizes several functions in this program
                sc.driveOpenLoop(np.array([0.,0.]))
                if myVector[0] <= 0.4 and myVector[1] >= 5 and myVector[1] <= 90:       #conditions if object is too close to the left
                    motor.sendLeft(-0.8)                                                #left wheel reverse
                    motor.sendRight(0.8)                                                #right wheel forward
                elif myVector[0] <= 0.4 and myVector[1] <= -5 and myVector[1] >= -90:   #conditions if object is too close to the right
                    motor.sendLeft(0.8)                                                 #left wheel forward
                    motor.sendRight(-0.8)                                               #right wheel reverse
                elif myVector[0] <= 0.4 and myVector[1] < 5 and myVector[1] >= 0:       #condition if object is too close in front left, turn 90ish degrees to the right
                    motor.sendLeft(-0.8)                                                #left wheel reverse
                    motor.sendRight(0.8)                                                #right wheel forward
                    sleep(2)                                                            #continue turn for 2 seconds
                elif myVector[0] <= 0.4 and myVector[1] > -5 and myVector[1] <= 0:      #condition if object is too close in front right, turn 90ish degrees to the left
                    motor.sendLeft(0.8)                                                 #left wheel forward
                    motor.sendRight(-0.8)                                               #right wheel reverse
                    sleep(2)                                                            #continue turn for 2 seconds
                else:
                    motor.sendLeft(0.8)                                                 #left wheel forward
                    motor.sendRight(0.8)                                                #right wheel forward
                
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        pass

    finally:
    	print("Exiting Color Tracking.")


if __name__ == "__main__":
    while True:
        main()