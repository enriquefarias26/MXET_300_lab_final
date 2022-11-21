#creating program  to do object avoidance
import numpy as np
import L1_lidar as lidar
import L1_motor as motor
from time import sleep
import ColorTracking as CTrack
#import ServoControl as SControl


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


if __name__ == "__main__":
    while True:
        myVector = getNearest()                                                 # call the function which utilizes several functions in this program
        if myVector[0] <= 0.4 and myVector[1] >= 5 and myVector[1] <= 90:       #conditions if object is too close to the left
            motor.sendLeft(-0.8)
            motor.sendRight(0.8)
        elif myVector[0] <= 0.4 and myVector[1] <= -5 and myVector[1] >= -90:   #conditions if object is too close to the right
            motor.sendLeft(0.8)
            motor.sendRight(-0.8)
        elif myVector[0] <= 0.4 and myVector[1] < 5 and myVector[1] >= 0:       #condition if object is too close in front left, turn 90ish degrees to the right
            motor.sendLeft(-0.8)
            motor.sendRight(0.8)
            sleep(2)
        elif myVector[0] <= 0.4 and myVector[1] > -5 and myVector[1] <= 0:      #condition if object is too close in front right, turn 90ish degrees to the left
            motor.sendLeft(0.8)
            motor.sendRight(-0.8)
            sleep(2)
        else:
            motor.sendLeft(0.8)
            motor.sendRight(0.8)