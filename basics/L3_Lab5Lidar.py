#Lidar

# This File performs the following:
# 1) grab a subset of the readings from the lidar for lightweight purposes
# 2) assign the proper angle value to the reading, with respect to robot x-axis
# 3) create a 2d array of [distances, angles] from the data

# Note: installation of pysicktim library is required for to run this program.
# perform "sudo pip3 install pyusb," then "sudo pip3 install pysicktim"

# Import external libraries
import numpy as np                                  # for array handling
import pysicktim as lidar                           # required for communication with TiM561 lidar sensor
import time                                         # for timekeeping
import L1_log as logging

np.set_printoptions(suppress=True)                  # Suppress Scientific Notation
start_angle = -135.0                                # lidar points will range from -135 to 135 degrees


def polarScan(num_points=54):                       # You may request up to 811 points, max.

    lidar.scan()                                    # take reading

    # LIDAR data properties
    dist_amnt = lidar.scan.dist_data_amnt                               # Number of distance data points reported from the lidar
    angle_res = lidar.scan.dist_angle_res                               # Angular resolution reported from lidar

    # create the column of distances
    scan_points = np.asarray(lidar.scan.distances)                      # store the reported readings and cast as numpy.array
    inc_ang = (dist_amnt/(num_points+1))*angle_res                      # Calculate angle increment for scan_points resized to num_points
    scan_points = np.asarray(np.array_split(scan_points, num_points))   # Split array into sections
    scan_points = [item[0] for item in scan_points]                     # output first element in each section into a list
    scan_points = np.asarray(scan_points)                               # cast the list into an array
    scan_points = np.reshape(scan_points, (scan_points.shape[0], 1))    # Turn scan_points row into column

    # create the column of angles
    angles = np.zeros(num_points)
    x = len(angles)
    for i in range(x):
        angles[i] = (i*lidar.scan.dist_angle_res*lidar.scan.dist_data_amnt/num_points)+(start_angle)

    angles = np.reshape(angles, (angles.shape[0], 1))                   # Turn angles row into column

    # create the polar coordinates of scan
    scan_points = np.hstack((scan_points, angles))                      # Turn two (54,) arrays into a single (54,2) matrix
    scan_points = np.round(scan_points, 3)                              # Round each element in array to 3 decimal places

    return(scan_points)

#Vector
# This program manipulates distance vectors in the robot coordinate frame,
# as well as arrays of vectors.  Pay attention to format of arguments since
# some functions are not yet optimized to handle numpy [1x2] vectors directly.
# Further functions will be added for rotation of vectors in various coordinate frames.

# Import internal programs
import L1_lidar as lidar2

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


def polar2cart(r, alpha):                           # convert an individual vector to cartesian coordinates (in the robot frame)
    alpha = np.radians(alpha)                       # alpha*(np.pi/180) # convert to radians
    x = r * np.cos(alpha)                           # get x
    y = r * np.sin(alpha)                           # get y
    cart = np.round(np.array([x, y]), 3)            # vectorize and round
    return cart


def rotate(vec, theta):                             # describe a vector in global coordinates by rotating from body-fixed frame
    c, s = np.cos(theta), np.sin(theta)             # define cosines & sines
    R = np.array(((c, -s), (s, c)))                 # generate a rotation matrix
    vecGlobal = np.matmul(R, vec)                   # multiply the two matrices
    return vecGlobal


def sumVec(vec, loc):                               # add two vectors. (origin to robot, robot to obstacle)
    mySum = vec + loc                               # element-wise addition takes place
    return mySum                                    # return [x,y]


def getNearest():                                   # combine multiple functions into one.  Call to get nearest obstacle.
    scan = lidar2.polarScan()                        # get a reading in meters and degrees
    valids = getValid(scan)                         # remove the bad readings
    vec = nearest(valids)                           # find the nearest
    return vec                                      # pass the closest valid vector [m, deg]

#Main

if __name__ == "__main__":
    while True:
        #Lidar Part
        lidarData = polarScan(54)
        print(lidarData)
        time.sleep(2)
        #Vector Part
        myVector = getNearest()                                 # call the function which utilizes several functions in this program
        #print("\n The nearest object (m,deg):\n", myVector)     # print the result
        time.sleep(0.1)                                         # small delay
        #Logging Distance
        logging.tmpFile(myVector[0], "TempLidarLog.txt")
        #Logging Angle
        logging.tmpFile(myVector[1], "TempLidarAngleLog.txt")
        #Polar to Cart Calculation
        alpha = np.radians(myVector[1])                       # alpha*(np.pi/180) # convert to radians
        r = myVector[0]
        x = r * np.cos(alpha)                           # get x
        y = r * np.sin(alpha)                           # get y
        #Logging X Distance
        logging.tmpFile(x, "TempLab5XLog.txt")
        #Logging Y Distance
        logging.tmpFile(y, "TempLab5YLog.txt")