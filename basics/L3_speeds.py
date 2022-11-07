#Imports
import smbus2       # a Python package to communicate over i2c
import numpy as np  # use numpy to build the angles array
import time         # for keeping time
import numpy as np  # library for math operations

import L1_encoder as enc                    # local library for encoders
import L1_log as log

#define encoder
bus=smbus2.SMBus(1) # declare the i2c bus object

encL = 0x40         # encoder i2c address for LEFT motor
encR = 0x41         # encoder i2c address for RIGHT motor (this encoder has A1 pin pulled high)

#define kinematics
R = 0.041                                   # wheel radius (meters)
L = 0.201                                   # half of wheelbase (meters)
res = (360/2**14)                           # resolution of the encoders (degrees per LSB)
pulleyRatio = 0.5                           # wheel movement per shaft movement
A = np.array([[R/2, R/2], [-R/(2*L), R/(2*L)]])     # This matrix relates [PDL, PDR] to [XD,TD]
wait = 0.02                                 # wait time between encoder measurements (s)


#Encoder Part
def singleReading(encoderSelection):                                            # return a reading for an encoder in degrees (motor shaft angle)
    try:
        twoByteReading = bus.read_i2c_block_data(encoderSelection, 0xFE, 2)     # request data from registers 0xFE & 0xFF of the encoder. Approx 700 microseconds.
        binaryPosition = (twoByteReading[0] << 6) | (twoByteReading[1])           # remove unused bits 6 & 7 from byte 0xFF creating 14 bit value
        degreesPosition = binaryPosition*(360/2**14)                            # convert to degrees
        degreesAngle = round(degreesPosition,1)                                 # round to nearest 0.1 degrees
    except:
        print("Encoder reading failed.")                                        # indicate a failed reading
        degreesAngle = 0
    return degreesAngle

def readShaftPositions():                                   # read both motor shafts.  approx 0.0023 seconds.
    try:
        rawAngle = singleReading(encL)                      # capture left motor shaft
        angle0 = 360.0 - rawAngle                           # invert the reading for left side only
        angle0 = round(angle0,1)                            # repeat rounding due to math effects
    except:
        print('Warning(I2C): Could not read left encoder')  # indicate which reading failed
        angle0 = 0
    try:
        angle1 = singleReading(encR)                        # capture right motor shaft
    except:
        print('Warning(I2C): Could not read right encoder') # indicate which reading failed
        angle1 = 0
    angles = np.array([angle0,angle1])
    return angles
    
#Kinematics Part

def getPdCurrent():
    global pdCurrents                       # make a global var for easy retrieval
    encoders_t1 = enc.readShaftPositions()  # grabs the current encoder readings in degrees
    t1 = time.monotonic()                   # time.monotonic() reports in seconds
    time.sleep(wait)                        # delay for the specified amount
    
    encoders_t2 = enc.readShaftPositions()  # grabs the current encoder readings in degrees
    t2 = time.monotonic()                   # usually takes about .003 seconds gap
    global deltaT
    deltaT = round((t2 - t1), 3)            # compute delta-time (t.ttt scalar)

    # calculate travel of both wheels simultaneously
    travel = encoders_t2 - encoders_t1      # compute change in both shaft encoders (degrees)
    travel = encoders_t2 - encoders_t1      # array, 2x1 to indicate travel
    trav_b = travel + 360                   # array variant b
    trav_c = travel - 360                   # array variant c
    mx = np.stack((travel, trav_b, trav_c)) # combine array variants
    mx_abs = np.absolute(mx)                # convert to absolute val
    mins = np.argmin(mx_abs,0)              # find the indices of minimum values (left and right hand)
    left = mx[mins[0],0]                    # pull corresponding indices from original array
    right = mx[mins[1],1]                   # pull corresponding index for RH
    shaftTravel = np.array([left,right])    # combine left and right sides to describe travel (degrees)
    
    # build an array of wheel speeds in rad/s
    wheelTravel = shaftTravel * pulleyRatio     # compute wheel turns from motor turns
    wheelSpeeds_deg = wheelTravel / deltaT      # compute wheel speeds (degrees/s)
    pdCurrents = wheelSpeeds_deg * np.pi / 180  # compute wheel speeds (rad/s) & store to global variable
    return(pdCurrents)                          # returns [pdl, pdr] in radians/second


def getMotion():                            # this function returns the chassis speeds
    B = getPdCurrent()                      # store phidots to array B (here still in rad/s)
    C = np.matmul(A, B)                     # perform matrix multiplication
    C = np.round(C, decimals=3)             # round the matrix
    return(C)                               # returns a matrix containing [xDot, thetaDot]

def phiTravels(encoders_t1, encoders_t2):   # get travel of wheels [deg, deg] (take no measurements)
    travel = encoders_t2 - encoders_t1      # compute change in both shaft encoders (degrees)
    travel = encoders_t2 - encoders_t1      # array, 2x1 to indicate travel
    trav_b = travel + 360                   # array variant b
    trav_c = travel - 360                   # array variant c
    mx = np.stack((travel, trav_b, trav_c)) # combine array variants
    mx_abs = np.absolute(mx)                # convert to absolute val
    mins = np.argmin(mx_abs,0)              # find the indices of minimum values (left and right hand)
    left = mx[mins[0],0]                    # pull corresponding indices from original array
    right = mx[mins[1],1]                   # pull corresponding index for RH
    shaftTravel = np.array([left,right])    # combine left and right sides to describe travel (degrees)
    wheelTravel = shaftTravel * pulleyRatio # compute wheel turns from motor turns [deg,deg]
    return(wheelTravel)                     # return the movement
    
#Motor Part
def computePWM(speed):              # take an argument in range [-1,1]
    if speed == 0:
        x = np.array([0,0])         # set all PWM to zero
    else:
        x = speed + 1.0             # change the range to [0,2]
        chA = 0.5 * x               # channel A sweeps low to high
        chB = 1 - (0.5 * x)         # channel B sweeps high to low
        x = np.array([chA, chB])    # store values to an array
        x = np.round(x,2)           # round the values
    return(x)

def sendLeft(mySpeed):          # takes at least 0.3 ms
    myPWM = computePWM(mySpeed)
    left_chB.value = myPWM[0]
    left_chA.value = myPWM[1]

def sendRight(mySpeed):         # takes at least 0.3 ms
    myPWM = computePWM(mySpeed)
    right_chB.value = myPWM[0]
    right_chA.value = myPWM[1]

# THIS LOOP RUNS IF THE PROGRAM IS CALLED DIRECTLY
if __name__ == "__main__":
    print("Testing Encoders")
    while True:
      #Encoder Part Phi Dots
      encValues = readShaftPositions() # read the values.  Reading will only change if motor pulley moves
      # round the values and print them separated by a tab
      print("Left: ", encValues[0], "\t","Right: ", encValues[1])
      log.tmpFile(encValues[0], "TempPDLLog.txt") #log Left Wheel Value
      log.tmpFile(encValues[1], "TempPDRLog.txt") #log Right Wheel Value
      time.sleep(0.5)
      
      #Kinematics Part Chassis Speeds
      C = getMotion()  # This take approx 25.1 ms if the delay is 20ms
      print("xdot(m/s), thetadot (rad/s):", C, "\t","deltaT: ", deltaT)
      log.tmpFile(C[0], "TempXDot.txt")
      log.tmpFile(C[1], "TempThetaDot.txt")
      time.sleep(0.2)
