# this code is intended to be used to create a path utilising inverse kinematics
# use this code as a template to create pre-determined paths using known chassis forward velocity (x dot) in m/s, 
# chassis angular velocity (theta dot) in rad/s, and motion duration in sec for each motion to create the path

import L1_log as log
import L2_inverse_kinematics as ik
import L2_speed_control as sc
import numpy as np
from time import sleep
import L1_log as logging

# define some variables that can be used to create the path
# make use of these definitions in the motions list
pi = np.pi                  # utilize the numpy library to define pi
d1 = 0.4                    # distance in meters of segment 1 in the path
d2 = 0.4                    # distance in meters of segment 2 in the path
forward_velocity = 0.4      # forward velocity (x dot) in m/s of SCUTTLE. NOTE that the max forward velocity of SCUTTLE is 0.4 m/s

# below is a list setup to run through each motion segment to create the path.
# the list elements within each list are in order as follows: chassis forward velocity (m/s), chassis angular velocity (rad/s), and motion duration (sec)
# enter the chassis forward velocity (x dot) in m/s, chassis angular velocity (theta dot) in rad/s, and motion duration in sec for each motion to create the path
motions = [
    [0.4, 0.0, 2.0],            # Motion 1 - move forward 0.4 m
    [0.0, 9.7, 2.5],            # Motion 2 - turn +90 degrees
    [0.4, 0.0, 2.0],            # Motion 3 - move forward 0.4 m
    [0.0, 9.7, 2.5],            # Motion 4 - turn +90 degrees
    [0.4, 0.0, 2.0],            # Motion 5 - move forward 0.4 m
    [0.0, -9.7, 2.5],           # Motion 6 - turn -90 degrees
    [0.4, 0.0, 2.0],            # Motion 7 - move forward 0.4 m
    [0.0, -9.7, 2.5],           # Motion 8 - turn -90 degrees
    [0.4, 0.0, 2.0],            # Motion 9 - move forward 0.4 m
    [0.0, 0.0, 0.0],            # Motion 10 - stop
]

# iterate through and perform each open loop motion and then wait the specified duration.
for  count, motion in enumerate(motions):
    print("Motion: ", count+1, "\t Chassis Forward Velocity (m/s): {:.2f} \t Chassis Angular Velocity (rad/s): {:.2f} \t Duration (sec): {:.2f}".format(motion[0], motion[1], motion[2]))
    wheel_speeds = ik.getPdTargets(motion[:2])                  # take the forward speed(m/s) and turning speed(rad/s) and use inverse kinematics to deterimine wheel speeds
    sc.driveOpenLoop(wheel_speeds)                              # take the calculated wheel speeds and use them to run the motors
    sleep(motion[2])                                            # wait the motion duration
    print("Linear Velocity: ", motion[0])
    logging.tmpFile(motion[0], "TempXLog.txt")
    print("Angular Velocity: ", motion[1])
    logging.tmpFile(motion[1], "TempThetaLog.txt")
    sleep(motion[2])