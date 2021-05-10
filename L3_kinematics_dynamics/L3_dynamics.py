#common stuff 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm

import os
from base_controller.utils.common_functions import *
from base_controller.utils.ros_publish_ur4 import RosPub
from base_controller.utils.kin_dyn_utils import getM
from base_controller.utils.kin_dyn_utils import getg
from base_controller.utils.kin_dyn_utils import getC

import L3_conf as conf

#instantiate graphic utils
ros_pub = RosPub()
robot = getRobotModel4()


# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0])
time = 0.0

# Init loggers
q_log = np.empty((4))*nan
q_des_log = np.empty((4))*nan
qd_log = np.empty((4))*nan
qd_des_log = np.empty((4))*nan
qdd_log = np.empty((4))*nan
qdd_des_log = np.empty((4))*nan
tau_log = np.empty((4))*nan
f_log = np.empty((3,0))*nan
x_log = np.empty((3,0))*nan
time_log =  np.empty((0,0))*nan

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des = zero
qd_des = zero
qdd_des = zero        # joint reference acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

error = np.array([1, 1, 1, 1])

# Main loop to simulate dynamics
while any(i >= 0.01 for i in np.abs(error)):

    robot.computeAllTerms(q, qd)

    # Exercise 3.1
    # gravity terms
    g = robot.gravity(q)
    g_ = getg()

    # joint space inertia matrix                
    M = robot.mass(q, False)
    M_ = getM()

    # vector of nonlinear effects (gravity + Coriolis)
    h = robot.nle(q, qd, False)
    # vector of Coriolis terms
    C = getC()

    # Exercise 3.2
    # Simulation of the forward dynamics

    # Exercise 3.3
    # Forward Euler Integration    

    # Log Data into a vector
    time_log = np.append(time_log, time)	
    q_log = np.vstack((q_log, q ))
    q_des_log= np.vstack((q_des_log, q_des))
    qd_log= np.vstack((qd_log, qd))
    qd_des_log= np.vstack((qd_des_log, qd_des))
    qdd_log= np.vstack((qdd_log, qd))
    qdd_des_log= np.vstack((qdd_des_log, qdd_des))
    # tau_log = np.vstack((tau_log, tau))            
 
    # update time
    time = time + conf.dt

    #publish joint variables
    ros_pub.publish(robot, q, qd,damping)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break
            
raw_input("Robot came to a stop. Press Enter to continue")
ros_pub.deregister_node()
        
                 
                
# plot joint variables                                                                              
plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
# plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
# plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('torque', 3, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)

raw_input("Press Enter to continue")





