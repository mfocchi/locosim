#common stuff 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm

from base_controller.utils.common_functions import *
from base_controller.utils.ros_publish_ur4 import RosPub
from base_controller.utils.kin_dyn_utils import directKinematics as dk
from base_controller.utils.kin_dyn_utils import computeEndEffectorJacobian as eeJ
from base_controller.utils.kin_dyn_utils import numericalInverseKinematics as ik
from base_controller.utils.kin_dyn_utils import fifthOrderPolynomialTrajectory as coeffTraj
from base_controller.utils.kin_dyn_utils import geometric2analyticJacobian as g2a
from base_controller.utils.kin_dyn_utils import rot2eul

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

q_des = conf.q0
qd_des = conf.qd0
qdd_des = conf.qdd0

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)
#################
# exercise 2.1
#################
# direct kinematics function
T_01, T_02, T_03, T_04, T_0e = dk(q)
# compare with Pinocchio built-in functions 
robot.computeAllTerms(q, qd)
x = robot.framePlacement(q, frame_ee).translation
o = robot.framePlacement(q, frame_ee).rotation
position_diff = x - T_0e[:3,3]
rotation_diff = o - T_0e[:3,:3]
print position_diff
print rotation_diff


#################
# exercise 2.2
#################
J,z1,z2,z3,z4 = eeJ(q)
# compare with Pinocchio      
Jee = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  
jacobian_diff = J - Jee
print jacobian_diff

##################
# exercise 2.3
##################
J_r = g2a(J, T_0e)

###################
# exercise 2.4
###################

# desired task space position
p = np.array([-0.5, -0.2, 0.5, math.pi/3])
# outside of workspace, gets the solution with minumum error
#p = np.array([-2.5, -0.2, 0.5, math.pi/3])

# initial value for numerical ik
#q_i  = np.array([ 0.5, -1.0, -0.8, -math.pi]) # good initialization
q_i  = np.array([ -5, 5.0, -0.8, -math.pi]) # bad initialization
# solution of the numerical ik
q_f = ik(p,q_i) 
# compare solution with values obtained through direct kinematics
T_01, T_02, T_03, T_04, T_0e = dk(q_f)
rpy = rot2eul(T_0e[:3,:3])
task_diff = p - np.hstack((T_0e[:3,3],rpy[0]))
print "task"
print task_diff
print "qf"
print q_f
    
###################
# exercise 2.5
###################    
while np.count_nonzero(q - q_f) :          

    # Polynomial trajectory
    for i in range(4):      
        a = coeffTraj(3,conf.q0[i],q_f[i])     
        qdd[i] = 2*a[2] + 6*a[3]*time + 12*a[4]*time**2 + 20*a[5]*time**3
        qd[i] = a[1] + 2*a[2]*time + 3*a[3]*time**2 + 4*a[4]*time**3 + 5*a[5]*time**4
        q[i] = a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5

    # Log Data into a vector
    time_log = np.append(time_log, time)	
    q_log = np.vstack((q_log, q ))
    q_des_log= np.vstack((q_des_log, q_des))
    qd_log= np.vstack((qd_log, qd))
    qd_des_log= np.vstack((qd_des_log, qd_des))
    qdd_log= np.vstack((qdd_log, qdd))
    qdd_des_log= np.vstack((qdd_des_log, qdd_des))
    # tau_log = np.vstack((tau_log, tau))            
 
    # update time
    time = time + conf.dt                  
    #publish joint variables
    ros_pub.publish(robot, q, qd)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break;

raw_input("Final position reached. Press Enter to continue and plot results")
ros_pub.deregister_node()  
                
# plot joint variables                                                                              
plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
raw_input("Press Enter to exit")






