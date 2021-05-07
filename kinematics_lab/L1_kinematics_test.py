#common stuff 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm
import eigenpy

import os
from base_controller.utils.common_functions import *
from base_controller.utils.optimTools import quadprog_solve_qp
from base_controller.utils.ros_publish_ur4 import RosPub
from kin_utils import directKinematics as dk
from kin_utils import computeEndEffectorJacobian as eeJ
from kin_utils import numericalInverseKinematics as ik
from kin_utils import fifthOrderPolynomialTrajectory as coeffTraj
from kin_utils import geometric2analyticJacobian as g2a

import ex_1_conf_kin as conf

#instantiate graphic utils
ros_pub = RosPub()
robot = getRobotModel4()


# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0])
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

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

# EXERCISE 9: 
conf.qd0 = two_pi_f_amp

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

T_01, T_02, T_03, T_04, T_0e = dk(q)

p = np.array([-0.5,-0.2,0.5,math.pi])

q_0  = np.array([ -0.5, -1.0, -0.3, -math.pi]) 
J_man,z1,z2,z3,z4 = eeJ(q)
print("J:")
print(J_man)
# q_0  = np.array([ 0.1, 0.1, 0.1, 0.1]) 
q_f = ik(p,q_0)


print("T_0e")
print(T_0e)

J_r = g2a(J_man, T_0e)

print("J:")
print(J_man)

print("J_r:")
print(J_r)
print()


a = coeffTraj(10,conf.q0[0],q[0])

print(q_f)
print()


# CONTROL LOOP
while True:          
 
    # Decimate print of time
    #if (divmod(time ,1.0)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= conf.exp_duration:
        break
       

    robot.computeAllTerms(q, qd) 
    x = robot.framePlacement(q, frame_ee).translation 
    o = robot.framePlacement(q, frame_ee).rotation
				    # compute jacobian of the end effector (in the WF)        
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                   
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 

    # print("time: ", time)
    # print("frame pos")
    # print(x)
    # print("desired pos:")
    # print(p)
    # print("J6:")
    # print(J6)
       
    # Forward Euler Integration 
    for i in range(4):   
        a = coeffTraj(3,conf.q0[i],q_f[i])
        qdd[i] = 2*a[2] + 6*a[3]*time + 12*a[4]*time**2 + 20*a[5]*time**3
        qd[i] = a[1] + 2*a[2]*time + 3*a[3]*time**2 + 4*a[4]*time**3 + 5*a[5]*time**4
        q[i] = a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5

        # if q[i] > 2*math.pi:
        #     q[i] = q[i] - 2*math.pi

    # print("a:")
    # print(a)
    # print("q:")
    # print(q)
    # qd = qd + 0*conf.dt    
    # q = q + conf.dt*qd  + 0.5*conf.dt*conf.dt*0

    

    # Log Data into a vector
    time_log = np.append(time_log, time)	
    q_log = np.vstack((q_log, q ))
    qd_log= np.vstack((qd_log, qd))            
 
    # update time
    time = time + conf.dt                  
    #publish joint variables
    ros_pub.publish(robot, q, qd)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break;
            
ros_pub.deregister_node()
        
                 
                
# plot joint variables                                                                              
# plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('torque', 3, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)





