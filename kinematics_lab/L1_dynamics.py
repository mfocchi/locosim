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
from kin_utils_test import RNEA as rnea
from kin_utils_test import getM
from kin_utils_test import geth

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

# CONTROL LOOP
while True:
    # EXERCISE 1: Sinusoidal reference Generation
    q_des  = conf.q0 +   np.multiply( conf.amp, np.sin(two_pi_f*time + conf.phi))
    qd_des = np.multiply(two_pi_f_amp , np.cos(two_pi_f*time + conf.phi))
    qdd_des = np.multiply( two_pi_f_squared_amp , -np.sin(two_pi_f*time + conf.phi))

    # Set constant reference after a while
    if time >= conf.exp_duration_sin:
        q_des  = conf.q0
        qd_des=zero
        qdd_des=zero          

    # EXERCISE 2: Step reference Generation
#    if time > 2.0:
#        q_des = conf.q0 + np.matrix([ 0.0, -0.4, 0.0, 0.0,  0.5, 0.0]).T 
#        qd_des =  zero
#        qdd_des = zero 
#    else:
#        q_des = conf.q0
#        qd_des =  zero
#        qdd_des = zero    

 
    # Decimate print of time
    #if (divmod(time ,1.0)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= conf.exp_duration:
        break
                            
    
    # print("q:")
    # print(q)
    # print("qd:")
    # print(qd)
    robot.computeAllTerms(q, qd) 
    # joint space inertia matrix                
    M = robot.mass(q, False)

    M_ = getM(q)

    print("M pinocchio:")
    print(M)
    print("M Tavo:")
    print(M_)
    if np.linalg.det(M_) <= 0.0:
        print("tavo")
        print(np.linalg.det(M_))
    if np.linalg.det(M) <= 0.0:
        print("pinocchio")
        print(np.linalg.det(M))
    # bias terms                
    h = robot.nle(q, qd, False)
    h_ = geth(9.81,q)
    print("h pinocchio:")
    print(h)
    print("h_ Tavo:")
    print(h_)
    #gravity terms                
    g = robot.gravity(q)
    g_ = geth(9.81,q)
    print("g pinocchio:")
    print(g)
    print("g_ Tavo:")
    print(g_)

        
    # EXERCISE  5: PD control critical damping
#    conf.kd[0,0] = 2*np.sqrt(conf.kp[0,0]*M[0,0])
#    conf.kd[1,1] = 2*np.sqrt(conf.kp[1,1]*M[1,1])
#    conf.kd[2,2] = 2*np.sqrt(conf.kp[2,2]*M[2,2])
#    conf.kd[3,3] = 2*np.sqrt(conf.kp[3,3]*M[3,3])
#    conf.kd[4,4] = 2*np.sqrt(conf.kp[4,4]*M[4,4])
#    conf.kd[5,5] = 2*np.sqrt(conf.kp[5,5]*M[5,5])
#    print (2*np.sqrt(300*M[4,4])    )
                                
    #CONTROLLERS                                    
    #Exercise 3:  PD control
    #tau = conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd)
    
    # Exercise 6: PD control + Gravity Compensation
    tau = 0*conf.kp.dot(q_des-q) + 0*conf.kd.dot(qd_des-qd)  + 0*g - 0.5*qd
    
    # Exercise 7: PD + gravity + Feed-Forward term
    #tau= np.diag(M).dot(qdd_des) + conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd) + g

    # EXERCISE 8_ Inverse Dynamics
    #tau= M.dot(qdd_des+ conf.kp.dot(q_des-q)+ conf.kd.dot(qd_des-qd)) + h    

   
    x = robot.framePlacement(q, frame_ee).translation 
				    # compute jacobian of the end effector (in the WF)        
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
    			
    # print("frame ee:", frame_ee)
    # print("End-effector Jacobian:", J6)

    # if conf.EXTERNAL_FORCE: 
    #  #compute ee position  in the world frame  
    #  x = robot.framePlacement(q, frame_ee).translation  
    #  # compute jacobian of the end effector (in the WF)
    #  J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    #  # take first 3 rows of J6 cause we have a point contact            
    #  J = J6[:3,:] 					

    #  # Add external force at T =2.0s (EXERCISE 11)
    #  if time>2.0:         
    #      F_env = conf.extForce
    #  else:
    #      F_env = np.array([0.0, 0.0, 0.0])  									
			
	# Add  unilateral compliant contact (EXERCISE 12)
#     xd = J.dot(qd)
#     if (x[2]<0.0):      
#        F_env = np.array([0, 0, -10000*(x[2]) -100*xd[2] ] )
#     else:
#        F_env = np.array([0.0, 0.0, 0.0])     								 
							
    #  tau += 0*J.transpose().dot(F_env)     		      
    #  ros_pub.add_arrow(x,F_env/100) 
    


    #SIMULATION of the forward dynamics    
    M_inv = np.linalg.inv(M)  
    qdd = M_inv.dot(tau-h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt    
    q = q + conf.dt*qd  + 0.5*conf.dt*conf.dt*qdd 
    

    # Log Data into a vector
    time_log = np.append(time_log, time)	
    q_log = np.vstack((q_log, q ))
    q_des_log= np.vstack((q_des_log, q_des))
    qd_log= np.vstack((qd_log, qd))
    qd_des_log= np.vstack((qd_des_log, qd_des))
    qdd_log= np.vstack((qdd_log, qd))
    qdd_des_log= np.vstack((qdd_des_log, qdd_des))
    tau_log = np.vstack((tau_log, tau))                
 
    # update time
    time = time + conf.dt                  
                
    #publish joint variables
    ros_pub.publish(robot, q, qd, tau)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break;
            
ros_pub.deregister_node()
        
                 
                
# plot joint variables                                                                              
plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('torque', 3, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)





