# -*- coding: utf-8 -*-
"""
Created on Thu Apr  2 18:07:44 2020

@author: mfocchi
"""
import os
import psutil
#from pinocchio.visualize import GepettoVisualizer
from custom_robot_wrapper import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt
import sys
from termcolor import colored
from urdf_parser_py.urdf import URDF
#make plot interactive
plt.ion()
plt.close() 


def getRobotModel():    
    
    # Import the model
    ERROR_MSG = 'You should set the environment variable UR5_MODEL_DIR to something like "$DEVEL_DIR/install/share"\n';
    path      = os.environ.get('UR5_MODEL_DIR', ERROR_MSG)
    urdf      = path + "/ur_description/urdf/ur.urdf";
    srdf      = path + '/ur_description/srdf/ur_gripper.srdf'
    robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])
                                     
    #get urdf from ros just in case you need
    #robot_urdf_ros = URDF.from_parameter_server()                                                                                                                                                                                                                                                                                                              
    
    return robot    

def getRobotModel4():    
    
    # Import the model
    ERROR_MSG = 'You should set the environment variable UR5_MODEL_DIR to something like "$DEVEL_DIR/install/share"\n';
    path      = os.environ.get('LOCOSIM_DIR', ERROR_MSG)
    urdf      = path + "/visualize_ur/ur4_description/ur4_robot.urdf";
    srdf      = path + '/ur_description/srdf/ur_gripper.srdf'
    robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])
                                     
    #get urdf from ros just in case you need
    #robot_urdf_ros = URDF.from_parameter_server()                                                                                                                                                                                                                                                                                                              
    
    return robot                  

def plotJoint(name, figure_id, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log, tau_ffwd_log = None):
    if name == 'position':
        plot_var_log = q_log
        plot_var_des_log = q_des_log
    elif name == 'velocity':
        plot_var_log = qd_log
        plot_var_des_log  = qd_des_log
    elif name == 'acceleration':
        plot_var_log = qdd_log
        plot_var_des_log  = qdd_des_log
    elif name == 'torque':
        plot_var_log = tau_log
        if   (tau_ffwd_log is not None):    								
            plot_var_des_log  = tau_ffwd_log 
        else:
		  plot_var_des_log = None												
    else:
       print(colored("plotJopnt error: wrong input string", "red") )
       return                                   

    lw_des=7
    lw_act=4          

    njoints = plot_var_log.shape[1]																
    
    #neet to transpose the matrix other wise it cannot be plot with numpy array    
    fig = plt.figure(figure_id)				
    fig.suptitle(name, fontsize=20)             
    labels_ur = ["1 - Shoulder Pan", "2 - Shoulder Lift","3 - Elbow","4 - Wrist 1","5 - Wrist 2","6 - Wrist 3"]
    labels_hyq = ["LF_HAA", "LF_HFE","LF_KFE","RF_HAA", "RF_HFE","RF_KFE","LH_HAA", "LH_HFE","LH_KFE","RH_HAA", "RH_HFE","RH_KFE"]

    if njoints == 4:
        labels = labels_ur 		
    if njoints == 12:
        labels = labels_hyq 	             

    
    for jidx in range(njoints):
				
	    plt.subplot(njoints/2,2,jidx+1)
	    plt.ylabel(labels[jidx])    
	    plt.plot(time_log, plot_var_des_log[:-1,jidx], linestyle='-', lw=lw_des,color = 'red')
	    plt.plot(time_log, plot_var_log[:-1,jidx],linestyle='-', lw=lw_act,color = 'blue')
	    plt.grid()
                
    

                
def plotEndeff(name, figure_id, time_log, x_log, x_des_log=None, xd_log=None, xd_des_log=None, euler = None, euler_des = None, f_log=None):
    plot_var_des_log = None
    if name == 'position':
        plot_var_log = x_log
        if   (x_des_log is not None):                                
           plot_var_des_log = x_des_log                            
    elif name == 'force':
        plot_var_log = f_log
    elif  name == 'velocity':    
        plot_var_log = xd_log
        if   (xd_des_log is not None):                                
             plot_var_des_log = xd_des_log         
    elif  name == 'orientation':    
        plot_var_log = euler                             
        plot_var_des_log = euler_des                               
    else:
       print("wrong choice")                    
       
    lw_act=4  
    lw_des=7
                    
    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)                   
    plt.subplot(3,1,1)
    plt.ylabel("end-effector x")
    if   (plot_var_des_log is not None):
         plt.plot(time_log, plot_var_des_log[0,:].T, lw=lw_des, color = 'red')                    
    plt.plot(time_log, plot_var_log[0,:].T, lw=lw_act, color = 'blue')
    plt.grid()
    
    plt.subplot(3,1,2)
    plt.ylabel("end-effector y")    
    if   (plot_var_des_log is not None):
         plt.plot(time_log, plot_var_des_log[1,:].T, lw=lw_des, color = 'red')                    
    plt.plot(time_log, plot_var_log[1,:].T, lw=lw_act, color = 'blue')
    plt.grid()
    
    plt.subplot(3,1,3)
    plt.ylabel("end-effector z")    
    if   (plot_var_des_log is not None):
        plt.plot(time_log, plot_var_des_log[2,:].T, lw=lw_des, color = 'red')                                        
    plt.plot(time_log, plot_var_log[2,:].T, lw=lw_act, color = 'blue')
    plt.grid()
      
    
def plotCoM(name, figure_id, time_log, des_basePoseW, basePoseW, des_baseTwistW, baseTwistW, des_baseAccW, wrenchW):
    plot_var_log = None
    if name == 'position':
        plot_var_log = basePoseW
        plot_var_des_log = des_basePoseW
    elif name == 'velocity':
        plot_var_log = baseTwistW
        plot_var_des_log  = des_baseTwistW
    elif name == 'acceleration':
        plot_var_des_log  = des_baseAccW    
    elif name == 'wrench':
        plot_var_des_log  = wrenchW                                    
    else:
       print("wrong choice")                                    

    lw_des=7
    lw_act=4          
                
    #neet to transpose the matrix other wise it cannot be plot with numpy array    
    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)             
    plt.subplot(3,2,1)
    plt.ylabel("CoM X")    
    plt.plot(time_log, plot_var_des_log[0,:], linestyle='-', lw=lw_des,color = 'red')
    if   (plot_var_log is not None):                
        plt.plot(time_log, plot_var_log[0,:],linestyle='-', lw=lw_act,color = 'blue')
    plt.grid()
                
    plt.subplot(3,2,3)
    plt.ylabel("CoM Y")
    plt.plot(time_log, plot_var_des_log[1,:], linestyle='-', lw=lw_des,color = 'red', label="q_des")
    if   (plot_var_log is not None):    
        plt.plot(time_log, plot_var_log[1,:],linestyle='-',lw=lw_act, color = 'blue', label="q")
    plt.legend(bbox_to_anchor=(-0.01, 1.115, 1.01, 0.115), loc=3, mode="expand")
    plt.grid()
    
    plt.subplot(3,2,5)
    plt.ylabel("CoM Z")    
    plt.plot(time_log, plot_var_des_log[2,:],linestyle='-',lw=lw_des,color = 'red')
    if   (plot_var_log is not None):    
       plt.plot(time_log, plot_var_log[2,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()    
    
    plt.subplot(3,2,2)
    plt.ylabel("Roll")   
    plt.plot(time_log, plot_var_des_log[3,:],linestyle='-',lw=lw_des,color = 'red')
    if   (plot_var_log is not None):    
        plt.plot(time_log, plot_var_log[3,:].T,linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    
    plt.subplot(3,2,4)
    plt.ylabel("Pitch")   
    plt.plot(time_log, plot_var_des_log[4,:],linestyle='-',lw=lw_des,color = 'red')
    if   (plot_var_log is not None):        
        plt.plot(time_log, plot_var_log[4,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    
    plt.subplot(3,2,6)
    plt.ylabel("Yaw")
    plt.plot(time_log, plot_var_des_log[5,:],linestyle='-',lw=lw_des,color = 'red')
    if   (plot_var_log is not None):        
        plt.plot(time_log, plot_var_log[5,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
                
         
    
        
def plotGRFs(figure_id, time_log, des_forces, act_forces):
	       
    lw_act=4  
    lw_des=7
    # %% Input plots

    fig = plt.figure(figure_id)
    fig.suptitle("ground reaction forces", fontsize=20)  
    plt.subplot(6,2,1)
    plt.ylabel("$LF_x$", fontsize=10)
    plt.plot(time_log, des_forces[0,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[0,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((-100,100))
				
    plt.subplot(6,2,3)
    plt.ylabel("$LF_y$", fontsize=10)
    plt.plot(time_log, des_forces[1,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[1,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((-100,100))
				
    plt.subplot(6,2,5)
    plt.ylabel("$LF_z$", fontsize=10)
    plt.plot(time_log, des_forces[2,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[2,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((0,450))

    #RF
    plt.subplot(6,2,2)
    plt.ylabel("$RF_x$", fontsize=10)
    plt.plot(time_log, des_forces[3,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[3,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((-100,100))
				
    plt.subplot(6,2,4)
    plt.ylabel("$RF_y$", fontsize=10)
    plt.plot(time_log, des_forces[4,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[4,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((-100,100))
				
    plt.subplot(6,2,6)
    plt.ylabel("$RF_z$", fontsize=10)
    plt.plot(time_log, des_forces[5,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[5,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((0,450))
				
     #LH
    plt.subplot(6,2,7)
    plt.ylabel("$LH_x$", fontsize=10)
    plt.plot(time_log, des_forces[6,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[6,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((-100,100))
				
    plt.subplot(6,2,9)
    plt.ylabel("$LH_y$", fontsize=10)
    plt.plot(time_log, des_forces[7,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[7,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((-100,100))
				
				
    plt.subplot(6,2,11)
    plt.ylabel("$LH_z$", fontsize=10)
    plt.plot(time_log, des_forces[8,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[8,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((0,450))
				
     #RH
    plt.subplot(6,2,8)
    plt.ylabel("$RH_x$", fontsize=10)
    plt.plot(time_log, des_forces[9,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[9,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((-100,100))
				
    plt.subplot(6,2,10)
    plt.ylabel("$RH_y$", fontsize=10)
    plt.plot(time_log, des_forces[10,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[10,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((-100,100))
						
    plt.subplot(6,2,12)
    plt.ylabel("$RH_z$", fontsize=10)
    plt.plot(time_log, des_forces[11,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[11,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    plt.ylim((0,450))

def plotConstraitViolation(figure_id,constr_viol_log):
    fig = plt.figure(figure_id)			
    plt.plot(constr_viol_log[0,:],label="LF")
    plt.plot(constr_viol_log[1,:],label="RF")
    plt.plot(constr_viol_log[2,:],label="LH")
    plt.plot(constr_viol_log[3,:],label="RH")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("Constr violation", fontsize=10)
    plt.grid()	                                                             
