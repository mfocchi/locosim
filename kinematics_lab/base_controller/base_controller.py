# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""



import copy
import numpy as np
import os

import rospy as ros
import sys
import time
import threading

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty, EmptyRequest
from termcolor import colored

#gazebo messages
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
#gazebo services
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

# ros utils
import roslaunch
import rosnode
import rosgraph

#other utils
from utils.ros_publish import RosPub
from utils.pidManager import PidManager
from utils.utils import Utils
from utils.math_tools import *
from numpy import nan

#robot specific 
from hyq_kinematics.hyq_kinematics import HyQKinematics

#dynamics
from utils.custom_robot_wrapper import RobotWrapper



class BaseController(threading.Thread):
    
    def __init__(self):  
        
       #clean up previous process                

        os.system("killall gzserver gzclient")                                
        if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
            print 'ROS MASTER is active'
            nodes = rosnode.get_node_names()
            if "/rviz" in nodes:
                 print("Rviz active")
                 rvizflag=" rviz:=false"
            else:                                                         
                 rvizflag=" rviz:=true" 
        #start ros impedance controller
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)   
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.environ['LOCOSIM_DIR'] + "/ros_impedance_controller/launch/ros_impedance_controller_stdalone.launch"])
        #only available in ros lunar
#        roslaunch_args=rvizflag                             
#        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.environ['LOCOSIM_DIR'] + "/ros_impedance_controller/launch/ros_impedance_controller_stdalone.launch"],roslaunch_args=[roslaunch_args])
        self.launch.start() 
        ros.sleep(4.0)        


        threading.Thread.__init__(self)
								
        # instantiating objects
        self.ros_pub = RosPub(True)                    
        self.joint_names = ""
        self.u = Utils()
        self.kin = HyQKinematics()			
                                
        self.comPoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.stance_legs = np.array([True, True, True, True])
        
        self.q = np.zeros(12)
        self.qd = np.zeros(12)
        self.tau = np.zeros(12)                                
        self.q_des =np.zeros(12)
        self.qd_des = np.zeros(12)
        self.tau_ffwd =np.zeros(12)
        
        self.b_R_w = np.eye(3)       
                                  
        self.grForcesW = np.zeros(12)
        self.basePoseW = np.zeros(6) 
        self.J = [np.eye(3)]* 4                                   
        self.wJ = [np.eye(3)]* 4                       
                                
        self.robot_name = ros.get_param('/robot_name')
        self.sub_contact = ros.Subscriber("/"+self.robot_name+"/contacts_state", ContactsState, callback=self._receive_contact, queue_size=100)
        self.sub_pose = ros.Subscriber("/"+self.robot_name+"/ground_truth", Odometry, callback=self._receive_pose, queue_size=1)
        self.sub_jstate = ros.Subscriber("/"+self.robot_name+"/joint_states", JointState, callback=self._receive_jstate, queue_size=1)                  
        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1)

        # freeze base  and pause simulation service 
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_gravity = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
                                
                                
       # Loading a robot model of HyQ (Pinocchio)
        ERROR_MSG = 'You should set the environment variable UR5_MODEL_DIR to something like "$DEVEL_DIR/install/share"\n';
        path      = os.environ.get('LOCOSIM_DIR', ERROR_MSG)
        urdf      = path + "/ros_impedance_controller/config/"+ self.robot_name+".urdf";
        srdf      = path + "/ros_impedance_controller/config/"+ self.robot_name+".srdf";
        self.robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])
								
								
	   #send data to param server
        self.verbose = True	#this can be read from config file																							
        self.u.putIntoGlobalParamServer("verbose", self.verbose)   
                                
    def _receive_contact(self, msg):
        # get the ground truth from gazebo (only works with framwork, dls_hw_sim has already LF RF LH RH convention)
#        self.grForcesW[0] = msg.states[0].wrenches[0].force.x
#        self.grForcesW[1] =  msg.states[0].wrenches[0].force.y
#        self.grForcesW[2] =  msg.states[0].wrenches[0].force.z
#        self.grForcesW[3] = msg.states[1].wrenches[0].force.x
#        self.grForcesW[4] =  msg.states[1].wrenches[0].force.y
#        self.grForcesW[5] =  msg.states[1].wrenches[0].force.z
#        self.grForcesW[6] = msg.states[2].wrenches[0].force.x
#        self.grForcesW[7] =  msg.states[2].wrenches[0].force.y
#        self.grForcesW[8] =  msg.states[2].wrenches[0].force.z
#        self.grForcesW[9] = msg.states[3].wrenches[0].force.x
#        self.grForcesW[10] =  msg.states[3].wrenches[0].force.y
#        self.grForcesW[11] =  msg.states[3].wrenches[0].force.z
        pass
                                                
    def _receive_pose(self, msg):
        
        self.quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(self.quaternion)

        self.basePoseW[self.u.sp_crd["LX"]] = msg.pose.pose.position.x
        self.basePoseW[self.u.sp_crd["LY"]] = msg.pose.pose.position.y
        self.basePoseW[self.u.sp_crd["LZ"]] = msg.pose.pose.position.z
        self.basePoseW[self.u.sp_crd["AX"]] = euler[0]
        self.basePoseW[self.u.sp_crd["AY"]] = euler[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = euler[2]

        self.baseTwistW[self.u.sp_crd["LX"]] = msg.twist.twist.linear.x
        self.baseTwistW[self.u.sp_crd["LY"]] = msg.twist.twist.linear.y
        self.baseTwistW[self.u.sp_crd["LZ"]] = msg.twist.twist.linear.z
        self.baseTwistW[self.u.sp_crd["AX"]] = msg.twist.twist.angular.x
        self.baseTwistW[self.u.sp_crd["AY"]] = msg.twist.twist.angular.y
        self.baseTwistW[self.u.sp_crd["AZ"]] = msg.twist.twist.angular.z
        
        mathJet = Math()
        # compute orientation matrix                                
        self.b_R_w = mathJet.rpyToRot(euler)
   
    def _receive_jstate(self, msg):
          #need to map to robcogen only the arrays coming from gazebo because of ROS convention is different
         self.joint_names = msg.name   
         q_ros = np.zeros(12)
         qd_ros = np.zeros(12)
         tau_ros = np.zeros(12)             
         for i in range(len(self.joint_names)):           
             q_ros[i] = msg.position[i]
             qd_ros[i] = msg.velocity[i]
             tau_ros[i] = msg.effort[i]
         #map from ROS (alphabetical) to our  LF RF LH RH convention
         self.q = self.u.mapFromRos(q_ros)
         self.qd = self.u.mapFromRos(qd_ros)                    
         self.tau = self.u.mapFromRos(tau_ros)  
         
    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         msg.position = q_des
         msg.velocity = qd_des
         msg.effort = tau_ffwd                
         self.pub_des_jstate.publish(msg)     

    def register_node(self):
        ros.init_node('controller_python', disable_signals=False, anonymous=False)

    def deregister_node(self):
        print "deregistering nodes"     
        os.system(" rosnode kill /hyq/ros_impedance_controller")    
        os.system(" rosnode kill /gazebo")    
 
    def get_contact(self):
        return self.contactsW
    def get_pose(self):
        return self.basePoseW
    def get_jstate(self):
        return self.q

        
    def freezeBase(self, flag):
        #toggle gravity
        req_reset_gravity = SetPhysicsPropertiesRequest()
        #ode config
        req_reset_gravity.time_step = 0.001
        req_reset_gravity.max_update_rate = 1000                
        req_reset_gravity.ode_config.sor_pgs_iters = 50
        req_reset_gravity.ode_config.sor_pgs_w = 1.3        
        req_reset_gravity.ode_config.contact_surface_layer = 0.001
        req_reset_gravity.ode_config.contact_max_correcting_vel = 100
        req_reset_gravity.ode_config.erp = 0.2
        req_reset_gravity.ode_config.max_contacts = 20        
        
        if (flag):
            req_reset_gravity.gravity.z =  0.0
        else:
            req_reset_gravity.gravity.z = -9.81                
        self.reset_gravity(req_reset_gravity)

        # create the message
        req_reset_world = SetModelStateRequest()
        #create model state
        model_state = ModelState()        
        model_state.model_name = "hyq"
        model_state.pose.position.x = 0.0
        model_state.pose.position.y = 0.0        
        model_state.pose.position.z = 0.8

        model_state.pose.orientation.w = 1.0
        model_state.pose.orientation.x = 0.0       
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0                
                                
        model_state.twist.linear.x = 0.0
        model_state.twist.linear.y = 0.0        
        model_state.twist.linear.z = 0.0
             
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0        
        model_state.twist.angular.z = 0.0
             
        req_reset_world.model_state = model_state
        # send request and get response (in this case none)
        self.reset_world(req_reset_world) 

    def initKinematics(self,kin):
        kin.init_homogeneous()
        kin.init_jacobians()  
                                
    def mapBaseToWorld(self, B_var):
        W_var = self.b_R_w.transpose().dot(B_var) + self.u.linPart(self.basePoseW)                            
        return W_var
                                                                                                                                
    def updateKinematics(self,kin):
        # q is continuously updated
        kin.update_homogeneous(self.q)
        kin.update_jacobians(self.q)
        self.B_contacts = kin.forward_kin(self.q) 
        # map feet contacts to wf
        self.W_contacts = np.zeros((3,4))
        for leg in range(4):
             self.W_contacts[:,leg] = self.mapBaseToWorld(self.B_contacts[leg, :].transpose())
        # update the feet jacobians
        self.J[self.u.leg_map["LF"]], self.J[self.u.leg_map["RF"]], self.J[self.u.leg_map["LH"]], self.J[self.u.leg_map["RH"]], flag = kin.getLegJacobians()
        #map jacobians to WF
        for leg in range(4):
            self.wJ[leg] = self.b_R_w.transpose().dot(self.J[leg])
             
        # Pinocchio Update the joint and frame placements
        gen_velocities  = np.hstack((self.baseTwistW,self.qd))
        configuration = np.hstack(( self.u.linPart(self.basePoseW), self.quaternion, self.q))
        self.robot.computeAllTerms(configuration, gen_velocities)    
        self.M = self.robot.mass(self.q, False)    
        self.h = self.robot.nle(configuration, gen_velocities, False)
        self.h_joints = self.h[6:]  
        #compute contact forces                        
        self.estimateContactForces()             

    def estimateContactForces(self):           
        # estimate ground reaxtion forces from tau 
        for leg in range(4):
            grf = np.linalg.inv(self.wJ[leg].T).dot(self.u.getLegJointState(leg, self.h_joints - self.tau ))                             
            self.u.setLegJointState(leg, grf, self.grForcesW)                                  
                                  
                                 
    def startupProcedure(self):
        self.unpause_physics_client(EmptyRequest()) #pulls robot up
        ros.sleep(0.2)  # wait for callback to fill in jointmnames
                                
        self.pid = PidManager(self.joint_names) #I start after cause it needs joint names filled in by receive jstate callback
        # set joint pdi gains
        self.pid.setPDs(400.0, 6.0, 0.0)
        # GOZERO Keep the fixed configuration for the joints at the start of simulation
        self.q_des = np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4])
        self.qd_des = np.zeros(12)
        self.tau_ffwd = np.zeros(12)
                                
       # these torques are to compensate the leg gravity
        self.gravity_comp = np.array(
            [24.2571, 1.92, 50.5, 24.2, 1.92, 50.5739, 21.3801, -2.08377, -44.9598, 21.3858, -2.08365, -44.9615])
                                                
        print("reset posture...")
        self.freezeBase(1)
        start_t = ros.get_time()
        while ros.get_time() - start_t < 1.0:
            self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
            ros.sleep(0.01)
        if self.verbose:
            print("q err prima freeze base", (self.q - self.q_des))
  
        print("put on ground and start compensating gravity...")
        self.freezeBase(0)                                                
        ros.sleep(1.0)
        if self.verbose:
            print("q err pre grav comp", (self.q - self.q_des))
                                                
        start_t = ros.get_time()
        while ros.get_time()- start_t < 1.0:
            self.send_des_jstate(self.q_des, self.qd_des, self.gravity_comp)
            ros.sleep(0.01)
        if self.verbose:
            print("q err post grav comp", (self.q - self.q_des))
                                                
        print("starting com controller (no joint PD)...")                
        self.pid.setPDs(0.0, 0.0, 0.0)

    def initVars(self):
 
        self.basePoseW_log = np.empty((6,0 ))*nan
        self.baseTwistW_log = np.empty((6,0 ))*nan
        self.q_des_log = np.empty((12,0 ))*nan    
        self.q_log = np.empty((12,0 )) *nan   
        self.qd_des_log = np.empty((12,0 ))*nan    
        self.qd_log = np.empty((12,0 )) *nan                                  
        self.tau_ffwd_log = np.empty((12,0 ))*nan    
        self.tau_log = np.empty((12,0 ))*nan                                  
        self.grForcesW_log = np.empty((12,0 ))  *nan 
        self.time_log = np.array([])*nan
        self.constr_viol_log = np.empty((4,0 ))*nan
        self.time = 0.0

    def logData(self):
        
        self.basePoseW_log = np.hstack((self.basePoseW_log , self.basePoseW.reshape(6,-1)))
        self.baseTwistW_log = np.hstack((self.baseTwistW_log , self.baseTwistW.reshape(6,-1)))
        self.q_des_log = np.hstack((self.q_des_log , self.q_des.reshape(12,-1)))   
        self.q_log = np.hstack((self.q_log , self.q.reshape(12,-1)))       
        self.qd_des_log = np.hstack((self.qd_des_log , self.qd_des.reshape(12,-1)))   
        self.qd_log = np.hstack((self.qd_log , self.qd.reshape(12,-1)))                                      
        self.tau_ffwd_log = np.hstack((self.tau_ffwd_log , self.tau_ffwd.reshape(12,-1)))                                
        self.tau_log = np.hstack((self.tau_log , self.tau.reshape(12,-1)))                                  
        self.grForcesW_log = np.hstack((self.grForcesW_log , self.grForcesW.reshape(12,-1)))    
        self.time_log = np.hstack((self.time_log, self.time))
	
def talker(p):
            
    p.start()
    p.register_node()
    p.initKinematics(p.kin) 
    p.initVars()        
    p.startupProcedure() 

    #looop frequency
    dt = 0.004                   
    rate = ros.Rate(1/dt) # 250Hz can be read from config file
             
    #control loop
    while True:  
        #update the kinematics
        p.updateKinematics(p.kin)    

        # controller                             
        p.tau_ffwd = 300.0 * np.subtract(p.q_des,   p.q)  - 10*p.qd + p.gravity_comp;
        #p.tau_ffwd  = np.zeros(12);       
								
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

	   # log variables
        p.logData()    
        
        # plot actual (green) and desired (blue) contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[:,leg], p.u.getLegJointState(leg, p.grForcesW/400),"green")        
        p.ros_pub.publishVisual()      				

        #wait for synconization of the control loop
        rate.sleep()     
        p.time = p.time + dt 								
	   # stops the while loop if  you prematurely hit CTRL+C                    
        if ros.is_shutdown():
            print ("Shutting Down")                    
            break;                                                
                             
    # restore PD when finished        
    p.pid.setPDs(400.0, 6.0, 0.0) 
    ros.sleep(1.0)                
    print ("Shutting Down")                 
    ros.signal_shutdown("killed")           
    p.deregister_node()   
    
if __name__ == '__main__':

    p = BaseController()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
        