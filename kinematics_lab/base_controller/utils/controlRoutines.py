# Description
# File contains some necessary control algorithms for HyQ
# Author: Niraj Rathod
# Date: 19-11-2019

# Standard packages
import scipy.io
import scipy.sparse as sparse
import numpy as np
import yaml
from math_tools import *
# User defined packages
from math_tools import Math
from utils import Utils

from optimTools import quadprog_solve_qp
from scipy.linalg import block_diag

def computeVirtualImpedanceWrench(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, W_base_to_com, isCoMControlled, GravityComp, ffwdOn):
    util = Utils()
    # The inertia matrix (for simplicity we will use the same for com and base frame)
    B_Inertia = np.array([     [conf.robotInertia.Ixx, conf.robotInertia.Ixy, conf.robotInertia.Ixz],
                             [conf.robotInertia.Ixy, conf.robotInertia.Iyy,  conf.robotInertia.Iyz],
                             [conf.robotInertia.Ixz,  conf.robotInertia.Iyz, conf.robotInertia.Izz]])
    # Load math functions 
    mathJet = Math()

    # Feedback wrench (Virtual PD)
    Kp_lin = np.diag([conf.Kp_lin_x, conf.Kp_lin_y, conf.Kp_lin_z])
    Kd_lin = np.diag([conf.Kd_lin_x, conf.Kd_lin_y, conf.Kd_lin_z])
    Kp_ang = np.diag([conf.KpRoll, conf.KpPitch, conf.KpYaw])
    Kd_ang = np.diag([conf.KdRoll, conf.KdPitch, conf.KdYaw])

    Wfbk = np.zeros(6)
                
    # linear part                
    Wfbk[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = Kp_lin.dot(util.linPart(des_pose) - util.linPart(act_pose)) + Kd_lin.dot(util.linPart(des_twist) - util.linPart(act_twist))
    # angular part                
    # actual orientation 
    b_R_w = mathJet.rpyToRot(util.angPart(act_pose))
                
    # Desired Orientation
    Rdes = mathJet.rpyToRot(util.angPart(des_pose))
    # compute orientation error
    Re = Rdes.dot(b_R_w.transpose())
    # express orientation error in angle-axis form                 
    err = rotMatToRotVec(Re)                  
                
    #the orient error is expressed in the base_frame so it should be rotated wo have the wrench in the world frame
    w_err = b_R_w.transpose().dot(err)        
    # map des euler tates into des omega
    Jomega =  mathJet.Jomega(util.angPart(act_pose))
   
    # Note we defined the angular part of the des twist as euler rates not as omega so we need to map them to an Euclidean space with Jomega                
    Wfbk[util.sp_crd["AX"]:util.sp_crd["AX"] + 3] = Kp_ang.dot(w_err) + Kd_ang.dot(Jomega.dot((util.angPart(des_twist) - util.angPart(act_twist))))


    #EXERCISE 3: Compute graviy wrench
    Wg = np.zeros(6)   
    if (GravityComp):         
        mg = conf.robotMass * np.array([0, 0, conf.gravity])
        Wg[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = mg
        #in case you are closing the loop on base frame
        if (not isCoMControlled):                 
            Wg[util.sp_crd["AX"]:util.sp_crd["AX"] + 3] = np.cross(W_base_to_com, mg)
                    
    # EXERCISE 4: Feed-forward wrench
    Wffwd = np.zeros(6)                                                                
    if (ffwdOn):    
        ffdLinear = conf.robotMass * util.linPart(des_acc) 
       # compute inertia in the WF  w_I = R' * B_I * R
        W_Inertia = np.dot(b_R_w.transpose(), np.dot(B_Inertia, b_R_w))
        # compute w_des_omega_dot  Jomega*des euler_rates_dot + Jomega_dot*des euler_rates
        Jomega_dot =  mathJet.Jomega_dot(util.angPart(des_pose),  util.angPart(des_twist))
        w_des_omega_dot = Jomega.dot(util.angPart(des_acc)) + Jomega_dot.dot(util.angPart(des_twist))                
        ffdAngular = W_Inertia.dot(w_des_omega_dot) 
        #ffdAngular = W_Inertia.dot(util.angPart(des_acc))
        Wffwd = np.hstack([ffdLinear, ffdAngular])
        

    return  Wffwd, Wfbk, Wg           
                 
# Whole body controller for HyQ that includes ffd wrench + fb Wrench (Virtual PD) + gravity compensation
# all vector is in the wf
def projectionBasedController(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, W_base_to_com, isCoMControlled, GravityComp, ffwdOn ):
    util = Utils()
                       
    # EXERCISE 2.1: compute virtual impedances  
    Wffwd, Wfbk, Wg = computeVirtualImpedanceWrench(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, W_base_to_com, isCoMControlled, GravityComp, ffwdOn)                
    # Total Wrench (FFwd + Feedback + Gravity)
    TotWrench =  Wffwd + Wfbk + Wg                     
                
    # EXERCISE 2.2: Compute mapping to grfs
    # Stance matrix
    S_mat = np.diag(np.hstack([stance_legs[util.leg_map["LF"]] * np.ones(3), stance_legs[util.leg_map["RF"]] * np.ones(3),
                              stance_legs[util.leg_map["LH"]] * np.ones(3), stance_legs[util.leg_map["RH"]] * np.ones(3)]))
    # This is a skew symmetric matrix for (xfi-xc)  corressponding  toe difference between the foothold locations
    # and COM trajectories)
    d1 = cross_mx(W_contacts[:,util.leg_map["LF"]] - util.linPart(act_pose))
    d2 = cross_mx(W_contacts[:,util.leg_map["RF"]] - util.linPart(act_pose))
    d3 = cross_mx(W_contacts[:,util.leg_map["LH"]] - util.linPart(act_pose))
    d4 = cross_mx(W_contacts[:,util.leg_map["RH"]] - util.linPart(act_pose))
    # Compute Jb^T
    JbT = np.vstack([np.hstack([np.eye(3), np.eye(3), np.eye(3), np.eye(3)]),
                        np.hstack([d1, d2, d3, d4])])
    #nullify columns relative to legs that are not in contact                                                                                                #
    JbT = JbT.dot(S_mat)

    # Map the total Wrench to grf
    w_des_grf = np.linalg.pinv(JbT, 1e-04).dot(TotWrench)
                
                                                                
    return w_des_grf, Wffwd, Wfbk, Wg

# Whole body controller for HyQ that includes ffd wrench + fb Wrench (Virtual PD) + gravity compensation
#every vector is in the wf                
def QPController(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, W_base_to_com, isCoMControlled, GravityComp, ffwdOn, frictionCones, normals, f_min, mu):
    util = Utils()
                
    # compute virtual impedances                
    Wffwd, Wfbk, Wg = computeVirtualImpedanceWrench(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, W_base_to_com, isCoMControlled, GravityComp, ffwdOn )
    # Total Wrench
    TotWrench =    Wfbk+ Wffwd+ Wg   
          
    
    #(Ax-b)T*(Ax-b)
    # G = At*A
    # g = -At*b
    #0.5 xT*G*x + gT*x
    #s.t. Cx<=d
        
    # compute cost function
    # Stance matrix
    S_mat = np.diag(np.hstack([stance_legs[util.leg_map["LF"]] * np.ones(3), stance_legs[util.leg_map["RF"]] * np.ones(3),
                              stance_legs[util.leg_map["LH"]] * np.ones(3), stance_legs[util.leg_map["RH"]] * np.ones(3)]))
    # This is a skew symmetric matrix for (xfi-xc)  corressponding  toe difference between the foothold locations
    # and COM trajectories)
    d1 = cross_mx(W_contacts[:,util.leg_map["LF"]] - util.linPart(act_pose))
    d2 = cross_mx(W_contacts[:,util.leg_map["RF"]] - util.linPart(act_pose))
    d3 = cross_mx(W_contacts[:,util.leg_map["LH"]] - util.linPart(act_pose))
    d4 = cross_mx(W_contacts[:,util.leg_map["RH"]] - util.linPart(act_pose))
    # Compute Jb^T
    JbT = np.vstack([np.hstack([np.eye(3), np.eye(3), np.eye(3), np.eye(3)]),
                        np.hstack([d1, d2, d3, d4])])
    #nullify columns relative to legs that are not in contact                                                                                                #
    JbT = JbT.dot(S_mat)
    
    W =  np.eye(12) * 1e-4           
    G = JbT.T.dot(JbT) + np.eye(12) * 1e-4  #regularize and make it definite positive
    g = -JbT.T.dot(TotWrench)                 
    
    # compute unilateral constraints Cx >= f_min => -Cx <= -f_min
    C =  - block_diag( normals[util.leg_map["LF"]] , 
                                         normals[util.leg_map["RF"]], 
                                         normals[util.leg_map["LH"]], 
                                         normals[util.leg_map["RH"]]    )                                                 
    #not need to nullify columns relative to legs that are not in contact because the QP solver removes 0 = 0 constraints                                                                                           #
    d = -f_min.reshape((4,))
                
    # EXERCISE 11: compute friction cones inequalities A_f x <= 0
    if (frictionCones):                                                                
        C_leg = [None]*4            
        for leg in range(4):
            #compute tangential components
            ty = np.cross(normals[leg], np.array([1,0,0]))
            tx = np.cross(ty, normals[leg])
                                            
            C_leg[leg] = np.array([
                tx  - mu[leg]*normals[leg] ,
                -tx - mu[leg]*normals[leg] ,
                ty  - mu[leg]*normals[leg] ,
                -ty - mu[leg]*normals[leg] ])
            
                       
        C =   block_diag( C_leg[util.leg_map["LF"]] , 
                       C_leg[util.leg_map["RF"]], 
                           C_leg[util.leg_map["LH"]], 
                           C_leg[util.leg_map["RH"]]    )   
        d = np.zeros(C.shape[0]) 
         
    w_des_grf = quadprog_solve_qp(G, g, C, d, None , None)   
 
    #compute constraint violations (take smallest distance from constraint)
    constr_viol = np.zeros(4)
    for leg in range(4): 
       if (frictionCones):
            constraintsPerLeg = 4  
       else:
          constraintsPerLeg = 1                                    
       # TODO check this										
       distance_to_violation = np.amin( - C [ constraintsPerLeg*leg : constraintsPerLeg*leg + constraintsPerLeg,:].dot(w_des_grf))  #d should be zeros so does not affect
       constr_viol[leg] = 1/(1.0 + distance_to_violation*distance_to_violation) #goes to 1 when constraints are violated                                
   
    return w_des_grf, Wffwd, Wfbk, Wg, constr_viol
