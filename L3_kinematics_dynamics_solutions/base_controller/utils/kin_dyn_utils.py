# -*- coding: utf-8 -*-
"""
Created on May 4 2021

@author: ovillarreal
"""

import numpy as np
import os
import math
import pinocchio as pin
from pinocchio.utils import *

def setRobotParameters():

    # link lengths
    l1 = 0.089159
    l2 = 0.13585
    l3 = 0.425
    l4 = 0.1197
    l5 = 0.39225
    l6 = 0.094
    l7 = 0.068  
    lengths = np.array([l1, l2, l3, l4, l5, l6, l7])

    m0 = 4
    m1 = 3.7
    m2 = 8.393
    m3 = 2.275
    m4 = 1.219

    link_masses = np.array([m0, m1, m2, m3, m4])
    
    # com of the links expressed in the respective link frame
    # com_link =  model.inertias[idx].lever		
    com0 = np.array([0., 0., 0.]) # base link
    com1 = np.array([0., 0., 0.]) #shoulder link
    com2 = np.array([0.  , 0.  , 0.28]) #upper_arm_link
    com3 = np.array([0.  , 0.  , 0.25]) #forearm_link
    com4 = np.array([0., 0., 0.]) #wrist_1_link
    #w_com_link = data.oMi[idx].rotation.dot(com_link) + data.oMi[idx].translation

    # inertia tensors of the links  w.r.t. to own CoM of each link expressed in the respective link frames
    I_0 = np.array([[0.00443333156,           0.0,    0.0],
                    [          0.0, 0.00443333156,    0.0],
                    [          0.0,           0.0, 0.0072]])

    I_1 = np.array([[0.010267495893,            0.0,     0.0],
                    [           0.0, 0.010267495893,     0.0],
                    [           0.0,            0.0, 0.00666]])

    I_2 = np.array([[0.22689067591,            0.0,       0.0],
                    [           0.0, 0.22689067591,       0.0],
                    [           0.0,           0.0, 0.0151074]])
    
    I_3 = np.array([[0.049443313556,            0.0,     0.0],
                    [           0.0, 0.049443313556,     0.0],
                    [           0.0,           0.0, 0.004095]])

    I_4 = np.array([[0.111172755531,            0.0,     0.0],
                    [           0.0, 0.111172755531,     0.0],
                    [           0.0,            0.0, 0.21942]])

    inertia_tensors = np.array([I_0, I_1, I_2, I_3, I_4])

    coms = np.array([com0, com1, com2, com3, com4])

    return lengths, inertia_tensors, link_masses, coms


def directKinematics(q):

    # define link lengths from urdf
    link_length,_,_,_ = setRobotParameters()
    l1 = link_length[0]
    l2 = link_length[1]
    l3 = link_length[2]
    l4 = link_length[3]
    l5 = link_length[4]
    l6 = link_length[5]
    l7 = link_length[6]

    # local homogeneous transformation matrices
    T_01 = np.array([[math.cos(q[0]), -math.sin(q[0]), 0,  0],
                     [math.sin(q[0]),  math.cos(q[0]), 0,  0],
                     [             0,               0, 1, l1],
                     [             0,               0, 0,  1]])
    
    T_12_ = np.array([[ 0, 0, 1,  0],
                      [ 0, 1, 0, l2],
                      [-1, 0, 0,  0],
                      [ 0, 0, 0,  1]])

    T_12 = np.array([[ math.cos(q[1]), 0, math.sin(q[1]), 0],
                     [              0, 1,              0, 0],
                     [-math.sin(q[1]), 0, math.cos(q[1]), 0],
                     [              0, 0,              0, 1]])

    T_23 = np.array([[ math.cos(q[2]), 0, math.sin(q[2]),   0],
                     [              0, 1,              0, -l4],
                     [-math.sin(q[2]), 0, math.cos(q[2]),  l3],
                     [              0, 0,              0,   1]])


    T_34_ = np.array([[ 0, 0, 1,  0],
                      [ 0, 1, 0,  0],
                      [-1, 0, 0, l5],
                      [ 0, 0, 0,  1]])

    T_34 = np.array([[ math.cos(q[3]), 0, math.sin(q[3]), 0],
                     [              0, 1,              0, 0],
                     [-math.sin(q[3]), 0, math.cos(q[3]), 0],
                     [              0, 0,              0, 1]])

    T_4e = np.array([[0, -1, 0,  0],
                     [1,  0, 0, l6],
                     [0,  0, 1, l7],
                     [0,  0, 0,  1]])

    # global homogeneous transformation matrices
    T_02_ = T_01.dot(T_12_) # rigid transform
    T_02 = T_02_.dot(T_12) # joint 2 transform
    T_03 = T_02.dot(T_23)  # joint 3 transform
    T_04_ = T_03.dot(T_34_) # rigid transform
    T_04 = T_04_.dot(T_34) # joint 4 transform
    T_0e = T_04.dot(T_4e) # rigid transform

    return T_01, T_02, T_03, T_04, T_0e 

def computeEndEffectorJacobian(q):

    # compute direct kinematics 
    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    # rigid 90 degrees rotation to have z axis corresponding to axis of rotation
    R_90 = np.array([[1,  0, 0],
                     [0,  0, 1],
                     [0, -1, 0]])

    # rotation matrix of z2 to w.r.t. the world frame 
    R_z2 = T_01[:3,:3].dot(R_90)

    # z vectors for rotations
    z1 = np.array([0, 0, 1])
    z2 = R_z2.dot(z1)
    z3 = z2
    z4 = z2

    # link position vectors
    p_01 = T_01[:3,3]
    p_02 = T_02[:3,3]
    p_03 = T_03[:3,3]
    p_04 = T_04[:3,3]
    p_0e = T_0e[:3,3]

    # vectors from link i to end-effector
    p_0_1e = p_0e - p_01
    p_0_2e = p_0e - p_02
    p_0_3e = p_0e - p_03
    p_0_4e = p_0e - p_04

    # linear and angular part of Jacobian matrix
    J_p = np.array([np.cross(z1,p_0_1e), np.cross(z2,p_0_2e), np.cross(z3,p_0_3e), np.cross(z4,p_0_4e)])
    J_o = np.array([z1, z2, z3, z4])

    # Jacobian matrix and joint axes both expressed in the world frame) 
    J = np.vstack((np.transpose(J_p),np.transpose(J_o)))

    return J,z1,z2,z3,z4

def rot2eul(R):
    phi = np.arctan2(R[1,0], R[0,0])
    theta = np.arctan2(-R[2,0], np.sqrt(pow(R[2,1],2) + pow(R[2,2],2) ))
    psi = np.arctan2(R[2,1], R[2,2])
   
    #unit test should return roll = 0.5 pitch = 0.2  yaw = 0.3
    # rot2eul(np.array([ [0.9363,   -0.1684,    0.3082], [0.2896 ,   0.8665  , -0.4065], [-0.1987 ,   0.4699  ,  0.8601]]))    
    
    # returns roll = psi, pitch = theta,  yaw = phi
    return np.array((psi, theta, phi))

def geometric2analyticJacobian(J,T_0e):
    R = T_0e[:3,:3]
    rpy = rot2eul(R)
    # compute the mapping between euler rates and angular velocity
    T = np.array([[math.cos(rpy[1])*math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                  [math.cos(rpy[1])*math.sin(rpy[2]),  math.cos(rpy[2]), 0],
                  [                 -math.sin(rpy[1]),                0, 1]])

    T_a = np.array([np.vstack((np.hstack((np.identity(3), np.zeros((3,3)))),
                                          np.hstack((np.zeros((3,3)),T))))])
    

    J_r = np.dot(np.linalg.inv(T_a), J)

    return J_r[0]

def numericalInverseKinematics(p,q0):

    epsilon = 0.000001
    alpha = 0.1
    gamma = 0.1
    e_bar1 = 1
    e_bar = 1
    lambda_ = 0.0000001
    beta = 0.5
    max_iter = 10000
    iter = 0
    q_init = q0

    while np.linalg.norm(e_bar) >= epsilon and iter < max_iter:
        
        J,_,_,_,_ = computeEndEffectorJacobian(q0)
        _, _, _, _, T_0e = directKinematics(q0)

        p_e = T_0e[:3,3]
        R = T_0e[:3,:3]
        rpy = rot2eul(R)
        roll = rpy[0]
        p_e = np.append(p_e,roll)
        e_bar = p_e - p

        J_bar = geometric2analyticJacobian(J,T_0e)
        J_bar = J_bar[:4,:]
        JtJ= np.dot(J_bar.T,J_bar) + np.identity(J_bar.shape[1])*lambda_
        JtJ_inv = np.linalg.inv(JtJ)
        P = JtJ_inv.dot(J_bar.T)
        dq = -P.dot(e_bar)
        q1 = q0 + dq*alpha
        q0 = q1
        iter += 1

    # while np.linalg.norm(e_bar) >= epsilon and iter < max_iter:
        
    #     J,_,_,_,_ = computeEndEffectorJacobian(q0)
    #     _, _, _, _, T_0e = directKinematics(q0)

    #     p_e = T_0e[:3,3]
    #     R = T_0e[:3,:3]
    #     rpy = rot2eul(R)
    #     roll = rpy[0]
    #     p_e = np.append(p_e,roll)
    #     e_bar = p_e - p

    #     J_bar = geometric2analyticJacobian(J,T_0e)
    #     J_bar = J_bar[:4,:]
    #     JtJ= np.dot(J_bar.T,J_bar) + np.identity(J_bar.shape[1])*lambda_
    #     JtJ_inv = np.linalg.inv(JtJ)
    #     P = JtJ_inv.dot(J_bar.T)
    #     dq = -P.dot(e_bar)
    #     q1 = q0 + dq*alpha

    #     _, _, _, _, T_0e1 = directKinematics(q1)
    #     p_e1 = T_0e1[:3,3]
    #     R1 = T_0e1[:3,:3]
    #     rpy1 = rot2eul(R1)
    #     roll1 = rpy1[0]
    #     p_e1 = np.append(p_e1,roll1)
    #     e_bar1 = p_e1 - p_e

    #     e_bar_check = -np.linalg.norm(e_bar) + np.linalg.norm(e_bar1)
    #     threshold = gamma*alpha*np.linalg.norm(e_bar)

    #     if e_bar_check >= threshold:
    #         alpha = beta*alpha
    #         print alpha

    #     q0 = q1
    #     iter += 1

    if iter >= max_iter:
        print("Maximum number of iterations reached no solution was found")
        q0 = q1
    else:
        print("Inverse kinematics solved in {} iterations".format(iter))
        
    # unwrapping prevents from outputs larger than 2pi
    for i in range(len(q0)):
        while q0[i] >= 2*math.pi:
            q0[i] -= 2*math.pi 
        while q0[i] < -2*math.pi:
            q0[i] += 2*math.pi 

    return q0




def fifthOrderPolynomialTrajectory(tf,q0,qf):

    # Matrix used to solve the linear system of equations for the polynomial trajectory
    polyMatrix = np.array([[1,  0,              0,               0,                  0,                0],
                           [1, tf, np.power(tf,2),  np.power(tf,3),     np.power(tf,4),   np.power(tf,5)],
                           [0,  1,              0,               0,                  0,                0],
                           [0,  1,           2*tf, 3*np.power(tf,2),  4*np.power(tf,3), 5*np.power(tf,4)],
                           [0,  0,              2,                0,                 0,                0],
                           [0,  0,              2,             6*tf, 12*np.power(tf,2), 20*np.power(tf,3)]])
    
    polyVector = np.array([q0, qf, 0, 0, 0, 0])
    matrix_inv = np.linalg.inv(polyMatrix)
    polyCoeff = matrix_inv.dot(polyVector)

    return polyCoeff
    
def RNEA(g0,q,qd,qdd, Fee = np.zeros((1,3)), Mee = np.zeros((1,3))):

    # setting values of inertia tensors w.r.t. to their CoMs from urdf and link masses
    _, tensors, m, coms = setRobotParameters()

    # get inertia tensors about the com expressed in the respective link frame    
    c_I_0 = tensors[0]    
    c_I_1 = tensors[1]
    c_I_2 = tensors[2]
    c_I_3 = tensors[3]
    c_I_4 = tensors[4]
    
    # get positions of the link com expressed in the respective link frame    
    c_com_0 = coms[0]    
    c_com_1 = coms[1]    
    c_com_2 = coms[2]    
    c_com_3 = coms[3]    
    c_com_4 = coms[4]

    # initializing variables
    n = len(q)
    
    #pre-pend a fake joint for base link
    q_link = np.insert(q, 0, 0.0, axis=0)
    qd_link = np.insert(qd, 0, 0.0, axis=0)
    qdd_link = np.insert(qdd, 0, 0.0, axis=0)
        
    
    zeroV = np.zeros((1,3))
    omega = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    v = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    omega_dot = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0],zeroV[0]])
    a = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    vc = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    ac = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0],zeroV[0]])

    F = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0], Fee[0]])
    M = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0], Mee[0]])

    tau = np.array([0.0, 0.0, 0.0, 0.0])

    # obtaining joint axes vectors required in the computation of the velocities and accelerations (expressed in the world frame)
    _,z1,z2,z3,z4 = computeEndEffectorJacobian(q)
    
    
    z = np.array([np.zeros((1,3)), z1,z2,z3,z4])
   
    # global homogeneous transformation matrices
    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    # link positions w.r.t. the world
    p_00 = np.array([0.0,0.0,0.0])
    p_01 = T_01[:3,3]
    p_02 = T_02[:3,3]
    p_03 = T_03[:3,3]
    p_04 = T_04[:3,3]
    p_0e = T_0e[:3,3]
     
    # rotation matrices w.r.t. to the world of each link
    R_00 = np.eye(3)    
    R_01 = T_01[:3,:3]
    R_02 = T_02[:3,:3]
    R_03 = T_03[:3,:3]
    R_04 = T_04[:3,:3]

    # positions of the CoMs w.r.t. to the world frame
    pc_0 = p_00 + c_com_0
    pc_1 = p_01 + np.dot(R_01, c_com_1)
    pc_2 = p_02 + np.dot(R_02, c_com_2)
    pc_3 = p_03 + np.dot(R_03, c_com_3)
    pc_4 = p_04 + np.dot(R_04, c_com_4) 
    pc_e = p_0e

    # array used in the recursion
    p = np.array([p_00, p_01, p_02, p_03, p_04, p_0e])
    pc = np.array([pc_0, pc_1, pc_2, pc_3, pc_4, pc_e])

    # expressing tensors of inertia of the links (about the com) in the world frame (time consuming)
    I_0 = np.dot(np.dot(R_00,c_I_0),R_00.T)    
    I_1 = np.dot(np.dot(R_01,c_I_1),R_01.T)
    I_2 = np.dot(np.dot(R_02,c_I_2),R_02.T)
    I_3 = np.dot(np.dot(R_03,c_I_3),R_03.T)
    I_4 = np.dot(np.dot(R_04,c_I_4),R_04.T)
    I = np.array([I_0, I_1, I_2, I_3, I_4])

    # forward pass: compute accelerations from 0 to ee
    for i in range(n+1):
        if i == 0: # we start from base link 0
            p_ = p[0]            
            #base frame is still (not true for a legged robot!)
            omega[0] = 0
            v[0] = 0 
            omega_dot[0] = 0
            a[0] = -g0 # acceleration of the base is just gravity so we remove it from Netwon equations
        else:
            p_ = p[i] - p[i-1] #p_i-1,i
            omega[i] = omega[i-1] + qd_link[i]*z[i]
            omega_dot[i] = omega_dot[i-1] + qdd_link[i]*z[i] + qd_link[i]*np.cross(omega[i-1],z[i])

            v[i] = v[i-1] + np.cross(omega[i-1],p_)
            a[i] = a[i-1] + np.cross(omega_dot[i-1],p_) + np.cross(omega[i-1],np.cross(omega[i-1],p_))

        pc_ = pc[i] - p[i] # p_i,c
        
        #compute com quantities
        vc[i] = v[i] + np.cross(omega[i],p_)
        ac[i] = a[i] + np.cross(omega_dot[i],pc_) + np.cross(omega[i],np.cross(omega[i],pc_))

    
    # backward pass: compute forces and moments from ee to 1 TODO CHECK FROM HERE
    for i in range(n,-1,-1):   

        pc_ = p[i]- pc[i]  
        pc_1 = p[i+1] - pc[i] 
        
        F[i] = F[i+1] + m[i]*(ac[i])
        
        M[i] = M[i+1] - \
               np.cross(pc_,F[i]) + \
               np.cross(pc_1,F[i+1]) + \
               np.dot(I[i],omega_dot[i]) + \
               np.cross(omega[i],np.dot(I[i],omega[i]))  

    # compute torque for all joints (revolute) by projection
    for i in range(n):
        tau[i] = np.dot(z[i+1],M[i+1]) 

    return tau

# computation of gravity terms
def getg(q,robot):
    qd = np.array([0.0, 0.0, 0.0, 0.0])
    qdd = np.array([0.0, 0.0, 0.0, 0.0])
    # Pinocchio
    # g = pin.rnea(robot.model, robot.data, q,qd ,qdd)
    g = RNEA(np.array([0.0, 0.0, -9.81]),q,qd,qdd)   
    return g


# computation of generalized mass matrix
def getM(q,robot):
    n = len(q)
    M = np.zeros((n,n))
    for i in range(n):
        ei = np.array([0.0, 0.0, 0.0, 0.0])
        ei[i] = 1
        # Pinocchio
        #g = getg(q,robot)
        # tau_p = pin.rnea(robot.model, robot.data, q, np.array([0,0,0,0]),ei) -g      
        tau = RNEA(np.array([0.0, 0.0, 0.0]), q, np.array([0.0, 0.0, 0.0, 0.0]),ei)
        # fill in the column of the inertia matrix
        M[:4,i] = tau        
        
    return M

def getC(q,qd,robot):   
    qdd = np.array([0.0, 0.0, 0.0, 0.0])
    # Pinocchio
    # g = getg(q,robot)
    # C = pin.rnea(robot.model, robot.data,q,qd,qdd) - g    
    C = RNEA(np.array([0.0, 0.0, 0.0]), q, qd, np.array([0.0, 0.0, 0.0, 0.0]))
    return C      

    



