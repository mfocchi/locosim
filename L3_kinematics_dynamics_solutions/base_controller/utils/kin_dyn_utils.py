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
    link_lengths = np.array([l1, l2, l3, l4, l5, l6, l7])

    m0 = 4
    m1 = 3.7
    m2 = 8.393
    m3 = 2.275
    m4 = 1.219

    link_masses = np.array([m0, m1, m2, m3, m4])

    # inertia tensors w.r.t. to own CoM of each link
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


    return link_lengths, inertia_tensors, link_masses


def directKinematics(q):

    # define link lengths from urdf
    links,_,_ = setRobotParameters()
    l1 = links[0]
    l2 = links[1]
    l3 = links[2]
    l4 = links[3]
    l5 = links[4]
    l6 = links[5]
    l7 = links[6]

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
    T_02_ = T_01.dot(T_12_)
    T_02 = T_02_.dot(T_12)
    T_03 = T_02.dot(T_23)
    T_04_ = T_03.dot(T_34_)
    T_04 = T_04_.dot(T_34)
    T_0e = T_04.dot(T_4e)

    return T_01, T_02, T_03, T_04, T_0e 

def computeEndEffectorJacobian(q):

    # compute direct kinematics 
    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    # rigid 90 degrees rotation to have z axis corresponding to axis of roation
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

    # Jacobian matrix
    J = np.vstack((np.transpose(J_p),np.transpose(J_o)))

    return J,z1,z2,z3,z4

def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

def geometric2analyticJacobian(J,T_0e):
    R = T_0e[:3,:3]
    rpy = rot2eul(R)
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
    
def RNEA(g0,q,qd,qdd):

    # setting values of inertia tensors w.r.t. to their CoMs from urdf and link masses
    _, tensors, m = setRobotParameters()
    I_1 = tensors[1]
    I_2 = tensors[2]
    I_3 = tensors[3]
    I_4 = tensors[4]

    # initializing variables
    n = len(q)
    zeroV = np.zeros((1,3))
    omega = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    v = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    omega_dot = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    a = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    ac = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])

    F = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    M = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0]])

    tau = np.array([0.0, 0.0, 0.0, 0.0])

    # obtaining z vectors required in the computation of the velocities and accelerations
    _,z1,z2,z3,z4 = computeEndEffectorJacobian(q)
    z = np.array([z1,z2,z3,z4])

    # global homogeneous transformation matrices
    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    # link positions w.r.t. the world
    p_01 = T_01[:3,3]
    p_02 = T_02[:3,3]
    p_03 = T_03[:3,3]
    p_04 = T_04[:3,3]
    p_0e = T_0e[:3,3]

    # rotation matrices w.r.t. to the world of each link
    R_01 = T_01[:3,:3]
    R_02 = T_02[:3,:3]
    R_03 = T_03[:3,:3]
    R_04 = T_04[:3,:3]

    # positions of the CoMs w.r.t. to the world
    pc_1 = p_01
    pc_2 = p_02 + np.dot(R_02, np.array([0.0, 0.0, 0.28]))
    pc_3 = p_03 + np.dot(R_03, np.array([0.0, 0.0, 0.25]))
    pc_4 = p_04
    pc_e = p_0e

    # array used in the recursion
    p = np.array([p_01, p_02, p_03, p_04, p_0e])
    pc = np.array([pc_1, pc_2, pc_3, pc_4, pc_e])

    # gravity vector acting on link 1 expressed in frame 1
    g = np.dot(R_01,np.array([0.0, 0.0, -g0]))
    
    # expressing tensors of inertia w.r.t. world frame (time consuming)
    I_1 = np.dot(np.dot(R_01,I_1),R_01.T)
    I_2 = np.dot(np.dot(R_02,I_2),R_02.T)
    I_3 = np.dot(np.dot(R_03,I_3),R_03.T)
    I_4 = np.dot(np.dot(R_04,I_4),R_04.T)
    I = np.array([I_1, I_2, I_3, I_4])

    # forward pass: compute accelerations from 0 to ee
    for i in range(n):

        if i == 0:
            p_ = p[i]

            omega[i] = qd[i]*z[i]
            v[i] = 0

            omega_dot[i] = qdd[i]*z[i] + qd[i]*np.cross(omega[i],z[i])
            a[i] = -g
            
        else:
            p_ = p[i] - p[i-1]

            omega[i] = omega[i-1] + qd[i]*z[i]
            omega_dot[i] = omega_dot[i-1] + qdd[i]*z[i] + qd[i]*np.cross(omega[i],z[i])

            v[i] = v[i-1] + np.cross(omega[i-1],p_)
            a[i] = a[i-1] + np.cross(omega_dot[i-1],p_) + np.cross(omega[i-1],np.cross(omega[i-1],p_))

        pc_ = pc[i] - p[i]
        ac[i] = a[i] + np.cross(omega_dot[i],pc_) + np.cross(omega[i],np.cross(omega[i],pc_))

    # backward pass: compute forces and moments from ee to 1
    for i in range(n-1,-1,-1):
        if i == 0:
            p_ = p[i]            
        else:
            p_ = p[i] - p[i-1]
        
        pc_ = pc[i] - p[i]
        pc_1 = pc[i] - p[i+1]

        F[i] = F[i+1] + m[i]*(ac[i])
        
        M[i] = M[i+1] - \
               np.cross(F[i],pc_) + \
               np.cross(F[i+1],pc_1) + \
               np.dot(I[i],omega_dot[i]) + \
               np.cross(omega[i],np.dot(I[i],omega[i]))  

    # compute torque for all joints (revolute)
    for i in range(n):
        tau[i] = np.dot(M[i].T,z[i]) 

    return tau

# computation of gravity terms
def getg(q,robot):
    qd = np.array([0.0, 0.0, 0.0, 0.0])
    qdd = np.array([0.0, 0.0, 0.0, 0.0])
    g = pin.rnea(robot.model, robot.data, q,qd ,qdd)
    return g


# computation of generalized mass matrix
def getM(q,robot):
    g = getg(q,robot)
    n = len(q)
    M = np.zeros((n,n))
    for i in range(n):
        ei = np.array([0.0, 0.0, 0.0, 0.0])
        ei[i] = 1
        taup = pin.rnea(robot.model, robot.data, q,np.array([0,0,0,0]),ei)
        M[:4,i] = taup - g
    return M

def getC(q,qd,robot):
    g = getg(q,robot)
    qdd = np.array([0.0, 0.0, 0.0, 0.0])
    C = pin.rnea(robot.model, robot.data,q,qd,qdd) - g
    return C      

    



