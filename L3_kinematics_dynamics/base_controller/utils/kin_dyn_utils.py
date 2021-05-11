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

# function to set parameters based on data from URDF
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

    m0 = 4
    m1 = 3.7
    m2 = 8.393
    m3 = 2.275
    m4 = 1.219

    link_masses = np.array([m0, m1, m2, m3, m4])

    return link_lengths, inertia_tensors, link_masses

# function to transform from rotation matrix to euler angles
def rot2eul(R):
    phi = np.arctan2(R[1, 0], R[0, 0])
    theta = np.arctan2(-R[2, 0], np.sqrt(pow(R[2, 1], 2) + pow(R[2, 2], 2)))
    psi = np.arctan2(R[2, 1], R[2, 2])
    # unit test should return roll = 0.5 pitch = 0.2  yaw = 0.3
    # rot2eul(np.array([ [0.9363,   -0.1684,    0.3082], [0.2896 ,   0.8665  , -0.4065], [-0.1987 ,   0.4699  ,  0.8601]]))

    # returns roll = psi, pitch = theta,  yaw = phi
    return np.array((psi, theta, phi))

# Exercise 2.1
# direct kinematics function
def directKinematics(q):

    # define link lengths from urdf
    links, _, _ = setRobotParameters()
    l1 = links[0]
    l2 = links[1]
    l3 = links[2]
    l4 = links[3]
    l5 = links[4]
    l6 = links[5]
    l7 = links[6]

    # type your computation of the direct kinematics here

    # local homogeneous transformation matrices
    T_01 = np.array([[math.cos(q[0]), -math.sin(q[0]), 0, 0],
                     [math.sin(q[0]), math.cos(q[0]), 0, 0],
                     [0, 0, 1, l1],
                     [0, 0, 0, 1]])

    T_12_ = np.array([[0, 0, 1, 0],
                      [0, 1, 0, l2],
                      [-1, 0, 0, 0],
                      [0, 0, 0, 1]])

    T_12 = np.array([[math.cos(q[1]), 0, math.sin(q[1]), 0],
                     [0, 1, 0, 0],
                     [-math.sin(q[1]), 0, math.cos(q[1]), 0],
                     [0, 0, 0, 1]])

    T_23 = np.array([[math.cos(q[2]), 0, math.sin(q[2]), 0],
                     [0, 1, 0, -l4],
                     [-math.sin(q[2]), 0, math.cos(q[2]), l3],
                     [0, 0, 0, 1]])

    T_34_ = np.array([[0, 0, 1, 0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, l5],
                      [0, 0, 0, 1]])

    T_34 = np.array([[math.cos(q[3]), 0, math.sin(q[3]), 0],
                     [0, 1, 0, 0],
                     [-math.sin(q[3]), 0, math.cos(q[3]), 0],
                     [0, 0, 0, 1]])

    T_4e = np.array([[0, -1, 0, 0],
                     [1, 0, 0, l6],
                     [0, 0, 1, l7],
                     [0, 0, 0, 1]])

    # global homogeneous transformation matrices

    # global homogeneous transformation matrices
    T_02_ = T_01.dot(T_12_)  # rigid transform
    T_02 = T_02_.dot(T_12)  # joint 2 transform
    T_03 = T_02.dot(T_23)  # joint 3 transform
    T_04_ = T_03.dot(T_34_)  # rigid transform
    T_04 = T_04_.dot(T_34)  # joint 4 transform
    T_0e = T_04.dot(T_4e)  # rigid transform

    return T_01, T_02, T_03, T_04, T_0e 


# Exercise 2.2
# Derive geometric Jacobian
def computeEndEffectorJacobian(q):

    # type your computation of the geometric Jacobian here

    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    # link position vectors
    p_01 = T_01[:3, 3]
    p_02 = T_02[:3, 3]
    p_03 = T_03[:3, 3]
    p_04 = T_04[:3, 3]
    p_0e = T_0e[:3, 3]

    # rigid 90 degrees rotation to have z axis corresponding to axis of rotation
    R_90 = np.array([[1, 0, 0],
                     [0, 0, 1],
                     [0, -1, 0]])

    # rotation matrix of z2 to w.r.t. the world frame
    R_z2 = T_01[:3, :3].dot(R_90)

    # z vectors for rotations
    z1 = np.array([0, 0, 1])
    z2 = R_z2.dot(z1)
    z3 = z2
    z4 = z2

    # vectors from link i to end-effector
    p_0_1e = p_0e - p_01
    p_0_2e = p_0e - p_02
    p_0_3e = p_0e - p_03
    p_0_4e = p_0e - p_04

    # linear and angular part of Jacobian matrix
    J_p = np.array([np.cross(z1, p_0_1e), np.cross(z2, p_0_2e), np.cross(z3, p_0_3e), np.cross(z4, p_0_4e)])
    J_o = np.array([z1, z2, z3, z4])

    # Jacobian matrix
    J = np.vstack((np.transpose(J_p), np.transpose(J_o)))

    return J,z1,z2,z3,z4

# Exercise 2.3
# deriving analytical Jacobian
def geometric2analyticJacobian(J,T_0e):
    R = T_0e[:3, :3]
    rpy = rot2eul(R)
    # compute the mapping between euler rates and angular velocity
    T = np.array([[math.cos(rpy[1]) * math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                  [math.cos(rpy[1]) * math.sin(rpy[2]), math.cos(rpy[2]), 0],
                  [-math.sin(rpy[1]), 0, 1]])

    T_a = np.array([np.vstack((np.hstack((np.identity(3), np.zeros((3, 3)))),
                               np.hstack((np.zeros((3, 3)), T))))])

    J_r = np.dot(np.linalg.inv(T_a), J)

    return J_r[0]

# Exercise 2.4
# computing the numerical inverse kinmatics
def numericalInverseKinematics(p, q0):

    # Error initialization
    e_bar = 1
    iter = 0

    # Recursion parameters
    epsilon = 0.000001  # Tolerance
    # alpha = 0.1
    alpha = 1  # Step size
    lambda_ = 0.0000001  # Damping coefficient for pseudo-inverse
    max_iter = 10000  # Maximum number of iterations

    # For line search only
    gamma = 0.5
    beta = 0.5

    # Inverse kinematics without line search
    while np.linalg.norm(e_bar) >= epsilon and iter < max_iter:
        J, _, _, _, _ = computeEndEffectorJacobian(q0)
        _, _, _, _, T_0e = directKinematics(q0)

        p_e = T_0e[:3, 3]
        R = T_0e[:3, :3]
        rpy = rot2eul(R)
        roll = rpy[0]
        p_e = np.append(p_e, roll)
        e_bar = p_e - p

        J_bar = geometric2analyticJacobian(J, T_0e)
        J_bar = J_bar[:4, :]
        JtJ = np.dot(J_bar.T, J_bar) + np.identity(J_bar.shape[1]) * lambda_
        JtJ_inv = np.linalg.inv(JtJ)
        P = JtJ_inv.dot(J_bar.T)
        dq = -P.dot(e_bar)
        q1 = q0 + dq * alpha
        q0 = q1
        iter += 1

    # Inverse kinematics with line search
    # while np.linalg.norm(e_bar) >= epsilon and iter < max_iter:
    #
    #     J,_,_,_,_ = computeEndEffectorJacobian(q0)
    #     _, _, _, _, T_0e = directKinematics(q0)
    #
    #     # Compute Newton step
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
    #
    #     # Update
    #     q1 = q0 + dq*alpha
    #
    #     #Compute error of next step
    #     _, _, _, _, T_0e1 = directKinematics(q1)
    #     p_e1 = T_0e1[:3,3]
    #     R1 = T_0e1[:3,:3]
    #     rpy1 = rot2eul(R1)
    #     roll1 = rpy1[0]
    #     p_e1 = np.append(p_e1,roll1)
    #     e_bar1 = p_e - p_e1
    #     e_bar_check = -np.linalg.norm(e_bar) + np.linalg.norm(e_bar1)
    #     threshold = gamma*alpha*np.linalg.norm(e_bar)
    #
    #     if e_bar_check >= threshold:
    #         alpha = beta*alpha
    #         print alpha
    #
    #     q0 = q1
    #     iter += 1

    if iter >= max_iter:
        print("Maximum number of iterations reached no solution was found, going to closest solution")
        q0 = q1
    else:
        print("Inverse kinematics solved in {} iterations".format(iter))

    # unwrapping prevents from outputs larger than 2pi
    for i in range(len(q0)):
        while q0[i] >= 2 * math.pi:
            q0[i] -= 2 * math.pi
        while q0[i] < -2 * math.pi:
            q0[i] += 2 * math.pi

    return q0


# Exercise 2.5
# define a trajectory for the joints to follow
def fifthOrderPolynomialTrajectory(tf,q0,qf):
    # Matrix used to solve the linear system of equations for the polynomial trajectory
    polyMatrix = np.array([[1, 0, 0, 0, 0, 0],
                           [1, tf, np.power(tf, 2), np.power(tf, 3), np.power(tf, 4), np.power(tf, 5)],
                           [0, 1, 0, 0, 0, 0],
                           [0, 1, 2 * tf, 3 * np.power(tf, 2), 4 * np.power(tf, 3), 5 * np.power(tf, 4)],
                           [0, 0, 2, 0, 0, 0],
                           [0, 0, 2, 6 * tf, 12 * np.power(tf, 2), 20 * np.power(tf, 3)]])

    polyVector = np.array([q0, qf, 0, 0, 0, 0])
    matrix_inv = np.linalg.inv(polyMatrix)
    polyCoeff = matrix_inv.dot(polyVector)

    return polyCoeff

# Exercise 3.1
# write the RNEA algorithm for th 4 DoF robot
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

    # type your RNEA function here

    return tau

# Exercise 3.2
# Compute the generalized mass matrix
def getM(q):
    n = len(q)
    M = np.zeros((n,n))

    # type your computation of the mass matrix here
    return M

# Exercise 3.3
# computation of gravity terms
def getg(g0,q):
    g = np.array([])
    # type your computation of the gravity terms here
    return g
        

    



