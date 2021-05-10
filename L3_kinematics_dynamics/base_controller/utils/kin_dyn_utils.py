# -*- coding: utf-8 -*-
"""
Created on May 4 2021

@author: ovillarreal
"""

import numpy as np
import os
import math

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
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

# Exercise 2.1
# direct kinematics function
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

    # type your computation of the direct kinematics here

    # global homogeneous transformation matrices

    T_01 = np.array([[],[],[],[]])
    T_02 = np.array([[],[],[],[]])
    T_03 = np.array([[],[],[],[]])
    T_04 = np.array([[],[],[],[]])
    T_0e = np.array([[],[],[],[]])

    return T_01, T_02, T_03, T_04, T_0e 


# Exercise 2.2
# Derive geometric Jacobian
def computeEndEffectorJacobian(q):

    # type your computation of the geometric Jacobian here

    # z vectors for rotations
    z1 = np.array([])
    z2 = np.array([])
    z3 = np.array([])
    z4 = np.array([])

    # Jacobian matrix
    J = np.array([])

    return J,z1,z2,z3,z4

# Exercise 2.3
# deriving analytical Jacobian
def geometric2analyticJacobian(J,T_0e):
    
    # type your computation of the analytic Jacobian here
    
    J_r = np.array([])

    return J_r 

# Exercise 2.4
# computing the numerical inverse kinmatics
def numericalInverseKinematics(p,q0):

    # type your numerical inverse kinematics computations here
        
    # unwrapping function prevents from outputs larger than 2pi
    return q0


# Exercise 2.5
# define a trajectory for the joints to follow
def fifthOrderPolynomialTrajectory(tf,q0,qf):

    # Type your function to find the coefficients of the 5th order polynomial here

    polyCoeff = np.array([])

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
        

    



