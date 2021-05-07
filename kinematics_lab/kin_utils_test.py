# -*- coding: utf-8 -*-
"""
Created on May 4 2021

@author: ovillarreal
"""

import numpy as np
import os
import math

def directKinematics(q):

    l1 = 0.089159
    l2 = 0.13585
    l3 = 0.425
    l4 = 0.1197
    l5 = 0.39225
    l6 = 0.094
    l7 = 0.068    

    T_01 = np.array([[math.cos(q[0]), -math.sin(q[0]), 0,  0],
                     [math.sin(q[0]),  math.cos(q[0]), 0,  0],
                     [             0,               0, 1, l1],
                     [             0,               0, 0,  1]])
    
    T_12_ = np.array([[                    0, 0, math.sin(1.570796325),  0],
                     [                     0, 1,                     0, l2],
                     [-math.sin(1.570796325), 0,                     0,  0],
                     [                     0, 0,                     0,  1]])

    T_12 = np.array([[ math.cos(q[1]), 0, math.sin(q[1]), 0],
                     [              0, 1,              0, 0],
                     [-math.sin(q[1]), 0, math.cos(q[1]), 0],
                     [              0, 0,              0, 1]])

    T_23 = np.array([[ math.cos(q[2]), 0, math.sin(q[2]),   0],
                     [              0, 1,              0, -l4],
                     [-math.sin(q[2]), 0, math.cos(q[2]),  l3],
                     [              0, 0,              0,   1]])

    T_34_ = np.array([[                    0, 0, math.sin(1.570796325),  0],
                     [                     0, 1,                     0,  0],
                     [-math.sin(1.570796325), 0,                     0, l5],
                     [                     0, 0,                     0,  1]])

    T_34 = np.array([[ math.cos(q[3]), 0, math.sin(q[3]), 0],
                     [              0, 1,              0, 0],
                     [-math.sin(q[3]), 0, math.cos(q[3]), 0],
                     [              0, 0,              0, 1]])

    T_4e = np.array([[0, -1, 0,  0],
                     [1,  0, 0, l6],
                     [0,  0, 1, l7],
                     [0,  0, 0,  1]])

    T_02_ = T_01.dot(T_12_)
    T_02 = T_02_.dot(T_12)
    T_03 = T_02.dot(T_23)
    T_04_ = T_03.dot(T_34_)
    T_04 = T_04_.dot(T_34)
    T_0e = T_04.dot(T_4e)

    return T_01, T_02, T_03, T_04, T_0e 

def computeEndEffectorJacobian(q):

    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    R_01 = np.array([[ math.cos(q[0]), math.sin(q[0]), 0],
                     [-math.sin(q[0]), math.cos(q[0]), 0],
                     [              0,              0, 1]])

    R_12_ = np.array([[1, 0, 0],
                      [0, math.cos(-1.570796325), -math.sin(-1.570796325)],
                      [0, math.sin(-1.570796325),  math.cos(-1.570796325)]])
    
    R_12 = R_01.dot(R_12_)

    z1 = np.array([0, 0, 1])
    z2 = R_12.dot(z1)
    z3 = z2
    z4 = z2

    p_01 = T_01[:3,3]
    p_02 = T_02[:3,3]
    p_03 = T_03[:3,3]
    p_04 = T_04[:3,3]
    p_0e = T_0e[:3,3]

    p_0_1e = p_0e - p_01
    p_0_2e = p_0e - p_02
    p_0_3e = p_0e - p_03
    p_0_4e = p_0e - p_04

    J_p = np.array([np.cross(z1,p_0_1e), np.cross(z2,p_0_2e), np.cross(z3,p_0_3e), np.cross(z4,p_0_4e)])

    J_o = np.array([z1, z2, z3, z4])

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
    
    print(J)

    J_r = np.dot(np.linalg.inv(T_a), J)

    return J_r[0]

def numericalInverseKinematics(p,q0):

    epsilon = 0.000001
    alpha = 0.1
    gamma = 0.01
    T_01, T_02, T_03, T_04, T_0e = directKinematics(q0)
    R = T_0e[:3,:3]
    rpy = rot2eul(R)
    yaw = rpy[2]
    p_e = T_0e[:3,3]
    p_e = np.append(p_e,yaw)
    e_bar = p_e - p
    lambda_ = 0.0001
    # e1_bar = e_bar

    while np.linalg.norm(e_bar) >= epsilon:
        
        J,z1,z2,z3,z4 = computeEndEffectorJacobian(q0)
        
        T_01, T_02, T_03, T_04, T_0e = directKinematics(q0)
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

        # T1_01, T1_02, T1_03, T1_04, T1_0e = directKinematics(q0)
        # p1_e = T1_0e[:3,3]
        # e1_bar = p - p1_e

        q0 = q1

        print("e_bar norm:")
        print(np.linalg.norm(e_bar))
    
    for i in range(4):
        if q0[i] > 2*math.pi:
            while q0[i] > 2*math.pi:
                q0[i] = q0[i] - 2*math.pi
        if q0[i] <= -2*math.pi:
            while q0[i] <= -2*math.pi:
                q0[i] = q0[i] + 2*math.pi
        
        

    return q0


def fifthOrderPolynomialTrajectory(tf,q0,qf):

    polyMatrix = np.array([[1,  0,              0,               0,                  0,                0],
                           [1, tf, np.power(tf,2),  np.power(tf,3),     np.power(tf,4),   np.power(tf,5)],
                           [0,  1,              0,               0,                  0,                0],
                           [0,  1,           2*tf, 3*np.power(tf,2),  4*np.power(tf,3), 5*np.power(tf,4)],
                           [0,  0,              2,                0,                 0,                0],
                           [0,  0,              2,             6*tf, 12*np.power(tf,2), 20*np.power(tf,3)]])

    # print("polyMatrix:")
    # print(polyMatrix)

    
    polyVector = np.array([q0, qf, 0, 0, 0, 0])

    matrix_inv = np.linalg.inv(polyMatrix)

    # print("matrix_inv:")
    # print(matrix_inv)

    polyCoeff = matrix_inv.dot(polyVector)

    return polyCoeff
    
def RNEA(g0,q,qd,qdd):

    g = np.array([0.0, 0.0, g0])

    n = len(q)

    zeroV = np.zeros((1,3))

    omega = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    v = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])

    omega_dot = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    a = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    a[0] = a[0] - g
    ac = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0]])

    J,z1,z2,z3,z4 = computeEndEffectorJacobian(q)
    z = np.array([z1,z2,z3,z4])

    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    p_01 = T_01[:3,3]
    p_02 = T_02[:3,3]
    p_03 = T_03[:3,3]
    p_04 = T_04[:3,3]
    p_0e = T_0e[:3,3]

    pc_1 = p_01
    pc_2 = p_02 + np.array([0.28, 0.0, 0.0])
    pc_3 = p_03 + np.array([0.25, 0.0, 0.0])
    pc_4 = p_04
    pc_e = p_0e

    R_01 = T_01[:3,:3]
    R_02 = T_02[:3,:3]
    R_03 = T_03[:3,:3]
    R_04 = T_04[:3,:3]
    R_0e = T_0e[:3,:3]

    R = np.array([R_01.T, R_02.T, R_03.T, R_04.T])

    p = np.array([p_01, p_02, p_03, p_04, p_0e])
    pc = np.array([pc_1, pc_2, pc_3, pc_4, pc_e])


    # print("p:")
    # print(p)

    F = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0]])
    M = np.array([zeroV[0], zeroV[0], zeroV[0], zeroV[0], zeroV[0]])

    m0 = 4
    m1 = 3.7
    m2 = 8.393
    m3 = 2.275
    m4 = 1.219

    m = np.array([m1, m2, m3, m4])

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
    
    I_1 = np.dot(np.dot(R_01,I_1),R_01.T)
    I_2 = np.dot(np.dot(R_02,I_2),R_02.T)
    I_3 = np.dot(np.dot(R_03,I_3),R_03.T)
    I_4 = np.dot(np.dot(R_04,I_4),R_04.T)

    I_e = np.array([[0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0]])

    I = np.array([I_1, I_2, I_3, I_4,I_e])

    tau = np.array([0.0, 0.0, 0.0, 0.0])

    for i in range(n):

        if i == 0:
            p_ = p[i]

            omega[i] = qd[i]*z[i]
            v[i] = 0

            omega_dot[i] = qdd[i]*z[i] + qd[i]*np.cross(omega[i],z[i])
            a[i] = 0
            
        else:
            p_ = p[i] - p[i-1]

            omega[i] = omega[i-1] + qd[i]*z[i]
            v[i] = v[i-1] + np.cross(omega[i-1],p_)

            omega_dot[i] = omega_dot[i-1] + qdd[i]*z[i] + qd[i]*np.cross(omega[i-1],z[i])
            a[i] = a[i-1] + np.cross(omega_dot[i-1],p_) + np.cross(omega[i-1],np.cross(omega[i-1],p_))

        pc_ = pc[i] - p[i]
        ac[i] = a[i] + np.cross(omega_dot[i-1],pc_) + np.cross(omega[i-1],np.cross(omega[i-1],pc_))

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

    
    for i in range(n):
        tau[i] = np.dot(M[i].T,z[i])

   
    # print("omega")
    # print(omega)
    # print("v:")
    # print(v)
    # print("ac")
    # print(ac)
    # print("omega_dot")
    # print(omega_dot)
    # print("F:")
    # print(F)
    # print("M:")
    # print(M)
    # print("z")
    # print(z)

    # print("tau:")
    # print(tau)


    return tau

def getM(q):
    n = len(q)
    M = np.zeros((n,n))
    qd = np.array([0.0, 0.0, 0.0, 0.0])
    for i in range(n):
        ei = np.array([0.0, 0.0, 0.0, 0.0])
        ei[i] = 1
        M[:n,i] = RNEA(0, q, qd, ei)
    return M

def geth(g0,q):
    qd = np.array([0.0, 0.0, 0.0, 0.0])
    qdd = np.array([0.0, 0.0, 0.0, 0.0])
    g = np.array([0.0, 0.0, g0])
    h = RNEA(g0,q, qd, qdd)
    return h

    



