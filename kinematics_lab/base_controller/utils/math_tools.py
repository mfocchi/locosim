# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 09:43:27 2018

@author: romeo orsolino
"""
#to be compatible with python3
from __future__ import print_function
import numpy as np
import scipy as sp
import math as math
from utils import *

class Math:
    def normalize(self, n):
        norm1 = np.linalg.norm(n)
        n = np.true_divide(n, norm1)
        return n

    def skew(self, v):
        if len(v) == 4: v = v[:3]/v[3]
        skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
        return skv - skv.T

    def rotation_matrix_from_normal(self, n):
        n = n.reshape((3,))
        e_x = np.array([1., 0., 0.])
        t = e_x - np.dot(e_x, n) * n
        t = t / np.linalg.norm(t)
        b = np.cross(n, t)
        return np.vstack([t, b, n]).T

    def getGraspMatrix(self, r):
        math = Math()
        G = block([[np.eye(3), np.zeros((3, 3))],
                       [math.skew(r), np.eye(3)]])
        return G    

    def plane_z_intercept(self, point_on_plane, plane_normal):
        return point_on_plane[2] + \
               plane_normal[0] / plane_normal[2] * point_on_plane[0] + \
               plane_normal[1] / plane_normal[2] * point_on_plane[1]

    def compute_z_component_of_plane(self, xy_components, plane_normal, z_intercept):
        return -plane_normal[0]/plane_normal[2]*xy_components[0] - \
               plane_normal[1]/plane_normal[2]*xy_components[1] + z_intercept

    def rpyToRot(self, roll, pitch, yaw):
    

        
        Rx =  np.array([ [   1   ,    0           ,        0], 
                         [0   ,    np.cos(roll) ,  np.sin(roll)],
                         [0   ,    -np.sin(roll),  np.cos(roll)]]);


        Ry = np.array([[np.cos(pitch)     ,     0  ,   -np.sin(pitch)],
              [      0       ,    1  ,   0],
              [np.sin(pitch)     ,    0   ,  np.cos(pitch)]]);
          
        
        Rz = np.array([[ np.cos(yaw)  ,  np.sin(yaw) ,        0],
                      [-np.sin(yaw) ,  np.cos(yaw) ,          0],
                      [0      ,     0     ,       1]]);
        
        

        R =  Rx.dot(Ry.dot(Rz));
        return R
                                
    def rpyToRot(self, rpy):
        c_roll =  np.cos(rpy[0])
        s_roll = np.sin(rpy[0])
        c_pitch =      np.cos(rpy[1])        
        s_pitch = np.sin(rpy[1])
        c_yaw = np.cos(rpy[2])
        s_yaw = np.sin(rpy[2])
                                
        Rx =  np.array([ [   1   ,    0           ,        0], 
                         [   0   ,        c_roll  ,  s_roll],
                         [   0   ,    -s_roll,      c_roll ]]);


        Ry = np.array([[c_pitch     ,     0  ,   -s_pitch],
              [      0       ,    1  ,   0],
              [ s_pitch     ,    0   ,  c_pitch]]);
          
        
        Rz = np.array([[ c_yaw  ,  s_yaw ,        0],
                      [  -s_yaw ,  c_yaw ,          0],
                      [0      ,     0     ,       1]]);
        
        

        R =  Rx.dot(Ry.dot(Rz));
        return R                                
                                
    # the dual of rpyToRot()
     # set of Euler angles (according to ZYX convention) representing the orientation of frame represented by b_R_w
    def rotTorpy(self, b_R_w):
        rpy = np.array([0.0,0.0,0.0])                    
        rpy[0] = np.arctan2(b_R_w[1,2], b_R_w[2,2])
        rpy[1] = -np.arcsin( b_R_w[0,2])
        rpy[2] = np.arctan2(b_R_w[0,1], b_R_w[0,0])
    
        return rpy;
                
     # used to compute 
    # omega_dot = J_omega * euler_rates_dot + J_omega_dot*euler_rates                
    def Jomega_dot(self, rpy, rpyd):
    
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]
        rolld = rpyd[0]
        pitchd = rpyd[1]
        yawd = rpyd[2]
    
        Jomega_dot = np.array([[ -np.cos(yaw)*np.sin(pitch)*pitchd - np.cos(pitch)*np.sin(yaw)*yawd, - np.cos(yaw)*yawd, 0],
                           [ np.cos(yaw)*np.cos(pitch)*yawd - np.sin(yaw)*np.sin(pitch)*pitchd, -np.sin(yaw)*yawd, 0  ],
                          [ -np.cos(pitch)*pitchd,  0, 0 ]])
    
        return Jomega_dot
    
    # returns w_omega = Jomega * euler_rates
    def Jomega(self, rpy):
    
        #convention yaw pitch roll
    
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        Jomega = np.array([[np.cos(pitch)*np.cos(yaw),       -np.sin(yaw),                 0],
                           [ np.cos(pitch)*np.sin(yaw),                  np.cos(yaw),                  0],
                           [ -np.sin(pitch),      0 ,        1]])
        return Jomega
                                

    def line(self, p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0]*p2[1] - p2[0]*p1[1])
        return A, B, -C

    def two_lines_intersection(self, L1, L2):
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x,y
        else:
            return False

    def is_point_inside_segment(self, first_input_point, second_input_point, point_to_check):
        epsilon = 0.001

        if (np.abs(first_input_point[0] - second_input_point[0]) < 1e-02):                     
            alpha = (point_to_check[1] - second_input_point[1]) / (first_input_point[1] - second_input_point[1]) 
        else:
            alpha = (point_to_check[0] - second_input_point[0]) / (first_input_point[0] - second_input_point[0])                 

        if(alpha>=-epsilon)&(alpha<=1.0+epsilon):
            new_point = point_to_check
        else:
            new_point = False

        return new_point, alpha

    def find_point_to_line_signed_distance(self, segment_point1, segment_point2, point_to_check):
        # this function returns a positive distance if the point is on the right side of the segment. This will return 
        # a positive distance for a polygon queried in clockwise order and with a point_to_check which lies inside the polygon itself 
        num = (segment_point2[0] - segment_point1[0])*(segment_point1[1] - point_to_check[1]) - (segment_point1[0] - point_to_check[0])*(segment_point2[1] - segment_point1[1])
        denum_sq = (segment_point2[0] - segment_point1[0])*(segment_point2[0] - segment_point1[0]) + (segment_point2[1] - segment_point1[1])*(segment_point2[1] - segment_point1[1])
        dist = num/np.sqrt(denum_sq)
#        print segment_point1, segment_point2, point_to_check, dist
        return dist

    def find_residual_radius(self, polygon, point_to_check):
        # this function returns a positive distance if the point is on the right side of the segment. This will return 
        # a positive distance for a polygon queried in clockwise order and with a point_to_check which lies inside the polygon itself 
        # print 'poly',polygon
        numberOfVertices = np.size(polygon,0)
        # print 'polygon in residual radius computation', polygon
        # print 'number of vertices', numberOfVertices
        residual_radius = 1000000.0
        for i in range(0,numberOfVertices-1):
            s1 = polygon[i,:]
            s2 = polygon[i+1,:]
            # print s1, s2, point_to_check
            d_temp = self.find_point_to_line_signed_distance(s1, s2, point_to_check)
#            print i, s1, s2, d_temp
            if d_temp < 0.0:
                print ('Warning! found negative distance. Polygon might not be in clockwise order...')
            elif d_temp < residual_radius:
                residual_radius = d_temp

        # we dont need to compute for the last edge cause we added an extra point to close the polytop (last point equal to the first)
        
#        print polygon[numberOfVertices-1,:], polygon[0,:], d_temp
        return residual_radius

    def find_polygon_segment_intersection(self, vertices_input, desired_direction, starting_point):
        desired_direction = desired_direction/np.linalg.norm(desired_direction)*10.0
        #print "desired dir: ", desired_direction

        desired_com_line = self.line(starting_point, starting_point+desired_direction)
        #print "des line : ", desired_com_line
        tmp_vertices = np.vstack([vertices_input, vertices_input[0]])
        intersection_points = np.zeros((0,2))
        points_along_direction = np.zeros((0,2))
        point_to_com_distance = np.zeros((0,1))

        for i in range(0,len(vertices_input)):
            v1 = tmp_vertices[i,:]
            v2 = tmp_vertices[i+1,:]
            actuation_region_edge = self.line(v1, v2)
            #print desired_com_line, actuation_region_edge
            new_point = self.two_lines_intersection(desired_com_line, actuation_region_edge)

            if new_point:
                intersection_points = np.vstack([intersection_points, new_point])
                new_point, alpha = self.is_point_inside_segment(starting_point, starting_point+desired_direction, new_point)
                if new_point:
                    points_along_direction = np.vstack([points_along_direction, new_point])
                    d = np.sqrt((new_point[0] - starting_point[0])*(new_point[0] - starting_point[0]) + (new_point[1] - starting_point[1])*(new_point[1] - starting_point[1]))
                    point_to_com_distance = np.vstack([point_to_com_distance, d])

            else:
                print( "lines are parallel!")
                #while new_point is False:
                #    desired_com_line = self.line(starting_point, starting_point+desired_direction)
                #    new_point = self.two_lines_intersection(desired_com_line, actuation_region_edge)
                #    intersection_points = np.vstack([intersection_points, new_point])
                #    print new_point

        #print points_along_direction, point_to_com_distance
        idx = np.argmin(point_to_com_distance)
        final_point = points_along_direction[idx,:]
        #print points_along_direction, point_to_com_distance, idx
        return final_point, intersection_points
        

def cross_mx(v):
    mx =np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return mx
def cross_mx_casadi(v):
    # mx =[[MX(0.0), -v[2], v[1]], [v[2], MX(0.0), -v[0]], [-v[1], v[0], MX(0.0)]]
    mx =MX.zeros(3,3)
    mx[0, 1] =  -v[2]
    mx[0, 2] =   v[1]
    mx[1, 0] =   v[2]
    mx[1, 2] =  -v[0]
    mx[2, 0] =  -v[1]
    mx[2, 1] =   v[0]
    return mx
def skew_simToVec(Ra):
    # This is similar to one implemented in the framework
    v = np.zeros(3)
    v[0] = 0.5*(Ra[2,1] - Ra[1,2])
    v[1] = 0.5*(Ra[0,2] - Ra[2,0])
    v[2] = 0.5*(Ra[1,0] - Ra[0,1])

    return v



def rotMatToRotVec(Ra):
    c = 0.5 * (Ra[0, 0] + Ra[1, 1] + Ra[2, 2] - 1)
    w = -skew_simToVec(Ra)
    s = np.linalg.norm(w) # w = sin(theta) * axis

    if abs(s) <= 1e-10:
        err = np.zeros(3)
    else:
        angle = math.atan2(s, c)
        axis = w / s
        err = angle * axis
    return err


# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> np.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not np.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)


def MxInv(A):
    # Determinant of matrix A
    sb1 = A[0, 0] * ((A[1, 1] * A[2, 2]) - (A[1, 2] * A[2, 1]))
    sb2 = A[0, 1] * ((A[1, 0] * A[2, 2]) - (A[1, 2] * A[2, 0]))
    sb3 = A[0, 2] * ((A[1, 0] * A[2, 1]) - (A[1, 1] * A[2, 0]))

    Adetr = sb1 - sb2 + sb3
    #    print(Adetr)
    # Transpose matrix A
    TransA = A.T

    # Find determinant of the minors
    a01 = (TransA[1, 1] * TransA[2, 2]) - (TransA[2, 1] * TransA[1, 2])
    a02 = (TransA[1, 0] * TransA[2, 2]) - (TransA[1, 2] * TransA[2, 0])
    a03 = (TransA[1, 0] * TransA[2, 1]) - (TransA[2, 0] * TransA[1, 1])

    a11 = (TransA[0, 1] * TransA[2, 2]) - (TransA[0, 2] * TransA[2, 1])
    a12 = (TransA[0, 0] * TransA[2, 2]) - (TransA[0, 2] * TransA[2, 0])
    a13 = (TransA[0, 0] * TransA[2, 1]) - (TransA[0, 1] * TransA[2, 0])

    a21 = (TransA[0, 1] * TransA[1, 2]) - (TransA[1, 1] * TransA[0, 2])
    a22 = (TransA[0, 0] * TransA[1, 2]) - (TransA[0, 2] * TransA[1, 0])
    a23 = (TransA[0, 0] * TransA[1, 1]) - (TransA[0, 1] * TransA[1, 0])

    # Inverse of determinant
    invAdetr = (float(1) / Adetr)
    #    print(invAdetr)
    # Inverse of the matrix A
    invA = MX.zeros(3,3)
    invA[0, 0] = invAdetr * a01
    invA[0, 1] = -invAdetr * a02
    invA[0, 2] = invAdetr * a03

    invA[0, 0] = -invAdetr * a11
    invA[0, 1] = invAdetr * a12
    invA[0, 2] = -invAdetr * a13

    invA[0, 0] = invAdetr * a21
    invA[0, 1] = -invAdetr * a22
    invA[0, 2] = invAdetr * a23

    # Return the matrix
    return invA


#/**
#brief motionVectorTransform Tranforms twists from A to B (b_X_a)   \in R^6 \times 6
#here A is the origin frame and B the destination frame.
#param position coordinate vector expressing OaOb in A coordinates
#param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
#return
#
def motionVectorTransform(position, rotationMx):
    utils = Utils()
    b_X_a = np.zeros((6,6))
    b_X_a[utils.sp_crd["AX"]:utils.sp_crd["AX"]+ 3 ,   utils.sp_crd["AX"]: utils.sp_crd["AX"] + 3] = rotationMx
    b_X_a[utils.sp_crd["LX"]:utils.sp_crd["LX"] + 3 ,   utils.sp_crd["AX"]: utils.sp_crd["AX"] + 3] = -rotationMx*cross_mx(position)
    b_X_a[utils.sp_crd["LX"]:utils.sp_crd["LX"] + 3 ,   utils.sp_crd["LX"]: utils.sp_crd["LX"] + 3] = rotationMx
    return b_X_a


#
#brief forceVectorTransform Tranforms wrenches from A to B (b_X_a)   \in R^6 \times 6
#here A is the origin frame and B the destination frame.
#param position coordinate vector expressing OaOb in A coordinates
#param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
#return
#


def forceVectorTransform(position, rotationMx):
    utils = Utils()
    b_X_a = np.zeros((6,6))
    b_X_a[utils.sp_crd("AX"):utils.sp_crd("AX") + 3 ,   utils.sp_crd("AX"): utils.sp_crd("AX") + 3] = rotationMx
    b_X_a[utils.sp_crd("AX"):utils.sp_crd("AX") + 3 ,   utils.sp_crd("LX"): utils.sp_crd("LX") + 3] = -rotationMx*cross_mx(position)
    b_X_a[utils.sp_crd("LX"):utils.sp_crd("LX") + 3 ,   utils.sp_crd("LX"): utils.sp_crd("LX") + 3] = rotationMx
    return b_X_a


def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else:
        print("Toc: start time not set")
