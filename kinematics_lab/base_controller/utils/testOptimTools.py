# -*- coding: utf-8 -*-
"""
Created on Mon Apr  6 14:15:50 2020

@author: mfocchi
"""

from optimTools import quadprog_solve_qp
import numpy as np
# Print options 
import sys
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
sys.dont_write_bytecode = True

#∥Mx−b∥2# 
# G = Mt*M
# g = -MT*b
#0.5 xT*G*x + g*x
#s.t. Cx≤d
#     Ax=b




#do this with numpy array
M = np.array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
b = np.array([3., 2., 3.])

G = np.dot(M.T, M)
g = -np.dot(b.T, M)

C = np.array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
d = np.array([3., 2., -2.]).reshape((3,))

result = quadprog_solve_qp(G, g, C, d)

print result 

#do the same with matrix array
M = np.matrix([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
b = np.matrix([3., 2., 3.]).T
G = M.T*M
g = -b.T*M
C = np.matrix([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
d = np.matrix([3., 2., -2.]).reshape((3,))
result = quadprog_solve_qp(G, g, C, d)

print result

#with equalituies and inequalities 
M = np.matrix([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
b = np.matrix([3., 2., 3.]).T
G = M.T*M
g = -b.T*M

#3 ineq constraints 
C = np.matrix([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
d = np.matrix([3., 2., -2.]).reshape((3,))

#1 ineq constraint
A = np.matrix([[0.5, 1., 2.]])
b = np.matrix([3. ]).reshape((1,))

result = quadprog_solve_qp(G, g, C, d,A , b)

print result

from scipy.linalg import block_diag
M = np.array([[ 1.    , 0.    , 0.    , 1.    , 0.    , 0.    , 1.    , 0.   ,  0.     ,1.   ,  0.    , 0.   ],
		     [ 0.    , 1.    , 0.    , 0.    , 1.    , 0.    , 0.    , 1.   ,  0.    , 0.   ,  1.    , 0.   ],
               [ 0.    , 0.    , 1.    , 0.    , 0.    , 1.    ,0.     ,0.    , 1.    , 0.    , 0.     ,1.   ],
               [ 0.    , 0.598  ,0.328 , 0.    , 0.598 ,-0.328  ,0.    , 0.598 , 0.328 , 0.    , 0.598 ,-0.328],
               [-0.598 , 0.    ,-0.368 ,-0.598 , 0.    ,-0.368 ,-0.598 , 0.    , 0.367 ,-0.598 , 0.    , 0.367],
               [-0.328 , 0.368  ,0.    , 0.328 , 0.368 , 0.    ,-0.328 ,-0.367 , 0.   ,  0.328 ,-0.367 , 0.   ]])
															
															
b =	np.array([  -0. ,     -100.   ,  838.225 ,  -0.  ,    -0.114 ,  -0.   ])
G = np.dot(M.T, M) + np.eye(12)*1e-07
g = -np.dot(b.T, M)
C =  - block_diag( np.array([0. ,0. ,1.]),  np.array([0. ,0. ,1.]), np.array([0. ,0. ,1.]), np.array([0. ,0. ,1.]))
d = -np.array([230., 10. ,10. ,10.]).reshape((4,))	

result = quadprog_solve_qp(G, g, C, d,None , None)
print result