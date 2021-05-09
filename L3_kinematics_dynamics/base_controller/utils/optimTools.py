# -*- coding: utf-8 -*-
"""
Created on Mon Apr  6 14:12:33 2020

@author: mfocchi
"""

import numpy as np
from quadprog import solve_qp  
#
#minimize (1/2)x.T*G*x+g*x
#s.t. Cx≤d
#     Ax=b


def quadprog_solve_qp(G, g, C=None, d=None, A=None, b=None):
    
    
 # python solve_qp requires
 # Minimize     1/2 x^T G x - a^T x
 # Subject to   C.T x >= b        
    
    qp_G = .5 * (G + G.T)   # make sure P is symmetric
    qp_a = -g.T
    if A is not None and C is not None:
        qp_C = -np.vstack([A, C]).T
        qp_b = -np.hstack([b, d])
        meq = A.shape[0]
    elif C is not None and A is None:  # only inequality constraints
        qp_C = -C.T
        qp_b = -d
        meq = 0
    elif C is  None and A is not None: #only equality constraints                                
    # no equality constraint
        qp_C = -A.T
        qp_b = -b
        meq = A.shape[0]
    else:
       print("quadprog_solve_qp error: you need to set at least one inequality or equality constraint")
       return 0                     
                                    
    if isinstance(qp_a, np.matrix) or isinstance(qp_b, np.matrix):                                
        return solve_qp(qp_G, qp_a.A1, qp_C, qp_b.A1, meq)[0]     
    else:
        return solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]                    