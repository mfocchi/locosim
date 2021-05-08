# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os
import math

dt = 0.001                   # controller time step
exp_duration_sin = 3.0 #sine reference duration
exp_duration = 5.0 #simulation duration

SLOW_FACTOR = 1 #to slow down simulation

frame_name = 'ee_link'    # name of the frame to control (end-effector)

# Initial configuration / velocity / Acceleration
q0 =   np.array([math.pi, -math.pi/8,  -math.pi/6, 0.0])
qd0 =  np.array([    0.0, 0.0, 0.0, 0.0])
qdd0 = np.array([    0.0, 0.0, 0.0, 0.0]) 

#UR4 robot parameters
l1 = 0.089159
l2 = 0.13585
l3 = 0.425
l4 = 0.1197
l5 = 0.39225
l6 = 0.094
l7 = 0.068  
urdf90degrees = 1.570796325

parameters = np.array([l1, l2, l3, l4, l5, l6, l7, urdf90degrees])





