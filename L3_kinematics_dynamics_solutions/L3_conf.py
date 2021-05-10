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





