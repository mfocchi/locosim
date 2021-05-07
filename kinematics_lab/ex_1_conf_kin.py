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

SLOW_FACTOR = 3 #to slow down simulation

frame_name = 'ee_link'    # name of the frame to control (end-effector)

#PD controller
## Matrix of gains
kp = np.eye(4)*600  # proportional gains 
kd = np.eye(4)*20 # derivative gains (critical damping)

## PARAMETERS OF REFERENCE SINUSOIDAL TRAJECTORY
amp = np.array([ 0.0, 0.2, 0.0, 0.0])    # amplitude
phi = np.array([ 0.0, 0.0, 0.0, 0.0])      # phase
freq = np.array([ 0.0, 1.0, 0.0, 0.0])    # frequency

# Initial configuration / velocity / Acceleration
# q0  = np.array([ math.pi/4, -math.pi/4, math.pi/2, 0.0]) 
q0 = np.array([0.     , 0.0,  0*math.pi/6, 0.0])
qd0 = np.array([ 0.0, 0.0, 0.0, 0.0])
qdd0 = np.array([ 0.0, 0.0, 0.0, 0.0]) 


