# -*- coding: utf-8 -*-
"""
Created on Mon Dec 17 11:39:13 2018

@author: rorsolino
"""

import numpy as np

class RigidBodyDynamics:
    def __init__(self): 
        self.LX = 0
        self.LY = 1
        self.LZ = 2
        self.AX = 3
        self.AY = 4
        self.AZ = 5