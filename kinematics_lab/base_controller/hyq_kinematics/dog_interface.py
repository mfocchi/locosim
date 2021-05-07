# -*- coding: utf-8 -*-
"""
Created on Mon Dec 17 11:30:12 2018

@author: rorsolino
"""

import numpy as np

class DogInterface:
    def __init__(self):
        self.LF = 0
        self.RF = 1
        self.LH = 2
        self.RH = 3
        
        self.HAA = 0
        self.HFE = 1
        self.KFE = 2
        
        self.legJoints = [self.HAA, self.HFE, self.KFE]
        
        self.legs = [self.LF, self.RF, self.LH, self.RH]
        
        temp = [0.0, 0.0, 0.0]
        self.footPos =[[temp],[temp],[temp],[temp]]