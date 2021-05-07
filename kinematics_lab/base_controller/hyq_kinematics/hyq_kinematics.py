# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
from __future__ import print_function
import numpy as np
import sys

if sys.version_info[:2] == (2, 7):
    from dog_interface import DogInterface
    from rigid_body_dynamics import RigidBodyDynamics
else:
    from hyq_kinematics.dog_interface import DogInterface
    from hyq_kinematics.rigid_body_dynamics import RigidBodyDynamics


class HyQKinematics:
    def __init__(self):
        
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()

        self.upperLegLength = 0.35;
        self.lowerLegLength = 0.341;

        self.BASE2HAA_offset_x = 0.3735;
        self.BASE2HAA_offset_y = 0.207;
        self.BASE2HAA_offset_z = 0.08;
        self.HAA2HFE = 0.08;

        # self.isOutOfWorkSpace = False
        
        self.fr_LF_lowerleg_Xh_LF_foot = np.zeros((4,4));	
        self.fr_RF_lowerleg_Xh_RF_foot = np.zeros((4,4));
        self.fr_LH_lowerleg_Xh_LH_foot = np.zeros((4,4));
        self.fr_RH_lowerleg_Xh_RH_foot = np.zeros((4,4));
        self.LF_foot_Xh_fr_trunk = np.zeros((4,4));
        self.RF_foot_Xh_fr_trunk = np.zeros((4,4));
        self.LH_foot_Xh_fr_trunk = np.zeros((4,4));
        self.RH_foot_Xh_fr_trunk = np.zeros((4,4));
        self.fr_trunk_Xh_LF_foot = np.zeros((4,4));    
        self.fr_trunk_Xh_fr_LF_HAA = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LF_HFE = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LF_KFE = np.zeros((4,4));
        self.fr_trunk_Xh_fr_RF_HAA = np.zeros((4,4));
        self.fr_trunk_Xh_RF_foot = np.zeros((4,4));
        self.fr_trunk_Xh_fr_RF_HFE = np.zeros((4,4));
        self.fr_trunk_Xh_fr_RF_KFE = np.zeros((4,4));
        self.fr_trunk_Xh_LH_foot = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LH_HAA = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LH_HFE = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LH_KFE = np.zeros((4,4));
        self.fr_trunk_Xh_RH_foot =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RH_HAA =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RH_HFE =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RH_KFE =np.zeros((4,4));
        self.fr_LF_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        self.fr_trunk_Xh_fr_LF_hipassembly =np.zeros((4,4));
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly =np.zeros((4,4));
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg =np.zeros((4,4));
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg =np.zeros((4,4));
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg =np.zeros((4,4));
        self.fr_RF_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RF_hipassembly =np.zeros((4,4));
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly =np.zeros((4,4));
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg =np.zeros((4,4));
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg =np.zeros((4,4));
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg =np.zeros((4,4));
        self.fr_LH_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        self.fr_trunk_Xh_fr_LH_hipassembly =np.zeros((4,4));
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly =np.zeros((4,4));
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg =np.zeros((4,4));
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg =np.zeros((4,4));
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg =np.zeros((4,4));
        self.fr_RH_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RH_hipassembly =np.zeros((4,4));
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly =np.zeros((4,4));
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg =np.zeros((4,4));
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg =np.zeros((4,4));
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg =np.zeros((4,4));

        self.s__q_LF_HFE = 0.0
        self.s__q_LF_KFE = 0.0
        self.s__q_LF_HAA = 0.0
        self.s__q_RF_HFE = 0.0
        self.s__q_RF_KFE = 0.0
        self.s__q_RF_HAA = 0.0
        self.s__q_LH_HFE = 0.0
        self.s__q_LH_KFE = 0.0
        self.s__q_LH_HAA = 0.0
        self.s__q_RH_HFE = 0.0
        self.s__q_RH_KFE = 0.0
        self.s__q_RH_HAA = 0.0
        self.c__q_LF_HFE = 0.0
        self.c__q_LF_KFE = 0.0
        self.c__q_LF_HAA = 0.0
        self.c__q_RF_HFE = 0.0
        self.c__q_RF_KFE = 0.0
        self.c__q_RF_HAA = 0.0
        self.c__q_LH_HFE = 0.0
        self.c__q_LH_KFE = 0.0
        self.c__q_LH_HAA = 0.0
        self.c__q_RH_HFE = 0.0
        self.c__q_RH_KFE = 0.0
        self.c__q_RH_HAA = 0.0
        
        '''jacobians'''
        self.fr_trunk_J_LF_foot = np.zeros((6,3));
        self.fr_trunk_J_RF_foot = np.zeros((6,3));
        self.fr_trunk_J_LH_foot = np.zeros((6,3));
        self.fr_trunk_J_RH_foot = np.zeros((6,3));
        
        '''initialize quantities'''
        self.init_jacobians()
        self.init_homogeneous()
        

    def init_jacobians(self):

        self.fr_trunk_J_LF_foot[0,0] = - 1.0;
        self.fr_trunk_J_RF_foot[0,0] = 1.0;
        self.fr_trunk_J_LH_foot[0,0] = - 1.0;
        self.fr_trunk_J_RH_foot[0,0] = 1.0;

    def init_homogeneous(self):
        
        self.fr_LF_lowerleg_Xh_LF_foot[0,2] = - 1.0;
        self.fr_LF_lowerleg_Xh_LF_foot[0,3] = self.lowerLegLength;
        self.fr_LF_lowerleg_Xh_LF_foot[1,0] = - 1;
        self.fr_LF_lowerleg_Xh_LF_foot[2,1] = 1;
        self.fr_LF_lowerleg_Xh_LF_foot[3,3] = 1;	
               
        self.fr_RF_lowerleg_Xh_RF_foot[0,2] = - 1.0;
        self.fr_RF_lowerleg_Xh_RF_foot[0,3] = self.lowerLegLength;
        self.fr_RF_lowerleg_Xh_RF_foot[1,0] = - 1;
        self.fr_RF_lowerleg_Xh_RF_foot[2,1] = 1;
        self.fr_RF_lowerleg_Xh_RF_foot[3,3] = 1;	
        
        self.fr_LH_lowerleg_Xh_LH_foot[0,2] = - 1.0;
        self.fr_LH_lowerleg_Xh_LH_foot[0,3] = self.lowerLegLength;
        self.fr_LH_lowerleg_Xh_LH_foot[1,0] = - 1;
        self.fr_LH_lowerleg_Xh_LH_foot[2,1] = 1;
        self.fr_LH_lowerleg_Xh_LH_foot[3,3] = 1;	
        
        self.fr_RH_lowerleg_Xh_RH_foot[0,2] = - 1.0;
        self.fr_RH_lowerleg_Xh_RH_foot[0,3] = self.lowerLegLength;
        self.fr_RH_lowerleg_Xh_RH_foot[1,0] = - 1;
        self.fr_RH_lowerleg_Xh_RH_foot[2,1] = 1;
        self.fr_RH_lowerleg_Xh_RH_foot[3,3] = 1;	
        
        self.LF_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.RF_foot_Xh_fr_trunk[3,3]= 1.0;	
        
        self.LH_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.RH_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_LF_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_HAA[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LF_HAA[1,1] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[1,3] = self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LF_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HFE[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LF_HFE[3,3] = 1.0;	
         
        self.fr_trunk_Xh_fr_LF_KFE[3,3] = 1.0;	  
         
        self.fr_trunk_Xh_RF_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_HAA[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RF_HAA[1,1] = 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[1,3] = - self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RF_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_RF_HFE[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RF_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_KFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_LH_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_HAA[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LH_HAA[1,1] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[1,3] = self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LH_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_HFE[0,0] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HFE[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LH_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_KFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_RH_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_HAA[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RH_HAA[1,1] = 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[1,3] = - self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RH_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_RH_HFE[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RH_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_KFE[3,3] = 1.0;	
        
        self.fr_LF_hipassembly_Xh_fr_trunk[2,0] = - 1.0;
        self.fr_LF_hipassembly_Xh_fr_trunk[2,3] = self.BASE2HAA_offset_x;
        self.fr_LF_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_hipassembly[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LF_hipassembly[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LF_hipassembly[1,3] = self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LF_hipassembly[3,3] = 1.0;	
            
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[2,1] = - 1;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[3,3] = 1.0;	
     
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,3] = self.BASE2HAA_offset_z;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[1,2] = - 1;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[3,3] = 1;	
        
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[2,2] = 1;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[3,3] = 1.0;	
        
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,3] = 0.35;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[2,2] = 1;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[3,3] = 1;	
        
        self.fr_RF_hipassembly_Xh_fr_trunk[2,0] = 1.0;
        self.fr_RF_hipassembly_Xh_fr_trunk[2,3] = - self.BASE2HAA_offset_x;
        self.fr_RF_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_hipassembly[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RF_hipassembly[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RF_hipassembly[1,3] = - self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RF_hipassembly[3,3] = 1.0;	
        
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[2,1] = 1;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[3,3] = 1.0;	
        
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,3] = self.BASE2HAA_offset_z;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[1,2] = 1;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[3,3] = 1;	
        
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[2,2] = 1;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[3,3] = 1.0;	
        
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,3] = self.upperLegLength;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[2,2] = 1;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[3,3] = 1;	
        
        self.fr_LH_hipassembly_Xh_fr_trunk[2,0] = - 1.0;
        self.fr_LH_hipassembly_Xh_fr_trunk[2,3] = - self.BASE2HAA_offset_x;
        self.fr_LH_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_hipassembly[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LH_hipassembly[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LH_hipassembly[1,3] = self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LH_hipassembly[3,3] = 1.0;	
        
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[2,1] = - 1;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[3,3] = 1.0;	
        
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,3] = self.BASE2HAA_offset_z;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[1,2] = - 1;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[3,3] = 1;	
        
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[2,2]= 1;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[3,3]= 1.0;	
        
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,3] = self.upperLegLength;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[2,2] = 1;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[3,3] = 1;	
        

        self.fr_RH_hipassembly_Xh_fr_trunk[2,0] = 1.0;
        self.fr_RH_hipassembly_Xh_fr_trunk[2,3] = self.BASE2HAA_offset_x;
        self.fr_RH_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_hipassembly[0,2]= 1.0;
        self.fr_trunk_Xh_fr_RH_hipassembly[0,3]= - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RH_hipassembly[1,3]= - self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RH_hipassembly[3,3]= 1.0;	
        
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[2,1]= 1;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[3,3]= 1.0;	
        
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,3] = self.BASE2HAA_offset_z;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[1,2] = 1;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[3,3] = 1;	
        
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[2,2] = 1;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[3,3] = 1.0;
        
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[0,3] = self.upperLegLength;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[2,2] = 1;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[3,3] = 1;
        
        return True
                
    def update_homogeneous(self, q):
        
        self.s__q_LF_HFE = np.sin( q[1]);
        self.s__q_LF_KFE = np.sin( q[2]);
        self.s__q_LF_HAA = np.sin( q[0]);
        self.s__q_RF_HFE = np.sin( q[4]);
        self.s__q_RF_KFE = np.sin( q[5]);
        self.s__q_RF_HAA = np.sin( q[3]);
        self.s__q_LH_HFE = np.sin( q[7]);
        self.s__q_LH_KFE = np.sin( q[8]);
        self.s__q_LH_HAA = np.sin( q[6]);
        self.s__q_RH_HFE = np.sin( q[10]);
        self.s__q_RH_KFE = np.sin( q[11]);
        self.s__q_RH_HAA = np.sin( q[9]);
        self.c__q_LF_HFE = np.cos( q[1]);
        self.c__q_LF_KFE = np.cos( q[2]);
        self.c__q_LF_HAA = np.cos( q[0]);
        self.c__q_RF_HFE = np.cos( q[4]);
        self.c__q_RF_KFE = np.cos( q[5]);
        self.c__q_RF_HAA = np.cos( q[3]);
        self.c__q_LH_HFE = np.cos( q[7]);
        self.c__q_LH_KFE = np.cos( q[8]);
        self.c__q_LH_HAA = np.cos( q[6]);
        self.c__q_RH_HFE = np.cos( q[10]);
        self.c__q_RH_KFE = np.cos( q[11]);
        self.c__q_RH_HAA = np.cos( q[9]);
        
        
        self.LF_foot_Xh_fr_trunk[0,0] = ( self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[0,1] = (- self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[0,2] = (- self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[0,3] = ((( self.BASE2HAA_offset_x *  self.s__q_LF_HFE) + ((( self.BASE2HAA_offset_y *  self.s__q_LF_HAA) -  self.BASE2HAA_offset_z) *  self.c__q_LF_HFE) -  self.upperLegLength) *  self.s__q_LF_KFE) + ((((( self.BASE2HAA_offset_y *  self.s__q_LF_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_LF_HFE) - ( self.BASE2HAA_offset_x *  self.c__q_LF_HFE)) *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[1,1] =  self.c__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[1,2] = - self.s__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[1,3] = - self.BASE2HAA_offset_y *  self.c__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[2,0] = ( self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,1] = ( self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,2] = ( self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,3] = ((((( self.BASE2HAA_offset_y *  self.s__q_LF_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_LF_HFE) - ( self.BASE2HAA_offset_x *  self.c__q_LF_HFE)) *  self.s__q_LF_KFE) + (((- self.BASE2HAA_offset_x *  self.s__q_LF_HFE) + (( self.BASE2HAA_offset_z - ( self.BASE2HAA_offset_y *  self.s__q_LF_HAA)) *  self.c__q_LF_HFE) +  self.upperLegLength) *  self.c__q_LF_KFE) +  self.lowerLegLength;
        
        
        
        self.RF_foot_Xh_fr_trunk[0,0] = ( self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,1] = ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,2] = (- self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,3] = ((( self.BASE2HAA_offset_x *  self.s__q_RF_HFE) + ((( self.BASE2HAA_offset_y *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_z) *  self.c__q_RF_HFE) -  self.upperLegLength) *  self.s__q_RF_KFE) + ((((( self.BASE2HAA_offset_y *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_RF_HFE) - ( self.BASE2HAA_offset_x *  self.c__q_RF_HFE)) *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[1,1] =  self.c__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[1,2] =  self.s__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[2,0] = ( self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,1] = ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,2] = ( self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,3] = ((((( self.BASE2HAA_offset_y *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_RF_HFE) - ( self.BASE2HAA_offset_x *  self.c__q_RF_HFE)) *  self.s__q_RF_KFE) + (((- self.BASE2HAA_offset_x *  self.s__q_RF_HFE) + (( self.BASE2HAA_offset_z - ( self.BASE2HAA_offset_y *  self.s__q_RF_HAA)) *  self.c__q_RF_HFE) +  self.upperLegLength) *  self.c__q_RF_KFE) +  self.lowerLegLength;
        
        
        
        self.LH_foot_Xh_fr_trunk[0,0] = ( self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,1] = (- self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,2] = (- self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,3] = (((- self.BASE2HAA_offset_x *  self.s__q_LH_HFE) + ((( self.BASE2HAA_offset_y *  self.s__q_LH_HAA) -  self.BASE2HAA_offset_z) *  self.c__q_LH_HFE) -  self.upperLegLength) *  self.s__q_LH_KFE) + ((((( self.BASE2HAA_offset_y *  self.s__q_LH_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_LH_HFE) + ( self.BASE2HAA_offset_x *  self.c__q_LH_HFE)) *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[1,1] =  self.c__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[1,2] = - self.s__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[1,3] = - self.BASE2HAA_offset_y *  self.c__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[2,0] = ( self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,1] = ( self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,2] = ( self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,3] = ((((( self.BASE2HAA_offset_y *  self.s__q_LH_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_LH_HFE) + ( self.BASE2HAA_offset_x *  self.c__q_LH_HFE)) *  self.s__q_LH_KFE) + ((( self.BASE2HAA_offset_x *  self.s__q_LH_HFE) + (( self.BASE2HAA_offset_z - ( self.BASE2HAA_offset_y *  self.s__q_LH_HAA)) *  self.c__q_LH_HFE) +  self.upperLegLength) *  self.c__q_LH_KFE) +  self.lowerLegLength;
        
        
        
        self.RH_foot_Xh_fr_trunk[0,0] = ( self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,1] = ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,2] = (- self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,3] = (((- self.BASE2HAA_offset_x *  self.s__q_RH_HFE) + ((( self.BASE2HAA_offset_y *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_z) *  self.c__q_RH_HFE) -  self.upperLegLength) *  self.s__q_RH_KFE) + ((((( self.BASE2HAA_offset_y *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_RH_HFE) + ( self.BASE2HAA_offset_x *  self.c__q_RH_HFE)) *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[1,1] =  self.c__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[1,2] =  self.s__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[2,0] = ( self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,1] = ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,2] = ( self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,3] = ((((( self.BASE2HAA_offset_y *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_RH_HFE) + ( self.BASE2HAA_offset_x *  self.c__q_RH_HFE)) *  self.s__q_RH_KFE) + ((( self.BASE2HAA_offset_x *  self.s__q_RH_HFE) + (( self.BASE2HAA_offset_z - ( self.BASE2HAA_offset_y *  self.s__q_RH_HAA)) *  self.c__q_RH_HFE) +  self.upperLegLength) *  self.c__q_RH_KFE) +  self.lowerLegLength;
        
        
        
        self.fr_trunk_Xh_LF_foot[0,0] = ( self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[0,2] = ( self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[0,3] = (- self.lowerLegLength *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.s__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.s__q_LF_HFE) +  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_LF_foot[1,0] = (- self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[1,1] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_LF_foot[1,2] = ( self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[1,3] = ( self.lowerLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.s__q_LF_HAA) +  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_LF_foot[2,0] = (- self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[2,1] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_LF_foot[2,2] = ( self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[2,3] = ( self.lowerLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LF_HAA);
        
        
        
        self.fr_trunk_Xh_fr_LF_HFE[1,0] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[1,2] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[1,3] =  self.BASE2HAA_offset_y - ( self.BASE2HAA_offset_z *  self.s__q_LF_HAA);
        self.fr_trunk_Xh_fr_LF_HFE[2,0] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[2,2] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[2,3] = - self.BASE2HAA_offset_z *  self.c__q_LF_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LF_KFE[0,0] = - self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[0,1] = - self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[0,3] =  self.BASE2HAA_offset_x - ( self.upperLegLength *  self.s__q_LF_HFE);
        self.fr_trunk_Xh_fr_LF_KFE[1,0] = - self.s__q_LF_HAA *  self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[1,1] =  self.s__q_LF_HAA *  self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[1,2] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_KFE[1,3] = (- self.upperLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.s__q_LF_HAA) +  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LF_KFE[2,0] = - self.c__q_LF_HAA *  self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[2,1] =  self.c__q_LF_HAA *  self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[2,2] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_KFE[2,3] = (- self.upperLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LF_HAA);
        
        
        
        self.fr_trunk_Xh_RF_foot[0,0] = ( self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[0,2] = ( self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[0,3] = (- self.lowerLegLength *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.s__q_RF_HFE *  self.c__q_RF_KFE) - ( self.upperLegLength *  self.s__q_RF_HFE) +  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_RF_foot[1,0] = ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[1,1] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_RF_foot[1,2] = ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[1,3] = (- self.lowerLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( self.upperLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_RF_foot[2,0] = (- self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[2,1] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_RF_foot[2,2] = ( self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[2,3] = ( self.lowerLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.upperLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_RF_HAA);
        
        
        
        self.fr_trunk_Xh_fr_RF_HFE[1,0] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[1,2] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[1,3] = ( self.BASE2HAA_offset_z *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RF_HFE[2,0] = - self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[2,2] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[2,3] = - self.BASE2HAA_offset_z *  self.c__q_RF_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RF_KFE[0,0] = - self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[0,1] = - self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[0,3] =  self.BASE2HAA_offset_x - ( self.upperLegLength *  self.s__q_RF_HFE);
        self.fr_trunk_Xh_fr_RF_KFE[1,0] =  self.s__q_RF_HAA *  self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[1,1] = - self.s__q_RF_HAA *  self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[1,2] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_KFE[1,3] = ( self.upperLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RF_KFE[2,0] = - self.c__q_RF_HAA *  self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[2,1] =  self.c__q_RF_HAA *  self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[2,2] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_KFE[2,3] = (- self.upperLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_RF_HAA);
        
        
        
        self.fr_trunk_Xh_LH_foot[0,0] = ( self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[0,2] = ( self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[0,3] = (- self.lowerLegLength *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.s__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.s__q_LH_HFE) -  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_LH_foot[1,0] = (- self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[1,1] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_LH_foot[1,2] = ( self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[1,3] = ( self.lowerLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.s__q_LH_HAA) +  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_LH_foot[2,0] = (- self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[2,1] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_LH_foot[2,2] = ( self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[2,3] = ( self.lowerLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LH_HAA);
        
        
        
        self.fr_trunk_Xh_fr_LH_HFE[1,0] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[1,2] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[1,3] =  self.BASE2HAA_offset_y - ( self.BASE2HAA_offset_z *  self.s__q_LH_HAA);
        self.fr_trunk_Xh_fr_LH_HFE[2,0] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[2,2] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[2,3] = - self.BASE2HAA_offset_z *  self.c__q_LH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LH_KFE[0,0] = - self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[0,1] = - self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[0,3] = (- self.upperLegLength *  self.s__q_LH_HFE) -  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LH_KFE[1,0] = - self.s__q_LH_HAA *  self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[1,1] =  self.s__q_LH_HAA *  self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[1,2] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_KFE[1,3] = (- self.upperLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.s__q_LH_HAA) +  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LH_KFE[2,0] = - self.c__q_LH_HAA *  self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[2,1] =  self.c__q_LH_HAA *  self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[2,2] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_KFE[2,3] = (- self.upperLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LH_HAA);
        
        
        
        self.fr_trunk_Xh_RH_foot[0,0] = ( self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[0,2] = ( self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[0,3] = (- self.lowerLegLength *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.s__q_RH_HFE *  self.c__q_RH_KFE) - ( self.upperLegLength *  self.s__q_RH_HFE) -  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_RH_foot[1,0] = ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[1,1] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_RH_foot[1,2] = ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[1,3] = (- self.lowerLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( self.upperLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_RH_foot[2,0] = (- self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[2,1] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_RH_foot[2,2] = ( self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[2,3] = ( self.lowerLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.upperLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_RH_HAA);
        
        
        
        self.fr_trunk_Xh_fr_RH_HFE[1,0] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[1,2] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[1,3] = ( self.BASE2HAA_offset_z *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RH_HFE[2,0] = - self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[2,2] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[2,3] = - self.BASE2HAA_offset_z *  self.c__q_RH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RH_KFE[0,0] = - self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[0,1] = - self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[0,3] = (- self.upperLegLength *  self.s__q_RH_HFE) -  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RH_KFE[1,0] =  self.s__q_RH_HAA *  self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[1,1] = - self.s__q_RH_HAA *  self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[1,2] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_KFE[1,3] = ( self.upperLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RH_KFE[2,0] = - self.c__q_RH_HAA *  self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[2,1] =  self.c__q_RH_HAA *  self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[2,2] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_KFE[2,3] = (- self.upperLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_RH_HAA);
        
        self.fr_LF_hipassembly_Xh_fr_trunk[0,1] = - self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[0,3] =  self.BASE2HAA_offset_y *  self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,1] = - self.c__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_LF_HAA;
        
        self.fr_trunk_Xh_fr_LF_hipassembly[1,0] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[1,1] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[2,0] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[2,1] =  self.s__q_LF_HAA;
        
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,0] =  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,2] =  self.s__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,3] = - self.BASE2HAA_offset_z *  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,0] = - self.s__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,2] =  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,3] =  self.BASE2HAA_offset_z *  self.s__q_LF_HFE;
        
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,0] =  self.c__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,1] = - self.s__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[2,0] =  self.s__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[2,1] =  self.c__q_LF_HFE;
        
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,0] =  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,1] =  self.s__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,3] = - self.upperLegLength *  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,0] = - self.s__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,1] =  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,3] =  self.upperLegLength *  self.s__q_LF_KFE;
        
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,0] =  self.c__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,1] = - self.s__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[1,0] =  self.s__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[1,1] =  self.c__q_LF_KFE;
        
        self.fr_RF_hipassembly_Xh_fr_trunk[0,1] =  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[0,3] =  self.BASE2HAA_offset_y *  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,1] =  self.c__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_RF_HAA;
        
        self.fr_trunk_Xh_fr_RF_hipassembly[1,0] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[1,1] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[2,0] = - self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[2,1] =  self.s__q_RF_HAA;
        
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,0] =  self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,2] = - self.s__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,3] = - self.BASE2HAA_offset_z *  self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,0] = - self.s__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,2] = - self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,3] =  self.BASE2HAA_offset_z *  self.s__q_RF_HFE;
        
        
        
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,0] =  self.c__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,1] = - self.s__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[2,0] = - self.s__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[2,1] = - self.c__q_RF_HFE;
        
        
        
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,0] =  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,1] =  self.s__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,3] = - self.upperLegLength *  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,0] = - self.s__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,1] =  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,3] =  self.upperLegLength *  self.s__q_RF_KFE;
        
        
        
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,0] =  self.c__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,1] = - self.s__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[1,0] =  self.s__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[1,1] =  self.c__q_RF_KFE;
        
        
        
        self.fr_LH_hipassembly_Xh_fr_trunk[0,1] = - self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[0,3] =  self.BASE2HAA_offset_y *  self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,1] = - self.c__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_LH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LH_hipassembly[1,0] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[1,1] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[2,0] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[2,1] =  self.s__q_LH_HAA;
        
        
        
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,0] =  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,2] =  self.s__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,3] = - self.BASE2HAA_offset_z *  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,0] = - self.s__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,2] =  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,3] =  self.BASE2HAA_offset_z *  self.s__q_LH_HFE;
        
        
        
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,0] =  self.c__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,1] = - self.s__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[2,0] =  self.s__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[2,1] =  self.c__q_LH_HFE;
        
        
        
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,0] =  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,1] =  self.s__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,3] = - self.upperLegLength *  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,0] = - self.s__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,1] =  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,3] =  self.upperLegLength *  self.s__q_LH_KFE;
        
        
        
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,0] =  self.c__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,1] = - self.s__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[1,0] =  self.s__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[1,1] =  self.c__q_LH_KFE;
        
        
        
        self.fr_RH_hipassembly_Xh_fr_trunk[0,1] =  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[0,3] =  self.BASE2HAA_offset_y *  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,1] =  self.c__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_RH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RH_hipassembly[1,0] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[1,1] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[2,0] = - self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[2,1] =  self.s__q_RH_HAA;
        
        
        
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,0] =  self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,2] = - self.s__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,3] = - self.BASE2HAA_offset_z *  self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,0] = - self.s__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,2] = - self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,3] =  self.BASE2HAA_offset_z *  self.s__q_RH_HFE;
        
        
        
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,0] =  self.c__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,1] = - self.s__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[2,0] = - self.s__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[2,1] = - self.c__q_RH_HFE;
        
        
        
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,0] =  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,1] =  self.s__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,3] = - self.upperLegLength *  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,0] = - self.s__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,1] =  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,3] =  self.upperLegLength *  self.s__q_RH_KFE;
        
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[0,0] =  self.c__q_RH_KFE;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[0,1] = - self.s__q_RH_KFE;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[1,0] =  self.s__q_RH_KFE;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[1,1] =  self.c__q_RH_KFE;

    def update_jacobians(self, q):
#        self.s__q_LF_HAA = np.sin(q[1-1]);
#        self.s__q_LF_HFE = np.sin(q[2-1]);
#        self.s__q_LF_KFE = np.sin(q[3-1]);
#        self.c__q_LF_HAA = np.cos(q[1-1]);
#        self.c__q_LF_HFE = np.cos(q[2-1]);
#        self.c__q_LF_KFE = np.cos(q[3-1]);

        self.fr_trunk_J_LF_foot[2-1,2-1] =  self.c__q_LF_HAA;
        self.fr_trunk_J_LF_foot[2-1,3-1] =  self.c__q_LF_HAA;
        self.fr_trunk_J_LF_foot[3-1,2-1] = - self.s__q_LF_HAA;
        self.fr_trunk_J_LF_foot[3-1,3-1] = - self.s__q_LF_HAA;
        self.fr_trunk_J_LF_foot[4-1,2-1] = ( self.lowerLegLength *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.c__q_LF_HFE);
        self.fr_trunk_J_LF_foot[4-1,3-1] = ( self.lowerLegLength *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.c__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_J_LF_foot[5-1,1-1] = ( self.lowerLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LF_HAA);
        self.fr_trunk_J_LF_foot[5-1,2-1] = ( self.lowerLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE) + ( self.upperLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE);
        self.fr_trunk_J_LF_foot[5-1,3-1] = ( self.lowerLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_J_LF_foot[6-1,1-1] = (- self.lowerLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) + ( self.upperLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_LF_HAA);
        self.fr_trunk_J_LF_foot[6-1,2-1] = ( self.lowerLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE) + ( self.upperLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE);
        self.fr_trunk_J_LF_foot[6-1,3-1] = ( self.lowerLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);

        self.s__q_RF_HAA = np.sin(q[4-1]);
        self.s__q_RF_HFE = np.sin(q[5-1]);
        self.s__q_RF_KFE = np.sin(q[6-1]);
        self.c__q_RF_HAA = np.cos(q[4-1]);
        self.c__q_RF_HFE = np.cos(q[5-1]);
        self.c__q_RF_KFE = np.cos(q[6-1]);

        self.fr_trunk_J_RF_foot[2-1,2-1] =  self.c__q_RF_HAA;
        self.fr_trunk_J_RF_foot[2-1,3-1] =  self.c__q_RF_HAA;
        self.fr_trunk_J_RF_foot[3-1,2-1] =  self.s__q_RF_HAA;
        self.fr_trunk_J_RF_foot[3-1,3-1] =  self.s__q_RF_HAA;
        self.fr_trunk_J_RF_foot[4-1,2-1] = ( self.lowerLegLength *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.upperLegLength *  self.c__q_RF_HFE);
        self.fr_trunk_J_RF_foot[4-1,3-1] = ( self.lowerLegLength *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_J_RF_foot[5-1,1-1] = (- self.lowerLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( self.upperLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE) + ( self.BASE2HAA_offset_z *  self.c__q_RF_HAA);
        self.fr_trunk_J_RF_foot[5-1,2-1] = (- self.lowerLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE) - ( self.upperLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE);
        self.fr_trunk_J_RF_foot[5-1,3-1] = (- self.lowerLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_J_RF_foot[6-1,1-1] = (- self.lowerLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( self.upperLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RF_HAA);
        self.fr_trunk_J_RF_foot[6-1,2-1] = ( self.lowerLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE) + ( self.upperLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE);
        self.fr_trunk_J_RF_foot[6-1,3-1] = ( self.lowerLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);

        self.s__q_LH_HAA = np.sin(q[7-1]);
        self.s__q_LH_HFE = np.sin(q[8-1]);
        self.s__q_LH_KFE = np.sin(q[9-1]);
        self.c__q_LH_HAA = np.cos(q[7-1]);
        self.c__q_LH_HFE = np.cos(q[8-1]);
        self.c__q_LH_KFE = np.cos(q[9-1]);

        self.fr_trunk_J_LH_foot[2-1,2-1] =  self.c__q_LH_HAA;
        self.fr_trunk_J_LH_foot[2-1,3-1] =  self.c__q_LH_HAA;
        self.fr_trunk_J_LH_foot[3-1,2-1] = - self.s__q_LH_HAA;
        self.fr_trunk_J_LH_foot[3-1,3-1] = - self.s__q_LH_HAA;
        self.fr_trunk_J_LH_foot[4-1,2-1] = ( self.lowerLegLength *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.c__q_LH_HFE);
        self.fr_trunk_J_LH_foot[4-1,3-1] = ( self.lowerLegLength *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.c__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_J_LH_foot[5-1,1-1] = ( self.lowerLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LH_HAA);
        self.fr_trunk_J_LH_foot[5-1,2-1] = ( self.lowerLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE) + ( self.upperLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE);
        self.fr_trunk_J_LH_foot[5-1,3-1] = ( self.lowerLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_J_LH_foot[6-1,1-1] = (- self.lowerLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) + ( self.upperLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_LH_HAA);
        self.fr_trunk_J_LH_foot[6-1,2-1] = ( self.lowerLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE) + ( self.upperLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE);
        self.fr_trunk_J_LH_foot[6-1,3-1] = ( self.lowerLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);

        self.s__q_RH_HAA = np.sin(q[10-1]);
        self.s__q_RH_HFE = np.sin(q[11-1]);
        self.s__q_RH_KFE = np.sin(q[12-1]);
        self.c__q_RH_HAA = np.cos(q[10-1]);
        self.c__q_RH_HFE = np.cos(q[11-1]);
        self.c__q_RH_KFE = np.cos(q[12-1]);

        self.fr_trunk_J_RH_foot[2-1,2-1] =  self.c__q_RH_HAA;
        self.fr_trunk_J_RH_foot[2-1,3-1] =  self.c__q_RH_HAA;
        self.fr_trunk_J_RH_foot[3-1,2-1] =  self.s__q_RH_HAA;
        self.fr_trunk_J_RH_foot[3-1,3-1] =  self.s__q_RH_HAA;
        self.fr_trunk_J_RH_foot[4-1,2-1] = ( self.lowerLegLength *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.upperLegLength *  self.c__q_RH_HFE);
        self.fr_trunk_J_RH_foot[4-1,3-1] = ( self.lowerLegLength *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_J_RH_foot[5-1,1-1] = (- self.lowerLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( self.upperLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE) + ( self.BASE2HAA_offset_z *  self.c__q_RH_HAA);
        self.fr_trunk_J_RH_foot[5-1,2-1] = (- self.lowerLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE) - ( self.upperLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE);
        self.fr_trunk_J_RH_foot[5-1,3-1] = (- self.lowerLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_J_RH_foot[6-1,1-1] = (- self.lowerLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( self.upperLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RH_HAA);
        self.fr_trunk_J_RH_foot[6-1,2-1] = ( self.lowerLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE) + ( self.upperLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE);
        self.fr_trunk_J_RH_foot[6-1,3-1] = ( self.lowerLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        #print self.fr_trunk_J_LF_foot[3:6,:]
        
#        print self.fr_trunk_J_LF_foot, self.fr_trunk_J_RF_foot, self.fr_trunk_J_LH_foot, self.fr_trunk_J_RH_foot
        return self.fr_trunk_J_LF_foot[3:6,:] , self.fr_trunk_J_RF_foot[3:6,:], self.fr_trunk_J_LH_foot[3:6,:], self.fr_trunk_J_RH_foot[3:6,:]

    def getLegJacobians(self):
        return self.fr_trunk_J_LF_foot[3:6,:] , self.fr_trunk_J_RF_foot[3:6,:], self.fr_trunk_J_LH_foot[3:6,:], self.fr_trunk_J_RH_foot[3:6,:], False


    def forward_kin(self, q):
        LF_foot = self.fr_trunk_Xh_LF_foot[0:3,3]
        RF_foot = self.fr_trunk_Xh_RF_foot[0:3,3]
        LH_foot = self.fr_trunk_Xh_LH_foot[0:3,3]
        RH_foot = self.fr_trunk_Xh_RH_foot[0:3,3]
        
        contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        
        return contacts

    def leg_inverse_kin_ikpy(self, legID, footPositionBF):
        target_vector = np.array(footPositionBF[legID])
        target_frame = np.eye(4)
        target_frame[:3, 3] = target_vector
        if legID == 0:
            q = self.hyq_LF_chain.inverse_kinematics(target_frame)
        elif legID == 1:
            q = self.hyq_RF_chain.inverse_kinematics(target_frame)
        elif legID == 2:
            q = self.hyq_LH_chain.inverse_kinematics(target_frame)
        elif legID == 3:
            q = self.hyq_RH_chain.inverse_kinematics(target_frame)
        else:
            print( "warning: leg ID is wrong")
        q_leg = q[1:4]
        return q_leg

    def inverse_kin_ikpy(self, contactsBF):
        q = []
        for legID in self.dog.legs:
            q_leg = self.leg_inverse_kin_ikpy(legID, contactsBF[legID,:])
            q = np.hstack([q, q_leg])
        return q

    def leg_inverse_kin(self, legID, footPositionBF, footVelocityBF):
#        BASE2HAAX = 0.3735;  //!< x component of LF_HAA origin, in base frame
#        BASE2HAAY = 0.207;   //!< y component of LF_HAA origin, in base frame
#        HAA2HFE  = 0.08;  //!< distance between HFE to HAA, in the xz plane of the HAA frame
#1        HFE2KFEZ = -0.35;  //!< distance of HFE to KFE in z direction  [is this used? is this at HFE=0rad?]
#        
#        self.upperLegLength = 0.35;
#        self.lowerLegLength = 0.341;

#        self.BASE2HAA_offset_x = 0.3735;
#        self.BASE2HAA_offset_y = 0.207;
#        self.BASE2HAA_offset_z = 0.08;
        
        M_PI = np.pi

#    double getHAA_x() { return BASE2HAAX; }
#    double getHAA_y() { return BASE2HAAY; }
#    double getHAA_z() { return 0; }
#
#    double getDist_HAA_HFE() { return HAA2HFE; }
#    double getDist_HFE_KFE() { return upleg_length; }
#    double getFoot_x() { return pGetter.getValue_foot_x(); }

#        upleg_length  = 0.35;    #//!< length of upper leg
#        upleg_length_sqr  = upleg_length*upleg_length;
#
        haa_sign_flip = [1.0, 1.0, 1.0, 1.0]
        kfe_sign_flip = [1.0, 1.0, 1.0, 1.0]
        haaXOffset_sign_flip = [1.0, 1.0, 1.0, 1.0]
        haaYOffset_sign_flip = [1.0, 1.0, 1.0, 1.0]

        haa_sign_flip[self.dog.RF] = -1.0 
        haa_sign_flip[self.dog.RH] = -1.0
        kfe_sign_flip[self.dog.LH] = -1.0
        kfe_sign_flip[self.dog.RH] = -1.0
        
        haaXOffset_sign_flip[self.dog.LH] = -1.0; 
        haaXOffset_sign_flip[self.dog.RH] = -1.0;
        haaYOffset_sign_flip[self.dog.RF] = -1.0;
        haaYOffset_sign_flip[self.dog.RH] = -1.0;
 
#        print 'foot pos ', footPositionBF
        q_leg = [np.nan, np.nan, np.nan]
        foot_haa_frame = footPositionBF - [self.BASE2HAA_offset_x*haaXOffset_sign_flip[legID], self.BASE2HAA_offset_y*haaYOffset_sign_flip[legID], 0.0]
#        print self.BASE2HAA_offset_x, self.BASE2HAA_offset_y, self.BASE2HAA_offset_z
#        print foot_haa_frame
        
        
        
        haa2foot_yz = np.sqrt(np.square(foot_haa_frame[self.rbd.LY]) + np.square(foot_haa_frame[self.rbd.LZ])) 
        hfe2foot_yz = 0.0
        fe_plane_line = [0.0, foot_haa_frame[self.rbd.LY], foot_haa_frame[self.rbd.LZ]]
        upleg_length_sqr = self.upperLegLength*self.upperLegLength
        lowleg_length_sqr = self.lowerLegLength*self.lowerLegLength
        
        if foot_haa_frame[self.rbd.LZ] > 0:
#            The damn foot is above the HAA origin. Just invert the vector to go
#            back to the negative half-plane. That would not change the HAA angle
            fe_plane_line = - fe_plane_line;
#            In addition, the HFE-to-foot distance has to be computed like this
            hfe2foot_yz = haa2foot_yz + self.HAA2HFE;
        else:
            hfe2foot_yz = haa2foot_yz - self.HAA2HFE;
            
            
#            // HFE-to-foot distance, squared and plain
        hfe2foot_sqr = hfe2foot_yz * hfe2foot_yz + foot_haa_frame[self.rbd.LX] * foot_haa_frame[self.rbd.LX];
        hfe2foot = np.sqrt( hfe2foot_sqr );

    
        ''' HAA '''
        
        q_leg[self.dog.HAA] = - np.arctan2(fe_plane_line[self.rbd.LY], -fe_plane_line[self.rbd.LZ])  * haa_sign_flip[legID];
            
        ''' HFE '''
            
        temp = (upleg_length_sqr + hfe2foot_sqr - lowleg_length_sqr) / (2 * self.upperLegLength * hfe2foot);
#    temp = (temp > 1 ? 1 : (temp < -1 ? -1 : temp));    
        if temp > 1:
            temp = 1
        else:
            if temp < -1:
                temp = -1

#    // Foot X coordinate in the HAA frame; swap the sign if necessary to map it
#    //  to the LF_HAA frame, so that we can use the equations that work with the
#    //  geometry of the LF leg only
        foot_x = foot_haa_frame[self.rbd.LX] * kfe_sign_flip[legID];

#    // Distinguish whether the foot is above or below the HFE line in the FE
#    //  plane (the line passing through HFE at an angle of PI/2 in HFE conventions)
        if(haa2foot_yz < self.HAA2HFE):
            q_leg[self.dog.HFE] = - M_PI + np.arcsin( foot_x / hfe2foot ) + np.arccos( temp )
        else:
            q_leg[self.dog.HFE] = - np.arcsin( foot_x / hfe2foot ) + np.arccos( temp )
    
#    // Now swap the sign again to account for the actual leg
        q_leg[self.dog.HFE] = q_leg[self.dog.HFE]  * kfe_sign_flip[legID]; # // not an errors, use the same sign_flip of KFE
        
        ''' KFE '''
        
        twiceUpperLower = 2 * self.upperLegLength * self.lowerLegLength;
#    // Compute the argument of the arc cosine; because of numerical errors, it
#    //  might be slightly outside [-1,1], therefore I cap it.
        temp = (upleg_length_sqr + lowleg_length_sqr - hfe2foot_sqr) / twiceUpperLower;
#        temp = (temp > 1 ? 1 : (temp < -1 ? -1 : temp));
        if temp > 1:
            temp = 1
        else:
            if temp < -1:
                temp = -1
                
        q_leg[self.dog.KFE] = (-M_PI + np.arccos(temp)) * kfe_sign_flip[legID];
        
#       //Check if the outputs are inf or nan
        for joint in self.dog.legJoints:
            if not np.isfinite(q_leg[joint]):
                print ("Position of joint ",joint," and leg ", 0," is not finite !!!")
#            return false;
            
        return q_leg
    
    def fixedBaseInverseKinematics(self, contactsBF, foot_vel):
        q = []
        for legID in self.dog.legs:
            q_leg = self.leg_inverse_kin(legID, contactsBF[legID,:], foot_vel[legID,:])
            q = np.hstack([q, q_leg])

        self.update_homogeneous(q)
        self.update_jacobians(q)
        return q

    def isOutOfJointLims(self, joint_positions, joint_limits_max, joint_limits_min):

        no_of_legs_to_check = joint_positions.size/3
        q = joint_positions.reshape((no_of_legs_to_check, 3))
        # print "q: ", q
        # print "leq than max ", np.all(np.less_equal(q, joint_limits_max))
        # print "geq than min ", np.all(np.greater_equal(q, joint_limits_min))
        return not np.all(np.less_equal(q, joint_limits_max)) \
               or not np.all(np.greater_equal(q, joint_limits_min))

    def isOutOfWorkSpace(self, contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel):

        q = self.fixedBaseInverseKinematics(contactsBF_check, foot_vel)
        q_to_check = np.concatenate([list(q[leg * 3: leg * 3 + 3]) for leg in stance_index])

        return self.isOutOfJointLims(q_to_check, joint_limits_max[stance_index,:], joint_limits_min[stance_index,:])