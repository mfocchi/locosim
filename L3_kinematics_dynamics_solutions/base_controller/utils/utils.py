
from roslib.scriptutil import get_param_server
from roslib.names import ns_join, get_ros_namespace, make_caller_id
import socket

import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import rospy as ros

class Utils:

    def __init__(self):
        self.leg_map = {
            "LF": 0,
            "RF": 1,
            "LH": 2,
            "RH": 3
        }
        self.crd = {
            "X": 0,
            "Y": 1,
            "Z": 2,
        }


        self.sp_crd = {
            "LX": 0,
            "LY": 1,
            "LZ": 2,
            "AX": 3,
            "AY": 4,
            "AZ": 5,
        }



    def getSegment(self, var, index, size):
        return var[index:index+size]

    def linPart(self, var):
        index = self.sp_crd["LX"]
        return var[index:index+3]

    def angPart(self, var):
        index = self.sp_crd["AX"]
        return var[index:index+3]

    ########################################################################

    #manage param server

    ########################################################################



    def succeed(self,args):
        """
        Utility routine for checking ROS XMLRPC API call
        @return: value field from ROS xmlrpc call
        @rtype: XmlRpcLegalValue
        @raise ROSParamException: if call to ROS xmlrpc API did not succeed
        """
        code, msg, val = args
        if code != 1:
            raise ROSParamException(msg)
        return val

    def get_caller_id(self):
        """
        @return: caller ID for rosparam ROS client calls
        @rtype: str
        """
        return make_caller_id('rosparam-%s' % os.getpid())

    def putIntoParamServer(self, data):

        try:
            self.succeed(get_param_server().setParam(self.get_caller_id(),  "hyq", data))

        except socket.error:
            raise ROSParamIOException("Unable to communicate with master!")
        print "set parameter [%s] to [%s]" % ('hyq', data)
        eval(help( get_param_server().setParam()) )								
        pass

    def putIntoGlobalParamServer(self,label, data):

        try:
            ros.set_param(label, data)
        except socket.error:
            raise ROSParamIOException("Unable to communicate with master!")
        print "set parameter %s into global param server" % label
        pass

#########################################################################

    def getIdx(self, leg, coord):
        return self.leg_map[leg]*3 + self.crd[coord]

    def setLegJointState(self, legid,  input, jointState):

        jointState[self.leg_map[legid]*3:self.leg_map[legid]*3+3] = input

    def setLegJointState(self, legid,  input, jointState):
        jointState[legid*3:legid*3+3] = input

    def getLegJointState(self, legid,  jointState):
        return jointState[self.leg_map[legid]*3:self.leg_map[legid]*3+3]

    def getLegJointState(self, legid,  jointState):
        return jointState[legid*3:legid*3+3]

    def spy(self, var):
        plt.spy(var)
        plt.show()


    def detectLiftOff(self, swing,idx, leg):
        if ((swing[leg, idx-1] == 0) and (swing[leg, idx] == 1)):

            return True
        else:

            return False

    def detectTouchDown(self, swing,idx, leg):
        if ((swing[leg, idx] == 1) and (swing[leg, idx+1] == 0)):

            return True
        else:

            return False

    def detectHapticTouchDown(self, grForcesW, leg, force_th):
        grfleg = self.getLegJointState(leg, grForcesW)
        if grfleg[2]>=force_th:
            return True
        else:
            return False

    def mapFromRos(self, ros_in):
        ros_out = np.zeros_like(ros_in)
        ros_out[0:3] = ros_in[0:3] 
        ros_out[3:6] = ros_in[6:9] 
        ros_out[6:9] = ros_in[3:6] 
        ros_out[9:12] = ros_in[9:12] 
        return ros_out