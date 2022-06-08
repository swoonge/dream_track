#!/usr/bin/env python
# -- coding: utf-8 --
from __future__ import print_function

import sys
import rospy
from open_manipulator_msgs.srv import SetJointPosition
import time
import numpy as np

from roboticstoolbox import ETS as ET, DHLink, DHRobot, RevoluteDH
from spatialmath.base import *
from spatialmath import SE3

pi = np.pi
theta0 = np.arctan2(0.024,0.178)
q0 = [0,0,0,0]

### x 0.188 y = 0.03 z =0.025 //offset:RefToCam

### invK
robot = DHRobot(
     [
          RevoluteDH(d=0.0765,alpha=-pi/2),
          RevoluteDH(a=0.178,offset=-pi/2+theta0),
          RevoluteDH(a=0.174,offset=pi/2-theta0),
          RevoluteDH(a=0.126)
     ]
)

def invK(x,y,z):
    T = [
        [1,0,0,x + 0.188],
        [0,1,0,y + 0.03],
        [0,0,1,z - 0.025],
        [0,0,0,1]
        ]
    T = np.array(T)
    T = SE3(T)
    sol = robot.ikine_LM(T,q0)
    return sol[0]

class Joint_position():
    def __init__(self, joint_name, position, max_accelerations_scaling_factor, max_velocity_scaling_factor):
        self.joint_name = joint_name
        self.position = position
        self.max_accelerations_scaling_factor = max_accelerations_scaling_factor
        self.max_velocity_scaling_factor = max_velocity_scaling_factor

### motor control
def movePoint(x,y,z,t):
    rospy.wait_for_service('/goal_joint_space_path')
    stateReq = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)

    j_name = ['joint1','joint2','joint3','joint4']
    theta = invK(x,y,z)
    # theta = [theta1,theta2,theta3,theta4]
    # theta = np.array(theta)
    # theta = theta*np.pi/180
    theta = list(theta)
    joint_position = Joint_position(j_name,theta,0,0)
    t = 3
    print(theta)
    respl = stateReq('',joint_position,t)

def initPose(pose='home'):
    rospy.wait_for_service('/goal_tool_control')
    stateReq = rospy.ServiceProxy('/goal_tool_control',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4']
    if pose=='home':
        theta = [0,-1.05,0.37,0.7]
        t = 5
    else:
        theta = [0,0.31,1,-1.22]
        t = 3

    joint_position = Joint_position(j_name,theta,0,0)
    respl = stateReq('',joint_position,t)

def standbyPose(t):
    rospy.wait_for_service('/goal_tool_control')
    stateReq = rospy.ServiceProxy('/goal_tool_control',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4','gripper']
    theta = [0,.69,1.269,-1.65,0.01]

    joint_position = Joint_position(j_name,theta,0,0)
    respl = stateReq('',joint_position,t)

def gripOpen():
    rospy.wait_for_service('/goal_tool_control')
    stateReq = rospy.ServiceProxy('/goal_tool_control',SetJointPosition)

    j_name = ['gripper']
    IsGrip = [0.01] ### 0.01 open, -0.01 close
    joint_position = Joint_position(j_name,IsGrip,0,0)
    respl = stateReq('',joint_position,3)

def gripClose():
    rospy.wait_for_service('/goal_tool_control')
    stateReq = rospy.ServiceProxy('/goal_tool_control',SetJointPosition)

    j_name = ['gripper']
    IsGrip = [-0.01] ### 0.01 open, -0.01 close
    joint_position = Joint_position(j_name,IsGrip,0,0)
    respl = stateReq('',joint_position,3)

def goPath():
    movePoint(0.22,0,0.02,t=1)
    time.sleep(2.5)
    gripClose()
    movePoint(0.09,0,0.15,t=1.8)
    time.sleep(5)
    standbyPose(4)

### Warning : radian, m ###
### Warning : radian, m ###
### Warning : radian, m ###
if __name__ == "__main__":
    goPath()