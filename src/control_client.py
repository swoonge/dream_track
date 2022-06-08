#!/usr/bin/env python
# -- coding: utf-8 --
from __future__ import print_function

import sys
import rospy
from open_manipulator_msgs.srv import SetJointPosition
from detection_msgs.msg import BoundingBox,BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np

from roboticstoolbox import ETS as ET, DHLink, DHRobot, RevoluteDH
from spatialmath.base import *
from spatialmath import SE3

pi = np.pi
theta0 = np.arctan2(0.024,0.178)
q0 = [0,0,0,0]
X_coord, Y_coord, Z_coord = 1,1,1
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
        [0,1,0,y + 0.035],
        [0,0,1,z - 0.025],
        [0,0,0,1]
        ]
    T = np.array(T)
    T = SE3(T)
    initT = [
        [1,0,0,0.1964],
        [0,1,0,0],
        [0,0,1,-0.02633],
        [0,0,0,1]
        ]
    # fkine = robot.fkine([0,0.76,1.15,-.15])
    # print(fkine)
    q0 = [0,0,0,0]
    sol = robot.ikine_LM(T,q0,ilimit=100)
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
    x = x*1.1
    y = y*1.1
    theta = invK(x,y,z)
    # theta = [theta1,theta2,theta3,theta4]
    # theta = np.array(theta)
    # theta = theta*np.pi/180
    print(theta)
    theta = list(theta)
    theta[0] = theta[0]*1.17 ### calibration
    thetaRot = [theta[0],.76,1.15,-1.5]
    joint_position = Joint_position(j_name,thetaRot,0,0)
    respl = stateReq('',joint_position,t)
    time.sleep(1)
    joint_position = Joint_position(j_name,theta,0,0)
    respl = stateReq('',joint_position,t)

def initPose(pose='home'):
    rospy.wait_for_service('/goal_joint_space_path')
    stateReq = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4']
    if pose=='home':
        theta = [0,-1.05,0.37,0.7]
        t = 3.2
    else:
        theta = [0,0.31,1,-1.22]
        t = 3

    joint_position = Joint_position(j_name,theta,0,0)
    respl = stateReq('',joint_position,t)
    print("init")

def standbyPose(t):
    rospy.wait_for_service('/goal_joint_space_path')
    stateReq = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4']
    theta = [0,.76,1.15,-1.5]

    joint_position = Joint_position(j_name,theta,0,0)
    respl = stateReq('',joint_position,t)

def collectionPose():
    rospy.wait_for_service('/goal_joint_space_path')
    stateReq = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4']  
    theta = [-3.142,0.6,1.43,-1.68]
    joint_position = Joint_position(j_name,theta,0,0)
    t = 3
    respl = stateReq('',joint_position,t)

def turnPose():
    rospy.wait_for_service('/goal_joint_space_path')
    stateReq = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4']  
    theta = [-3.142,-1.05,0.37,0.7]
    joint_position = Joint_position(j_name,theta,0,0)
    t = 3.6
    respl = stateReq('',joint_position,t)

def gripOpen():
    rospy.wait_for_service('/goal_tool_control')
    stateReq = rospy.ServiceProxy('/goal_tool_control',SetJointPosition)

    j_name = ['gripper']
    IsGrip = [0.01] ### 0.01 open, -0.01 close
    joint_position = Joint_position(j_name,IsGrip,0,0)
    respl = stateReq('',joint_position,0.5)

def gripClose():
    rospy.wait_for_service('/goal_tool_control')
    stateReq = rospy.ServiceProxy('/goal_tool_control',SetJointPosition)

    j_name = ['gripper']
    IsGrip = [-0.01] ### 0.01 open, -0.01 close
    joint_position = Joint_position(j_name,IsGrip,0,0)
    respl = stateReq('',joint_position,0.5)

# def goPath(x,y,z):
#     gripOpenClient()
#     jointReqClient(0.09,0,0.15,t=1.2)
#     stateReqClient(0.09,0,0.15,t=3)
#     stateReqClient(x,y,z,t=1)
#     time.sleep(2)
#     gripCloseClient()
#     stateReqClient(0.09,0,0.15,t=1.8)
#     time.sleep(3)
#     gripOpenClient()

def goPath(x,y,z):
    standbyPose(2)
    movePoint(x,y,z,t=2.5) # offset for depth, 
    time.sleep(1.7)
    gripClose()
    time.sleep(.8)
    initPose('home')
    time.sleep(1.5)
    turnPose()
    time.sleep(.8)
    collectionPose()
    time.sleep(2.5)
    gripOpen()
    turnPose()
    time.sleep(.4)
    initPose('home')
    # gripOpen()

def bbox(data):
    c_Z = 239
    c_Y = 319

    for box in data.bounding_boxes:
        center_y = (box.ymin+box.ymax)//2
        center_x = (box.xmin+box.xmax)//2
        depth_array = np.array(depth_image, dtype=np.float32)/1000 # mm to m
        Y_pix = center_x - c_Y
        Z_pix = center_y - c_Z
        theta = np.arctan2(np.sqrt(Y_pix**2+Z_pix**2),640)

        ### can depth offset
        X_coord = np.dot((depth_array[center_y, center_x]+0.0375),np.cos(theta)) # projection
        Y_coord = -1*Y_pix/640*X_coord
        Z_coord = -1*Z_pix/640*X_coord

        rospy.loginfo(
            "\n Obj : {} \n depth : {:.3f}m \n X : {:.3f}m \n Y : {:.3f}m \n Z : {:.3f}m \n".format(
                box.Class, depth_array[center_y, center_x], X_coord, Y_coord, Z_coord
            )
        )
        isCatch = np.sqrt(np.square(X_coord)+np.square(Y_coord)+np.square(Z_coord))
        if isCatch < 0.242 and isCatch > 0.15:
            goPath(X_coord,Y_coord,Z_coord)
            print("go through the path")

def convert_depth_image(image):
    bridge = CvBridge()
    try:
        global depth_image
        depth_image = bridge.imgmsg_to_cv2(image, image.encoding)
    except CvBridgeError as e:
        print(e)

def ctrl():
    rospy.init_node('ctr_node', anonymous=True)
    rospy.Subscriber('/yolov5/detections',BoundingBoxes,bbox,queue_size=1)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,convert_depth_image,queue_size=1)
    rospy.spin()

### Warning : radian, m ###
### Warning : radian, m ###
### Warning : radian, m ###
if __name__ == "__main__":
    ctrl()

###
