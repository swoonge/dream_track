#!/usr/bin/env python
# -- coding: utf-8 --
from __future__ import print_function

import sys
import rospy
from open_manipulator_msgs.srv import SetJointPosition
from detection_msgs.msg import BoundingBox,BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np

from roboticstoolbox import ETS as ET, DHLink, DHRobot, RevoluteDH
from spatialmath.base import *
from spatialmath import SE3

### initialize
pi = np.pi
theta0 = np.arctan2(0.024,0.178)
X_coord, Y_coord, Z_coord = 1,1,1

### manipualtor DH parameter
robot = DHRobot(
     [
          RevoluteDH(d=0.0765,alpha=-pi/2),
          RevoluteDH(a=0.178,offset=-pi/2+theta0),
          RevoluteDH(a=0.174,offset=pi/2-theta0),
          RevoluteDH(a=0.126)
     ]
)

### inverse kinematics
def invK(x,y,z):
    T = [
        [1,0,0,x + 0.188], ###
        [0,1,0,y + 0.035], ### offset
        [0,0,1,z - 0.005], ###
        [0,0,0,1]
        ]
    T = np.array(T)
    T = SE3(T)
    # fkine = robot.fkine([0,0.76,1.15,-.15])
    # print(fkine)
    q0 = [0,0,0,0]
    sol = robot.ikine_LM(T,q0,ilimit=50)
    return sol[0]

class Joint_position():
    def __init__(self, joint_name, position, max_accelerations_scaling_factor, max_velocity_scaling_factor):
        self.joint_name = joint_name
        self.position = position
        self.max_accelerations_scaling_factor = max_accelerations_scaling_factor
        self.max_velocity_scaling_factor = max_velocity_scaling_factor

###########################
###### motor control ######
###########################
def movePoint(x,y,z,t):
    rospy.wait_for_service('/goal_joint_space_path')
    stateReq = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4']
    x = x*1.1 ### end effector에 대한 보정
    y = y*1.1 ###
    theta = invK(x,y,z)
    print(theta)
    theta = list(theta)
    theta[0] = theta[0]*1.17 ### theta1 calibration. 사유 : 종합적인 오차로 인해 theta1이 조금 모자라게 나타나서
    thetaRot = [theta[0],.665,.71,-1]

    joint_position = Joint_position(j_name,thetaRot,0,0)
    respl = stateReq('',joint_position,t)
    time.sleep(1) ### 1번 모터 우선 회전 후 물체 집기

    joint_position = Joint_position(j_name,theta,0,0)
    respl = stateReq('',joint_position,t)
    print("move to Point")

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
    print("initPose")

def standbyPose(t):
    rospy.wait_for_service('/goal_joint_space_path')
    stateReq = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4']
    theta = [0,.665,.71,-1] # stanby pose

    joint_position = Joint_position(j_name,theta,0,0)
    respl = stateReq('',joint_position,t)
    print("standbyPose")

def collectionPose():
    rospy.wait_for_service('/goal_joint_space_path')
    stateReq = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
    j_name = ['joint1','joint2','joint3','joint4']  
    theta = [-3.142,0.6,1.43,-1.68]
    print("collectionPose")

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
    print("turnPose")

def gripOpen():
    rospy.wait_for_service('/goal_tool_control')
    stateReq = rospy.ServiceProxy('/goal_tool_control',SetJointPosition)
    j_name = ['gripper']
    IsGrip = [0.01] ### 0.01 open, -0.01 close

    joint_position = Joint_position(j_name,IsGrip,0,0)
    respl = stateReq('',joint_position,0.5)
    print("gripOpen")

def gripClose():
    rospy.wait_for_service('/goal_tool_control')
    stateReq = rospy.ServiceProxy('/goal_tool_control',SetJointPosition)
    j_name = ['gripper']
    IsGrip = [-0.01] ### 0.01 open, -0.01 close

    joint_position = Joint_position(j_name,IsGrip,0,0)
    respl = stateReq('',joint_position,0.5)
    print("gripClose")

def goPath(x,y,z):
    standbyPose(2) # 준비 자세
    movePoint(x,y,z,t=2.5) # 캔 이동
    time.sleep(1.7)
    gripClose() # 캔 집기
    time.sleep(.8)
    initPose('home') # home pose
    time.sleep(1.5)
    turnPose() # home turn pose
    time.sleep(.8)
    collectionPose() # 수거함
    time.sleep(2.5)
    gripOpen() # 캔 놓기
    turnPose() # home turn pose
    time.sleep(.4)
    initPose('home') # home pose , 대기

# 잡을 캔 종류
catchLabel = ["gatorade",
              "letsbe",
              "milkis",
              "powerade",
              "vita500"]

# 안 잡을 캔 종류
noncatchLabel =["pepsi_zero"]

# 잡을 캔인지
def isCatch(label):
    for i in range(len(catchLabel)):
        if label==catchLabel[i]:
            return 1
    return 0

def bbox(data):
    global can_st_pub
    global Z_coord
    c_Z = 239
    c_Y = 319

    ### bounding boxes 감지된 경우
    for box in data.bounding_boxes:
        can_st = [0.0,0.0,0.0]
        center_y = (box.ymin+box.ymax)//2
        center_x = (box.xmin+box.xmax)//2
        depth_array = np.array(depth_image, dtype=np.float32)/1000 # mm to m
        Y_pix = center_x - c_Y
        Z_pix = center_y - c_Z
        theta = np.arctan2(np.sqrt(Y_pix**2+Z_pix**2),640)

        ### 좌표 추정
        X_coord = np.dot((depth_array[center_y, center_x]+0.0375),np.cos(theta)) # projection
        Y_coord = -1*Y_pix/640*X_coord
        Z_coord = -1*Z_pix/640*X_coord

        rospy.loginfo(
            "\n Obj : {} \n depth : {:.3f}m \n X : {:.3f}m \n Y : {:.3f}m \n Z : {:.3f}m \n".format(
                box.Class, depth_array[center_y, center_x], X_coord, Y_coord, Z_coord
            )
        )

        if isCatch(box.Class): 
            objDist = np.sqrt(np.square(X_coord)+np.square(Y_coord)+np.square(Z_coord))
            can_st[0:2] = X_coord*1.1, Y_coord*1.1
            ### 물체의 거리 15~27cm 이내인 경우
            if objDist < 0.242+0.018 and objDist > 0.15: # state 2
                can_st[2] = 2
                can_st_pub.publish(data = can_st)
                goPath(X_coord,Y_coord,Z_coord)
                can_st = [0.0,0.0,0.0]
                can_st_pub.publish(data = can_st)
                print("go through the path")
            ### 물체의 거리가 해당 영역을 벗어난 경우
            else: # state 1
                can_st[2] = 1
                can_st_pub.publish(data = can_st)
        # if isCatch(box.Class)==0: # state 0
        #     print("non-catch label")
        #     can_st[2] = 0
        #     can_st_pub.publish(data = can_st)
    ### bounding boxes 감지되지 않은 경우
    if len(data.bounding_boxes)==0:
        can_st = [0.0,0.0,0.0]
        can_st_pub.publish(data = can_st)
        
# /can_position_state // x,y 

def convert_depth_image(image):
    bridge = CvBridge()
    try:
        global depth_image
        depth_image = bridge.imgmsg_to_cv2(image, image.encoding)
    except CvBridgeError as e:
        print(e)

def ctrl():
    rospy.init_node('ctr_node', anonymous=True)
    global can_st_pub
    can_st_pub = rospy.Publisher('/can_position_state',Float64MultiArray,queue_size=1)
    rospy.Subscriber('/yolov5/detections',BoundingBoxes,bbox,queue_size=1)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,convert_depth_image,queue_size=1)
    rospy.spin()

### Warning : radian, m ###
### Warning : radian, m ###
### Warning : radian, m ###
if __name__ == "__main__":
    ctrl()

###