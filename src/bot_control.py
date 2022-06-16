#!/usr/bin/env python
# -- coding: utf-8 --
import imp
import rospy
import time
# import numpy as np
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Bool, Empty
from move_base_msgs.msg import MoveBaseActionFeedback
from tf.msg import tfMessage
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from open_manipulator_msgs.srv import SetActuatorState

class bot:
    def __init__(self, offset):
        ## 구독자 선언
        self.current_pose = rospy.Subscriber('/tf', tfMessage, self.pose_callback, queue_size = 1)

        ## 발행자 선언
        self.motor_power_pub = rospy.Publisher('/motor_power', Bool, queue_size = 1)
        self.motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.reset_pub = rospy.Publisher('/reset', Empty, queue_size = 1)

        ## Sub 받은 데이터
        self.heading = 0.0
        self.pos = self.offset = offset # [x좌표, y좌표]
        self.can_state = [0.0, 0.0, 0.0]

    ########## callback 함수 #########
    ## manipulator's actuator 전원 On/Off
    def actuator_power(self, st):
        rospy.wait_for_service('/set_actuator_state')
        try:
            can_mission_srvs = rospy.ServiceProxy('/set_actuator_state', SetActuatorState)
            bak = can_mission_srvs(set_actuator_state=st)
        except:
            pass

    ## 뎁스카메라에서 측정된 캔의 위치와 판단 여부
    def can_callback(self, can):
        self.can_state = can.data

    ## 로봇의 위치 정보
    def pose_callback(self, tf):
        for t in tf.transforms:
            # base_footframe 을 로봇의 좌표계로 사용
            if t.child_frame_id == "base_footprint":
                self.pos = [t.transform.translation.x, t.transform.translation.y]
                orientation_list = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
                (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
                self.heading = yaw # heading을 rad으로 표현 / x축 기준 반시계방향 -pi ~ pi
            else: pass
        
    ##################################
    ## 로봇의 위치와 센서를 초기화
    def f_set(self):
        self.reset_pub.publish()
        time.sleep(2)
        self.motor_power_pub.publish(data = True)
        time.sleep(1)
        print("turtle is ready!!\n--------------------------------------------------------\n"),

    ## speed와 steer를 주면 로봇의 이동을 제어
    def move(self, v, steer):
        mt = Twist(linear=Vector3(v, 0, 0), angular=Vector3(0, 0, steer))
        self.motor_pub.publish(mt)