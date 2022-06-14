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
        # 구독자 선언
        self.current_pose = rospy.Subscriber('/tf', tfMessage, self.pose_callback, queue_size = 1) # (x, y)좌표
        # self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)

        self.motor_power_pub = rospy.Publisher('/motor_power', Bool, queue_size = 1)
        self.motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1) # 모터
        self.reset_pub = rospy.Publisher('/reset', Empty, queue_size = 1)

        # Sub 받은 데이터
        self.heading = 0.0
        
        # self.scan_raw = []
        self.pos = self.offset = offset
        self.can_state = [0.0, 0.0, 0.0]
        
        self.move(0.0, 0.0)

    ########## callback 함수 #########
    def actuator_power(self, st):
        rospy.wait_for_service('/set_actuator_state')
        try:
            can_mission_srvs = rospy.ServiceProxy('/set_actuator_state', SetActuatorState)
            bak = can_mission_srvs(set_actuator_state=st)
        except:
            pass

    def can_callback(self, can):
        self.can_state = can.data

    # def scan_callback(self, scan):
    #     self.scan_raw = scan.ranges

    def pose_callback(self, tf):
        for t in tf.transforms:
            if t.child_frame_id == "base_footprint":
                self.pos = [t.transform.translation.x, t.transform.translation.y]
            else: pass
        # ori = [fb.feedback.base_position.pose.orientation.x  + self.offset[0], pos.transforms[0].transform.translation.y + self.offset[1]]
        orientation_list = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.heading = yaw # heading을 rad으로 표현 / x축 기준 반시계방향 -pi ~ pi
    ##################################

    def f_set(self):
        self.reset_pub.publish()
        time.sleep(2)
        self.motor_power_pub.publish(data = True)
        time.sleep(1)
        print("turtle is ready!!\n--------------------------------------------------------\n"),

    def move(self, v, steer):
        mt = Twist(linear=Vector3(v, 0, 0), angular=Vector3(0, 0, steer))
        self.motor_pub.publish(mt)