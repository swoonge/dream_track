#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import time
# import numpy as np
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Bool, Empty
from tf.msg import tfMessage
from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import LaserScan

class bot:
    def __init__(self, offset):
        # 구독자 선언
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1) # 라이다
        self.deg_sub = rospy.Subscriber('/tf', tfMessage, self.tf_callback, queue_size = 1) # (x, y)좌표

        self.motor_power_pub = rospy.Publisher('/motor_power', Bool, queue_size = 1)
        self.motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1) # 모터
        self.reset_pub = rospy.Publisher('/reset', Empty, queue_size = 1)

        # Sub 받은 데이터
        self.sub_scan = [0.0, 0.0]
        self.heading = 0.0
        self.pos = self.offset = offset
        self.orientation_q = Quaternion(0,0,0,1)

    ########## callback 함수 #########
    def scan_callback(self, scan):
        self.sub_scan = scan.ranges

    def tf_callback(self, pos):
        self.pos = [pos.transforms[0].transform.translation.x + self.offset[0], pos.transforms[0].transform.translation.y + self.offset[1]]

        orientation = pos.transforms[0].transform.rotation
        self.orientation_q = Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
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