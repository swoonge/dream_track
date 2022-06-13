#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time
# import numpy as np
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Bool, Empty
from tf.msg import tfMessage
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from open_manipulator_msgs.srv import SetActuatorState
from nav_msgs.msg import Path

class Pp:
    def __init__(self):
        # 구독자 선언
        self.pos_sub = rospy.Subscriber('/tf', tfMessage, self.tf_callback, queue_size = 1) # (x, y)좌표
        self.plan_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.plan_callback, queue_size = 1)

        # Sub 받은 데이터
        self.pos = [0.0, 0.0]
        self.heading = 0.0
        self.plan = [[0.0, 0.0]]

    ########## callback 함수 #########
    def plan_callback(self, plan):
        # p = [plan.poses.pose.position.x, plan.poses.pose.position.y]
        print(plan.poses.pose.position.x)

    def tf_callback(self, pos):
        # print(pos)
        try:
            # print(pos)
            self.pos = [pos.transforms[0].transform.translation.x + self.offset[0], pos.transforms[0].transform.translation.y + self.offset[1]]
            orientation_list = [pos.transforms[0].transform.rotation.x, pos.transforms[0].transform.rotation.y, pos.transforms[0].transform.rotation.z, pos.transforms[0].transform.rotation.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            self.heading = yaw # heading을 rad으로 표현 / x축 기준 반시계방향 -pi ~ pi
            # print(yaw)
        except:
            pass
    ##################################


def main():
    rate = rospy.Rate(5)
    pp = Pp()

    while not rospy.is_shutdown():

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Pp',anonymous=True)
    main()