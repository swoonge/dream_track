#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import math
import numpy as np
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose,Vector3
from tf.transformations import euler_from_quaternion
from scipy.spatial import distance
from bot_control import *

from tf.msg import tfMessage
from nav_msgs.msg import Path

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw

class Pp:
    def __init__(self):
        # 구독자 선언
        self.plan_sub = rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, self.plan_callback, queue_size = 1)
        self.a_point_rviz_pub = rospy.Publisher('/a_point_rviz', Marker, queue_size=3)
        self.c_point_rviz_pub = rospy.Publisher('/c_point_rviz', Marker, queue_size=3)

        # Sub 받은 데이터
        # self.pos = [0.0, 0.0]
        # self.heading = 0.0
        self.plan = [[0.0, 0.0]]
        self.Ld = 5
        self.S = 0

    ########## callback 함수 #########
    def plan_callback(self, plan):
        new_plan = []
        # print(plan.poses[0])
        for xp in plan.poses:
            # print(xp.pose.position.x)
            new_plan.append([xp.pose.position.x, xp.pose.position.y])
        self.plan = new_plan[:]
    ##################################

    def min_dis_ind(self, data):
        d_min = float('inf'); min_ind = 0
        for idp, p in enumerate(self.plan):
            d = distance.euclidean(data, p)
            if  d <= d_min:
                min_ind = idp
                d_min = d
        return min_ind, d

    # def find_S(self, pose):#(x,y)
    #     self.min_dis_ind(pose)
    #     a_point = self.plan[self.min_dis_ind(pose)]

    def pur_p(self, pose, heading):
        min_ind, D = self.min_dis_ind(pose)
        self.a_point_viz(self.plan[min_ind+self.Ld])
        self.c_point_viz(pose, heading)
        rad_to_next = math.atan2(self.plan[min_ind+self.Ld][0] - pose[0], self.plan[min_ind+self.Ld][1] - pose[1])
        speed = 6*D+0.02 if D < 0.1 else 0.08
        return speed, (rad_to_next - heading)*0.5

    def a_point_viz(self, point):
        pose = Pose()
        pose.orientation.x=0.0
        pose.orientation.y=0.0
        pose.orientation.z=0.0
        pose.orientation.w=1.0

        pose.position.x=point[0]
        pose.position.y=point[1]
        pose.position.z=0

        rviz_apoint=Marker(
            header=Header(frame_id='map',stamp=rospy.get_rostime()),
            ns="apoint",
            id=100,
            type=Marker.CUBE,
            lifetime=rospy.Duration(),
            action=Marker.ADD,
            pose=pose,
            scale=Vector3(x=0.05,y=0.05,z=0.05),
            color=ColorRGBA(r=1.0,g=0.7,b=0.0,a=1.0)
            )

        self.a_point_rviz_pub.publish(rviz_apoint)

    def c_point_viz(self, point, heading):
        pose = Pose()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = euler_to_quaternion(0,0,heading)

        pose.position.x=point[0]
        pose.position.y=point[1]
        pose.position.z=0

        rviz_cpoint=Marker(
            header=Header(frame_id='map',stamp=rospy.get_rostime()),
            ns="cpoint",
            id=101,
            type=Marker.ARROW,
            lifetime=rospy.Duration(),
            action=Marker.ADD,
            pose=pose,
            scale=Vector3(x=0.2,y=0.05,z=0.05),
            color=ColorRGBA(r=0.2,g=0.0,b=7.0,a=1.0)
            )

        self.c_point_rviz_pub.publish(rviz_cpoint)

def main():
    rate = rospy.Rate(5)
    pp = Pp()
    Turtle = bot([0.0, 0.0])

    while not rospy.is_shutdown():
        print(Turtle.heading)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Pp',anonymous=True)
    main()