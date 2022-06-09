#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import numpy as np
import time
import math
from std_msgs.msg import Float64MultiArray
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from dbscan import *

class Map():
    def __init__(self):
        # self.map_update()
        self.can_sub = rospy.Subscriber('/can_position_state', Float64MultiArray, self.can_callback, queue_size = 1) # (x, y, state)좌표
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)

        self.mission_state = 0
        self.maching_can_pos = [0.0, 0.0]
        self.sub_scan = list()
        self.DB_can_cluster = DBSCAN(0.05, 3)
    
    def matching_clear(self):
        self.mission_state = 0
        self.maching_can_pos = [0.0, 0.0]
        self.sub_scan = list()

    def can_callback(self, can): #sub 받는 캔의 위치는 리얼센스 위치 기준. 나는 추적 기준이니까 로봇의 중심 좌표 기준으로 변환해주자
        sub_can_pos = [can.data[0]+0.188, can.data[1]-0.03] # offset = x좌표 기준 캠 위치 - 중심위치
        if can.data[2] == 2 or self.mission_state == 2: self.mission_state = 2
        if can.data[2] == 1: #2였다면 위에 if문에서 미리 바꼈으니 1인경우 
            self.maching_can_pos = self.matching_point(sub_can_pos, 0.1)
            if self.maching_can_pos == [0.0,0.0]: 
                self.maching_can_pos = sub_can_pos
                self.mission_state = 1
        elif can.data[2] == 0:
            if self.mission_state == 1:
                self.maching_can_pos = self.matching_point(self.maching_can_pos, 0.1)
                if self.maching_can_pos == [0.0,0.0]:
                    self.mission_state = 0

            
    def scan_callback(self, scan):
        self.sub_scan = list(scan.ranges)[:]
        self.get_obj()
        
    def tf_tm(self, dis, i):
        obs_y = dis*math.sin(np.deg2rad(float(i)))
        obs_x = dis*math.cos(np.deg2rad(float(i))) + 0.03 ## 이거도 라이다랑 로봇 중심좌표 거리로 바꾸자
        return [obs_x, obs_y]

    def tf_scan_to_xy(self):
        obs = list()
        for i, dis in enumerate(self.sub_scan):
            if 90 < i and i < 270 and 0.2 < dis and dis < 1.0: obs.append(self.tf_tm(dis, i))
        if len(obs) < 2: obs = [[0,0]]
        return obs

    def get_obj(self):
        self.DB_can_cluster.run(np.array(self.tf_scan_to_xy()))
            
    def matching_point(self, point, R):
        dist = list()
        for c_avg in self.DB_can_cluster.cluster_avg:
            dist.append(np.linalg.norm([c_avg[0]-point[0], c_avg[1]-point[1]]))
        matching_min_pos = self.DB_can_cluster.cluster_avg[dist.index(min(dist))]
        maching_pos = matching_min_pos if min(dist) < R else [0.0, 0.0]         
        return maching_pos

    def map_update(self):
        rospy.wait_for_service('/dynamic_map')
        try:
            call_map = rospy.ServiceProxy('/dynamic_map', GetMap)
            resp_map = call_map()
            self.raw_map = resp_map
            print("Map ready")
        except:
            print("Map call failed")

def main():
    rospy.Rate(2)
    map = Map()
    while not rospy.is_shutdown():
        print("map test")
        rospy.sleep()

if __name__ == '__main__':
    rospy.init_node('bot_map',anonymous=True)
    main()