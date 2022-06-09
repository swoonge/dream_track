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
        self.deg_sub = rospy.Subscriber('/can_position_state', Float64MultiArray, self.can_callback, queue_size = 1) # (x, y, state)좌표
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.can_pos_state = [0.0, 0.0, 0.0] # x,y,count
        self.can_pos = [0.0, 0.0]
        self.obj = list()
        self.scan0 = list()
        self.scan1 = list()
        self.scan2 = list()
        self.DB_can_cluster = DBSCAN(0.05, 3)

    def can_callback(self, can):
        self.can_pos_state = can.data # moving averge can pos
        if self.can_pos_state[2] > 0:
            self.can_pos = self.can_pos_state[0:2]

    def scan_callback(self, scan):
        self.scan1 = self.scan0.copy()
        self.scan2 = self.scan1.copy()
        self.scan0 = scan.ranges
        self.get_obj()
        if self.can_pos != 0:
            self.trace_can()

    def tf_tm(self, dis, i):
        obs_y = dis*math.sin(np.deg2rad(float(i)))
        obs_x = dis*math.cos(np.deg2rad(float(i)))
        return [obs_x + 0.05, obs_y]

    def tf_scan_to_xy(self, scan_data):
        obs = list()
        for i, dis in enumerate(scan_data):
            if 0.2 < dis and dis < 3.0:
                obs.append(self.tf_tm(dis, i))
        return obs

    def get_obj(self):
        obs = self.tf_scan_to_xy(self.scan0 + self.scan1 + self.scan2)
        self.DB_can_cluster.run(np.array(obs))

    def trace_can(self):
        dist = list()
        for x in self.DB_can_cluster.cluster_avg:
            dist.append(np.sqrt(np.sum(((self.can_pos - x)**2),2)))
        if self.can_pos_state[2] > 0:
            self.can_pos = self.DB_can_cluster.cluster_avg[dist.index(min(dist))]
            # nearist self.DB_can_cluster point matching
        else:
            if min(dist) < 0.05:
                self.can_pos = self.DB_can_cluster.cluster_avg[dist.index(min(dist))]
            # past matching point near in 5cm near point tracking
            else: # can lost
                self.can_pos = [0.0, 0.0]
        
    def map_update(self):
        rospy.wait_for_service('/dynamic_map')
        try:
            call_map = rospy.ServiceProxy('/dynamic_map', GetMap)
            resp_map = call_map()
            self.raw_map =  resp_map
            print("Map ready")
        except:
            print("Map call failed")

def main():
    bot_map = Map()
    time.sleep(1)
    bot_map.map_update()
    print(bot_map.raw_map)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('bot_map',anonymous=True)
    main()