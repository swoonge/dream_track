#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import numpy as np
import time
from dbscan import *
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from dream_track.srv import can_move

def mission_point_client(can_pos):
    rospy.wait_for_service('/can_mission_point')
    try:
        can_mission_srvs = rospy.ServiceProxy('/can_mission_point', can_move)
        bak = can_mission_srvs(x = can_pos[0], y = can_pos[1])
        print(bak.suc)
    except:
        pass

def main():
    # a = [1.0,2.0,0]
    # b = [[1.3,2.2],[2.1,0.8],[3,2.0]]
    # dist = list()
    # c=a.copy
    # print("dasfdasf",a[0:2])
    # for c_avg in b:
    #     print("dasfdasf",c_avg)
    #     dist.append(np.linalg.norm([c_avg[0] - a[0],c_avg[1] - a[1]]))
    # print(dist)
    # a = [[1.0,2.0],[2.0,1.0],[1.5,2.0],[5.0,5.0],[4.5,4.7]]
    # b = [[1.3,2.2],[2.1,0.8],[3,2.0]]
    # data = np.array(a+b)
    # data = [[]]
    # DB_test = DBSCAN(1.0, 2)
    # DB_test.run(data)
    # DB_test.plot()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        mission_point_client([3.0, 5.0])

if __name__ == '__main__':
    rospy.init_node('test1',anonymous=True)
    main()