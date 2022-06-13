#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import numpy as np
import math
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float64MultiArray, Header
# from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from dbscan import *

class Map():
    def __init__(self):
        # self.map_update()
        self.can_sub = rospy.Subscriber('/can_position_state', Float64MultiArray, self.can_callback, queue_size = 1) # (x, y, state)좌표
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size = 1)

        self.goal_pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)

        self.mission_state = 0
        self.matching_can_pos = [0.0, 0.0]
        self.sub_scan = list()
        self.DB_can_cluster = DBSCAN(0.05, 3)
        self.goal_seq = 0
        self.res = 0.05
        self.offset = [30, 30]
        # self.slam_map = np.zeros((1184,1984))
    
    # 미션이 수행 되고 난 후, can 위치 관련 정보 초기화
    def matching_clear(self):
        self.mission_state = 0
        self.matching_can_pos = [0.0, 0.0]
        self.sub_scan = list()

    # can의 위치 정보가 들어 왔을 때 callback함수. 라이다 매칭 미련을 조금 못버려서 추적하는데만 조금 매칭 사용(조금 사용함)
    def can_callback(self, can):
        sub_can_pos = [can.data[0]+0.170, can.data[1]-0.035] # offset = x좌표 기준 캠 위치 - 중심위치
        if can.data[2] == 2 or self.mission_state == 2: self.mission_state = 2 # state가 2(로봇팔 작동 시작)가 들어오면 무조건 상태 2로 변경 and 2로 유지
        if can.data[2] == 1: #2였다면 위에 if문에서 미리 바꼈으니 1인경우 -> 그래도 한번 매칭 해봄 -> 매칭 되면 라이다 좌표로 움직임 / 매칭 실패시 그냥 리얼센스 값 사용 
            self.matching_can_pos = self.matching_point(sub_can_pos, 0.1)
            if self.matching_can_pos == [0.0,0.0]: 
                self.matching_can_pos = sub_can_pos
            self.mission_state = 1 # -> 매칭 되던 안되던 state는 1
        elif can.data[2] == 0: # 혹시나 지금 카메라에서는 인식 못했는데
            if self.mission_state == 1: # 직전에는 can의 위치정보가 있었다면
                self.matching_can_pos = self.matching_point(self.matching_can_pos, 0.1) #한번 직전의 점과 매칭은 해봄 매칭 되면 state를 그대로 캔 추정모드(1)로 두고 trace된 캔 좌표로 주행
                if self.matching_can_pos == [0.0,0.0]: # 매칭 안됬어
                    self.mission_state = 0 # 응 완전히 놓친거야~ state 자율주행모드(0) 으로 완전 변경
        # print(self.matching_can_pos)

    # 라이다 거리정보 sub
    def scan_callback(self, scan):
        self.sub_scan = list(scan.ranges)[:]
        self.get_obj()
    
    def map_callback(self, map):
        ## self.map 정보 -> resolution == 0.05, offset = 30,30
        self.slam_map = np.array(map.data).reshape(map.info.height, map.info.width)
        # print(np.where(self.slam_map==100)) ## [600,599] - > 0,0
    
    # 라이다 거리&각도 정보를 로봇 xy좌표계로 변환
    def tf_tm(self, dis, i):
        obs_y = dis*math.sin(np.deg2rad(float(i)))
        obs_x = dis*math.cos(np.deg2rad(float(i))) + 0.04 ## 이거도 라이다랑 로봇 중심좌표 거리로 바꾸자
        return [obs_x, obs_y]

    # sub받은 라이다 거리정보를 정면 180도, 0.2~1.0m거리의 거리정보만 xy좌표로 변환
    def tf_scan_to_xy(self):
        obs = list()
        for i, dis in enumerate(self.sub_scan):
            if 0.2 < dis and dis < 1.0: obs.append(self.tf_tm(dis, i))
        if len(obs) < 2: obs = [[0,0]]
        return obs

    # 라이다로 얻은 주변 xy점들을 클러스터링. DBClustering으로 0.05m정도 간격으로 3점 이상 뭉쳐있으면 cluster하고, 평균점 정보도 저장
    def get_obj(self):
        self.DB_can_cluster.run(np.array(self.tf_scan_to_xy()))
            
    # 어떤 point와 주변환경을 매칭할건지 매칭해주는 함수. R은 얼만큼 가까운 점이 있으면 제일 가까운 점으로 cluster할지 정하는 매칭 반경
    def matching_point(self, point, R):
        dist = list()
        for c_avg in self.DB_can_cluster.cluster_avg:
            dist.append(np.linalg.norm([c_avg[0]-point[0], c_avg[1]-point[1]]))
        matching_min_pos = self.DB_can_cluster.cluster_avg[dist.index(min(dist))]
        maching_pos = matching_min_pos if min(dist) < R else [0.0, 0.0]         
        return maching_pos

    def pub_goal(self, goal, heading):
        self.goal_seq += 1
        q = quaternion_from_euler(0, 0, heading)
        P = PoseStamped(header = Header(seq = self.goal_seq, stamp = rospy.Time.now(), frame_id = "map"),
                        pose = Pose(position = Point(x = goal[0], y = goal[1]),
                                    orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
        self.goal_pose_pub.publish(P)

    def is_wall(self, point):
        p_ind = [int((point[1]+self.offset[0])/self.res), int((point[0]+self.offset[1])/self.res)]
        cost = np.sum(self.slam_map[p_ind[0]-2:p_ind[0]+3, p_ind[1]-2:p_ind[1]+3])
        cost = 1 if cost < 1 else 0
        return cost

    def pub_goal_point(self, point, heading):
        if self.is_wall(point) == 1:
            self.pub_goal(point, heading)
        else:
            



    # slam 코드에 서비스를 요청하여 지도 정보를 받아오는 서비스 함수
    # def map_update(self):
    #     rospy.wait_for_service('/dynamic_map')
    #     try:
    #         call_map = rospy.ServiceProxy('/dynamic_map', GetMap)
    #         resp_map = call_map()
    #         self.raw_map = resp_map
    #         print("Map ready")
    #     except:
    #         print("Map call failed")

def main():
    rate = rospy.Rate(2)
    map = Map()
    print("map test")
    rospy.sleep(3)
    while not rospy.is_shutdown():
        # print(np.where(map.slam_map>100))
        # map.pub_goal([0.0, 1.0], math.pi/2)
        print(map.is_wall([0.25, 0]))
        input("print map")
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('bot_map',anonymous=True)
    main()