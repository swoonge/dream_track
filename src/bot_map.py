#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import numpy as np
import math
from tf.transformations import quaternion_from_euler
from scipy.spatial import distance

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Float64MultiArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from dbscan import *

class Map():
    def __init__(self):
        self.can_sub = rospy.Subscriber('/can_position_state', Float64MultiArray, self.can_callback, queue_size = 1) # (x, y, state)좌표
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size = 1)

        self.mean_pub = rospy.Publisher('/mean_rviz', Marker, queue_size=3)
        self.goal_pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)

        self.mission_state = 0
        self.matching_can_pos = [0.0, 0.0]
        self.sub_scan = list()
        self.DB_can_cluster = DBSCAN(0.05, 3)

        self.mini_obj_check = 1
        self.mini_obj = [[2000.0, 0.0]]

        self.goal_cost_map = 0
        self.goal_seq = 0
        self.res = 0.05
        self.offset = [50, 50]
        self.path_point = [[2.5,1.0],[3.0, -2.0],[2.0,-0.5],[0.0, 0.0]]

    ## 뎁스 카메라로 캔의 위치를 받으면 라이다와 매칭.
    def can_callback(self, can):
        sub_can_pos = [can.data[0]+0.170, can.data[1]-0.035] # offset = x좌표 기준 캠 위치 - 중심위치
        if can.data[2] == 2 or self.mission_state == 2: self.mission_state = 2 # 캔이 workspace에 들어온 경우. 캔을 집는 동안 멈추는 state 유지
        if can.data[2] == 1: # 캔을 카메라로 발견했지만 workspace 범위에 없는 경우 -> 더 정확한 위치 추적을 위해 라이다정보와 매칭시도 
            self.matching_can_pos = self.matching_point(sub_can_pos, 0.1)
            if self.matching_can_pos == [1000.0, 1000.0]:
                self.matching_can_pos = sub_can_pos
            self.mission_state = 1
        elif can.data[2] == 0: # 캔을 카메라로 발견하지 못했을 경우
            if self.mission_state == 1: # 캔의 위치정보가 있었을 경우
                self.matching_can_pos = self.matching_point(self.matching_can_pos, 0.1) # 라이다정보만으로 움직인 캔의 위치를 매칭하여 추정
                if self.matching_can_pos == [1000.0, 1000.0]: # 추정 실패시 캔 lost
                    self.matching_can_pos == [0.0, 0.0]
                    self.mission_state = 0

    ## 라이다 거리정보  Sub
    def scan_callback(self, scan):
        self.sub_scan = list(scan.ranges)[:]
        self.get_obj() # 받은 거리정보를 robot frame으로 좌표 변환 후 dbscan클러스터링
        self.find_mini_obj() # 근처 캔과 비슷한 크기의 물체 찾기
    
    def map_callback(self, map):
        ## self.map 정보 -> resolution == 0.05, offset = 30,30 // 행이 y, 열이 x값. [600,599] - > 0,0
        if self.goal_cost_map == 0:
            self.make_map_copy()
        self.slam_map = np.array(map.data).reshape(map.info.height, map.info.width)
    
    ## 미션이 수행 되고 난 후, can 위치 관련 정보 초기화
    def matching_clear(self):
        self.mission_state = 0
        self.matching_can_pos = [0.0, 0.0]
        self.sub_scan = list()

    def find_mini_obj(self):
        if len(self.DB_can_cluster.cluster_avg) > 0: # 클러스터가 있다면 작동
            if len(self.mini_obj) == 1: # 처음 발견한 점 이라면 클러스터중에 작은 물체가 가까이 있으면 추적할 점 추가
                for ic, c in enumerate(self.DB_can_cluster.cluster):
                    if len(c[0]) < 7 and np.linalg.norm(self.DB_can_cluster.cluster_avg[ic]) < 0.5:
                        self.mini_obj.append(self.DB_can_cluster.cluster_avg[ic])
                        self.mini_obj_check += 1
            else:# 처음받은 점이 아닐 때
                # 기존에 추적해 왔던 물체 추적
                # 추적 실패시 추적 중지
                for io, obj in enumerate(self.mini_obj):
                    obj_near = self.matching_point(obj, 0.2)
                    if obj_near != [1000.0, 1000.0]:
                        self.mini_obj[io] = obj_near
                    elif obj_near == [1000.0, 1000.0]:
                        self.mini_obj[io] = [2000.0, 0.0]
                # 새로운 물체(추적해오지 않았던 물체)는 새로운 점으로 추가
                for ic, c in enumerate(self.DB_can_cluster.cluster):
                    if len(c[0]) < 7 and np.linalg.norm(self.DB_can_cluster.cluster_avg[ic]) < 0.5:
                        dist = list()
                        for o_avg in self.mini_obj:
                            dist.append(np.linalg.norm([o_avg[0]-self.DB_can_cluster.cluster_avg[ic][0], o_avg[1]-self.DB_can_cluster.cluster_avg[ic][1]]))
                        if min(dist) < 0.2: pass
                        else: self.mini_obj.append(self.DB_can_cluster.cluster_avg[ic])
        # 클러스터가 없다면 모든 물체 추적 중지
        elif len(self.DB_can_cluster.cluster_avg) == 0:
            self.mini_obj[:][:] = [2000.0, 0.0]
        
    # 라이다 거리&각도 정보를 로봇 xy좌표계로 변환
    def tf_tm(self, dis, i):
        obs_y = dis*math.sin(np.deg2rad(float(i)))
        obs_x = dis*math.cos(np.deg2rad(float(i))) + 0.03
        return [obs_x, obs_y]

    # sub받은 라이다 거리정보를 정면 180도, 0.38~2.0m거리의 거리정보만 xy좌표로 변환
    def tf_scan_to_xy(self):
        obs = list()
        for i, dis in enumerate(self.sub_scan):
            if 0.38 < dis and dis < 2.0: obs.append(self.tf_tm(dis, i))
        if len(obs) < 2: obs = [[0,0]]
        return obs

    # 라이다로 얻은 주변 xy점들을 클러스터링.
    def get_obj(self):
        self.DB_can_cluster.run(np.array(self.tf_scan_to_xy()))
            
    # 주변 환경의 물체와 point사이의 거리가 R 보다 작은 점 중에서 가장 가까운 점 매칭
    def matching_point(self, point, R):
        dist = list()
        for c_avg in self.DB_can_cluster.cluster_avg:
            dist.append(np.linalg.norm([c_avg[0]-point[0], c_avg[1]-point[1]]))
        matching_min_pos = self.DB_can_cluster.cluster_avg[dist.index(min(dist))]
        maching_pos = matching_min_pos if min(dist) < R else [1000.0, 1000.0]
        return maching_pos

    ## 테스트 작업공간 path point 생성
    def get_path_point(self, space_range): #space_range is 작업공간의 아래 좌우 x,y좌표. 좌표는 왼-우 좌표
        x_interval = 6*0.45/5 #복도 폭/5
        robot_wall_tolerance = 0.6 #(m)
        space_range[0] = [space_range[0][0], space_range[0][1] - robot_wall_tolerance]
        space_range[1] = [space_range[1][0], space_range[1][1] + robot_wall_tolerance]
        self.path_point = [[space_range[0][0]+2*x_interval, space_range[0][1]],
                            [space_range[0][0]+4*x_interval, space_range[0][1]],[space_range[1][0]+4*x_interval, space_range[1][1]],
                            [space_range[1][0]+3*x_interval, space_range[1][1]],[space_range[0][0]+3*x_interval, space_range[0][1]],
                            [space_range[0][0]+2*x_interval, space_range[0][1]],[space_range[1][0]+2*x_interval, space_range[1][1]],
                            [space_range[1][0]+x_interval, space_range[1][1]],[0.0, 0.0]]

    # 가야할 지점의 x,y,yaw 값을 Navigation 모듈에 Pub
    def pub_goal(self, goal, heading):
        self.goal_seq += 1
        q = quaternion_from_euler(0, 0, heading)
        P = PoseStamped(header = Header(seq = self.goal_seq, stamp = rospy.Time.now(), frame_id = "map"),
                        pose = Pose(position = Point(x = goal[0], y = goal[1]),
                                    orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
        self.goal_pose_pub.publish(P)

    # 격자맵에서 point 주변 공간 n의 범위 안에에 벽이 있는지 판단
    def is_wall(self, point, n = 2):
        p_ind = [int((point[1]+self.offset[0])//self.res), int((point[0]+self.offset[1])//self.res)]
        cost = np.sum(self.slam_map[p_ind[0]-n:p_ind[0]+n+1, p_ind[1]-n:p_ind[1]+n+1])
        cost = 1 if cost < 1 else 0
        return cost # 0 : 포인트 주변에 벽이 있음

    # x,y,yaw값을 지정하면 근처에 벽이 있는경우 조금 떨어트려서 위치 전달
    def pub_goal_point(self, point, heading):
        if self.is_wall(point) == 1:
            self.pub_goal(point, heading)
        else:
            pa = 0
            while(pa == 0):
                s_point = []
                for x in [-0.1, 0.1]:
                    for y in [-0.1, 0.1]:
                        if self.is_wall([point[0]+x, point[1]+y]) == 1:
                            s_point.append([point[0]+x, point[1]+y])
                if len(s_point) == 0:
                    pa = 0
                else:
                    pa = 1
                    shift_point = s_point[0]
            self.pub_goal(shift_point, heading)

    ## 경로 코스트 맵 생성
    def make_map_copy(self):
        self.goal_cost_map = np.zeros(self.slam_map.shape)

    ## path point 업데이트 함수.
    # def get_new_goal_point(self, pose):
    #     p_ind = [int((pose[1]+self.offset[0])//self.res), int((pose[0]+self.offset[1])//self.res)] # 격자점에서 현재 몇by몇에 위치해 있는지 계산, 
    #     a = np.sum(self.goal_cost_map[p_ind[0]])

    #####벽과 갈수 있는 공간의 평균점으로 path point update 시도 함수#####
    def get_space_senter(self, c): # 0이면 빈공간 계산
        scaned_space = np.where(self.slam_map == 0) if c == 0 else np.where(self.slam_map > 1)
        scaned_space_mean = np.mean(np.stack([scaned_space[0],scaned_space[1]],1), axis=0)*self.res-self.offset[0]
        self.space_mean[0] = self.space_mean[1]
        self.space_mean[1] = [scaned_space_mean[1], scaned_space_mean[0]]
        self.mean_viz([(scaned_space_mean[1]),(scaned_space_mean[0])])

    
    def check_space(self, c): #움직였으면 1, 안움직였으면 0 반환
        self.get_space_senter(c)
        if distance.euclidean(self.space_mean[0],self.space_mean[1]) < 0.00001:
            return 0
        else: return 1
    ###################################################################