#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time
import numpy as np

from move_base_msgs.msg import MoveBaseActionResult

from bot_control import *
from bot_map import *

class mode():
    def __init__(self):
        self.motor_sub = rospy.Subscriber('/cmd_vel_navi', Twist, self.navi_cmd_callback, queue_size = 1)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback, queue_size = 1)

        self.state = 0 # 0 is scan_mode, 1 is get_mode
        self.result_state = 0
        self.navi_mt = [0.0, 0.0]

    ## navigation에서 계산된 speed와 steer 값 Sub
    def navi_cmd_callback(self, navi):
        self.navi_mt = [navi.linear.x, navi.angular.z]

    ## navigation에서의 상태 feedback
    def result_callback(self, result):
        self.result_state = result.status.status

    ## scan모드일 경우, navigation의 값 사용
    def scan_mode(self):
        return self.navi_mt[0], self.navi_mt[1]
    
    ## get모드일 경우, 캔을 향해 조향하거나 정지 명령
    def get_mode(self, can_pos):
        if self.state == 1: # 캔을 집으러 감. steer는 p-control
            if np.linalg.norm(can_pos) < 0.9:
                steer = can_pos[1]*2 if can_pos[1] > 0.01 else can_pos[1]*2 if can_pos[1] < -0.01 else 0.0
                speed = 0.05
            else:
                speed = self.navi_mt[0]
                steer = self.navi_mt[1]
        else: # 캔이 workspace안에 들어온 경우 정지
            steer = speed = 0.0
        return speed, steer

    ## state에 따라 적절한 speed와 steer값을 반환
    def run(self, can_pos, st, pos, heading):
        self.state = st # state 업데이트
        if self.state == 0:
            return self.scan_mode(pos, heading)
        if self.state > 0:
            return self.get_mode(can_pos)


def main():
    rate = rospy.Rate(5)
    mission = mode()
    Turtle = bot([0.0, 0.0])
    map = Map()

    map_check_count = 0
    path_point_i = 0

    Turtle.f_set() #터틀봇 위치 초기화

    map.get_path_point([[0.1,0.45*2.5],[0.1,-0.45*6.5]])
    map.pub_goal_point(map.path_point[path_point_i], Turtle.heading)
    
    # 시작하자마자 있는 작은 물체는 확인 생략
    map.mini_obj_check = len(map.mini_obj)

    while not rospy.is_shutdown():
        ## 로봇이 도착했다고 판단 된 경우 다음 navigation에게 새로운 goal point 전달
        if mission.result_state == 3:
            path_point_i += 1
            if path_point_i == (len(map.path_point)-1): pass
            elif path_point_i == (len(map.path_point)): break
            mission.result_state = 0
            map.pub_goal_point(map.path_point[path_point_i], Turtle.heading)

        ## 4초에 한번씩 새로 발견된 작은 물체가 있는지 확인
        ## 새로 발견된 물체가 있다면 시야를 돌려 카메라로 확인
        if map_check_count >= 20:
            if len(map.mini_obj) > map.mini_obj_check:
                steer = 0.5 if map.mini_obj[map.mini_obj_check][1] > 0.05 else -0.5 if map.mini_obj[map.mini_obj_check][1] < -0.05 else 0.0
                speed = 0.0
                Turtle.move(speed, steer)
                if steer == 0.0:
                    time.sleep(0.5)
                    map_check_count = 0
                    map.mini_obj_check += 1
                rate.sleep()
                continue
        else: map_check_count += 1

        ## 미션 스테이트에 따라 speed와 steer를 가져오는 함수
        speed, steer = mission.run(map.matching_can_pos, map.mission_state, Turtle.pos, Turtle.heading)

        ## 미션 스테이트가 2(집는 모션)이라면 미션관련 모두 초기화 후, 정지 후 집는 모션 대기
        if mission.state == 2: 
            map.matching_clear()
            Turtle.move(0.0, 0.0)
            time.sleep(8)

        Turtle.move(speed, steer)
        rate.sleep()
    print("------------end------------")

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()
