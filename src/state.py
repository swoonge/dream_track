#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy

from bot_control import *
from bot_map import *

class mode():
    def __init__(self):
        self.state = 0 # 0 is scan_mode, 1 is get_mode

    def scan_mode(self): # scan mode # speed max == 0.1 #지금은 일단 직진.
        steer = 0
        speed = 0.8
        return speed, steer

    def get_mode(self, can_pos): # state가 1 일때는 y좌표가 좌우 10cm씩 넘어갈 때만 조향
        if self.state == 1: # 스티어는 간단한 p제어와 같음
            steer = can_pos[1]*2 if can_pos[1] > 0.1 else can_pos[1]*2 if can_pos[1] < -0.1 else 0.0
            speed = 0.05
        else: # state가 2가 들어오는 경우. 정지
            steer = speed = 0.0
        return speed, steer

    def run(self, can_pos, st):
        self.state = st # state 업데이트
        if self.state == 0:
            return self.scan_mode()
        if self.state > 0:
            return self.get_mode(can_pos)


def main():
    rate = rospy.Rate(5)
    mission = mode()
    Turtle = bot([0.0, 0.0]) #로봇의 시작위치 offset -> 지도 볼 때 필요할 수도 있겠다 싶었음
    map = Map() #맵 모듈. 지도 정보와 can의 정보가 담겨있음

    Turtle.f_set() #터틀봇 위치 초기화

    while not rospy.is_shutdown():
        speed, steer = mission.run(map.maching_can_pos, map.mission_state)
        if mission.state == 2: map.matching_clear() # state가 2라면 can 관련 정보 초기화
        Turtle.move(speed, steer) # 로봇 이동
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()