#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy

from bot_control import *
from bot_map import *

class mode():
    def __init__(self):
        self.motor_sub = rospy.Subscriber('/cmd_vel_navi', Twist, self.navi_cmd_callback, queue_size = 1) # 모터
        self.state = 0 # 0 is scan_mode, 1 is get_mode
        self.navi_mt = [0.0,0.0] #speed, steer

    def navi_cmd_callback(self, navi):
        self.navi_mt = [navi.linear.x, navi.angular.z]
        # print(self.navi_mt)

    def scan_mode(self): # scan mode # speed max == 0.1 #지금은 일단 직진.
        return self.navi_mt[0], self.navi_mt[1]

    def get_mode(self, can_pos): # state가 1 일때는 y좌표가 좌우 10cm씩 넘어갈 때만 조향
        if self.state == 1: # 스티어는 간단한 p제어와 같음
            if np.linalg.norm(can_pos) < 0.6:
                steer = can_pos[1]*2 if can_pos[1] > 0.01 else can_pos[1]*2 if can_pos[1] < -0.01 else 0.0
                speed = 0.05
            else:
                speed = self.navi_mt[0]
                steer = self.navi_mt[1]
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
    # input("After enter, actuator power off ")
    # Turtle.actuator_power(False)
    # input("Set actuator and enter to power on ")
    # Turtle.actuator_power(True)
    
    Turtle.f_set() #터틀봇 위치 초기화

    input("Enter to start bot")

    while not rospy.is_shutdown():
        speed, steer = mission.run(map.matching_can_pos, map.mission_state)
        # print(mission.state)
        if mission.state == 2: 
            map.matching_clear() # state가 2라면 can 관련 정보 초기화
            Turtle.move(0.0, 0.0)
            rospy.sleep(10)
        Turtle.move(speed, steer) # 로봇 이동
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()