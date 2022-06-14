#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time

from move_base_msgs.msg import MoveBaseActionResult

from bot_control import *
from bot_map import *
from Pp import *

class mode():
    def __init__(self):
        self.motor_sub = rospy.Subscriber('/cmd_vel_navi', Twist, self.navi_cmd_callback, queue_size = 1) # 모터
        self.motor_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback, queue_size = 1) # 모터
        self.state = 0 # 0 is scan_mode, 1 is get_mode
        self.result_state = 0
        self.navi_mt = [0.0,0.0] #speed, steer
        self.pp = Pp()

    def navi_cmd_callback(self, navi):
        self.navi_mt = [navi.linear.x, navi.angular.z]
        # print(self.navi_mt)

    def result_callback(self, result):
        self.result_state = result.status.status

    def scan_mode(self, pos, heading): # scan mode # speed max == 0.1 #지금은 일단 직진.
        # print(self.pp.pur_p(pos, heading))
        # return 0.08, 0.0
        return self.navi_mt[0], self.navi_mt[1]

    def get_mode(self, can_pos): # state가 1 일때는 y좌표가 좌우 10cm씩 넘어갈 때만 조향
        if self.state == 1: # 스티어는 간단한 p제어와 같음
            if np.linalg.norm(can_pos) < 0.7:
                steer = can_pos[1]*2 if can_pos[1] > 0.01 else can_pos[1]*2 if can_pos[1] < -0.01 else 0.0
                speed = 0.05
            else:
                speed = self.navi_mt[0]
                steer = self.navi_mt[1]
        else: # state가 2가 들어오는 경우. 정지
            steer = speed = 0.0
        return speed, steer

    def run(self, can_pos, st, pos, heading):
        self.state = st # state 업데이트
        if self.state == 0:
            return self.scan_mode(pos, heading)
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
    map_check_count = 0
    # map.get_path_point([[0.37+0.1,0.45*2.5],[0.37+0.1,-0.45*6.5]])
    map.get_path_point([[-0.2,0.45*2.5],[-0.2,-0.45*2.5]])

    path_point_i = 0
    map.pub_goal_point(map.path_point[path_point_i], Turtle.heading)

    ## 첫 공간 완벽 스캔
    # while(Turtle.heading > math.pi/2):
    #     Turtle.move(0.0, 0.2)
    #     rate.sleep()
    # rospy.sleep(2)

    # Turtle.move(0.0, 1.0)
    # rospy.sleep(5)
    # Turtle.move(0.0, 0.0)
    # rospy.sleep(3)
    # Turtle.move(0.0, -1.0)
    # rospy.sleep(5)
    # Turtle.move(0.0, 0.0)
    # rospy.sleep(1)
    
    map.mini_obj_check = len(map.mini_obj)

    while not rospy.is_shutdown():
        ## goal_point_update
        # print(mission.result_state)
        if mission.result_state == 3:
            path_point_i += 1
            if path_point_i == (len(map.path_point)-1): pass
            elif path_point_i == (len(map.path_point)): break
            mission.result_state = 0
            map.pub_goal_point(map.path_point[path_point_i], Turtle.heading)
            # rospy.sleep(0.3)

        ## 닫힌 공간 판단 -> 2초에 한번씩
        # if map_check_count == 25:
        #     if map.check_space(0) == 0:
        #         map.pub_goal_point(0.0, 0.0, math.pi)
        #     else: map_check_count += 1
        #     map_check_count = 0

        # print(map_check_count, map.mini_obj_check)

        ## 닫힌 공간 판단 -> 4초에 한번씩
        if map_check_count >= 20:
            if len(map.mini_obj) > map.mini_obj_check:
                #map.mini_obj[map.mini_obj_check] 방향으로#헤딩 돌리기
                # steer = map.mini_obj[map.mini_obj_check][1]*2 if map.mini_obj[map.mini_obj_check][1]> 0.1 else map.mini_obj[map.mini_obj_check][1]*2 if map.mini_obj[map.mini_obj_check][1] < -0.1 else 0.0
                steer = 0.2 if map.mini_obj[map.mini_obj_check][1] > 0.1 else -0.2 if map.mini_obj[map.mini_obj_check][1] < -0.1 else 0.0
                speed = 0.0
                Turtle.move(speed, steer)
                if steer == 0.0:
                    map_check_count = 0
                    map.mini_obj_check += 1
                rate.sleep()
                continue
        map_check_count += 1

        ## 미션 스테이트에 따라 speed와 steer를 가져오는 함수
        speed, steer = mission.run(map.matching_can_pos, map.mission_state, Turtle.pos, Turtle.heading)

        ## 미션 스테이트가 2(집는 모션)이라면 미션관련 모두 초기화 후, 정지 후 집는 모션 대기
        if mission.state == 2: 
            map.matching_clear() # state가 2라면 can 관련 정보 초기화
            Turtle.move(0.0, 0.0)
            time.sleep(8)

        ## 로봇 이동
        Turtle.move(speed, steer)
        # Turtle.move(0.08, 0)
        rate.sleep()
    print("end------------------------")

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()