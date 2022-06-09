#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time

from bot_control import *
from bot_map import *
from dream_track.srv import can_move

class mode():
    def __init__(self):
        self.mission_pub = rospy.Publisher('/can_mission_point', Float64MultiArray, queue_size = 1)
        self.state = 0 # 0 is def_mod, 1 is get_mod

    def mission_point_client(self, can_pos):
        rospy.wait_for_service('/can_mission_point')
        try:
            can_mission_srvs = rospy.ServiceProxy('/can_mission_point', can_move)
            bak = can_mission_srvs(x = can_pos[0], y = can_pos[1])
            print("mission : ",bak.suc)
            
        except:
            self.state = 0
            pass

    def scan_mode(self): # scan mode # speed max == 0.1
        steer = 0
        speed = 0.1
        return speed, steer

    def get_mode(self, can_pos): # get mode
        steer = can_pos[1]*5 if can_pos[1] > 0.001 else can_pos[1]*5 if can_pos[1] < -0.001 else 0.0
        speed = 0.05
        return speed, steer

    def run(self, can_pos):
        if self.state == 0:
            return self.scan_mode()
        if self.state != 0:
            return self.get_mode(can_pos)

    def mode_update(self, can_pos): # (0 : can not dect -> scan mode, 1 : can dect, but far, 2 : can get mode)
        self.state = 0 if np.linalg.norm(can_pos) <= 0 else 1 if np.linalg.norm(can_pos) > 0.35 else 2
        print(self.state)

def main():
    rate = rospy.Rate(5)
    mission = mode()
    Turtle = bot([0.0, 0.0]) # 터틀봇 모듈 : 터틀봇의 센서데이터와 제어 담당
    map = Map()

    Turtle.f_set()

    while not rospy.is_shutdown():
        # map.map_update()
        mission.mode_update(map.can_pos)
        speed, steer = mission.run(map.can_pos)

        if mission.state == 2:
            print(map.can_pos)
            mission.mission_point_client(map.can_pos)
            map.clear()
            mission.state == 0
            speed = steer = 0.0

        Turtle.move(speed, steer)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()