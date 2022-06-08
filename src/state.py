#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time

from bot_control import *
from bot_map import *
# from control_client import *

class mode():
    def __init__(self):
        self.state = 0 # 0 is def_mod, 1 is get_mod

    def scan_mode(self): # scan mode # speed max == 0.1
        speed = 0.1
        steer = 0
        return speed, steer

    def get_mode(self): # get mode
        if self.state == 1:
            speed = 0.05
            steer = 0.0
        elif self.state == 2:
            speed = 0.0
            steer = 0.0
        return speed, steer

    def run(self):
        if self.state == 0:
            return self.scan_mode()
        if self.state != 0:
            return self.get_mode()

    def mode_update(self, state): # (0 : can not dect -> scan mode, 1 : can dect, but far, 2 : can get mode)  
        self.state = state
        
def main():
    rate = rospy.Rate(5)
    mission = mode()
    Turtle = bot([0.0, 0.0]) # 터틀봇 모듈 : 터틀봇의 센서데이터와 제어 담당
    # map = Map()
    Turtle.f_set()
    print_count = 0

    while not rospy.is_shutdown():
        # map.map_update()
        mission.mode_update(Turtle.can_state[2])
        speed, steer = mission.run()
        
        print_count += 1
        if print_count > 5:
            print(mission.state)
            print_count = 0

        Turtle.move(speed, steer)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()