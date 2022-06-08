#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time

from bot_control import *
from bot_map import *

class mode():
    def __init__(self):
        self.state = 0 # 0 is def_mod, 1 is get_mod

    def mode_0(self): # scan mode
        speed = 0
        steer = 0
        theta = [0, 0, 0, 0] # or int
        return speed, steer, theta

    def mode_1(self): # get mode
        speed = 0
        steer = 0
        theta = [0, 0, 0, 0] # or int
        return speed, steer, theta

    def run(self):
        if self.state == 0:
            return self.mode_0()
        if self.state == 1:
            return self.mode_1()

    def mode_update(self, can):
        if can != 0:
            self.state = 1
        if can == 0:
            self.state = 0
        
def main():
    rate = rospy.Rate(1)
    mission = mode()
    Turtle = bot() # 터틀봇 모듈 : 터틀봇의 센서데이터와 제어 담당
    map = Map()
    
    while not rospy.is_shutdown():
        map.map_update()
        mission.mode_update(can_dis):
        speed, steer = mission.run()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()