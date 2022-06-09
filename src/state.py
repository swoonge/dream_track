#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time

from bot_control import *
from bot_map import *

class mode():
    def __init__(self):
        self.state = 0 # 0 is def_mod, 1 is get_mod

    def scan_mode(self): # scan mode # speed max == 0.1
        steer = 0
        speed = 0.8
        return speed, steer

    def get_mode(self, can_pos): # get mode
        if self.state == 1:
            steer = can_pos[1]*2 if can_pos[1] > 0.1 else can_pos[1]*2 if can_pos[1] < -0.1 else 0.0
            speed = 0.05
        else:
            steer = speed = 0.0
        return speed, steer

    def run(self, can_pos, st):
        self.state = st
        if self.state == 0:
            return self.scan_mode()
        if self.state > 0:
            return self.get_mode(can_pos)


def main():
    rate = rospy.Rate(5)
    mission = mode()
    Turtle = bot([0.0, 0.0])
    map = Map()

    Turtle.f_set()

    while not rospy.is_shutdown():
        speed, steer = mission.run(map.maching_can_pos, map.mission_state)
        if mission.state == 2: map.matching_clear()
        Turtle.move(speed, steer)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()