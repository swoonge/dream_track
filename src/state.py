#!/usr/bin/env python
# -- coding: utf-8 --
import re
import rospy
import time

from bot_control import *
from bot_map import *
from slam import *

map_size = 20 # (m)
scale = 0.05 # (0.05m 간격으로 격자화)
offset = [map_size/2.0, map_size/2.0] 

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
    global map_size, scale, offset
    rate = rospy.Rate(1)
    Turtle = bot(offset) # 터틀봇 모듈 : 터틀봇의 센서데이터와 제어 담당
    Slam = slam() # Slam 모듈 : (xy(m), scale(m))
    mode = mode()

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()