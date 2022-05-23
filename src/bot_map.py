#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import numpy as np
import time
from nav_msgs.srv import GetMap

class Map():
    def __init__(self):
        rospy.wait_for_service('/dynamic_map')
        try:
            call_map = rospy.ServiceProxy('/dynamic_map', GetMap)
            resp_map = call_map()
            self.raw_map =  resp_map.map.data
            print(self.raw_map)
        except:
            print("Service call failed")

    def map_update(self):
        rospy.wait_for_service('/dynamic_map')
        call_map = rospy.ServiceProxy('/dynamic_map', GetMap)
        resp_map = call_map()
        self.raw_map =  resp_map

def main():
    bot_map = Map()
    time.sleep(1)
    bot_map.map_update()
    print(bot_map.raw_map)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('bot_map',anonymous=True)
    main()