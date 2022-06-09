#!/usr/bin/env python3
# -- coding: utf-8 --
from sre_constants import SUCCESS
import rospy
import time

from std_srvs.srv import SetBool, SetBoolResponse
from dream_track.srv import can_moveResponse, can_move
from bot_control import *
from bot_map import *

#########
def callback(data):
    print(data.x, data.y)
    time.sleep(5) # arm move//////////////////////
    return can_moveResponse(suc = True)
##########


def main():
    rate = rospy.Rate(5)

    ########
    can_mission_cli = rospy.Service('/can_mission_point', can_move, callback)
    ########


    while not rospy.is_shutdown():
        # map.map_update()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('test2',anonymous=True)
    main()

##
# bool success
# string message