#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from open_manipulator_msgs.srv import SetJointPosition

def callback(request):
    print(request)
    res = bool()
    res= True
    return res

rospy.init_node('control_srv_test_server')
service_server = rospy.Service('/goal_tool_control', SetJointPosition, callback)
rospy.spin()