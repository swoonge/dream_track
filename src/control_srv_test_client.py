#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import time
from open_manipulator_msgs.srv import SetJointPosition
   
def main():
    rospy.init_node('day1_sub',anonymous=True)
    rospy.wait_for_service('/goal_joint_space_path')
    try:
        cli = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        result = cli(t = msg.stamp, x = int(msg.x), y = int(msg.y))
        print("최종합: "),
        print(result.e)
        print("계산시간: "),
        print(result.t),
        print("nsec")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    rospy.spin()

if __name__ == '__main__':
    main()