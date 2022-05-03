
#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
# import time
from open_manipulator_msgs.srv import SetJointPosition

# class server():
#     def __init__(self):
#         self.server = rospy.Service('/goal_joint_space_path', SetJointPosition, self.call)

#     def call(self, sr):
#         print(sr)
#         return True
        
# def main():
#     rospy.init_node('day1_pub', anonymous=True)
#     server()
#     rospy.spin()

# if __name__ == '__main__':
#     main()  

def call(srv):
    print(srv)
    return SetJointPositionResponse(True)

def mot_server(req):
    rospy.init_node('control_srv_test_server', anonymous=True)
    print("a")
    s = rospy.Service('/goal_joint_space_path', SetJointPosition, call)
    print("start")
    rospy.spin()

if __name__ == '__main__':
    mot_server() 