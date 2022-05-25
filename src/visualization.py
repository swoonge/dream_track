#!/usr/bin/env python
import rospy
from math import cos, sin, pi
import numpy as np

from geometry_msgs.msg import Vector3, Pose, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Float64, ColorRGBA

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class Visualization():
    def __init__(self):
        # self.Map_pub=rospy.Publisher('/map_rviz', Marker, queue_size=100)
        self.pose_pub = rospy.Publisher('/pose_rviz',Marker, queue_size=1)
        # self.path_pub = rospy.Publisher('/path_rviz',Marker, queue_size=1)

    def presentPOSE(self,x,y,heading):
        pose = Pose()

        q=euler_to_quaternion(0,0,heading)
        pose.orientation.x=q[0]
        pose.orientation.y=q[1]
        pose.orientation.z=q[2]
        pose.orientation.w=q[3]

        pose.position.x=x
        pose.position.y=y
        pose.position.z=0

        rviz_msg_pose=Marker(
            header=Header(frame_id='map',stamp=rospy.get_rostime()),
            ns="current_Pose",
            id=100,
            type=Marker.ARROW,
            lifetime=rospy.Duration(),
            action=Marker.ADD,
            pose=pose,
            scale=Vector3(x=2.0,y=0.5,z=0.5),
            color=ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0),
            )

        self.pose_pub.publish(rviz_msg_pose)

    # def map(self):
    #     i=0
    #     for st in self.PATH:
    #         if st[6:8] == "ce":
    #             cl=ColorRGBA(1.0,1.0,0.0,1.0)
    #         elif st[6:8] == "li":
    #             cl=ColorRGBA(1.0,1.0,1.0,1.0)
    #         elif st[6:8] == "st":
    #             cl=ColorRGBA(1.0,0.0,0.0,1.0)
    #         elif st[0:2] == "DG":
    #             cl=ColorRGBA(1.0,1.0,1.0,1.0)
    #         elif st[6:8] == "bu":
    #             cl=ColorRGBA(0.0,0.0,1.0,1.0)

    #         rviz_msg_global=Marker(
    #             header=Header(frame_id='map'),
    #             ns="global_path",
    #             type=Marker.LINE_STRIP,
    #             action=Marker.ADD,
    #             id=i,
    #             scale=Vector3(0.1,0.1,0),
    #             color=cl)

    #         path_arr=np.load(file=PATH_ROOT+"global_map/"+self.PATH[i])
    #         s=range(len(path_arr))
    #         for a in s:
    #             p=Point()
    #             p.x=float(path_arr[a,0])-self.offset[0]
    #             p.y=float(path_arr[a,1])-self.offset[1]
    #             p.z=0
    #             rviz_msg_global.points.append(p)
    #         self.global_pub.publish(rviz_msg_global)
    #         i+=1

    # def path(self):
    #     rviz_msg_tracking=Marker(
    #         header=Header(frame_id='map'),
    #         ns="tracking_line",
    #         type=Marker.LINE_STRIP,
    #         action=Marker.ADD,
    #         id=102,
    #         scale=Vector3(0.1,0.1,0),
    #         color=ColorRGBA(0.0,1.0,0.0,0.7))

    #     path_arr=np.load(file=PATH_ROOT+"path/"+Tracking_path)
    #     s=range(len(path_arr))
    #     for a in s:
    #         p=Point()
    #         p.x=float(path_arr[a,0])-self.offset[0]
    #         p.y=float(path_arr[a,1])-self.offset[1]
    #         p.z=0
    #         rviz_msg_tracking.points.append(p)
    #     self.tracking_pub.publish(rviz_msg_tracking)

    def rviz_update(self, pose, heading):
        self.presentPOSE(pose[0], pose[1], heading)


def main():
    rospy.init_node('visualization',anonymous=True)

if __name__ == '__main__':
    main()