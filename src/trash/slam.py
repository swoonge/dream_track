#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import math
import map as mp
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

class slam:
    def __init__(self, map_size = 20, scale = 0.05):
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size = 1)
        self.map = np.zeros((int(map_size//scale), int(map_size//scale)), dtype='i8') # x(m) x y(m) 의 공간을 설정
        self.map -= 1
        self.map[:,0] = 100
        self.map[:,len(self.map[0])-1] = 100
        self.map[0,:] = 100
        self.map[len(self.map)-1,:] = 100
        self.scale = scale
        self.map_size = map_size
        self.map_seq = 0
        self.map_wh = self.map.shape

    def reset(self):
        self.map = np.zeros((int(self.map_size//self.scale), int(self.map_size//self.scale)), dtype='i8')
        self.map -= 1
        self.map[:,0] = 100
        self.map[:,len(self.map[0])-1] = 100
        self.map[0,:] = 100
        self.map[len(self.map)-1,:] = 100

    def tf_tm(self, dis, i, T):
        obs_xy = np.empty((1, 3))
        obs_y = dis*math.sin(np.deg2rad(float(i)))
        obs_x = dis*math.cos(np.deg2rad(float(i)))
        obs_xy = np.dot(T, np.transpose([obs_x, obs_y, 1]))
        return [obs_xy[0], obs_xy[1]]

    def tf_scan_to_xy(self, pos, heading, scan_data):
        obs = []

        Trans_z = [[math.cos(heading), -math.sin(heading), pos[0]],
                  [math.sin(heading),  math.cos(heading), pos[1]],
                  [        0        ,         0         ,    1  ]]

        for i, dis in enumerate(scan_data):
            if 0.2 < dis and dis < 3.0:
                obs.append(self.tf_tm(dis, i, Trans_z))

        return obs

    def make_map(self, obs):
        for obj in obs:
            try:
                n = int(round(obj[0]/self.scale))
                m = int(round(obj[1]/self.scale))
                self.map[int(n)][int(m)] = int(100)
            except:
                continue

    def bot_point(self, pos):
        try:
            n = int(round(pos[0]/self.scale))
            m = int(round(pos[1]/self.scale))
            self.map[int(n)][int(m)] = int(88)
        except:
            pass
            
    def slam(self, pos, heading, scan):
        obs = self.tf_scan_to_xy(pos, heading, scan)
        self.make_map(obs)
        self.bot_point(pos)
        self.map_seq += 1

    def map_vis_pub(self, pos, orientation_s):
        rviz_map=OccupancyGrid( header=Header(
                                    seq = self.map_seq, 
                                    frame_id='/map', 
                                    stamp=rospy.get_rostime()),
                                info = MapMetaData(
                                    resolution = 0.05,
                                    width = self.map_wh[0],
                                    height = self.map_wh[1],
                                    origin = Pose(  position = Point(x=pos[0], y=pos[1]),
                                                    orientation = orientation_s)),
                                data = self.map.flatten().tolist())
        print(self.map[190:208, 190:208])
        self.map_pub.publish(rviz_map)

#############################################################################################################        
def main():
    slam = slam()

if __name__ == '__main__':
    rospy.init_node('slam',anonymous=True)
    main()