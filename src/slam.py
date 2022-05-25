#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import math
import numpy as np

class slam:
    def __init__(self, map_size = 20, scale = 0.05):
        self.map = np.zeros((int(map_size[0]//scale), int(map_size[1]//scale))) # x(m) x y(m) 의 공간을 설정
        self.map[:,0] = 1
        self.map[:,len(self.map[0])-1] = 1
        self.map[0,:] = 1
        self.map[len(self.map)-1,:] = 1
        self.scale = scale, self.map_size = map_size

    def reset(self):
        self.map = np.zeros((int(self.map_size[0]//self.scale), int(self.map_size[1]//self.scale)))
        self.map[:,0] = 1
        self.map[:,len(self.map[0])-1] = 1
        self.map[0,:] = 1
        self.map[len(self.map)-1,:] = 1

    def tf_tm(self, dis, i, T):
        obs_xy = np.empty((1, 3))
        obs_y = dis*math.sin(np.deg2rad(float(i)))
        obs_x = dis*math.cos(np.deg2rad(float(i)))
        obs_xy = np.dot(T, np.transpose([obs_x, obs_y, 1]))
        return [obs_xy[0], obs_xy[1]]

    def tf_scan_to_xy(self, pos, heading, scan_data):
        i = 0
        obs = []

        Trans_z = [[math.cos(heading), -math.sin(heading), pos[0]],
                  [math.sin(heading),  math.cos(heading), pos[1]],
                  [        0        ,         0         ,    1  ]]

        for dis in scan_data:
            if 0.2 < dis and dis < 3.0:
                obs.append(self.tf_tm(dis, i, Trans_z)) # 변환된 좌표가 self.obs에 저장됨
            i = i + 1 # 각도 올림
        return obs

    def make_map(self, obs):
        for obj in obs:
            try:
                n = int(round(obj[0]/self.scale))
                m = int(round(obj[1]/self.scale))
                self.map[int(n)][int(m)] += 1
            except:
                continue
            
    def slam(self, pos, heading, scan):
        obs = self.tf_scan_to_xy(pos, heading, scan)
        self.make_map(obs)
        
#############################################################################################################        
def main():
    slam = slam()

if __name__ == '__main__':
    rospy.init_node('slam',anonymous=True)
    main()