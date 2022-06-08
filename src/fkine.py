# -- coding: utf-8 --
import cv2
import time
import numpy as np

from roboticstoolbox import ETS as ET, DHLink, DHRobot, RevoluteDH
from spatialmath.base import *
from spatialmath import SE3

pi = np.pi
theta0 = np.arctan2(0.024,0.178)
q0 = [0,0,0,0]
X_coord, Y_coord, Z_coord = 1,1,1
### x 0.188 y = 0.03 z =0.025 //offset:RefToCam

### invK
robot = DHRobot(
     [
          RevoluteDH(d=0.0765,alpha=-pi/2),
          RevoluteDH(a=0.178,offset=-pi/2+theta0),
          RevoluteDH(a=0.174,offset=pi/2-theta0),
          RevoluteDH(a=0.126)
     ]
)

def invK(x,y,z):
    T = [
        [1,0,0,x + 0.188],
        [0,1,0,y + 0.03],
        [0,0,1,z - 0.025],
        [0,0,0,1]
        ]
    T = np.array(T)
    T = SE3(T)
    initT = [
        [1,0,0,0.1201],
        [0,1,0,0],
        [0,0,1,0.2918],
        [0,0,0,1]
        ]
    fkine = robot.fkine([0,0.76,1.15,-1.5])
    print(fkine)
    sol = robot.ikine_LM(T,fkine)
    return sol[0]
invK(0.3,0,0)