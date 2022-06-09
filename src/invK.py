# -*- coding: utf-8 -*-
from roboticstoolbox import ETS as ET, DHLink, DHRobot, RevoluteDH
from spatialmath.base import *
from spatialmath import SE3
import numpy as np

### x 0.188 y = 0.03 z =0.025

pi = np.pi

theta0 = np.arctan2(0.024,0.178)
robot = DHRobot(
     [
          RevoluteDH(d=0.0765,alpha=-pi/2),
          RevoluteDH(a=0.178,offset=-pi/2+theta0),
          RevoluteDH(a=0.174,offset=pi/2-theta0),
          RevoluteDH(a=0.126)
     ]
)
q0 = [0,0,0,0]
T = [
     [1,0,0,0.17415],
     [0,1,0,0.25],
     [0,0,1,0.7],
     [0,0,0,1]
     ]
T = np.array(T)
T = SE3(T)
sol = robot.ikine_LM(T,q0)
print(sol)