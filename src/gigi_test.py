#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import numpy as np
import time
from dbscan import *

def main():
    # a = [[1.0,2.0],[2.0,1.0],[1.5,2.0],[5.0,5.0],[4.5,4.7]]
    # b = [[1.3,2.2],[2.1,0.8],[3,2.0]]
    # data = np.array(a+b)
    # DB_test = DBSCAN(1.0, 2)
    # DB_test.run(data.tolist())
    # DB_test.plot()
    print(np.linalg.norm([3,4]))

if __name__ == '__main__':
    main()