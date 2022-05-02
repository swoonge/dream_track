#!/usr/bin/env python
# -- coding: utf-8 --
import rospy

def main():
    print("clean test")

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()