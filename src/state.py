#!/usr/bin/env python
# -- coding: utf-8 --
import rospy

class bot():
    def __init__(self):
        self.state = 0 # 0 is def_mod, 1 is get_mod

# class server():
#     def __init__(self):

def main():
    print("clean test")

if __name__ == '__main__':
    rospy.init_node('state',anonymous=True)
    main()