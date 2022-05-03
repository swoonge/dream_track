#!/usr/bin/env python
# -- coding: utf-8 --

## manipulator launch
# roslaunch open_manipulator_controller open_manipulator_controller.launch
# roslaunch open_manipulator_description open_manipulator_rviz.launch
# roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch

### read state msgs
## open_manipulator_controller's msg
# 1. /joint_states
#  header: 
#   seq: 35611
#   stamp: 
#     secs: 1651559599
#     nsecs:  64099015
#   frame_id: ''
# name: 
#   - joint1
#   - joint2
#   - joint3
#   - joint4
#   - gripper
# position: [-0.03528155758976936, -0.03834952041506767, 1.1550875902175903, 0.1840776950120926, 0.011780972778797149]
# velocity: [0.0, 0.0, 0.0, 0.0, 0.0]
# effort: [0.0, 0.0, 0.0, -2.690000057220459, 0.0]

# 2. /gripper/kinematics_pose
# pose: 
#   position: 
#     x: 0.11900239408258872
#     y: -0.0037767783556407995
#     z: -0.027545372443777152
#   orientation: 
#     x: 0.010681132727537306
#     y: 0.6054168497682235
#     z: -0.014038454340823698
#     w: 0.7957130596013224
# max_accelerations_scaling_factor: 0.0
# max_velocity_scaling_factor: 0.0
# tolerance: 0.0

## srv
# /set_actuator_state
#     open_manipulator_msgs/SetActuatorState
#     The user can use this service to control the state of actucators.
#     If the user set true at set_actuator_state valuable, the actuator will be enabled.
#     If the user set false at set_actuator_state valuable, the actuator will be disabled.

# /goal_joint_space_path
#     open_manipulator_msgs/SetJointPosition
#     The user can use this service to create a trajectory in the joint space.
#     The user inputs the angle of the target joint and the total time of the trajectory.


import rospy
from open_manipulator_msgs.msg import *

def main():
    print("clean test")

if __name__ == '__main__':
    rospy.init_node('control',anonymous=True)
    main()