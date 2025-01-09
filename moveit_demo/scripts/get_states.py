#!/usr/bin/python
# -*- coding: utf-8 -*-


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotTrajectory, Grasp, PlaceLocation, Constraints
from sensor_msgs.msg import JointState
## END_SUB_TUTORIAL

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

print(move_group.get_current_joint_values())

# joint_goal = move_group.get_current_joint_values()
# joint_goal[0] = -0.09710210646870741
# joint_goal[1] = 0.14022892195670564
# joint_goal[2] = -0.9363551844333426
# joint_goal[3] = -2.057766297941847
# joint_goal[4] = 0.1336904613169746
# joint_goal[5] = 2.1374147936139565
# joint_goal[6] = -0.3160219769148445

# move_group.set_joint_value_target(joint_goal)
# plan = RobotTrajectory()
# print(plan)

# 更改planners
# arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
# arm_group.set_planner_id('PRM')


