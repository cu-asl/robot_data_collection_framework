#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# initialize moveit_commander and a rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

# instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

# instantiate a PlanningSceneeInterface object
scene = moveit_commander.PlanningSceneInterface()

# instantiate a MoveGroupCommander object
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

planning_frame = move_group.get_planning_frame()
print('Planning frame: %s' % planning_frame)

eef_link = move_group.get_end_effector_link()
print ("============ End effector link: %s" % eef_link)

group_names = robot.get_group_names()
print ("============ Available Planning Groups:", robot.get_group_names())
print ("============ Printing robot state")
print (robot.get_current_state())
print ("=============")

joint_current = move_group.get_current_joint_values()

print('Current Joint valuses: ', joint_current)
print('Enter joint angles in radius')
joint_goal = [float(input("Enter angle: ")) for i in range(6)]
print('New goals for the robot:', joint_goal)

move_group.go(joint_goal, wait = True)

move_group.stop()