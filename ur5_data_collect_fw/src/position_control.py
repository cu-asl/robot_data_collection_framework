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
from tf.transformations import quaternion_from_euler

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
print (move_group.get_current_rpy())
print ("=============")
# print(robot.move_group.get_current_pose().pose)
# print ("=============")

# joint_current = move_group.get_current_joint_values()

# print('Current Joint valuses: ', joint_current)
# print('Enter joint angles in radius')
# joint_goal = [float(input("Enter angle: ")) for i in range(6)]
# print('New goals for the robot:', joint_goal)

# move_group.go(joint_goal, wait = True)

# move_group.stop()
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = float(input('position x = '))
pose_goal.position.y = float(input('position y = '))
pose_goal.position.z = float(input('position z = '))
roll_angle = float(input('roll_angle = '))
pitch_angle = float(input('pitch_angle = '))
yaw_angle = float(input('yaw_angle = '))
quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
pose_goal.orientation.x = quaternion[0]
pose_goal.orientation.y = quaternion[1]
pose_goal.orientation.z = quaternion[2]
pose_goal.orientation.w = quaternion[3]

move_group.set_pose_target(pose_goal)
print('New goals for the robot:', pose_goal)

plan = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()