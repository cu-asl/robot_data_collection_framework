#!/usr/bin/env python3
from math import radians
from numpy import sign
import rospy, moveit_commander
from gazebo_msgs.srv import *
import copy
from checker import Checker
class ControlPick():
    def __init__(self):
        rospy.init_node("control_pick")
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        get_wolrd_prop = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        self.manipulator = moveit_commander.MoveGroupCommander("manipulator")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        rate = rospy.Rate(10)
        checker = Checker()
        # rospy.set_param("asl/spawn_complete", True) # <---- for testing this program
        while not rospy.is_shutdown():
            if rospy.get_param("asl/spawn_complete", False):
                rospy.set_param("asl/spawn_complete", False)
                self.manipulator.set_named_target("home")
                self.manipulator.go(wait=True)
                rospy.set_param("asl/start_record", True)
                rospy.sleep(2)
                items = [item for item in get_wolrd_prop().model_names if not item in ["ground_plane","kinect","kinect_pilar","robot"]]
                for item in items:
                    if rospy.is_shutdown():
                        break
                    xyz = get_model_state(item,"world").pose.position
                    if item != items[0]:
                        self.pickKnown(xyz.x, xyz.y, xyz.z, -1,0)
                    else: 
                        self.pickKnown(xyz.x, xyz.y, xyz.z, -1,0,False)
                checker.bin()
                rospy.sleep(2)
                rospy.set_param("asl/spawn_finish",True)
                rospy.set_param("asl/stop_record", True)
                rospy.sleep(2)
                rospy.set_param("asl/spawn_next", True)
                if checker.save_data[0][0].lower() == "fail":
                    rospy.set_param("asl/task_fail",True)
                    rospy.sleep(5)
            if rospy.get_param("asl/spawn_finish",False):
                rospy.set_param("asl/spawn_finish",False)
                checker.export("checker_bin",True)
                rospy.set_param('asl/stop_ros',True) # to stop this program in run_loop.py
            rate.sleep()
        
    def pickKnown(self,x,y,z,bin_x,bin_y,reset=True):
        # open gripper
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        if reset:
            # go to edge of table
            waypoints = []
            wpose = self.get_link_state("robot::wrist_3_link","world").link_state.pose
            wpose.position.x = -0.4
            wpose.position.y = 0.3*sign(y)
            waypoints.append(copy.deepcopy(wpose))
            self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        # go above item
        waypoints = []
        wpose = self.get_link_state("robot::wrist_3_link","world").link_state.pose
        wpose.position.x = x-0.015
        wpose.position.y = y
        wpose.position.z = z + 0.4
        waypoints.append(copy.deepcopy(wpose))
        self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        # move down
        waypoints = []
        wpose = self.get_link_state("robot::wrist_3_link","world").link_state.pose
        wpose.position.z -= 0.1
        waypoints.append(copy.deepcopy(wpose))
        self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        # close gripper to grasp item
        self.gripper.set_joint_value_target([0,0,radians(25),0,0,radians(25),0,0,radians(25)])
        self.gripper.go(wait=True)
        # move up
        waypoints = []
        wpose = self.get_link_state("robot::wrist_3_link","world").link_state.pose
        wpose.position.z += 0.2
        waypoints.append(copy.deepcopy(wpose))
        self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        # go to edge of table
        waypoints = []
        wpose = self.get_link_state("robot::wrist_3_link","world").link_state.pose
        wpose.position.x = -0.4
        wpose.position.y = 0.3*sign(y)
        waypoints.append(copy.deepcopy(wpose))
        self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        # go above bin
        waypoints = []
        wpose = self.get_link_state("robot::wrist_3_link","world").link_state.pose
        wpose.position.x = bin_x
        wpose.position.y = bin_y
        waypoints.append(copy.deepcopy(wpose))
        self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        # open gripper to release item
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
    
if __name__ == "__main__":
    try:
        ControlPick()
    except:
        pass