#!/usr/bin/env python3
import rospy, moveit_commander
from gazebo_msgs.srv import *

import copy

class ControlRandomPick():
    def __init__(self):
        rospy.init_node("control_random_pick")
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        get_wolrd_prop = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        self.manipulator = moveit_commander.MoveGroupCommander("manipulator")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if rospy.get_param("asl/spawn_complete", False):
                rospy.set_param("asl/spawn_complete", False)
                self.manipulator.set_named_target("home")
                self.manipulator.go(wait=True)
                rospy.set_param("asl/start_record", True)
                items = [item for item in get_wolrd_prop().model_names if not item in ["ground_plane","kinect","kinect_pilar","robot"]]
                for item in items:
                    if rospy.is_shutdown():
                        break
                    xyz = get_model_state(item,"world").pose.position
                    self.pickKnown(xyz.x, xyz.y ,-1,0) 
                rospy.set_param("asl/stop_record", True)
                rospy.sleep(3)
                rospy.set_param("asl/spawn_next", True)
            rate.sleep()
        
    def pickKnown(self,x,y,bin_x,bin_y):
        # open gripper
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        # go above item
        self.manipulator.set_pose_target([x,y,1.25,-1.5708,0,0])
        self.manipulator.go(wait=True)
        # move down
        waypoints = []
        wpose = self.manipulator.get_current_pose().pose
        wpose.position.z -= 0.1
        waypoints.append(copy.deepcopy(wpose))
        self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        # close gripper to grasp item
        self.gripper.set_joint_value_target([0,0.3,0.3,0,0.3,0.3,0,0.3,0.3])
        self.gripper.go(wait=True)
        # go above bin
        self.manipulator.set_pose_target([bin_x,bin_y,1.25,-1.5708,0,0])
        self.manipulator.go(wait=True)
        # open gripper to release item
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        
if __name__ == "__main__":
    try:
        ControlRandomPick()
    except:
        pass