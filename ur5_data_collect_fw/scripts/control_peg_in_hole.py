#!/usr/bin/env python3
from time import sleep
import rospy, moveit_commander
from gazebo_msgs.srv import GetModelState
import copy
from checker import Checker
class ControlPegInHole():
    def __init__(self):
        rospy.init_node("control_peg_in_hole")
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.manipulator = moveit_commander.MoveGroupCommander("manipulator")
        self.manipulator.set_named_target("home")
        self.manipulator.go(wait=True)
        rate = rospy.Rate(10)
        checker1 = Checker()
        checker2 = Checker()
        rospy.set_param("asl/spawn_complete", True) # <---- delete this!!!!!!!!!!!!
        while not rospy.is_shutdown():
            if rospy.get_param("asl/spawn_complete", False):
                rospy.set_param("asl/spawn_complete", False)
                rospy.set_param("asl/start_record", True)
                rospy.sleep(1)
                self.move_peg_to_hole()
                rospy.sleep(1)
                checker1.force()
                checker2.distance()
                rospy.set_param("asl/spawn_finish",True)
                rospy.set_param("asl/stop_record", True)
                rospy.sleep(1)
                self.manipulator.set_named_target("home")
                self.manipulator.go(wait=True)
                rospy.set_param("asl/spawn_next", True)
                if checker1.save_data[0][0].lower() == "fail":
                    rospy.set_param("asl/task_fail",True)
                    rospy.sleep(5)
                if checker2.save_data[0][0].lower() == "fail":
                    rospy.set_param("asl/task_fail",True)
                    rospy.sleep(5)
            rate.sleep()
            if rospy.get_param("asl/spawn_finish",False):
                rospy.set_param("asl/spawn_finish",False)
                checker1.export("checker_force",True)
                checker2.export("checker_dist",True)
                rospy.set_param('asl/stop_ros',True) # to stop this program in run_loop.py
    
    def move_peg_to_hole(self):
        xyz = self.get_model_state("box_sq","world").pose.position
        waypoints = []
        wpose = self.manipulator.get_current_pose().pose
        wpose.position = xyz
        wpose.position.z += 0.3
        wpose.orientation.x = 0
        wpose.orientation.y = 1
        wpose.orientation.z = 0
        wpose.orientation.w = 0
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z -= 0.18
        waypoints.append(copy.deepcopy(wpose))
        self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        # waypoints = []
        # wpose = self.manipulator.get_current_pose().pose
        # wpose.position.z -= 0.18
        # waypoints.append(copy.deepcopy(wpose))
        # self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
    
if __name__ == "__main__":
    try:
        ControlPegInHole()
    except:
        pass