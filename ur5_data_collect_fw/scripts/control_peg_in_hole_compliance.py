#!/usr/bin/env python3
from time import sleep
import rospy, moveit_commander
from gazebo_msgs.srv import GetModelState, GetLinkState
import copy
from moveit_msgs.msg import DisplayTrajectory
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from checker import Checker
class ControlPegInHole():
    def __init__(self):
        rospy.init_node("control_peg_in_hole_compliance")
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.pub = rospy.Publisher("/my_motion_control_handle/feedback", InteractiveMarkerFeedback, queue_size=10)
        self.old_time = 0
        rospy.Subscriber("/move_group/display_planned_path",DisplayTrajectory,self.callback)
        self.manipulator = moveit_commander.MoveGroupCommander("manipulator")
        self.manipulator.set_named_target("home")
        self.manipulator.plan()
        self.wait()
        rate = rospy.Rate(10)
        checker1 = Checker()
        checker2 = Checker()
        # rospy.set_param("asl/spawn_complete", True) # <---- delete this!!!!!!!!!!!!
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
                self.manipulator.plan()
                self.wait()
                rospy.set_param("asl/spawn_next", True)
                if checker1.save_data[0][0].lower() == "fail":
                    rospy.set_param("asl/task_fail",True)
                    rospy.sleep(5)
            rate.sleep()
            if rospy.get_param("asl/spawn_finish",False):
                rospy.set_param("asl/spawn_finish",False)
                checker1.export("checker_force",True)
                checker2.export("checker_dist",True)
                rospy.set_param('asl/stop_ros',True) # to stop this program in run_loop.py
    
    def callback(self,data):
        self.move_time = data.trajectory[0].joint_trajectory.points[-1].time_from_start.to_sec()
    
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
        self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)
        self.wait()
        waypoints = []
        wpose.position.z -= 0.18
        waypoints.append(copy.deepcopy(wpose))
        self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)
        self.wait()
    
    def wait(self):
        while not rospy.is_shutdown():
            if self.move_time != self.old_time:
                self.old_time = self.move_time
                break
        rospy.sleep(self.move_time)
    
if __name__ == "__main__":
    try:
        ControlPegInHole()
    except:
        pass