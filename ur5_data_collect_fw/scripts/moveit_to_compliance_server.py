#!/usr/bin/env python3

# This file is used to publish trajectory plan or a point calculated from moveit
# to a marker showed on RViz which created from motion_control_handle.
# By doing so, the cartesian motion controller or the cartesian compliance controller
# can control the "UR5 robotic arm" to the destination requested.
# To use this file with other robot, user might chage (not test) the file destination 
# (pack and filename) in fuction: urdf_reader to the new urdf (not xacro) for FK calculation.

from urdfpy import URDF
from rospkg.rospack import RosPack
from tf.transformations import quaternion_from_matrix, translation_from_matrix
import os, rospy, moveit_commander
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory, MoveGroupActionGoal, MoveGroupActionFeedback
from controller_manager_msgs.srv import ListControllers

class MoveItToComplianceServer():
    def __init__(self):
        rospy.init_node("moveit_to_compliance_server", anonymous=True)
        rospy.wait_for_service("/controller_manager/list_controllers")
        self.list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers",ListControllers)
        self.pub = rospy.Publisher("/my_motion_control_handle/feedback", InteractiveMarkerFeedback, queue_size=10)
        self.manipulator = moveit_commander.MoveGroupCommander("manipulator")
        self.urdf_reader()
        rospy.Subscriber("/move_group/display_planned_path",DisplayTrajectory,self.path_handle)
        rospy.Subscriber("/move_group/goal",MoveGroupActionGoal,self.point_handle)
        rospy.spin()

    def check_running_controller(self,controller_name):
        current_controllers = dict([[controller.name,controller.state] for controller in self.list_controllers().controller])
        if controller_name in current_controllers:
            if current_controllers[controller_name] == "running":
                return True
            else:
                return False
    
    def joint_to_marker(self,joint):
        fk = self.robot.link_fk(cfg={'shoulder_pan_joint': joint[0],'shoulder_lift_joint': joint[1],'elbow_joint': joint[2],'wrist_1_joint': joint[3],'wrist_2_joint': joint[4],'wrist_3_joint': joint[5]})
        self.pub_marker(fk[self.robot.links[6]]) 
        
    def path_handle(self,data):
        now = rospy.Time.now()
        if self.check_running_controller("my_cartesian_compliance_controller"):
            for point in data.trajectory[0].joint_trajectory.points:
                time_from_start = point.time_from_start.secs*1e9 + point.time_from_start.nsecs
                self.joint_to_marker(point.positions)
                while not rospy.is_shutdown():
                    if (rospy.Time.now() - now).to_nsec() > time_from_start:
                        break
    
    def point_handle(self,data):
        if not "succes" in rospy.wait_for_message("/move_group/feedback",MoveGroupActionFeedback).status.text:
            goal_state = [joint.position for joint in data.goal.request.goal_constraints[0].joint_constraints]
            self.joint_to_marker(goal_state)
    
    def pub_marker(self,point):
        tran = translation_from_matrix(point)
        quar = quaternion_from_matrix(point)
        marker = InteractiveMarkerFeedback()
        marker.header.frame_id = "base_link"
        marker.client_id = "/rviz/InteractiveMarkers"
        marker.marker_name = "motion_control_handle"
        marker.event_type = 1
        pose = Pose()
        pose.position.x = tran[0]
        pose.position.y = tran[1]
        pose.position.z = tran[2]
        pose.orientation.x = quar[0]
        pose.orientation.y = quar[1]
        pose.orientation.z = quar[2]
        pose.orientation.w = quar[3]
        marker.pose = pose
        self.pub.publish(marker)
        
    def urdf_reader(self):
        pack = RosPack().get_path('ur5_data_collect_fw')
        filename = pack+"/urdf/ur5.urdf" 
        with open(filename) as file:
            data = file.read()
        count = 0
        while True:
            count += 1
            a = data.find("package")
            if a == -1:
                break
            b = data[a+10:].find("/")
            package = RosPack().get_path(data[a+10:a+10+b])
            data = data.replace(data[a:a+10+b],package)
        textfile = open(filename+".tmp","w")
        textfile.write(data)
        textfile.close()
        self.robot = URDF.load(filename+".tmp")
        os.remove(filename+".tmp")
        
if __name__ == "__main__":
    try:
        MoveItToComplianceServer()
    except rospy.ROSInterruptException:
        pass