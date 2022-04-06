#!/usr/bin/env python3
import rospy
import os
from ur5_data_collect_fw.srv import PoseGoal_RPY
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
import csv
from std_srvs.srv import Empty, EmptyResponse
from rospkg import RosPack
import time
import copy

class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial',
        #                 anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Panda:
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path_cartesian',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print ("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print ("============ Printing robot state")
        print (robot.get_current_state())
        print ("")
        print(group.get_path_constraints())
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        
    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.y += 1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 1
        waypoints.append(copy.deepcopy(wpose))


        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.001,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

def control_position(req):
    # initialize moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)

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
    pose_goal.position.x = req.position.x
    pose_goal.position.y = req.position.y
    pose_goal.position.z = req.position.z
    roll_angle = req.rotation.x
    pitch_angle = req.rotation.y
    yaw_angle = req.rotation.z
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
    move_group.clear_pose_targets()
    return True

def csv_go(req):
    csv_path = os.path.join(RosPack().get_path('ur5_data_collect_fw'),"src/go_position.csv")
    print(csv_path)
    # rospy.wait_for_service('/control_position')
    csv_go_service = rospy.ServiceProxy('/control_position', PoseGoal_RPY)
    print("csv ready")
    with open(csv_path, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        csv_go_goal = PoseGoal_RPY()
        csv_go_position = geometry_msgs.msg.Pose().position
        csv_go_rotation = geometry_msgs.msg.Pose().position
        for row in csv_reader:
            print(f'\tX = {row["X"]}, Y = {row["Y"]}, Z = {row["Z"]}, R = {row["Roll"]}, P = {row["Pitch"]}, Y = {row["Yaw"]}.')
            csv_go_position.x = float(row["X"])
            csv_go_position.y = float(row["Y"])
            csv_go_position.z = float(row["Z"])
            csv_go_rotation.x = float(row["Roll"])
            csv_go_rotation.y = float(row["Pitch"])
            csv_go_rotation.z = float(row["Yaw"])
            csv_go_goal.position = csv_go_position
            csv_go_goal.rotation = csv_go_rotation
            # csv_go_service(csv_go_position, csv_go_rotation, "", "", "")
            try:
                csv_go_service(csv_go_position, csv_go_rotation, "", "", "")
                time.sleep(0)
            except rospy.ServiceException as e:
                print("rosservice: csv_go called failed")
    return EmptyResponse()

def cartesian_path(req):
    tutorial = MoveGroupPythonIntefaceTutorial()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()
    tutorial.display_trajectory(cartesian_plan)
    tutorial.execute_plan(cartesian_plan)
    return EmptyResponse()

def control_server():
    rospy.init_node('control_server')
    s1 = rospy.Service('/control_position',PoseGoal_RPY,control_position)
    s2 = rospy.Service('/csv_go',Empty,csv_go)
    s3 = rospy.Service('/cartesian_path', Empty, cartesian_path)
    print("Ready to Control.")
    rospy.spin()
    
if __name__ == "__main__":
    control_server()
