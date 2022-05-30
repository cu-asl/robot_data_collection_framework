#!/usr/bin/env python3
from turtle import clear
import rospy, moveit_commander, rostopic
from sensor_msgs.msg import JointState
from std_msgs.msg import *
import copy

class Playback():
    def __init__(self):
        rospy.init_node("playback")
        rospy.Subscriber("joint_states_playback",JointState,self.joint_to_controllers)
        rospy.spin()
    
    def joint_to_controllers(self,data):
        pub, sub = rostopic.get_topic_list()
        controllers = [[topic[0],topic[1][topic[1].rindex("/")+1:],joint_order] for topic in sub for joint_order in range(len(data.name)) if "command" in topic[0] and data.name[joint_order] in topic[0]]
        for controller in controllers:
            rospy.Publisher(controller[0],eval(controller[1]),queue_size=10).publish(data.position[controller[2]])

    
if __name__ == "__main__":
    try:
        Playback()
    except:
        pass