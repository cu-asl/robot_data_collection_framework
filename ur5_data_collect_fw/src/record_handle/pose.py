import rospy, moveit_commander
import numpy as np
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion

class Pose:
    def __init__(self,group="manipulator",topic="/tf"):
        self.topic = topic
        self.group = moveit_commander.MoveGroupCommander(group)
        self.start()
        rospy.Subscriber(topic,TFMessage,self.callback)
        
    def callback(self,data):
        time = data.transforms[0].header.stamp.to_nsec()/1e9
        pose = self.group.get_current_pose().pose
        self.data.extend([time,pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def export(self,name="",time_relative=True,to_euler=True):
        self.array = np.array(self.data).reshape((-1,8))
        if time_relative:
            sub = np.array(self.array[:,0])
            sub -= sub[0]
            self.array = np.concatenate((sub.reshape((-1,1)),self.array[:,1:]),axis=1)
        if to_euler:
            self.array = np.concatenate((self.array[:,:-4],euler_from_quaternion_array(np.array(self.array[:,-4:]))),axis=1)
            header = ["Time (s)","Position.X","Position.Y","Position.Z","Angle.X","Angle.Y","Angle.Z"]
            np.savetxt(name+"_pose.csv",self.array,delimiter=",",header=",".join(header),comments="")
        else:
            header = ["Time (s)","Position.X","Position.Y","Position.Z","Orientation.X","Orientation.Y","Orientation.Z","Orientation.W"]
            np.savetxt(name+"_pose.csv",self.array,delimiter=",",header=",".join(header),comments="")
       
    def start(self):
        self.data=[]

def euler_from_quaternion_array(data):
    euler = []
    for quaternion in data:
        euler.extend(euler_from_quaternion(quaternion))
    return np.array(euler).reshape((-1,3))