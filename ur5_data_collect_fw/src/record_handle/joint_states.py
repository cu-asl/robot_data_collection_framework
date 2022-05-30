import rospy
from sensor_msgs.msg import JointState
import numpy as np

class JointStates:
    def __init__(self,topic="joint_states",position=True,velocity=False,effort=False):
        self.topic = topic
        self.pos_rec = position
        self.vel_rec = velocity
        self.eff_rec = effort
        self.start()
        if position:
            rospy.Subscriber(topic, JointState, self.callback_position)
        if velocity:
            rospy.Subscriber(topic, JointState, self.callback_velocity)
        if effort:
            rospy.Subscriber(topic, JointState, self.callback_effort)
        
    def callback_effort(self,data):
        self.effort.append(data.header.stamp.to_nsec()/1e9)
        self.effort.extend(data.effort)
            
    def callback_position(self,data):
        self.position.append(data.header.stamp.to_nsec()/1e9)
        self.position.extend(data.position)
        
    def callback_velocity(self,data):
        self.velocity.append(data.header.stamp.to_nsec()/1e9)
        self.velocity.extend(data.velocity)
        
    def export(self,name="",time_rel=True):
        name += "_" + self.topic[self.topic.rfind("/")+1:]
        self.name = name
        position = list(self.position)
        velocity = list(self.velocity)
        effort = list(self.effort)
        if self.pos_rec:
            self.export_with_time(position,time_rel,"position")
        if self.vel_rec:
            self.export_with_time(velocity,time_rel,"velocity")
        if self.eff_rec:
            self.export_with_time(effort,time_rel,"effort")
    
    def export_with_time(self,data,time_rel,topic):
        data = np.array(data).reshape((-1,self.length+1))
        if time_rel:
            data[:,0] -= data[0,0]
        np.savetxt(self.name+"_"+topic+".csv",data,delimiter=",",header=",".join(["Time (s)"]+self.joint_names),comments="")
    
    def start(self):
        self.joint_names = rospy.wait_for_message(self.topic,JointState).name
        self.length = len(self.joint_names)
        self.position = []
        self.velocity = []
        self.effort = []
