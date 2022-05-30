import rospy
from gazebo_msgs.msg import ContactsState
import numpy as np

class ContactsStates:
    def __init__(self,topic="asl/contact_sensor"):
        self.topic = topic
        self.start()
        rospy.Subscriber(topic, ContactsState, self.callback)
        
    def callback(self,data):
        if len(data.states) > 0:
            info = data.states[0].info.strip()
            self.time.append(float(info[info.rfind("time:")+5:]))
            force = data.states[0].total_wrench.force
            torque = data.states[0].total_wrench.torque
            self.state.extend([force.x,force.y,force.z,torque.x,torque.y,torque.z])
        
    def export(self,name,time_relative=True):
        name += "_" + self.topic[self.topic.rfind("/")+1:]
        if not name.endswith(".csv"):
            name += ".csv"
        time_array = np.array(self.time)
        state_array = np.array(self.state).reshape((-1,6))
        if time_relative and len(time_array)>0:
            time_array -= time_array[0]
        data = np.concatenate((time_array.reshape((-1,1)),state_array),axis=1)
        header = ["Time(s)","force.x","force.y","force.z","torque.x","torque.y","torque.z"]
        np.savetxt(name,data,delimiter=",",header=",".join(header),comments="")
    
    def start(self):
        self.time = []
        self.state = []