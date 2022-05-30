from math import sqrt
import rospy
from gazebo_msgs.msg import ContactsState, ModelStates, LinkStates
from gazebo_msgs.srv import GetLinkState, GetWorldProperties
from os.path import exists
import numpy as np
import pandas as pd
class Checker:
    def __init__(self):
        self.reset()
        
    def distance(self):
        last_link = [name for name in rospy.wait_for_message('gazebo/link_states',LinkStates).name if "robot" in name][-1]
        get_link_state = rospy.ServiceProxy('gazebo/get_link_state',GetLinkState)
        position = get_link_state(last_link,'box_sq::box_sq').link_state.pose.position
        position.z -= 0.1823 # distance between wrist_3 to end of peg
        disp = sqrt(position.x**2+position.y**2+position.z**2)
        if disp < 0.05:
            self.save_data.append(["Success",position.x,position.y,position.z])
        else:
            self.save_data.append(["Fail", 'out of reach',disp,''])
        
    def export(self,name="success_rate",add=False,timeout=False):
        path = rospy.get_param("asl/save_path")+name+".csv"
        if timeout:
            array = pd.DataFrame([["ERROR","Timeout"]])
        else:    
            array = pd.DataFrame(self.save_data)    
        if add and exists(path):
            array.to_csv(path,mode='a',index=False,header=False)
        else:
            array.to_csv(path,index=False)
        
    def force(self):
        count = 0
        forcetorque = []
        for i in range(10):
            msg = rospy.wait_for_message("asl/contact_sensor",ContactsState).states
            if len(msg)>0:
                count += 1
                force = msg[0].total_wrench.force
                torque = msg[0].total_wrench.torque
                forcetorque.append([force.x,force.y,force.z,torque.x,torque.y,torque.z])
        if count==10:
            self.save_data.append(["Success"] + [e for e in np.mean(np.array(forcetorque),0)/len(forcetorque)])
        else:
            self.save_data.append(["Fail", 'out of reach',count,'','','',''])
    
    def bin(self):
        data = rospy.wait_for_message("/gazebo/model_states",ModelStates)
        index_list = [index for index, item in enumerate(data.name) if not item in ["ground_plane","kinect","kinect_pilar","robot"]]
        on = 0
        inside = 0
        out = 0
        for index in index_list:
            xyz = data.pose[index].position
            if abs(xyz.x+1) < 0.25 and abs(xyz.y) < 0.25:
                inside += 1
            elif xyz.z > 0.8:
                on += 1
            else:
                out += 1
        if inside == len(index_list):
            self.save_data.append(["Success",inside,on,out])
        else:
            self.save_data.append(["Fail",inside,on,out])
            
    def reset(self):
        self.save_data = []