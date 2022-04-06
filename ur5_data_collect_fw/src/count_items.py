#!/usr/bin/env python3
import rospy
import rospy
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelStates
import tkinter as tk
import tkinter.font as tkFont

class CountItems():
    def __init__(self):
        rospy.init_node("count_items")
        rospy.wait_for_service('/gazebo/get_world_properties')
        self.window = tk.Tk()
        self.window.title("Count Items")
        font15 = tkFont.Font(size=15)
        font20 = tkFont.Font(size=20)
        get_wolrd_prop = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        self.world_prop = [item for item in get_wolrd_prop().model_names if not item in ["ground_plane","kinect","kinect_pilar","robot"]]
        sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.callback)
        self.on = tk.StringVar()
        self.inside = tk.StringVar()
        self.out = tk.StringVar()
        tk.Label(self.window,text="On Desk",font=font15).pack()
        tk.Label(self.window,textvariable=self.on,font=font20).pack()
        tk.Label(self.window,text="Inside Bin",font=font15).pack()
        tk.Label(self.window,textvariable=self.inside,font=font20).pack()
        tk.Label(self.window,text="Out Of Reach",font=font15).pack()
        tk.Label(self.window,textvariable=self.out,font=font20).pack()
        self.window.mainloop()   

    def callback(self,data):
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
        self.on.set(str(on))
        self.inside.set(str(inside))
        self.out.set(str(out))
        rospy.set_param("/asl/count/on_desk",on)
        rospy.set_param("/asl/count/inside_bin",inside)
        rospy.set_param("/asl/count/out_of_reach",out)
        if rospy.is_shutdown():
            self.window.destroy()

if __name__ == "__main__":
    try:
        CountItems()
    except:
        pass