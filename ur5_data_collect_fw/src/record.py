#!/usr/bin/env python3
import subprocess, shlex, psutil
import argparse
import cv2
import rospy
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from rospkg.rospack import RosPack
import numpy as np
import pandas as pd
from os.path import exists
class Record():
    def __init__(self):
        rospy.init_node("record")
        parser = argparse.ArgumentParser()
        parser.add_argument("-topic", help="list of topics to record", nargs="*")
        parser.add_argument("-joint_topic", default=[], help="list of sensor_msgs/JointState to record as csv", nargs="*")
        parser.add_argument("-video_topic", default=[], help="list of sensor_msgs/Image topic to record as other video format", nargs="*")
        parser.add_argument("-video_type", default="mp4", type=lambda x: "avi" if str(x).lower() == "avi" else "mp4", help="video recording format -> [avi, mp4]")
        parser.add_argument("-save_path", default=RosPack().get_path('ur5_data_collect_fw') + "/rec/", help="place to save file")
        parser.add_argument("-compress", default="lz4", type=lambda x: " --" + str(x).lower() if str(x).lower() in ["lz4","bz2"] else "", help="compress bag file or not -> [No, lz4, bz2]")
        parser.add_argument("-unique", default=False, type=lambda x: (str(x).lower() in ['true','1', 'yes']), help="extract topic which already save as other type from bag file")
        parser.add_argument("-no_bag", default=False, type=lambda x: (str(x).lower() in ['true','1', 'yes']), help="record to bag file or not")
        self.args = parser.parse_args(rospy.myargv()[1:])
        print(self.args)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if rospy.get_param("asl/start_record", False):
                self.name = str(rospy.get_param("asl/output_name",""))
                if self.args.save_path == RosPack().get_path('ur5_data_collect_fw') + "/rec/" and rospy.get_param("asl/save_path",False)!=False:
                    self.save_path = rospy.get_param("asl/save_path")
                print(self.save_path)
                for topic in self.args.joint_topic:
                    exec("self.joint" + topic.replace("/","_") + "_start = False")
                    exec("self.joint" + topic.replace("/","_") + "_isRecord = False")    
                    rospy.Subscriber(topic,JointState,self.joint_record,topic)
                for topic in self.args.video_topic:
                    exec("self.video" + topic.replace("/","_") + "_start = False")
                    exec("self.video" + topic.replace("/","_") + "_isRecord = False")    
                    rospy.Subscriber(topic,Image,self.video_record,topic)
                break
        
        while not rospy.is_shutdown():
            # waiting to start record
            if rospy.get_param("asl/start_record", False):
                self.name = str(rospy.get_param("asl/output_name",""))
                if self.args.save_path == RosPack().get_path('ur5_data_collect_fw') + "/rec/" and rospy.get_param("asl/save_path",False)!=False:
                    self.save_path = rospy.get_param("asl/save_path")
                print("recording")
                rospy.set_param("asl/start_record", False)
                rospy.set_param("asl/stop_record", False)
                if self.args.topic == None:
                    self.args.topic = [topic[0] for topic in rospy.get_published_topics()]
                if self.args.video_topic != None and self.args.unique:
                    self.args.topic = [topic for topic in self.args.topic if not topic in self.args.video_topic and not topic in self.args.joint_topic]
                    if len(self.args.topic) == 0:
                        self.args.no_bag = True
                if not self.args.no_bag:
                    self.bag_start_record()
                for topic in self.args.joint_topic:
                    exec("self.joint" + topic.replace("/","_") + "_start = True")     
                for topic in self.args.video_topic:
                    exec("self.video" + topic.replace("/","_") + "_start = True")                      
                
                while not rospy.is_shutdown():
                    # wating to stop record
                    if rospy.get_param("asl/stop_record", False):
                        print("stoping")
                        if not self.args.no_bag:
                            self.bag_stop_record()
                        for topic in self.args.joint_topic:
                            self.joint_stop_record(topic)
                        for topic in self.args.video_topic:
                            self.video_stop_record(topic)
                        break
                    rate.sleep()
            rate.sleep()
            
    def bag_start_record(self):
        self.command = "rosbag record" + self.args.compress + " -o " + self.save_path + self.name + ".bag " + " ".join(self.args.topic)
        self.command = shlex.split(self.command)
        self.rosbag_proc = subprocess.Popen(self.command)

    def bag_stop_record(self):
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(self.command[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)
        self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
        rospy.sleep(2)
    
    def joint_record(self,data,args):
        exec("self.joint_name"+args.replace("/","_")+" = data.name")
        if eval("self.joint"+args.replace("/","_")+"_start"):
            exec("self.joint_position"+args.replace("/","_")+" = np.empty((0,len(data.position)), int)")
            exec("self.joint_velocity"+args.replace("/","_")+" = np.empty((0,len(data.velocity)), int)")
            exec("self.joint"+args.replace("/","_")+'''_start = False''')
            exec("self.joint"+args.replace("/","_")+'''_isRecord = True''')
        if eval("self.joint"+args.replace("/","_")+"_isRecord"):
            exec("self.joint_position"+args.replace("/","_")+" = np.append(self.joint_position"+args.replace("/","_")+",np.array([data.position]),axis=0)")
            exec("self.joint_velocity"+args.replace("/","_")+" = np.append(self.joint_velocity"+args.replace("/","_")+",np.array([data.velocity]),axis=0)")
        
    def joint_stop_record(self,topic):
        exec("pd.DataFrame(self.joint_position"+topic.replace("/","_")+''').to_csv(self.save_path+self.name+topic.replace("/","_")+"_position.csv",header=self.joint_name'''+topic.replace("/","_")+", index=None)")
        exec("pd.DataFrame(self.joint_velocity"+topic.replace("/","_")+''').to_csv(self.save_path+self.name+topic.replace("/","_")+"_velocity.csv",header=self.joint_name'''+topic.replace("/","_")+", index=None)")
        
    def video_record(self,data,args):
        if data.encoding == "32FC1": # depth image convertor
            cv_image = CvBridge().imgmsg_to_cv2(data, "32FC1")
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            cv_image_array = 255 * cv_image_array / np.nanmax(cv_image_array)
            cv_image_uint8_1c = cv_image_array.astype(np.uint8)
            cv_image = np.repeat(cv_image_uint8_1c[:, :, np.newaxis], 3, axis=2)
        else:
            cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8") # rgb/bgr convertor for picture from GAZEBO
        if eval("self.video"+args.replace("/","_")+"_start"):
            fourcc = lambda x: cv2.VideoWriter_fourcc(*"mp4v") if x == "mp4" else cv2.VideoWriter_fourcc(*"MJPG")
            exec("self.video"+args.replace("/","_")+'''= cv2.VideoWriter(self.save_path+self.name+args.replace("/","_")+'.' +self.args.video_type,fourcc(self.args.video_type), 20, (data.width,data.height))''')
            exec("self.video"+args.replace("/","_")+'''_start = False''')
            exec("self.video"+args.replace("/","_")+'''_isRecord = True''')
            if not exists(self.save_path+self.name+args.replace("/","_")+'_before.jpg'):
                cv2.imwrite(self.save_path+self.name+args.replace("/","_")+'_before.jpg',cv_image)
        if eval("self.video"+args.replace("/","_")+"_isRecord"):
            exec("self.video"+args.replace("/","_")+'''.write(cv_image)''')
            cv2.imwrite(self.save_path+self.name+args.replace("/","_")+'_after.jpg',cv_image)
        cv2.waitKey(3)

    def video_stop_record(self,topic):
        exec("self.video"+topic.replace("/","_")+"_isRecord = False")
        exec("self.video"+topic.replace("/","_")+'''.release()''')
        
if __name__ == "__main__":
    try:
        Record()
        
    except:
        pass