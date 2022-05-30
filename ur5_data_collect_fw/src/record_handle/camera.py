import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from shutil import move
from os.path import exists
from os import remove, rename
from datetime import datetime

fourcc = lambda x: cv2.VideoWriter_fourcc(*"mp4v") if x == "mp4" else cv2.VideoWriter_fourcc(*"MJPG")

class Camera():
    def __init__(self,topic="camera/rgb/image_raw",ext="mp4"):
        self.topic = topic
        self.ext = ext
        last = topic.rfind("/")
        self.type = topic[topic[:last].rfind("/")+1:last]
        self.out = self.type + "." + ext
        self.start()
        rospy.Subscriber(topic,Image,self.callback)
        
    def callback(self,data):
        if data.encoding == "32FC1": # depth image convertor
            cv_image = CvBridge().imgmsg_to_cv2(data, "32FC1")
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            cv_image_array = 255 * cv_image_array / np.nanmax(cv_image_array)
            cv_image_uint8_1c = cv_image_array.astype(np.uint8)
            self.cv_image = np.repeat(cv_image_uint8_1c[:, :, np.newaxis], 3, axis=2)
        else:
            self.cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8") # rgb/bgr convertor for picture from GAZEBO
        if self.record:
            self.output.write(self.cv_image)
            
        
    def export(self,name):
        name += "_"+self.type
        if not name.endswith("."+self.ext):
            name += "."+self.ext
        self.output.release()
        self.record = False
        if exists(self.out):
            rename(self.out,name)
    
    def snapshot(self,name="",name2=""):
        if name2!="":
            name = name + "_" + self.type + "_" + name2
        if not name.endswith(".jpg"):
            name += ".jpg"
        cv2.imwrite(name,self.cv_image)
    
    def start(self):
        print(self.topic)
        data = rospy.wait_for_message(self.topic,Image)
        if exists(self.out):
            remove(self.out)
        self.record = True
        self.output = cv2.VideoWriter(self.out,fourcc(self.ext), 20, (data.width,data.height))