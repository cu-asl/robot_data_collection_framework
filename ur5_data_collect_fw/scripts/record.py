#!/usr/bin/env python3
import rospy, argparse
import record_handle

class Record():
    def __init__(self):
        rospy.init_node('record')
        rospy.set_param("asl/start_record", False)
        self.parser()
        print(self.args)
        if len(self.args.bag_topic)>0:
            if "all" in self.args.bag_topic:
                self.args.bag_topic = ["-a"]
            bag = record_handle.Bag()
        handle = []
        for topic in self.args.contact_topic:
            handle.append(record_handle.ContactsStates(topic))
        for topic in self.args.joint_topic:
            handle.append(record_handle.JointStates(topic))
        if self.args.pose_record:
            handle.append(record_handle.Pose(self.args.pose_planning_group))
        if self.args.video_rgb_topic!="":
            rgb = record_handle.Camera(self.args.video_rgb_topic,ext=self.args.video_type)
        if self.args.video_depth_topic!="":
            depth = record_handle.Camera(self.args.video_depth_topic,ext=self.args.video_type)
        print("wait to start")
        while not rospy.is_shutdown():
            if rospy.get_param("asl/start_record", False):
                rospy.set_param("asl/start_record", False)
                rospy.set_param("asl/stop_record", False)
                save = rospy.get_param("asl/save_path","") + str(rospy.get_param("asl/output_name",-1))
                print(save)
                # start recording
                if len(self.args.bag_topic)>0:
                    bag.start(self.args.bag_topic,self.args.bag_compress)
                if len(handle)>0:
                    for topic_handle in handle:
                        topic_handle.start()   
                if self.args.video_rgb_topic!="":
                    rgb.start()
                    rgb.snapshot(save,"before")  
                if self.args.video_depth_topic!="":
                    depth.start()
                    depth.snapshot(save,"before")
                print("wait to stop")
                while not rospy.is_shutdown():
                    if rospy.get_param("asl/stop_record", False):
                        save = rospy.get_param("asl/save_path","") + str(rospy.get_param("asl/output_name",-1))
                        print(save)
                        rospy.set_param("asl/stop_record", False)
                        # stop recording
                        if len(self.args.bag_topic)>0:
                            bag.export(save)
                        if len(handle)>0:
                            for topic_handle in handle:
                                topic_handle.export(save)
                        if self.args.video_rgb_topic!="":
                            rgb.snapshot(save,"after")
                            rgb.export(save)
                        if self.args.video_depth_topic!="":
                            depth.snapshot(save,"after")
                            print("after snapshot")
                            depth.export(save)
                        print("wait to start")
                        break
                    rospy.sleep(0.2)
            rospy.sleep(0.2)

    def parser(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("-bag_topic", default=[], help="list of topics to record in bag, 'all' means record all topics", nargs="*")
        parser.add_argument("-bag_compress", default="lz4", type=lambda x: " --" + str(x).lower() if str(x).lower() in ["lz4","bz2"] else "", help="compress bag file or not -> [No, lz4, bz2]")
        parser.add_argument("-contact_topic", default=[], help="list of gazebo_msgs/ContactsState topic to record as csv", nargs="*")
        parser.add_argument("-joint_topic", default=[], help="list of sensor_msgs/JointState to record as csv", nargs="*")
        parser.add_argument("-pose_record", default=False, type=lambda x: (str(x).lower() in ['true','1', 'yes']), help="record pose using moveit or not")
        parser.add_argument("-pose_planning_group", default="manipulator", help="moveit's planning group to get pose")
        parser.add_argument("-video_rgb_topic", default="", help="a sensor_msgs/Image topic, rgb type, to record as other video format")
        parser.add_argument("-video_depth_topic", default="", help="a sensor_msgs/Image topic, depth type, to record as other video format")
        parser.add_argument("-video_type", default="mp4", type=lambda x: "avi" if str(x).lower() == "avi" else "mp4", help="video recording format -> [avi, mp4]")
        self.args = parser.parse_args(rospy.myargv()[1:])
        
if __name__ == '__main__':
    try:
        Record()
    except:
        pass    