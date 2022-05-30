import rospy, subprocess, shlex, psutil
from datetime import datetime, timedelta
from time import time, sleep
from os.path import exists
from shutil import move

time2name = lambda dt,diff: (dt+timedelta(seconds=diff)).strftime("%Y-%m-%d-%H-%M-%S")+".bag"

class Bag:
    def export(self,name="",rename=True):
        if not name.endswith(".bag"):
            name += ".bag"
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(self.command[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)
        self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
        now = time()
        if rename:
            while not rospy.is_shutdown():
                if time()-now > 5:
                    rospy.loginfo("Requested Bag File Not Existed!")
                    break
                elif exists(time2name(self.time,0)):
                    move(time2name(self.time,0),name)
                    break
                elif exists(time2name(self.time,1)):
                    move(time2name(self.time,1),name)
                    break
                elif exists(time2name(self.time,2)):
                    move(time2name(self.time,2),name)
                    break
        sleep(0.1)
       
    def start(self,topic=[],compress=" --lz4"):
        topic = " ".join(topic)
        self.command = "rosbag record" + compress + " " + topic
        self.command = shlex.split(self.command)
        self.rosbag_proc = subprocess.Popen(self.command)
        self.time = datetime.now()