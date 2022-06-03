import os, signal, subprocess
import time, rospy, argparse
from rospkg.rospack import RosPack
from checker import Checker
from os.path import join, isdir, exists
import sys, rosnode
def main(args):
    num_open = 1
    # init save folder
    if isdir(RosPack().get_path('ur5_data_collect_fw')+"/"+args.open):
        filelist = [file for file in os.listdir(RosPack().get_path('ur5_data_collect_fw')+"/"+args.open) if file.endswith('.xml')]
        num_open = len(filelist)
    if not args.num_open == 0:
        num_open = args.num_open
    if args.save == None:
        path = RosPack().get_path('ur5_data_collect_fw')+"/rec/"
        onlyfolders = [folder for folder in os.listdir(path) if isdir(join(path, folder))]
        int_folder = 1
        while True:
            if str(int_folder) in onlyfolders:
                int_folder += 1
            else:
                path += str(int_folder) + "/"
                os.makedirs(path)
                args.save = f"rec/{int_folder}"
                break
    # init full command for roslaunch
    control_run = ""
    task_cmd = ["roslaunch","ur5_data_collect_fw"]
    if args.task == "1":
        task_cmd += ["peg.launch","control_peg:=true"]
        checker_list = ["checker_force","checker_dist"]
    elif args.task == "2":
        task_cmd += ["peg.launch","compliance_controller:=true","control_peg_compliance:=true"]
        checker_list = ["checker_force","checker_dist"]
    elif args.task == "3":
        task_cmd += ["gripper.launch","control_pick:=true"]
        checker_list = ["checker_bin"]
        control_run = "/control_pick"
    elif args.task == "4":
        task_cmd += ["peg_panda.launch","control_peg:=true"]
        checker_list = ["checker_force","checker_dist"]
    else:
        print(f"Please specify '-task' at value between 1-3")
        sys.exit()
    task_cmd += [f"spawn_save:={args.save}","rviz:=false","moveit_rviz:=false"]
    open_cmd = lambda x: [f"spawn_open:={args.open}"] if args.open.endswith(".xml") else [f"spawn_open:={args.open}/{x}.xml"]
    checker = Checker()
    print(" ".join(task_cmd))
    # start loop
    for i in range(num_open):
        breakloop = False
        repeat = 0
        while not breakloop:
            print(f"Loop No.{i}")
            rosrun = None
            process = subprocess.Popen(task_cmd+open_cmd(i), stdout=subprocess.PIPE, shell=False, preexec_fn=os.setsid)
            time.sleep(5) # wait for starting ROS
            rospy.init_node("run_loop",anonymous=True)
            now = time.time()
            time.sleep(10)
            print(f"Process PID = {process.pid}")
            while not control_run == "" and not control_run in rosnode.get_node_names():
                print("execute rosrun")
                rosrun = subprocess.Popen(["rosrun","ur5_data_collect_fw",f"{control_run[1:]}.py"], stdout=subprocess.PIPE, shell=False, preexec_fn=os.setsid)
                time.sleep(3)
            time.sleep(2)
            while True:
                if rospy.get_param('asl/stop_ros',False):
                    print("1st condition")
                    rospy.signal_shutdown("")
                    breakloop = True
                    break
                if (time.time()-now>args.timeout or rospy.get_param("asl/task_fail",False)) and repeat < 5:
                    print("2nd condition")
                    rospy.set_param('asl/stop_record',True)
                    time.sleep(5)
                    rospy.signal_shutdown("")
                    repeat += 1
                    print(f"repeat = {repeat}")
                    break
                elif time.time()-now>args.timeout:
                    print("3rd condition")
                    rospy.set_param('asl/stop_record',True)
                    time.sleep(5)
                    for name in checker_list:
                        checker.export(name,True,True)
                    rospy.signal_shutdown("")
                    breakloop = True
                    break
                time.sleep(0.2)
            if not rosrun == None:
                os.killpg(os.getpgid(rosrun.pid), signal.SIGINT)
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            print("Killing")
            time.sleep(30)
            print("After first kill")
            os.system(f"kill -9 {process.pid}")
            print("Kill complete")
    
if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-open", default="config/item_spawn.xml", help="path to xml file or folder")
        parser.add_argument("-num_open", default=0, type=lambda x: int(x), help="number of file config in the -open folder to open")
        parser.add_argument("-save", help="path to save generated complete xml file")
        parser.add_argument("-task",default="1", help="task specification: 1 for peg with trajectory, 2 for peg with complaince, 3 for pick and place, 4 for panda peg")
        parser.add_argument("-timeout",default=300, type=lambda x: float(x), help="time limit in sec for doing the task in one round")
        args = parser.parse_args(rospy.myargv()[1:])
        main(args)
    except:
        pass
