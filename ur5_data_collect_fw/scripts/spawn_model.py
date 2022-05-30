#!/usr/bin/env python3
from math import sqrt
import rospy, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties
from geometry_msgs.msg import *
from urdf_parser_py import urdf 
from std_srvs.srv import Empty
from xml.dom import minidom
from rospkg.rospack import RosPack
import copy
from datetime import datetime
import argparse
from os import listdir, makedirs
from os.path import join, isdir, exists

class SpawnModels():
    def __init__(self):
        rospy.init_node("spawn_model")
        rate =rospy.Rate(10)
        
        # read arguments
        parser = argparse.ArgumentParser()
        parser.add_argument("-open", default=RosPack().get_path('ur5_data_collect_fw') + "/config/item_spawn.xml", help="path to xml file or folder")
        parser.add_argument("-save_path", default=RosPack().get_path('ur5_data_collect_fw') + "/rec/", help="path to save generated complete xml file")
        parser.add_argument("-fix_path", help="fix path to save generated complete xml file (no new folders)")
        parser.add_argument("-world_items", default=['ground_plane', 'kinect', 'kinect_pilar', 'robot'], help="items which will not be deleted",nargs="*")
        args = parser.parse_args(rospy.myargv()[1:])
        self.world_items = args.world_items
        # wait for GAZEBO to start
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_urdf_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        self.unpause_physics = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
        self.pause_physics = rospy.ServiceProxy("gazebo/pause_physics", Empty)
        self.get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        # init save path folder
        if args.fix_path == None:
            if not args.save_path[-1] == "/":
                args.save_path += "/"
            if not exists(args.save_path):
                makedirs(args.save_path)
            onlyfolders = [folder for folder in listdir(args.save_path) if isdir(join(args.save_path, folder))]
            int_folder = 1
            while True:
                if str(int_folder) in onlyfolders:
                    int_folder += 1
                else:
                    save_path = args.save_path + str(int_folder) + "/"
                    makedirs(save_path)
                    rospy.set_param("asl/save_path",save_path)
                    break
        else:
            if not args.fix_path[-1] == "/":
                args.fix_path += "/"
            rospy.set_param("asl/save_path",args.fix_path)
            
        # file reading and spawning items
        if isdir(args.open):
            filelist = [file for file in listdir(args.open) if file.endswith('.xml')]
        else:
            filelist = [args.open[args.open.rfind('/')+1:]]
            args.open = args.open[:args.open.rfind('/')+1]
        if not args.open.endswith("/"):
            args.open += "/"
        for file in sorted(filelist):
            rospy.set_param("asl/output_name",file[:file.find('.xml')])
            self.xml_path = args.open + file
            self.pause_physics()
            self.readXml()
            self.spawnModel()
            self.unpause_physics()
            rospy.set_param("asl/spawn_complete",True)
            rospy.set_param("asl/spawn_next",False)
            if file == sorted(filelist)[-1]:
                rospy.set_param("asl/spawn_finish",True)
            else:
                while not rospy.is_shutdown():
                    if rospy.get_param("asl/spawn_next",False):
                        break
                    rate.sleep()
            
                    
    def readXml(self):
        # parse an xml file by name
        file = minidom.parse(self.xml_path)
        prefull_file = list()
        self.full_file = list()
        models = file.getElementsByTagName('model')
        unknown_object_name = 0
        name_dict = dict()
        for model in models:
            temp = FilledObject("","","","","","","","","","","","")
            if model.hasAttribute('id'):
                name = model.getAttribute('id')
            else:
                unknown_object_name += 1
                name = "object_"+str(unknown_object_name)
            if name in name_dict:
                name_dict[name] += 1
                name = name + "_" + chr(64+name_dict[name])
                name_dict[name] = 0
            else:
                name_dict[name] = 0
            temp.id = name
            if model.hasAttribute('type') and model.getAttribute('type').lower() in ["box","cylinder","sphere"]:
                filetype = model.getAttribute('type')
                if filetype.lower() == "box":
                    if model.hasAttribute('box_size'):
                        box_size = model.getAttribute('box_size')
                        if len(box_size.split())!=3 or len([True for i in box_size.split() if i.count(".")<=1])<3 or \
                        len(box_size)!=len([True for i in box_size if i in ". 0123456789"]): 
                            temp.box_size = 'error, please specify 3 float, "0.5 0.5 0.5"'
                            temp.error = True
                        else:
                            temp.box_size = box_size
                elif filetype.lower() == "cylinder":
                    if model.hasAttribute('radius'):
                        radius = model.getAttribute('radius')
                        if len(radius)!=len([True for i in radius.strip() if i in ".0123456789"]) or radius.count(".")>1: 
                            temp.radius = 'error, please specify 1 float'
                            temp.error = True
                        else:
                            temp.radius = radius
                    if model.hasAttribute('length'):
                        length = model.getAttribute('length')
                        if len(length)!=len([True for i in length.strip() if i in ".0123456789"]) or length.count(".")>1: 
                            temp.length = 'error, please specify 1 float'
                            temp.error = True
                        else:
                            temp.length = length
                elif filetype.lower() == "sphere":
                    if model.hasAttribute('radius'):
                        radius = model.getAttribute('radius')
                        if len(radius)!=len([True for i in radius.strip() if i in ".0123456789"]) or radius.count(".")>1: 
                            temp.radius = 'error, please specify 1 float'
                            temp.error = True
                        else:
                            temp.radius = radius
            elif model.hasAttribute('filename'):
                filename = model.getAttribute('filename')
                temp.filename = filename
                filetype = filename[filename.rfind(".")+1:]
            else:
                filetype = "error, invalid type"
            temp.type = filetype
            if not filetype.lower() in ["urdf","box","cylinder","sphere"]:
                temp.error = True
            if model.hasAttribute('mass'):
                mass = model.getAttribute('mass')
                if len(mass)!=len([True for i in mass.strip() if i in ".0123456789"]) or mass.count(".")>1: 
                    temp.mass = 'error, please specify 1 float'
                    temp.error = True
                else:
                    temp.mass = mass
            if model.hasAttribute('mu1'):
                mu1 = model.getAttribute('mu1')
                if len(mu1)!=len([True for i in mu1.strip() if i in ".0123456789"]) or mu1.count(".")>1: 
                    temp.mu1 = 'error, please specify 1 float'
                    temp.error = True
                else:
                    temp.mu1 = mu1
            if model.hasAttribute('mu2'):
                mu2 = model.getAttribute('mu2')
                if len(mu2)!=len([True for i in mu2.strip() if i in ".0123456789"]) or mu2.count(".")>1: 
                    temp.mu2 = 'error, please specify 1 float'
                    temp.error = True
                else:
                    temp.mu2 = mu2
            if model.hasAttribute('color'):
                temp.color = model.getAttribute('color')
            if model.hasAttribute('xyz'):
                xyz = model.getAttribute('xyz')
                if len(xyz.split())!=3 or len([True for i in xyz.split() if isfloat(i)])<3: 
                    temp.xyz = 'error, please specify 3 float, "0.5 0.5 0.5"'
                    temp.error = True
                else:
                    temp.xyz = xyz
            if model.hasAttribute('amount'):
                amount = model.getAttribute("amount")
                if amount.isdigit() and not "_" in amount:
                    if int(amount) >= 1:
                        temp.amount = amount
                    elif int(amount) == 0:
                        temp.amount = "0"
                        temp.error = True  
                    else: 
                        temp.amount = "error, invalid amount"
                        temp.error = True    
                else:
                    temp.amount = "error, invalid amount"
                    temp.error = True                    
            else:
                temp.amount = "1"
            prefull_file.append(temp)
        
        randXyz = RandomXyz()
        for model in prefull_file:
            models = copy.deepcopy(model)
            amount = models.amount
            name = models.id
            if amount.isdigit():
                if int(amount) > 1:
                    plus_number = 0
                    for i in range(int(amount)):
                        models = copy.deepcopy(model)
                        models.id = name + "_" + str(i+plus_number)
                        while models.id in name_dict:
                            plus_number += 1
                            models.id = name + "_" + str(i+plus_number)    
                        models.amount = 1
                        if models.xyz == "":
                            models.xyz = randXyz.xyz()
                        self.full_file.append(models)
                else:
                    if models.xyz == "":
                        models.xyz = randXyz.xyz()
                    self.full_file.append(models)
            else:
                if models.xyz == "":
                    models.xyz = randXyz.xyz()
                self.full_file.append(models)
        
        self.exportXml(self.full_file,True)  
    
    def exportXml(self, object_list, sorting=False):
        if sorting:
            name_list = []
            for num, val in enumerate(object_list):
                Id = [e[::-1] for e in val.id[::-1].partition("_")][::-2]
                if Id[0] == "":
                    Id = Id[::-1]
                elif not Id[1].isdigit():
                    Id = ["_".join(Id),""]
                if Id[1] == "":
                    Id[1] = "-1"
                name_list.append([num, Id])
            object_order = [obj[0] for obj in sorted(name_list, key=lambda x:(x[1][0], int(x[1][1])))]
        else:
            object_order = range(len(object_list))
        
        doc = minidom.Document()
        root = doc.createElement('root')
        doc.appendChild(root)
        
        models = doc.createElement('models')
        root.appendChild(models)
        attributeList = ["id","type","color","xyz","filename","amount","mass","box_size","radius","length","mu1","mu2"]
        for order in object_order:
            obj = object_list[order]
            if not obj.error:
                model = doc.createElement('model')
                models.appendChild(model)
                for attr in attributeList:
                    if eval("obj."+attr) != "":
                        model.setAttribute(attr, str(eval("obj."+attr)))
        #open & write text file
        with open(rospy.get_param("asl/save_path")+rospy.get_param("asl/output_name","")+".xml", "w") as text_file:
            text_file.write(doc.toprettyxml(indent = '   '))        
        
    def createObjectXml(self,geometry="box", color="Green", mass=1.0,
                    box_size="0.05 0.05 0.05",
                    radius="0.025", length="0.05",
                    mu1=1.0, mu2=1.0):
        
        item = urdf.Robot(name=geometry+"_"+color)
        if color == "":
            color = "Green"
        else:
            color = color[0].upper() + color[1:].lower()
        if mass == "":
            mass = 1.0
        else:
            mass = float(mass)
        if mu1 == "":
            mu1 = 1.0
        if mu2 == "":
            mu2 = 1.0
        if geometry == "box":
            if box_size == "":
                box_size = "0.05 0.05 0.05"
            item = urdf.Robot(name=geometry+"_"+color)
            length = [float(num) for num in box_size.split()]
            shape = urdf.Box(length)
            inertia = urdf.Inertia(1/12*mass*(length[1]**2+length[2]**2),0,0,
                                1/12*mass*(length[0]**2+length[2]**2),0,
                                1/12*mass*(length[0]**2+length[1]**2))
            
        elif geometry == "cylinder":
            if radius == "":
                radius = 0.025
            else:      
                radius = float(radius)
            if length == "":
                length = 0.05
            else:
                length = float(length)
            item = urdf.Robot(name=geometry+"_"+color)
            shape = urdf.Cylinder(float(radius),float(length))
            inertia = urdf.Inertia(1/12*mass*(3*radius**2+length**2),0,0,
                                1/12*mass*(3*radius**2+length**2),0,
                                1/2*mass*radius**2)
        
        elif geometry == "sphere":
            if radius == "":
                radius = 0.025
            else:      
                radius = float(radius)
            item = urdf.Robot(name=geometry+"_"+color)
            shape = urdf.Sphere(float(radius))
            inertia = urdf.Inertia(2/5*mass*radius**2,0,0,
                                2/5*mass*radius**2,0,
                                2/5*mass*radius**2)    
        
        item.add_link(urdf.Link("link",urdf.Visual(shape), 
                                urdf.Inertial(mass,inertia),
                                urdf.Collision(shape)))
        
        item_xml = item.to_xml_string()
        item_xml = item_xml[:item_xml.find("</robot>")] + \
        """  <gazebo reference="link">
            <material>Gazebo/{color}</material>
            <mu1>{mu1}</mu1>
            <mu2>{mu2}</mu2>
        </gazebo>
        </robot>"""
        
        return item_xml.format(color=color, mu1=float(mu1), mu2=float(mu2))

    def spawnModel(self):
        for model in self.get_world_properties().model_names:
            if not model in self.world_items:
                self.delete_model(model)
        time_delay(2)
        for model in self.full_file:
            if not model.error:        
                if model.type.lower() in ["box","cylinder","sphere"]:
                    product_xml = self.createObjectXml(geometry=model.type.lower(), 
                                                       color=model.color, mass=model.mass,
                                                       box_size=model.box_size, radius=model.radius,
                                                       length=model.length, mu1=model.mu1, mu2=model.mu2)     
                elif model.type.lower() == "urdf":
                    if model.filename.find("$(find") >= 0:
                        package = model.filename.strip()[7:model.filename.strip().find(")")]
                        path = RosPack().get_path(package)+model.filename[model.filename.find("/"):]
                    else:
                        path = RosPack().get_path('ur5_data_collect_fw')+model.filename
                    product_xml = minidom.parse(path).toxml()
                item_pose = Pose()
                if model.xyz != "":
                    pose = [float(num) for num in model.xyz.split()]
                    item_pose.position.x = pose[0]
                    item_pose.position.y = pose[1]
                    item_pose.position.z = pose[2]
                else:
                    item_pose.position.x = 0.4
                    item_pose.position.y = 0.1
                    item_pose.position.z = 2
                print(model.id, product_xml, "", item_pose, "ground_plane::link")
                self.spawn_urdf_model(model.id, product_xml, "", item_pose, "ground_plane::link")

class FilledObject():
    
    def __init__(self, id, type, color, xyz, filename,amount,mass,box_size,radius,length,mu1,mu2):
        self.id = id
        self.type = type
        self.color = color
        self.xyz = xyz
        self.filename = filename
        self.amount = amount
        self.mass = mass
        self.box_size = box_size
        self.radius = radius
        self.length = length
        self.mu1 = mu1
        self.mu2 = mu2
        self.error = False
        
def time_delay(sec):
    now = datetime.now()
    delta_time = datetime.now() - now
    while delta_time.seconds < sec:
        delta_time = datetime.now() - now
        
class RandomXyz():
    
    def __init__(self):
        if exists(RosPack().get_path('ur5_data_collect_fw')+"/urdf/desk.xacro"):
            desk_file = minidom.parse(RosPack().get_path('ur5_data_collect_fw')+"/urdf/desk.xacro")
            desk_property = desk_file.getElementsByTagName('xacro:property')
            desk_dict = dict()
            for prop in desk_property:
                desk_dict[prop.getAttribute("name")] = prop.getAttribute("value")
            self.width = float(desk_dict['plate_width'])
            self.length = float(desk_dict['plate_length'])
            self.height = float(desk_dict['desk_height'])
        else:
            self.width = 2
            self.length = 2
            self.height = 2
        
    def xyz(self):
        x = 0.4 * self.width * (-1+2*random.random())
        y = 0.4 * self.length * (-1+2*random.random())
        rospy.loginfo("Random XYZ: Y = "+str(y)+" X = " + str(x)+ " " + str(0.8**2-(x+0.4)**2))
        if abs(y) > sqrt(0.8**2-(x+0.4)**2): # for avoid spawning items out of UR5 workspace
            y = sqrt(0.8**2-(x+0.4)**2)
            if y < 0:
                y *= -1
        z = 1.5 * self.height
        print(1)
        print(x,y,z)
        return (" ").join([str(x),str(y),str(z)])

def isfloat(num):
    try:
        float(num)
        return True
    except ValueError:
        return False

if __name__ == "__main__":
    try:
        SpawnModels()
        
    except:
        pass