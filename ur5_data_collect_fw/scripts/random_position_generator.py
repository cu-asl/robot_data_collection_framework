#!/usr/bin/env python3
import argparse, rospy,random,sys
from copy import deepcopy
from os import makedirs
from xml.dom import minidom
from datetime import datetime
from math import sqrt
class RandomPositionGenerator:
    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('filename',help="file input as format of config/item_spawn.xml")
        parser.add_argument('num_output',help="number of file output")
        parse = parser.parse_args(rospy.myargv()[1:]).filename
        filename = parser.parse_args(rospy.myargv()[1:]).filename
        num_output = parser.parse_args(rospy.myargv()[1:]).num_output
        if not filename.endswith("xml"):
            print("Please specify a file in format of config/item_spawn.xml and filename endswith xml")
            sys.exit()
        print(filename[filename.rfind("/")+1:filename.rfind(".")])
        folder = filename[:filename.rfind("/")+1]+datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        makedirs(folder)
        
        # read file
        file = minidom.parse(filename)
        org_model = file.getElementsByTagName('model')
        for num in range(int(num_output)):
            doc = minidom.Document()
            root = doc.createElement('root')
            doc.appendChild(root)
            models = doc.createElement('models')
            root.appendChild(models)
            for itemm in org_model:
                item = deepcopy(itemm)
                if item.hasAttribute('amount'):
                    if int(item.getAttribute('amount'))>1:
                        amount = int(item.getAttribute('amount'))
                        item.setAttribute('amount',"1")
                        print("set complete")
                        for i in range(amount):
                            new_item = deepcopy(item)
                            new_item.setAttribute('id',item.getAttribute('id')+f"_{i}")
                            new_item.setAttribute('xyz',self.random_position())
                            models.appendChild(new_item)
                    else:
                        models.appendChild(item)
                else:
                    models.appendChild(item)

            # export file
            with open(folder+f"/{num}.xml", "w") as text_file:
                text_file.write(doc.toprettyxml(indent = '   '))     

    def random_position(self,width=1.0,length=1.6,height=0.8):
        x = 0.4 * width * (-1+2*random.random())
        y = 0.4 * length * (-1+2*random.random())
        if abs(y) > sqrt(0.8**2-(x+0.4)**2): # for avoid spawning items out of UR5 workspace
            y = sqrt(0.8**2-(x+0.4)**2)
            if y < 0:
                y *= -1
        z = 1.5 * height
        return (" ").join([str(x),str(y),str(z)])

if __name__ == '__main__':
    try:
        RandomPositionGenerator()
    except:
        pass    
