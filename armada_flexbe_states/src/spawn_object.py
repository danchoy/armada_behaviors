#!/usr/bin/env python

#from uml_hri_nerve_pick_and_place.srv import SpawnObject, SpawnObjectResponse
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    print("Got it.")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    xml_file = open(("/home/csrobot/.gazebo/gazebo_models/table/model.sdf",'r')
    model_xml = xml_file.read()

#    Pose(position= Point(0,0,2),orientation=Quaternion(0,0,0,0)),"world")

except rospy.ServiceException as e:
        print("Service call failed: ",e)
