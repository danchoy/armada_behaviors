#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def insert_object(a, b):
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 0

    f = open(a,'r')
    sdff = f.read()

    rospy.wait_for_service(b)
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox("object", sdff, "robotos_name_space", initial_pose, "world")


if __name__ == "__main__":
    rospy.init_node('insert_object',log_level=rospy.INFO)
    insert_object('/home/csrobot/.gazebo/gazebo_models/table/model.sdf', 'gazebo/spawn_sdf_model')

else:
    print('what?')




