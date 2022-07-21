#!/usr/bin/env python

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from tf import transformations





def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()


    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(sys.argv[1])
    goal.target_pose.pose.position.y = float(sys.argv[2])
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.w = 1.0



    pt = Point(x = float(sys.argv[1]), y = float(sys.argv[2]))
    qt = transformations.quaternion_from_euler(0, 0, float(sys.argv[3]))

    goal.target_pose.pose = Pose(position = pt,
                                 orientation = Quaternion(*qt))
    # Gazebo
    # x: 3.10445455822
    # y: -1.88048388782
    # RVIZ
    # x: 0.58872
    # y:-3.9773
#    -3.8571; -5.1199


    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
