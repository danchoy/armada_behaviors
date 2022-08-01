#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from tf import transformations
import rospy
import actionlib
from math import sin, cos
from moveit_python import (MoveGroupInterface, PlanningSceneInterface, PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


'''
Created on Wed Jul 20 2022
@author: Dan Choy
'''


class stow(EventState):

    def __init__(self):
        """Constructor"""

        super(stow, self).__init__(outcomes = ['continue', 'failed'])

        self.move_group = MoveGroupInterface("arm", "base_link")

        self._action_topic = "basic_grasping_perception/find_objects"
#        rospy.loginfo("Waiting for %s..." % self._action_topic)
        self._client = ProxyActionClient({self._action_topic: FindGraspableObjectsAction})
        self.find_client = actionlib.SimpleActionClient(self._action_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()
        self._continue = False
        self._failed = False



    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        """Constructor"""

        if self._continue:
            return 'continue'
        if self._failed:
            return 'failed'
        rospy.loginfo("in execute waiting for topic")

        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._continue = True
                return 'continue'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'









    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        pose = [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0]
#        while not rospy.is_shutdown():
        self.move_group.moveToJointPosition(joints, pose, 0.02)
        pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        self.move_group.moveToJointPosition(joints, pose, 0.02)
        self._continue = True

    def on_exit(self, userdata):
            # This method is called when an outcome is returned and another state gets active.
            # It can be used to stop possibly running processes started by on_enter.

            pass # Nothing to do in this state.

    def on_start(self):
            # This method is called when the behavior is started.
            # If possible, it is generally better to initialize used resources in the constructor
            # because if anything failed, the behavior would not even be started.

            pass # Nothing to do in this state.

    def on_stop(self):
            # This method is called whenever the behavior stops execution, also if it is cancelled.
            # Use this event to clean up things like claimed resources.

            pass # Nothing to do in this state.






