#!/usr/bin/env python


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D

from trajectory_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal

import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import MoveGroupInterface,PlanningSceneInterface, PickPlaceInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes



'''
Created on Wed Jul 20 2022
@author: Dan Choy
'''

# Send a trajectory to controller
class TrajectoryAction(EventState):

    def __init__(self, pose, joint_name):

        super(TrajectoryAction, self).__init__(outcomes = ['continue', 'failed'])

        self._action_topic = "torso_controller/follow_joint_trajectory"

        self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})
        self._continue = False
        self._failed = False
        self._x = pose
        self._joint_name = [joint_name]


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._continue:
            return 'continue'
        if self._failed:
            return 'failed'

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

        self._continue = False
        self._failed = False

        positions= [self._x, ]
        duration=5.0
        if len(self._joint_name) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self._joint_name
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory

        try:
                    self._client.send_goal(self._action_topic, goal)
        except Exception as e:
                    Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
                    self._failed = True




