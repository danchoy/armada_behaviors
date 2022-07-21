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


# Send a trajectory to controller


# Point the head using controller
class PointHeadState(EventState):

    def __init__(self, x, y, z, frame_name):

        super(PointHeadState, self).__init__(outcomes = ['continue', 'failed'])

        self._action_topic = "head_controller/point_head"

        self._client = ProxyActionClient({self._action_topic: PointHeadAction})

        self._continue = False
        self._failed = False
        self._x = (x / 100.0)
        self._y = (y / 100.0)
        self._z = (z / 100.0)
        self._frame_name = frame_name

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

        duration=1.0
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = self._frame_name
        goal.target.point.x = self._x
        goal.target.point.y = self._y
        goal.target.point.z = self._z
        goal.min_duration = rospy.Duration(duration)
        rospy.logerr("self.y = %.1f", self._z)

        try:
                    self._client.send_goal(self._action_topic, goal)
        except Exception as e:
                    Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
                    self._failed = True

