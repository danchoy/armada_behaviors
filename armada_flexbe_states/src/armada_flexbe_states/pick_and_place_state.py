#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
#from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from geometry_msgs.msg import PoseStamped

from tf import transformations
import rospy
import actionlib
import scene
import copy


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





class pick_and_place(EventState):

    def __init__(self):
        super(pick_and_place, self).__init__(outcomes = ['continue', 'failed'],
                                    input_keys = ['object_list', 'object_block', 'object_grasps'])

        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
#            self.move_group = MoveGroupInterface("arm", "base_link")

        self._action_topic = "basic_grasping_perception/find_objects"
#            rospy.loginfo("Waiting for %s..." % self._action_topic)
        self._client = ProxyActionClient({self._action_topic: FindGraspableObjectsAction})
#            self.find_client = actionlib.SimpleActionClient(self._action_topic, FindGraspableObjectsAction)

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
            cube_in_grapper = False
            objects = userdata.object_list

            block = userdata.object_block
            grasps = userdata.object_grasps


            success, pick_result = self.pickplace.pick_with_retry(block.name, grasps, support_name=block.support_surface, scene=self.scene)
            self.pick_result = pick_result
            cube_in_grapper = True
#            rospy.loginfo("printing objects...%s" % userdata.object_block)

#            print("obj.object.primitive_poses[0], obj.object.primitives[0]")
#            print(block.primitive_poses[0], block.primitives[0])



            rospy.loginfo("Placing object...")
            pose = PoseStamped()
            pose.pose = block.primitive_poses[0]
            pose.pose.position.y *= -1.0
            pose.pose.position.z += 0.02
            pose.header.frame_id = block.header.frame_id

            places = list()
            l = PlaceLocation()
            l.place_pose.pose = pose.pose
            l.place_pose.header.frame_id = pose.header.frame_id

            # copy the posture, approach and retreat from the grasp used
            l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
            l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
            l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
            places.append(copy.deepcopy(l))
            # create another several places, rotate each by 360/m degrees in yaw direction
            m = 16  # number of possible place poses
            pi = 3.141592653589
            for i in range(0, m-1):
                l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 5 * pi / m)
                places.append(copy.deepcopy(l))

            success, place_result = self.pickplace.place_with_retry(block.name, places, scene=self.scene)
            cube_in_grapper = False


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



