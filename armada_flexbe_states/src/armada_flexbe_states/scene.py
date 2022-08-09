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



class update_scene(EventState):

    def __init__(self):
        """Constructor"""

        super(update_scene, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['object_list'])

        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        self._action_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % self._action_topic)
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

            goal = FindGraspableObjectsGoal()
            goal.plan_grasps = True

            rospy.loginfo("in on_enter waiting for topic")
            self._client.send_goal(self._action_topic, goal)
            self.find_client.send_goal(goal)
            self.find_client.wait_for_result(rospy.Duration(5.0))


            # find objects
            find_result = self._client.get_result(self._action_topic)
#            rospy.loginfo("%s"% find_result)


            # remove previous objects
            for name in self.scene.getKnownCollisionObjects():
                self.scene.removeCollisionObject(name, False)
            for name in self.scene.getKnownAttachedObjects():
                self.scene.removeAttachedObject(name, False)
            self.scene.waitForSync()

            # insert objects to scene
            objects = list()
            idx = -1
            for obj in find_result.objects:
                idx += 1
                obj.object.name = "object%d" % idx
                print(obj.object.name)
                self.scene.addSolidPrimitive(obj.object.name,
                                             obj.object.primitives[0],
                                             obj.object.primitive_poses[0],
                                             use_service=False)
                if obj.object.primitive_poses[0].position.x < 0.85:
                    objects.append([obj, obj.object.primitive_poses[0].position.z])

            for obj in find_result.support_surfaces:
                # extend surface to floor, and make wider since we have narrow field of view
                height = obj.primitive_poses[0].position.z
                obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                                1.5,  # wider
                                                obj.primitives[0].dimensions[2] + height]
                obj.primitive_poses[0].position.z += -height/2.0

                # add to scene
                self.scene.addSolidPrimitive(obj.name,
                                             obj.primitives[0],
                                             obj.primitive_poses[0],
                                             use_service=False)
            self.scene.waitForSync()

            # store for grasping
            #self.objects = find_result.objects
            self.surfaces = find_result.support_surfaces

            # store graspable objects by Z
            objects.sort(key=lambda object: object[1])
            objects.reverse()
            self.objects = [object[0] for object in objects]
            # for object in objects:
            #    print(object[0].object.name, object[1])
            # exit(-1)
            Logger.loginfo('attempting to segment planes from pointcloud...' )

#            rospy.loginfo("Am I going access objects")

#            rospy.loginfo("Waiting for self.object 2 %s... "% objects)
#            block = self.objects.object
#            grasps = self.objects.grasps

            userdata.object_list = self.objects

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



