#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.head_action_state import PointHeadState as armada_flexbe_states__PointHeadState
from armada_flexbe_states.move_base_state import MoveBaseState as armada_flexbe_states__MoveBaseState
from armada_flexbe_states.torso_action_state import TrajectoryAction as armada_flexbe_states__TrajectoryAction
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 18 2022
@author: Dan
'''
class robot_moveSM(Behavior):
	'''
	robot approching to the table
	'''


	def __init__(self):
		super(robot_moveSM, self).__init__()
		self.name = 'robot_move'

		# parameters of this behavior
		self.add_parameter('theta', 5)
		self.add_parameter('robot_goal_pose_x', 0)
		self.add_parameter('robot_goal_pose_y', -600)
		self.add_parameter('robot_ori_pose_z', 0.8565)
		self.add_parameter('robot_ori_pose_w', 0.516)
		self.add_parameter('torso_pose_x', 0.4)
		self.add_parameter('torso_joint_name', 'torso_lift_joint')
		self.add_parameter('head_pose_x', 0)
		self.add_parameter('head_pose_y', -800)
		self.add_parameter('head_pose_z', 50)
		self.add_parameter('head_frame', 'map')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:797 y:39, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:36 y:28
			OperatableStateMachine.add('torso_action',
										armada_flexbe_states__TrajectoryAction(pose=self.torso_pose_x, joint_name=self.torso_joint_name),
										transitions={'continue': 'move', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:215 y:26
			OperatableStateMachine.add('move',
										armada_flexbe_states__MoveBaseState(a=self.robot_goal_pose_x, b=self.robot_goal_pose_y, t=self.theta),
										transitions={'arrived': 'finished', 'failed': 'torso_action'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:380 y:22
			OperatableStateMachine.add('wait',
										WaitState(wait_time=2),
										transitions={'done': 'head_move_action'},
										autonomy={'done': Autonomy.Off})

			# x:531 y:24
			OperatableStateMachine.add('head_move_action',
										armada_flexbe_states__PointHeadState(x=self.head_pose_x, y=self.head_pose_y, z=self.head_pose_z, frame_name=self.head_frame),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
