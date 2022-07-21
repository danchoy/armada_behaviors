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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 20 2022
@author: Dan
'''
class head_moveSM(Behavior):
	'''
	robot moving head
	'''


	def __init__(self):
		super(head_moveSM, self).__init__()
		self.name = 'head_move'

		# parameters of this behavior
		self.add_parameter('frame_name', 'map')
		self.add_parameter('head_pose_x', 0)
		self.add_parameter('head_pose_y', 0)
		self.add_parameter('head_pose_z', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('move_head',
										armada_flexbe_states__PointHeadState(x=self.head_pose_x, y=self.head_pose_y, z=self.head_pose_z, frame_name=self.frame_name),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
