#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.graspable_item import graspable_item
from armada_flexbe_states.head_action_state import PointHeadState as armada_flexbe_states__PointHeadState
from armada_flexbe_states.move_base_state import MoveBaseState as armada_flexbe_states__MoveBaseState
from armada_flexbe_states.pick_and_place_state import pick_and_place
from armada_flexbe_states.scene import update_scene
from armada_flexbe_states.stow import stow
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 26 2022
@author: Dan
'''
class graspable_update_sceneSM(Behavior):
	'''
	graspable item and update scene
	'''


	def __init__(self):
		super(graspable_update_sceneSM, self).__init__()
		self.name = 'graspable_update_scene'

		# parameters of this behavior
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
		# x:1012 y:555, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.object_list = 0
		_state_machine.userdata.object_block = 0
		_state_machine.userdata.object_grasps = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:88 y:13
			OperatableStateMachine.add('approch_again',
										armada_flexbe_states__MoveBaseState(a=0, b=-720, t=5),
										transitions={'arrived': 'wait_state', 'failed': 'approch'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1011 y:219
			OperatableStateMachine.add('graspable_item',
										graspable_item(),
										transitions={'continue': 'pick_and_place_state', 'failed': 'wait_state'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_list': 'object_list', 'object_block': 'object_block', 'object_grasps': 'object_grasps'})

			# x:526 y:18
			OperatableStateMachine.add('head_move',
										armada_flexbe_states__PointHeadState(x=self.head_pose_x, y=self.head_pose_y, z=self.head_pose_z, frame_name=self.head_frame),
										transitions={'continue': 'wait_update', 'failed': 'wait_state'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1007 y:330
			OperatableStateMachine.add('pick_and_place_state',
										pick_and_place(),
										transitions={'continue': 'stow', 'failed': 'stow'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_list': 'object_list', 'object_block': 'object_block', 'object_grasps': 'object_grasps'})

			# x:996 y:433
			OperatableStateMachine.add('stow',
										stow(),
										transitions={'continue': 'wait_state', 'failed': 'stow'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1006 y:20
			OperatableStateMachine.add('update_scene',
										update_scene(),
										transitions={'continue': 'graspable_item', 'failed': 'wait_state'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_list': 'object_list'})

			# x:279 y:16
			OperatableStateMachine.add('wait_state',
										WaitState(wait_time=2),
										transitions={'done': 'head_move'},
										autonomy={'done': Autonomy.Off})

			# x:803 y:15
			OperatableStateMachine.add('wait_update',
										WaitState(wait_time=2),
										transitions={'done': 'update_scene'},
										autonomy={'done': Autonomy.Off})

			# x:426 y:211
			OperatableStateMachine.add('approch',
										armada_flexbe_states__MoveBaseState(a=0, b=-710, t=5),
										transitions={'arrived': 'approch_again', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
