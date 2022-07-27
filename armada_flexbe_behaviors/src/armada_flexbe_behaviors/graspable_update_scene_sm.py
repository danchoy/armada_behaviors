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
from armada_flexbe_states.pick_and_place_state import pick_and_place
from armada_flexbe_states.scene import update_scene
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

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:591 y:220, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.object_list = 0
		_state_machine.userdata.object_block = 0
		_state_machine.userdata.object_grasps = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:36 y:52
			OperatableStateMachine.add('wait_update',
										WaitState(wait_time=2),
										transitions={'done': 'update_scene'},
										autonomy={'done': Autonomy.Off})

			# x:716 y:35
			OperatableStateMachine.add('pick_and_place_state',
										pick_and_place(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_list': 'object_list', 'object_block': 'object_block', 'object_grasps': 'object_grasps'})

			# x:152 y:55
			OperatableStateMachine.add('update_scene',
										update_scene(),
										transitions={'continue': 'graspable_item', 'failed': 'wait_update'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_list': 'object_list'})

			# x:163 y:157
			OperatableStateMachine.add('wait_state',
										WaitState(wait_time=4),
										transitions={'done': 'graspable_item'},
										autonomy={'done': Autonomy.Off})

			# x:472 y:63
			OperatableStateMachine.add('graspable_item',
										graspable_item(),
										transitions={'continue': 'pick_and_place_state', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_list': 'object_list', 'object_block': 'object_block', 'object_grasps': 'object_grasps'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
