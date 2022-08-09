#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_behaviors.graspable_update_scene_sm import graspable_update_sceneSM
from armada_flexbe_behaviors.robot_move_sm import robot_moveSM
from armada_flexbe_behaviors.spawn_objects_sm import spawn_objectsSM
from armada_flexbe_states.stow import stow
from armada_flexbe_states.tuck import tuck
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 20 2022
@author: Dan
'''
class pick_demoSM(Behavior):
	'''
	picking soda can demo
	'''


	def __init__(self):
		super(pick_demoSM, self).__init__()
		self.name = 'pick_demo'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(graspable_update_sceneSM, 'graspable_update_scene')
		self.add_behavior(robot_moveSM, 'robot_move')
		self.add_behavior(spawn_objectsSM, 'spawn_objects')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:824 y:520, x:466 y:340
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:34 y:23
			OperatableStateMachine.add('spawn_objects',
										self.use_behavior(spawn_objectsSM, 'spawn_objects'),
										transitions={'finished': 'robot_move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:255 y:37
			OperatableStateMachine.add('robot_move',
										self.use_behavior(robot_moveSM, 'robot_move'),
										transitions={'finished': 'stow_before_pick_state', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:827 y:339
			OperatableStateMachine.add('stow_after_place_state',
										stow(),
										transitions={'continue': 'tuck', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:556 y:22
			OperatableStateMachine.add('stow_before_pick_state',
										stow(),
										transitions={'continue': 'graspable_update_scene', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:224 y:489
			OperatableStateMachine.add('tuck',
										tuck(),
										transitions={'continue': 'stow_before_pick_state', 'failed': 'stow_before_pick_state'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:878 y:52
			OperatableStateMachine.add('graspable_update_scene',
										self.use_behavior(graspable_update_sceneSM, 'graspable_update_scene'),
										transitions={'finished': 'stow_after_place_state', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
